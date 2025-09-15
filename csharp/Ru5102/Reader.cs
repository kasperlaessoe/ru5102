using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO.Ports;

namespace Ru5102
{
    /// <summary>
    /// Driver for the CF-RU5102 UHF RFID reader (C# port of ru5102 Rust crate).
    /// </summary>
    public sealed class Reader : IDisposable
    {
        private readonly SerialPort _serialPort;
        private byte _address;
        private bool _disposed;

        public Reader(string portName)
        {
            if (string.IsNullOrWhiteSpace(portName))
            {
                throw new ArgumentException("Port name must be provided", nameof(portName));
            }

            _serialPort = new SerialPort(portName, 57600, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                ReadTimeout = 1000,
                WriteTimeout = 1000,
                NewLine = "\n"
            };

            try
            {
                if (!_serialPort.IsOpen)
                {
                    _serialPort.Open();
                }
            }
            catch (Exception ex)
            {
                throw new ReaderException(ReaderErrorKind.Io, $"Unable to connect to serial port {portName}: {ex.Message}", ex);
            }

            _address = 0;
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            _disposed = true;
            try
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.Close();
                }
            }
            catch
            {
                // Swallow exceptions during dispose
            }
            _serialPort.Dispose();
        }

        // ========================= CRC =========================

        private static ushort CalculateCrc(ReadOnlySpan<byte> data)
        {
            // CRC-16/MCRF4XX (poly 0x1021, init 0xFFFF, refin=false, refout=false, xorout=0x0000)
            ushort crc = 0xFFFF;
            for (int i = 0; i < data.Length; i++)
            {
                crc ^= (ushort)(data[i] << 8);
                for (int bit = 0; bit < 8; bit++)
                {
                    if ((crc & 0x8000) != 0)
                    {
                        crc = (ushort)((crc << 1) ^ 0x1021);
                    }
                    else
                    {
                        crc <<= 1;
                    }
                }
            }
            return crc;
        }

        // ========================= Model types =========================

        private enum CommandType : byte
        {
            // EPC C1 G2（ISO18000-6C) Commands
            Inventory = 0x01,
            ReadData = 0x02,
            WriteData = 0x03,
            WriteEpc = 0x04,
            KillTag = 0x05,
            Lock = 0x06,
            BlockErase = 0x07,
            ReadProtect = 0x08,
            ReadProtectWithoutEpc = 0x09,
            ResetReadProtect = 0x0a,
            CheckReadProtect = 0x0b,
            EasAlarm = 0x0c,
            CheckEasAlarm = 0x0d,
            BlockLock = 0x0e,
            InventorySingle = 0x0f,
            BlockWrite = 0x10,

            // ISO18000-6B Commands
            InventorySignal6B = 0x50,
            InventoryMultiple6B = 0x51,
            ReadData6B = 0x52,
            WriteData6B = 0x53,
            CheckLock6B = 0x54,
            Lock6B = 0x55,

            // Reader Commands
            GetReaderInformation = 0x21,
            SetRegion = 0x22,
            SetAddress = 0x24,
            SetScanTime = 0x25,
            SetBaudRate = 0x28,
            SetPower = 0x2F,
            AcoustoOpticControl = 0x33,
        }

        public enum ResponseStatus : byte
        {
            Ok = 0x00,
            ReturnBeforeInventoryFinished = 0x01,
            ScanTimeOverflow = 0x02,
            MoreData = 0x03,
            ReaderFlashFull = 0x04,
            AccessPasswordError = 0x05,
            KillTagError = 0x09,
            KillPasswordZero = 0x0A,
            CommandNotSupported = 0x0B,

            SaveFail = 0x13,
            CannotAdjust = 0x14,

            CommandExecuteError = 0xF9,
            PoorCommunication = 0xFA,
            NoTags = 0xFB,
            TagError = 0xFC,
            WrongLength = 0xFD,
            IllegalCommand = 0xFE,
            ParameterError = 0xFF,
        }

        private static bool IsSuccess(ResponseStatus status)
        {
            return status == ResponseStatus.Ok
                || status == ResponseStatus.ReturnBeforeInventoryFinished
                || status == ResponseStatus.ScanTimeOverflow
                || status == ResponseStatus.MoreData;
        }

        private readonly struct Command
        {
            public readonly byte Address;
            public readonly CommandType CommandCode;
            public readonly byte[] Data;

            public Command(byte address, CommandType commandCode, byte[] data)
            {
                Address = address;
                CommandCode = commandCode;
                Data = data ?? Array.Empty<byte>();
            }

            public byte[] ToBytes()
            {
                int packetLength = Data.Length + 4; // len + addr + cmd + data + crc(2) => len excludes its own byte but includes addr..data + crc
                var buffer = new byte[packetLength + 1];
                int index = 0;
                buffer[index++] = (byte)packetLength;
                buffer[index++] = Address;
                buffer[index++] = (byte)CommandCode;
                if (Data.Length > 0)
                {
                    Buffer.BlockCopy(Data, 0, buffer, index, Data.Length);
                    index += Data.Length;
                }
                ushort crc = CalculateCrc(new ReadOnlySpan<byte>(buffer, 0, index));
                buffer[index++] = (byte)(crc & 0xFF);
                buffer[index++] = (byte)((crc >> 8) & 0xFF);
                return buffer;
            }
        }

        private readonly struct Response
        {
            public readonly byte Address;
            public readonly byte Command;
            public readonly ResponseStatus Status;
            public readonly byte[] Data;

            private Response(byte address, byte command, ResponseStatus status, byte[] data)
            {
                Address = address;
                Command = command;
                Status = status;
                Data = data;
            }

            public static Response FromBytes(ReadOnlySpan<byte> bytes)
            {
                if (bytes.Length < 4)
                {
                    throw new ReaderException(ReaderErrorKind.Program, "Response too short");
                }

                if (bytes[0] != bytes.Length - 1)
                {
                    throw new ReaderException(ReaderErrorKind.Program, "Bad length prefix in response");
                }

                int len = bytes.Length;
                ushort expectedCrc = CalculateCrc(bytes.Slice(0, len - 2));
                ushort payloadCrc = (ushort)(bytes[len - 1] << 8 | bytes[len - 2]);
                if (payloadCrc != expectedCrc)
                {
                    throw new ReaderException(ReaderErrorKind.Program, "Bad CRC");
                }

                var payload = bytes.Slice(1, len - 3);
                byte address = payload[0];
                byte command = payload[1];
                byte statusByte = payload[2];
                ResponseStatus status = Enum.IsDefined(typeof(ResponseStatus), statusByte)
                    ? (ResponseStatus)statusByte
                    : throw new ReaderException(ReaderErrorKind.Program, $"Invalid status response: 0x{statusByte:X2}");

                var data = payload.Slice(3).ToArray();
                return new Response(address, command, status, data);
            }
        }

        public sealed class ReaderInformation
        {
            public byte[] Version { get; }
            public byte ReaderType { get; }
            public byte SupportedProtocols { get; }
            public byte MaxFrequency { get; }
            public byte MinFrequency { get; }
            public byte Power { get; }
            public byte ScanTime { get; }

            internal ReaderInformation(byte[] version, byte readerType, byte supportedProtocols, byte maxFrequency, byte minFrequency, byte power, byte scanTime)
            {
                Version = version;
                ReaderType = readerType;
                SupportedProtocols = supportedProtocols;
                MaxFrequency = maxFrequency;
                MinFrequency = minFrequency;
                Power = power;
                ScanTime = scanTime;
            }

            internal static ReaderInformation FromBytes(ReadOnlySpan<byte> bytes)
            {
                if (bytes.Length != 8)
                {
                    throw new ReaderException(ReaderErrorKind.Program, "Unexpected reader information length");
                }
                return new ReaderInformation(bytes.Slice(0, 2).ToArray(), bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]);
            }
        }

        public enum MemoryLocation : byte
        {
            Password = 0x00,
            Epc = 0x01,
            Tid = 0x02,
            User = 0x03,
        }

        public sealed class ReadCommand
        {
            public byte[] Epc { get; set; } = Array.Empty<byte>();
            public MemoryLocation Location { get; set; }
            public byte StartAddress { get; set; }
            public byte Count { get; set; }
            public byte[]? Password { get; set; }
            public byte? MaskAddress { get; set; }
            public byte? MaskLength { get; set; }

            internal byte[] ToBytes()
            {
                var packet = new List<byte>(2 + Epc.Length + 8);
                // EPC size is in words (2 bytes)
                packet.Add((byte)(Epc.Length / 2));
                packet.AddRange(Epc);
                packet.Add((byte)Location);
                packet.Add(StartAddress);
                packet.Add(Count);
                if (Password != null && Password.Length > 0)
                {
                    packet.AddRange(Password);
                }
                else
                {
                    packet.AddRange(new byte[] { 0, 0, 0, 0 });
                }
                if (MaskAddress.HasValue) packet.Add(MaskAddress.Value);
                if (MaskLength.HasValue) packet.Add(MaskLength.Value);
                return packet.ToArray();
            }
        }

        public sealed class WriteCommand
        {
            public byte[] Epc { get; set; } = Array.Empty<byte>();
            public MemoryLocation Location { get; set; }
            public byte StartAddress { get; set; }
            public byte[] Data { get; set; } = Array.Empty<byte>();
            public byte[]? Password { get; set; }
            public byte? MaskAddress { get; set; }
            public byte? MaskLength { get; set; }

            internal byte[] ToBytes()
            {
                var packet = new List<byte>(2 + Epc.Length + Data.Length + 8);
                // EPC and write size is in words (2 bytes)
                packet.Add((byte)(Data.Length / 2));
                packet.Add((byte)(Epc.Length / 2));
                packet.AddRange(Epc);
                packet.Add((byte)Location);
                packet.Add(StartAddress);
                packet.AddRange(Data);
                if (Password != null && Password.Length > 0)
                {
                    packet.AddRange(Password);
                }
                else
                {
                    packet.AddRange(new byte[] { 0, 0, 0, 0 });
                }
                if (MaskAddress.HasValue) packet.Add(MaskAddress.Value);
                if (MaskLength.HasValue) packet.Add(MaskLength.Value);
                return packet.ToArray();
            }
        }

        public sealed class KillCommand
        {
            public byte[] Epc { get; set; } = Array.Empty<byte>();
            public byte[] Password { get; set; } = Array.Empty<byte>();
            public byte? MaskAddress { get; set; }
            public byte? MaskLength { get; set; }

            internal byte[] ToBytes()
            {
                var packet = new List<byte>(2 + Epc.Length + 6);
                // EPC size is in words (2 bytes)
                packet.Add((byte)(Epc.Length / 2));
                packet.AddRange(Epc);
                packet.AddRange(Password);
                if (MaskAddress.HasValue) packet.Add(MaskAddress.Value);
                if (MaskLength.HasValue) packet.Add(MaskLength.Value);
                return packet.ToArray();
            }
        }

        // ========================= Public API =========================

        public ReaderInformation GetReaderInformation()
        {
            var cmd = new Command(_address, CommandType.GetReaderInformation, Array.Empty<byte>());
            var response = SendReceive(cmd);
            if (!IsSuccess(response.Status))
            {
                throw ReaderException.FromStatus(response.Status);
            }
            return ReaderInformation.FromBytes(response.Data);
        }

        /// <summary>
        /// Inventory all tags in the reader's range.
        /// Returns a list of EPC values as byte arrays.
        /// </summary>
        public List<byte[]> Inventory()
        {
            var cmd = new Command(_address, CommandType.Inventory, Array.Empty<byte>());
            var response = SendReceive(cmd);

            if (response.Status == ResponseStatus.NoTags)
            {
                return new List<byte[]>();
            }
            if (!IsSuccess(response.Status))
            {
                throw ReaderException.FromStatus(response.Status);
            }

            byte numTags = response.Data[0];
            int offset = 1;
            var tags = new List<byte[]>(numTags);
            for (int i = 0; i < numTags; i++)
            {
                byte tagLen = response.Data[offset++];
                var tag = new byte[tagLen];
                Buffer.BlockCopy(response.Data, offset, tag, 0, tagLen);
                offset += tagLen;
                tags.Add(tag);
            }
            return tags;
        }

        public byte[] ReadData(ReadCommand readCommand)
        {
            var cmd = new Command(_address, CommandType.ReadData, readCommand.ToBytes());
            var response = SendReceive(cmd);
            if (!IsSuccess(response.Status))
            {
                throw ReaderException.FromStatus(response.Status);
            }
            return response.Data;
        }

        public void WriteData(WriteCommand writeCommand)
        {
            var cmd = new Command(_address, CommandType.WriteData, writeCommand.ToBytes());
            var response = SendReceive(cmd);
            if (!IsSuccess(response.Status))
            {
                throw ReaderException.FromStatus(response.Status);
            }
        }

        public void Kill(KillCommand killCommand)
        {
            var cmd = new Command(_address, CommandType.KillTag, killCommand.ToBytes());
            var response = SendReceive(cmd);
            if (!IsSuccess(response.Status))
            {
                throw ReaderException.FromStatus(response.Status);
            }
        }

        // ========================= I/O =========================

        private Response SendReceive(Command cmd)
        {
            byte[] bytes = cmd.ToBytes();
            try
            {
                _serialPort.Write(bytes, 0, bytes.Length);

                int lengthPrefix = ReadExact(1)[0];
                var payload = ReadExact(lengthPrefix);
                var responseBuffer = new byte[1 + payload.Length];
                responseBuffer[0] = (byte)lengthPrefix;
                Buffer.BlockCopy(payload, 0, responseBuffer, 1, payload.Length);
                return Response.FromBytes(responseBuffer);
            }
            catch (TimeoutException tex)
            {
                throw new ReaderException(ReaderErrorKind.Io, "Timed out waiting for reader response", tex);
            }
            catch (ReaderException)
            {
                throw;
            }
            catch (Exception ex)
            {
                throw new ReaderException(ReaderErrorKind.Io, $"Serial I/O error: {ex.Message}", ex);
            }
        }

        private byte[] ReadExact(int count)
        {
            var buffer = ArrayPool<byte>.Shared.Rent(count);
            try
            {
                int totalRead = 0;
                while (totalRead < count)
                {
                    int read = _serialPort.Read(buffer, totalRead, count - totalRead);
                    if (read <= 0)
                    {
                        throw new TimeoutException("No data available from serial port");
                    }
                    totalRead += read;
                }
                var result = new byte[count];
                Buffer.BlockCopy(buffer, 0, result, 0, count);
                return result;
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(buffer);
            }
        }
    }

    public enum ReaderErrorKind
    {
        Io,
        Communication,
        Protocol,
        Program
    }

    public sealed class ReaderException : Exception
    {
        public ReaderErrorKind Kind { get; }
        public Reader.ReaderStatusWrapper? StatusWrapper { get; }

        public ReaderException(ReaderErrorKind kind, string message, Exception? innerException = null)
            : base(message, innerException)
        {
            Kind = kind;
        }

        internal static ReaderException FromStatus(Reader.ResponseStatus status)
        {
            return status switch
            {
                Reader.ResponseStatus.PoorCommunication => new ReaderException(ReaderErrorKind.Communication, $"Transient error communicating with tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.NoTags => new ReaderException(ReaderErrorKind.Communication, $"Transient error communicating with tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },

                Reader.ResponseStatus.AccessPasswordError => new ReaderException(ReaderErrorKind.Protocol, $"Error returned from tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.KillTagError => new ReaderException(ReaderErrorKind.Protocol, $"Error returned from tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.KillPasswordZero => new ReaderException(ReaderErrorKind.Protocol, $"Error returned from tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.CommandNotSupported => new ReaderException(ReaderErrorKind.Protocol, $"Error returned from tag: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },

                Reader.ResponseStatus.WrongLength => new ReaderException(ReaderErrorKind.Program, "Wrong command length") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.IllegalCommand => new ReaderException(ReaderErrorKind.Program, "Illegal command") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },
                Reader.ResponseStatus.ParameterError => new ReaderException(ReaderErrorKind.Program, "Parameter error") { StatusWrapper = new Reader.ReaderStatusWrapper(status) },

                _ => new ReaderException(ReaderErrorKind.Program, $"Invalid status response: {status}") { StatusWrapper = new Reader.ReaderStatusWrapper(status) }
            };
        }
    }

    public static class ReaderExtensions
    {
        public static string ToHex(this byte[] bytes)
        {
            return BitConverter.ToString(bytes).Replace("-", string.Empty);
        }
    }

    public partial class Reader
    {
        // Helper wrapper to expose ResponseStatus via exceptions without tight coupling
        public readonly struct ReaderStatusWrapper
        {
            public ResponseStatus Status { get; }
            public ReaderStatusWrapper(ResponseStatus status) { Status = status; }
            public override string ToString() => Status.ToString();
        }
    }
}

