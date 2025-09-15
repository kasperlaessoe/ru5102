using System;
using Ru5102;

namespace ReaderExample
{
    internal static class Program
    {
        private static void Main(string[] args)
        {
            if (args.Length < 1)
            {
                Console.Error.WriteLine("Usage: ReaderExample <serial-port>");
                return;
            }

            using var reader = new Reader(args[0]);
            var info = reader.GetReaderInformation();
            Console.WriteLine($"Reader info: Version={BitConverter.ToString(info.Version)} Type={info.ReaderType} Protocols={info.SupportedProtocols} Freq={info.MinFrequency}-{info.MaxFrequency} Power={info.Power} ScanTime={info.ScanTime}");

            while (true)
            {
                var inventory = reader.Inventory();
                if (inventory.Count > 0)
                {
                    foreach (var epc in inventory)
                    {
                        Console.WriteLine($"Found tag: {epc.ToHex()}");

                        // Example of reading additional data (commented out by default)
                        // var readCmd = new Reader.ReadCommand
                        // {
                        //     Epc = epc,
                        //     Location = Reader.MemoryLocation.Tid,
                        //     StartAddress = 0,
                        //     Count = 100,
                        //     Password = null,
                        //     MaskAddress = null,
                        //     MaskLength = null
                        // };
                        // var data = reader.ReadData(readCmd);
                        // Console.WriteLine($"Tag data: {data.ToHex()}");
                        Console.WriteLine();
                    }
                }
            }
        }
    }
}

