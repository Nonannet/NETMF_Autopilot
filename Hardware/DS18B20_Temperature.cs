using System;
using Microsoft.SPOT;
using GHIElectronics.NETMF.Hardware;
using Microsoft.SPOT.Hardware;
using System.Threading;
using GHIElectronics.NETMF.FEZ;

namespace PlaneOnBoardSoftware
{
    class DS18B20_Temperature
    {
        private const byte SearchROM = 0xF0;
        private const byte ReadROM = 0x33;
        private const byte MatchROM = 0x55;
        private const byte SkipROM = 0xCC;
        private const byte AlarmSearch = 0xEC;
        private const byte StartTemperatureConversion = 0x44;
        private const byte ReadScratchPad = 0xBE;
        private const byte WriteScratchPad = 0x4E;
        private const byte CopySratchPad = 0x48;
        private const byte RecallEEPROM = 0xB8;
        private const byte ReadPowerSupply = 0xB4;

        private OneWire DataPin;

        public float Temperature1 = -99;
        public float Temperature2 = -99;

        private bool ConvStarted = false;

        private byte[][] IDs = new byte[][] { new byte[8], new byte[8], new byte[8] };

        public DS18B20_Temperature(Cpu.Pin pin)
        {
            DataPin = new OneWire(pin);

            DataPin.Search_Restart();
            int i = 0;
            while (DataPin.Search_GetNextDevice(IDs[i++]))
            {
               Debug.Print("Found thermometer" + i);
            }
        }

        public void ReadTemperature()
        {
            long data = 0;

            if (DataPin.ReadByte() > 0 && ConvStarted)
            {
                //Read temperature1
                DataPin.Reset();
                DataPin.WriteByte(MatchROM);
                DataPin.Write(IDs[0], 0, 8);
                DataPin.WriteByte(ReadScratchPad);

                data = DataPin.ReadByte(); // LSB 
                data |= (ushort)(DataPin.ReadByte() << 8); // MSB
                Temperature1 = data * 0.0625f;

                //Read temperature2
                DataPin.Reset();
                DataPin.WriteByte(MatchROM);
                DataPin.Write(IDs[1], 0, 8);
                DataPin.WriteByte(ReadScratchPad);

                data = DataPin.ReadByte(); // LSB 
                data |= (ushort)(DataPin.ReadByte() << 8); // MSB
                Temperature2 = data * 0.0625f;


                ConvStarted = false;
            }

            DataPin.Reset();
            DataPin.WriteByte(MatchROM);
            DataPin.Write(IDs[0], 0, 8);
            DataPin.WriteByte(StartTemperatureConversion);

            DataPin.Reset();
            DataPin.WriteByte(MatchROM);
            DataPin.Write(IDs[1], 0, 8);
            DataPin.WriteByte(StartTemperatureConversion);

            ConvStarted = true;
        }
    }
}