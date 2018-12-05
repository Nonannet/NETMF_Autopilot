using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PlaneOnBoardSoftware
{
    public static class I2CBus
    {
        private const int Timeout = 1000;
        private static I2CDevice Device { get; set; }

        static I2CBus()
        {
            Device = new I2CDevice(new I2CDevice.Configuration(0x00, 400));
        }

        public static void Write(I2CDevice.Configuration configuration, byte[] bytes, int timeout = Timeout)
        {
            var actions = new I2CDevice.I2CTransaction[] {
                I2CDevice.CreateWriteTransaction(bytes)
            };

            var result = Execute(configuration, actions);
            if (result != bytes.Length)
            {
                //throw new ApplicationException("Expected to write " + bytes.Length + " bytes but only wrote " + result + " bytes");
            }
        }

        public static int Execute(I2CDevice.Configuration configuration, I2CDevice.I2CTransaction[] actions, int timeout = Timeout)
        {
            lock (Device)
            {
                Device.Config = configuration;
                return Device.Execute(actions, timeout);
            }
        }
    }
}
