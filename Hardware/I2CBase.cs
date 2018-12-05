using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PlaneOnBoardSoftware
{
    public abstract class I2CBase
    {
        public abstract I2CDevice.Configuration Config { get; }

        protected byte Read(byte address)
        {
            Byte[] data = new Byte[1];
            I2CDevice.I2CWriteTransaction write = I2CDevice.CreateWriteTransaction(new Byte[] { address });
            I2CDevice.I2CReadTransaction read = I2CDevice.CreateReadTransaction(data);
            I2CDevice.I2CTransaction[] readTransaction = new I2CDevice.I2CTransaction[] { write, read };

            var result = I2CBus.Execute(Config, readTransaction, 1000);

            return data[0];
        }

        protected float CombineBytes(byte high, byte low)
        {
            var data = (Int16)((high << 8) | low);

            return (float)data;
        }

        protected byte Read(byte address, byte[] buffer)
        {
            I2CDevice.I2CWriteTransaction write = I2CDevice.CreateWriteTransaction(new Byte[] { address });
            I2CDevice.I2CReadTransaction read = I2CDevice.CreateReadTransaction(buffer);
            I2CDevice.I2CTransaction[] readTransaction = new I2CDevice.I2CTransaction[] { write, read };

            var result = I2CBus.Execute(Config, readTransaction, 1000);

            return buffer[0];
        }

        protected void Write(byte address, byte value)
        {
            var bytes = new byte[] { address, value };

            I2CBus.Write(Config, bytes, 1000);
        }
    }
}
