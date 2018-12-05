using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PlaneOnBoardSoftware
{
    public class LSM303DLM_Magnetometer : I2CBase
    {
        private class Register
        {
            public const byte CRA_REG_M = 0x00;
            public const byte CRB_REG_M = 0x01;
            public const byte MR_REG_M  = 0x02;
            public const byte OUT_X_L_M = 0x04;
            public const byte OUT_X_H_M = 0x03;
            public const byte OUT_Y_L_M = 0x06;
            public const byte OUT_Y_H_M = 0x05;
            public const byte OUT_Z_L_M = 0x08;
            public const byte OUT_Z_H_M = 0x07;
        }

        private I2CDevice.Configuration _config;
        private bool _PowerDown = false;

        public float CalXoffSet = 0;
        public float CalYoffSet = 0;
        public float CalZoffSet = 0;
        public float CalXscale = 1f / 320;
        public float CalYscale = 1f / 320;
        public float CalZscale = 1f / 320;

        private float maxX = 0;
        private float minX = 0;
        private float maxY = 0;
        private float minY = 0;
        private float maxZ = 0;
        private float minZ = 0;
        private bool _calibrationmode = false;

        public Vector3D MagValue = new Vector3D();

        public bool CalibrationMode
        {
            get { return _calibrationmode; }
            set
            {
                _calibrationmode = value;
                if (_calibrationmode)
                {
                    maxX = 0;
                    minX = 0;
                    maxY = 0;
                    minY = 0;
                    maxZ = 0;
                    minZ = 0;
                }
                else
                {
                    CalXoffSet = (maxX + minX) / 2;
                    CalYoffSet = (maxY + minY) / 2;
                    CalZoffSet = (maxZ + minZ) / 2;
                    CalXscale = 2f / (maxX - minX);
                    CalYscale = 2f / (maxY - minY);
                    CalZscale = 2f / (maxZ - minZ);
                }
            }
        }

        public LSM303DLM_Magnetometer()
        {
            _config = new I2CDevice.Configuration(0x1E, 400);

            // 0x10 = 00010000 -> 15 Hz update rate
            Write(Register.CRA_REG_M, 0x10);

            // 0xC0 = 11000000 -> Gain: ±5.6 Gauss, 320 LSB/Gauss
            Write(Register.CRB_REG_M, 0xC0);

            // 0x00 = 00000000  Normal mode
            Write(Register.MR_REG_M, 0x00);
        }

        public bool PowerDown
        {
            get
            {
                return _PowerDown;
            }
            set
            {
                if (value)
                    Write(Register.MR_REG_M, 0x03);  // 0x03 = 00000011  Power-down
                else
                    Write(Register.MR_REG_M, 0x00);  // 0x00 = 00000000  Normal mode

                _PowerDown = value;
            }
        }

        private float GetMagValue(byte high, byte low)
        {
            var data = (Int16)((Read(high) << 8) | Read(low));

            return (((float)(data)));
        }

        public int RequestMagValue()
        {
            float x, y, z;
            byte[] bufferXhi = new byte[1];
            byte[] bufferYhi = new byte[1];
            byte[] bufferZhi = new byte[1];
            byte[] bufferXlo = new byte[1];
            byte[] bufferYlo = new byte[1];
            byte[] bufferZlo = new byte[1];
            I2CDevice.I2CTransaction[] readTransaction = new I2CDevice.I2CTransaction[12];
            I2CDevice.I2CWriteTransaction write;
            I2CDevice.I2CReadTransaction read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_X_H_M });
            read = I2CDevice.CreateReadTransaction(bufferXhi);
            readTransaction[0] = write;
            readTransaction[1] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_X_L_M });
            read = I2CDevice.CreateReadTransaction(bufferXlo);
            readTransaction[2] = write;
            readTransaction[3] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Y_H_M });
            read = I2CDevice.CreateReadTransaction(bufferYhi);
            readTransaction[4] = write;
            readTransaction[5] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Y_L_M });
            read = I2CDevice.CreateReadTransaction(bufferYlo);
            readTransaction[6] = write;
            readTransaction[7] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Z_H_M });
            read = I2CDevice.CreateReadTransaction(bufferZhi);
            readTransaction[8] = write;
            readTransaction[9] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Z_L_M });
            read = I2CDevice.CreateReadTransaction(bufferZlo);
            readTransaction[10] = write;
            readTransaction[11] = read;

            var result = I2CBus.Execute(Config, readTransaction, 1000);

            x = CombineBytes(bufferXhi[0], bufferXlo[0]);
            z = CombineBytes(bufferYhi[0], bufferYlo[0]); // Y <--> Z Exchanged
            y = CombineBytes(bufferZhi[0], bufferZlo[0]); // Z <--> Y Exchanged

            if (_calibrationmode)
            {
                if (x > maxX) maxX = x;
                if (x < minX) minX = x;
                if (y > maxY) maxY = y;
                if (y < minY) minY = y;
                if (z > maxZ) maxZ = z;
                if (z < minZ) minZ = z;
            }

            MagValue.X = -(x - CalXoffSet) * CalXscale;
            MagValue.Z = (z - CalZoffSet) * CalZscale; // Y <--> Z Exchanged
            MagValue.Y = -(y - CalYoffSet) * CalYscale; // Z <--> Y Exchanged

            return result;
        }

        public float GetX()
        {
            return GetMagValue(Register.OUT_X_H_M, Register.OUT_X_L_M) * CalXscale + CalXoffSet;
        }

        public float GetZ()
        {
            return GetMagValue(Register.OUT_Y_H_M, Register.OUT_Y_L_M) * CalYscale + CalYoffSet;
        }

        public float GetY()
        {
            return GetMagValue(Register.OUT_Z_H_M, Register.OUT_Z_L_M) * CalZscale + CalZoffSet;
        }

        public override I2CDevice.Configuration Config
        {
            get { return _config; }
        }
    }
}
