using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PlaneOnBoardSoftware
{
    public class LSM303DLM_Accelerometer : I2CBase
    {
        private class Register
        {
            public const byte CTRL_REG1_A = 0x20;
            public const byte OUT_X_L_A = 0x28;
            public const byte OUT_X_H_A = 0x29;
            public const byte OUT_Y_L_A = 0x2a;
            public const byte OUT_Y_H_A = 0x2b;
            public const byte OUT_Z_L_A = 0x2c;
            public const byte OUT_Z_H_A = 0x2d;
        }

        private I2CDevice.Configuration _config;
        private const float _scale =  1 / (0x4000 * 9.81f);
        private bool _PowerDown = false;

        public Vector3D Acceleration = new Vector3D();

        public LSM303DLM_Accelerometer()
        {
            _config = new I2CDevice.Configuration(0x18, 400);

            // 0x27 = 00100111  Normal mode
            Write(Register.CTRL_REG1_A, 0x27);
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
                    Write(Register.CTRL_REG1_A, 0x07);  // 0x07 = 00000111  Power-down
                else
                    Write(Register.CTRL_REG1_A, 0x27);  // 0x27 = 00100111  Normal mode

                _PowerDown = value;
            }
        }

        private float GetAccelerationInGs(byte high, byte low)
        {
            // 12 bit - left justified
            var data = (Int16)((Read(high) << 8) | Read(low) >> 4);

            return (((float)(data)) / _scale);
        }

        public int RequesAcceleration()
        {
            byte[] bufferXhi = new byte[1];
            byte[] bufferYhi = new byte[1];
            byte[] bufferZhi = new byte[1];
            byte[] bufferXlo = new byte[1];
            byte[] bufferYlo = new byte[1];
            byte[] bufferZlo = new byte[1];
            I2CDevice.I2CTransaction[] readTransaction = new I2CDevice.I2CTransaction[12];
            I2CDevice.I2CWriteTransaction write;
            I2CDevice.I2CReadTransaction read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_X_H_A });
            read = I2CDevice.CreateReadTransaction(bufferXhi);
            readTransaction[0] = write;
            readTransaction[1] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_X_L_A });
            read = I2CDevice.CreateReadTransaction(bufferXlo);
            readTransaction[2] = write;
            readTransaction[3] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Y_H_A });
            read = I2CDevice.CreateReadTransaction(bufferYhi);
            readTransaction[4] = write;
            readTransaction[5] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Y_L_A });
            read = I2CDevice.CreateReadTransaction(bufferYlo);
            readTransaction[6] = write;
            readTransaction[7] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Z_H_A });
            read = I2CDevice.CreateReadTransaction(bufferZhi);
            readTransaction[8] = write;
            readTransaction[9] = read;

            write = I2CDevice.CreateWriteTransaction(new Byte[] { Register.OUT_Z_L_A });
            read = I2CDevice.CreateReadTransaction(bufferZlo);
            readTransaction[10] = write;
            readTransaction[11] = read;

            var result = I2CBus.Execute(Config, readTransaction, 1000);

            Acceleration.X = -CombineBytes(bufferXhi[0], bufferXlo[0]) * _scale;
            Acceleration.Y = -CombineBytes(bufferYhi[0], bufferYlo[0]) * _scale;
            Acceleration.Z = CombineBytes(bufferZhi[0], bufferZlo[0]) * _scale;

            return result;
        }

        public float GetX()
        {
            return GetAccelerationInGs(Register.OUT_X_H_A, Register.OUT_X_L_A);
        }

        public float GetY()
        {
            return GetAccelerationInGs(Register.OUT_Y_H_A, Register.OUT_Y_L_A);
        }

        public float GetZ()
        {
            return GetAccelerationInGs(Register.OUT_Z_H_A, Register.OUT_Z_L_A);
        }

        public override I2CDevice.Configuration Config
        {
            get { return _config; }
        }
    }
}
