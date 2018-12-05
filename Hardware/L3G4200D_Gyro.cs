using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PlaneOnBoardSoftware
{
    /// <summary>
    /// Accelerometer
    /// </summary>
    public class L3G4200D_Gyro : I2CBase
    {
        private class Register
        {
            public const byte CTRL_REG1 = 0x20;
            public const byte OUT_TEMP =  0x26;
            public const byte OUT_X_L_A = 0x28;
            public const byte OUT_X_H_A = 0x29;
            public const byte OUT_Y_L_A = 0x2a;
            public const byte OUT_Y_H_A = 0x2b;
            public const byte OUT_Z_L_A = 0x2c;
            public const byte OUT_Z_H_A = 0x2d;
        }

        private I2CDevice.Configuration _config;
        private const float _scale = 250f / 0x8000 * 1.2f;
        private bool _PowerDown = false;

        private static float calXoffSet = -0.238403308f;
        private static float calYoffSet = -0.572611464f;
        private static float calZoffSet = +0.439249654f;

        public Vector3D AngularRate = new Vector3D();

        public L3G4200D_Gyro()
        {
            _config = new I2CDevice.Configuration(0x69, 400); //68

            // 0x1F = 00011111 Normal mode
            Write(Register.CTRL_REG1, 0x1F);
        }

        private float GetAngularRate(byte high, byte low)
        {
            var data = (Int16)((Read(high) << 8) | Read(low));

            return (((float)(data)) / _scale);
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
                    Write(Register.CTRL_REG1, 0x17);  // 0x17 = 00010111  Power-down
                else
                    Write(Register.CTRL_REG1, 0x1F);  // 0x1F = 00011111 Normal mode

                _PowerDown = value;
            }
        }

        public float GetTemperature()
        {
            return (sbyte)(Read(Register.OUT_TEMP));
        }

        public int RequestAngularRate()
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

            AngularRate.X = -CombineBytes(bufferXhi[0], bufferXlo[0]) * _scale + calXoffSet;
            AngularRate.Y = -CombineBytes(bufferYhi[0], bufferYlo[0]) * _scale + calYoffSet;
            AngularRate.Z = CombineBytes(bufferZhi[0], bufferZlo[0]) * _scale + calZoffSet;

            return result;
        }



        public float GetX()
        {
            return GetAngularRate(Register.OUT_X_H_A, Register.OUT_X_L_A) + calXoffSet;
        }

        public float GetY()
        {
            return GetAngularRate(Register.OUT_Y_H_A, Register.OUT_Y_L_A) + calYoffSet;
;
        }

        public float GetZ()
        {
            return GetAngularRate(Register.OUT_Z_H_A, Register.OUT_Z_L_A) + calZoffSet;
        }

        public override I2CDevice.Configuration Config
        {
            get { return _config; }
        }
    }
}
