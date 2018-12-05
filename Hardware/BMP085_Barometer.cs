using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    /// <summary>
    /// Absolute Pressure
    /// </summary>
    public class BMP085_Barometer : I2CBase
    {
        private Int16 AC1, AC2, AC3, B1, B2, MB, MC, MD;
        private UInt16 AC4, AC5, AC6;

        private byte OS = 3;
        private Int32 UT = 0;

        public float Temperature = 0;   // in Celcius
        public float Pressure = 0;   // in Pa
        public float Altitude = 0;  //in m
        public float OffSetAltitude = 0;  //in m

        private I2CDevice.Configuration _config;

        private int stCount = 0;

        public BMP085_Barometer()
        {
            _config = new I2CDevice.Configuration(0x77, 400); //68

            readCalibrationData();
        }

        public void RequestSensorData()
        {
            stCount++;

            if (stCount == 1)
            {
                Write(0xF4, 0x2E); //Start Temperature Measurement
            }
            else if (stCount == 2)
            {
                ReadUtMeasurement(); //Read UT Temperature
            }
            else if (stCount == 3)
            {
                Write(0xF4, 0x34 + 0xC0); //Start Pressure Measurement with oversampling = 3 (0xC0 = 3*2^6)
            }
            else
            {
                ReadMeasurement();
                stCount = 0;
            }
        }

        private void ReadUtMeasurement()
        {
            //Temperatur
            byte[] rawData = new byte[2];
            Read(0xF6, rawData);
            UT = (Int32)((Int32)(rawData[0] << 8) + (Int32)(rawData[1]));
        }

        private void ReadMeasurement()
        {
            Int32 X1, X2, B5, B6, X3, B3, P, UP;
            UInt32 B4, B7;

            //Temperatur
            X1 = (UT - AC6) * AC5 >> 15;

            if (X1 + MD > 0)
            {
                X2 = ((Int32)MC << 11) / (X1 + MD);
                B5 = X1 + X2;
                Temperature = ((float)((B5 + 8) >> 4)) / 10;

                //Pressure
                byte[] rawData2 = new byte[3];
                Read(0xF6, rawData2);
                UP = (((Int32)(rawData2[0]) << 16) + ((Int32)(rawData2[1]) << 8) + (Int32)rawData2[2]) >> (8 - OS);

                B6 = B5 - 4000;
                X1 = (B2 * (B6 * B6 >> 12)) >> 11;
                X2 = AC2 * B6 >> 11;
                X3 = X1 + X2;
                B3 = ((((Int32)AC1 * 4 + X3) << OS) + 2) >> 2;
                X1 = AC3 * B6 >> 13;
                X2 = (B1 * (B6 * B6 >> 12)) >> 16;
                X3 = ((X1 + X2) + 2) >> 2;
                B4 = AC4 * (UInt32)(X3 + 32768) >> 15;
                B7 = ((UInt32)(UP - B3)) * (UInt32)(50000 >> OS);
                P = (B7 < 0x80000000) ? (Int32)((B7 * 2) / B4) : (Int32)((B7 / B4) * 2);
                X1 = (P >> 8) * (P >> 8);
                X1 = (X1 * 3038) >> 16;
                X2 = (-7357 * P) >> 16;
                Pressure = P + ((X1 + X2 + 3791) >> 4);
            }
            else
            {
                Pressure = 100000;
            }
        }

        private void CalculateAltitude()
        {
            Altitude = OffSetAltitude + (float)(44330 * (1 - MathEx.Pow(Pressure / 101325, 0.1903)));
        }

        private void readCalibrationData()
        {
            byte[] rawData = new byte[22];

            Read(0xAA, rawData);

            AC1 = (short)((short)(rawData[0] << 8) + (short)(rawData[1]));
            AC2 = (short)((short)(rawData[2] << 8) + (short)(rawData[3]));
            AC3 = (short)((short)(rawData[4] << 8) + (short)(rawData[5]));
            AC4 = (ushort)((ushort)(rawData[6] << 8) + (ushort)(rawData[7]));
            AC5 = (ushort)((ushort)(rawData[8] << 8) + (ushort)(rawData[9]));
            AC6 = (ushort)((ushort)(rawData[10] << 8) + (ushort)(rawData[11]));
            B1 = (short)((short)(rawData[12] << 8) + (short)(rawData[13]));
            B2 = (short)((short)(rawData[14] << 8) + (short)(rawData[15]));
            MB = (short)((short)(rawData[16] << 8) + (short)(rawData[17]));
            MC = (short)((short)(rawData[18] << 8) + (short)(rawData[19]));
            MD = (short)((short)(rawData[20] << 8) + (short)(rawData[21]));

            //if (AC1 * AC2 * AC3 * AC4 * AC5 * AC6 * B1 * B2 * MB * MC * MD == 0) return;
        }

        public override I2CDevice.Configuration Config
        {
            get { return _config; }
        }
    }
}
