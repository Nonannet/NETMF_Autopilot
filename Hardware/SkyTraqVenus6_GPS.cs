using System;
using System.IO;
using System.IO.Ports;
using Microsoft.SPOT;
using System.Threading;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    public class SkyTraqVenus6_GPS
    {
        private byte MESSAGE_TYPE = 0x09;
        private byte POSITION_RATE = 0x0E;
        private DateTime GpsStartDate = new DateTime(1980, 1, 6);

        SerialPort Uart;
        byte[] rxBuffer = new byte[256];
        int bufferPos = 0;
        int packOffSet = 0;

        private int gpsTimeOfWeek;
        private int gpsWeekNumber;

        //private Vector3D ECEFCoordinate = new Vector3D();
        private Vector3D ECEFSpeed = new Vector3D();

        public bool Has3DFix = false;
        public double Latitude = 0;
        public double Longitude = 0;
        public float Altitude = 0;
        public float EllipsoidAltitude = 0;
        public float HorizontalDilution = 0;
        public float VerticalDilution = 0;
        public Vector3D Speed = new Vector3D();
        public DateTime GpsTime = new DateTime();
        public float Heading = 0;
        public float GroundSpeed = 0;
        
        public SkyTraqVenus6_GPS(SerialPort SerialPort)
        {
            Uart = SerialPort;

            Uart.BaudRate = 9600;
            Uart.Parity = Parity.None;
            Uart.DataBits = 8;
            Uart.StopBits = StopBits.One;

            Uart.Open();
            if (SerialPort.PortName.ToUpper() == "COM4")
            {
                FEZ_Low_Level.RemapCOM4to_TXAn2_RXAn3(Uart);
            }

            SendCommand(MESSAGE_TYPE, 2, 0); //Switch to Binary Output
            //SendCommand(POSITION_RATE, 3, 0); //Switch to 1 Hz update rate
        }

        private void SendCommand(byte Command, byte Value1, byte Value2)
        {
            //<0xA0,0xA1>< PL><09>< message body><CS><0x0D,0x0A>
            byte[] newCom = new byte[] { 0xA0, 0xA1, 0, 3, Command, Value1, Value2, 0xFF, 0x0D, 0x0A };

            SetChecksum(newCom, 3);

            Uart.Write(newCom, 0, 10);
            Uart.Flush();
        }

        private byte GetChecksum(byte[] data, int PayloadLen)
        {
            int cs = 0;

            for (int i = 0; i < PayloadLen; i++)
            {
                cs = cs ^ data[i + 4];
            }

            return (byte)cs;
        }

        private void SetChecksum(byte[] data, int PayloadLen)
        {
            data[PayloadLen + 4] = GetChecksum(data, PayloadLen);
        }

        public void ReadGPSData()
        {
            int read_count = 1;
            int readLenght = 1;
            int portBuffCount = Uart.BytesToRead;

            while (portBuffCount > 0)
            {
                readLenght = portBuffCount;
                if (readLenght > 128) readLenght = 128;

                if (bufferPos > 128) bufferPos = 0;
                read_count = Uart.Read(rxBuffer, bufferPos, readLenght);

                portBuffCount -= read_count;
                bufferPos += read_count;
            }

            for (int i = 0; i < 66; i++)
            {
                packOffSet = bufferPos - i - 66;

                if (packOffSet < 0) break;
                if (rxBuffer[packOffSet + 65] == 10 && rxBuffer[packOffSet + 64] == 13 &&
                    rxBuffer[packOffSet + 1] == 0xA1 && rxBuffer[packOffSet] == 0xA0)
                    break;
            }

            if (packOffSet > -1)
            {
                Has3DFix = (GetByte(2)==2);

                Latitude = GetSignedInteger(10) * 10e-8;
                Longitude = GetSignedInteger(14) * 10e-8;

                /*ECEFCoordinate.X = GetSignedInteger(36) * 0.01f;
                ECEFCoordinate.Y = GetSignedInteger(40) * 0.01f;
                ECEFCoordinate.Z = GetSignedInteger(44) * 0.01f;*/

                ECEFSpeed.X = GetSignedInteger(48) * 0.01f;
                ECEFSpeed.Y = GetSignedInteger(52) * 0.01f;
                ECEFSpeed.Z = GetSignedInteger(56) * 0.01f;

                EllipsoidAltitude = GetSignedInteger(18) * 0.01f; //ellipsoid altitude
                Altitude = GetSignedInteger(22) * 0.01f; //mean sea level altitude

                HorizontalDilution = GetUSignInt16(30) * 0.01f;
                VerticalDilution = GetUSignInt16(32) * 0.01f;

                gpsWeekNumber = GetUSignInt16(4);
                gpsTimeOfWeek = GetSignedInteger(6);

                GpsTime = GpsStartDate.AddDays(gpsWeekNumber * 7).AddMilliseconds(gpsTimeOfWeek * 10);

                Speed = ECEFtoLTPspeed(ECEFSpeed, Latitude, Longitude);
            }
        }

        private Vector3D ECEFtoLTPspeed(Vector3D ECEFSpeed, double Latitude, double Longitude)
        {
            Vector3D GetLTPSpeed = new Vector3D();

            double SinLat = MathEx.Sin(Latitude);
            double SinLon = MathEx.Sin(Longitude);
            double CosLat = MathEx.Cos(Latitude);
            double CosLon = MathEx.Cos(Longitude);


            GetLTPSpeed.X = (float)(-ECEFSpeed.X * SinLat * SinLon - ECEFSpeed.Y * SinLat * SinLon + ECEFSpeed.Z * CosLat); //Vnorth
            GetLTPSpeed.Y = (float)(-ECEFSpeed.X * SinLon + ECEFSpeed.Y * CosLon); //Veast
            GetLTPSpeed.Z = (float)(-ECEFSpeed.X * CosLat * CosLon - ECEFSpeed.Y * CosLat * SinLon + ECEFSpeed.Z * SinLat); //Vdown

            return GetLTPSpeed;
        }

        private byte GetByte(int Field)
        {
            return rxBuffer[packOffSet + 3 + Field];
        }

        private int GetUSignInt16(int StartField)
        {
            int bufPos = packOffSet + 3 + StartField;
            return (rxBuffer[bufPos] << 8) + rxBuffer[bufPos + 1];
        }

        private int GetSignedInteger(int StartField)
        {
            int buffer = 0;

            for (int i = 0; i < 4; i++)
            {
                int bufPos = packOffSet + 3 + StartField + i;

                buffer = (buffer << 8) + rxBuffer[bufPos];
            }

            return buffer;
        }

    }
}
