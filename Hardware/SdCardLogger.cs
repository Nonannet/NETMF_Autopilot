using System;
using Microsoft.SPOT;
using System.IO;
using Microsoft.SPOT.IO;
using System.Text;
using System.Threading;
using System.IO.Ports;

namespace PlaneOnBoardSoftware
{
    class SdCardLogger
    {
        SerialPort Uart;
        static byte[] newLineBytes = { 13, 10 };

        public SdCardLogger(SerialPort SerialPort)
        {
            Uart = SerialPort;

            Uart.BaudRate = 9600;
            Uart.Parity = Parity.None;
            Uart.DataBits = 8;
            Uart.StopBits = StopBits.One;

            Uart.Open();
        }

        public void Write(string Text)
        {
            sendToOpenLog(Encoding.UTF8.GetBytes(Text));
        }

        public void Write(Single value)
        {
            sendToOpenLog(Encoding.UTF8.GetBytes(value.ToString()));
        }

        public void Write(Double value)
        {
            sendToOpenLog(Encoding.UTF8.GetBytes(value.ToString()));
        }

        public void NewLine()
        {
            sendToOpenLog(newLineBytes);
        }

        void sendToOpenLog(byte[] data)
        {
            Uart.Write(data, 0, data.Length);
            //Uart.Flush();
        }
    }
}
