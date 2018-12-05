using System;
using GHIElectronics.NETMF.Hardware.LowLevel;
using System.IO.Ports;

namespace PlaneOnBoardSoftware
{
    class FEZ_Low_Level
    {
        // add this function anywhere
        static public void RemapCOM4to_TXAn2_RXAn3(SerialPort ser)
        {
            // call this function **after** you open COM4 port
            if (ser.PortName != "COM4" || ser.IsOpen == false)
            throw new Exception("Only use COM4 and make sure it is open");
            // remap COM4 RX (in) pin from P4.29/DIO17 to P0.26 (that is An3)
            // remap COM4 TX (out) pin from P4.28/DIO13 to P0.25 (that is An2)
            Register PINSEL9 = new Register(0xE002C024);
            PINSEL9.Write(0);// COM4 is now disconnected from P4.28 and P4.29
            Register PINSEL1 = new Register(0xE002C004);
            PINSEL1.SetBits(0xf << 18);// COM4 is now connected to An3 and An4
        }
    }
}
