using System;
using Microsoft.SPOT;
using Microsoft.SPOT.IO;
using Microsoft.SPOT.Hardware;
using System.Threading;

namespace PlaneOnBoardSoftware
{
    class Igniter
    {
        private int onTime;
        private OutputPort igniterPort;

        public Igniter(Cpu.Pin PortPin, int OnTime)
        {
            igniterPort = new OutputPort(PortPin, false);
            this.onTime = OnTime;
        }

        public void Fire()
        {
            igniterPort.Write(true);
            Thread.Sleep(onTime);
            igniterPort.Write(false);//<-temp
        }
    }
}
