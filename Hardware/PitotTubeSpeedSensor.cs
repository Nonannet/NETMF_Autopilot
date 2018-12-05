using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using GHIElectronics.NETMF.Hardware;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    class PitotTubeSpeedSensor
    {
        AnalogIn aiPort;
        const float RLuft = 287.058f;
        public float BarometricPressure = 100000; //Pa;
        public float DynamicPressure; //Pa
        public float Temperature = 20;

        public PitotTubeSpeedSensor(AnalogIn AnalogInPort)
        {
            aiPort = AnalogInPort;

            aiPort.SetLinearScale(0, 3300); //mV
        }

        public float GetSpeed()
        {
            float roh = BarometricPressure / (RLuft * (Temperature + 273.15f));
            DynamicPressure = (aiPort.Read() / 5000f - 0.04f) / 0.00009f; //in Pa
            if (roh > 0 && DynamicPressure > 0)
                return (float)MathEx.Sqrt(2 / roh * DynamicPressure);
            else
                return 0;
        }
    }
}
