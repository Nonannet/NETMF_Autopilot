using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using GHIElectronics.NETMF.Hardware;

namespace PlaneOnBoardSoftware
{
    class BatteryMonitor
    {
        AnalogIn aiPortU;
        AnalogIn aiPortI;

        static float CurrentOffSet = 1.477f;
        static float ShuntResistens = 0.005776f;
        static float ShuntGain = 100.0f;
        static float CurrFactor = 1 / ShuntGain / ShuntResistens;
        float _voltage;
        float _current;
        double _charge = 0;
        double _energy = 0;

        DateTime lastTimePoint = DateTime.Now;
            
        public float Voltage { get { return _voltage; } }
        public float Current { get { return _current; } }
        public float Charge { get { return (float)_charge; } }
        public float Energy { get { return (float)_energy; } }

        public BatteryMonitor(AnalogIn VoltageAnalogInPort, AnalogIn CurrentAnalogInPort)
        {
            aiPortU = VoltageAnalogInPort;
            aiPortI = CurrentAnalogInPort;

            aiPortU.SetLinearScale(0, 4861); //mV (4.8V = 3.3V*(22kOhm+10kOhm)/22kOhm), 4.861V <- calibration
            aiPortI.SetLinearScale(0, 3300); //mV
        }

        public void UpdateData()
        {
            float dt;

            DateTime newTime = DateTime.UtcNow;
            dt = newTime.Subtract(lastTimePoint).Ticks / (float)TimeSpan.TicksPerSecond;
            lastTimePoint = newTime;

            _voltage = aiPortU.Read() * 0.001f; //in V

            if (_voltage > 2.0f)
                _current = (CurrentOffSet - aiPortI.Read() * 0.001f) * CurrFactor; //in A (1 V/A)
            else
                _current = 0;

            if (dt > 0 && dt < 10)
            {
                _charge += _current * dt / 3.6f;
                _energy += _current * _voltage * dt;
            }
        }
    }
}
