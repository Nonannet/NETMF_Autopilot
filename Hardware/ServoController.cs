using System;
using Microsoft.SPOT;
using GHIElectronics.NETMF.Hardware;
using GHIElectronics.NETMF.FEZ;
using System.Threading;

namespace PlaneOnBoardSoftware
{
    class ServoController
    {
        float _position = 0;
        float usedAngle = 0;
        PWM servoPwm;

        public ServoController(PWM.Pin ServoPin, float Angle)
        {
            servoPwm = new PWM(ServoPin);
            this.Position = 0;
            usedAngle = Angle / 2;
        }

        public float Position
        {
            get {
                return _position;
            }
            set {
                _position = value;
                if (_position > usedAngle) _position = usedAngle;
                if (_position < -usedAngle) _position = -usedAngle;

                servoPwm.SetPulse(20000000, (uint)(1500000 + _position / 90 * 1250000));
            }
        }
    }
}
