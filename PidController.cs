using System;

namespace PlaneOnBoardSoftware
{
	class PidController
	{
        private float lastError = 0;
        private DateTime lastTime;
        private float integral = 0;
        private float _minOutput = 0;
        private float _maxOutput = 0;
        private float Spann = 0;

        public float P = 0;
        public float I = 0;
        public float D = 0;
        public float SetPoint = 0;
        public float InputVariable = 0;

        public bool Circle360DegMode = false;

        public PidController(float MinOutput, float MaxOutput, float PropValue, float InteValue, float DiffValue, bool Circle360DegreeMode)
        {
            Circle360DegMode = Circle360DegreeMode;

            P = PropValue;
            I = InteValue;
            D = DiffValue;

            lastTime = DateTime.UtcNow;
            _minOutput = MinOutput;
            _maxOutput = MaxOutput;
            Spann = MaxOutput - MinOutput;
        }

        public PidController(float MinOutput, float MaxOutput, float PropValue, float InteValue, float DiffValue)
		{
            P = PropValue;
            I = InteValue;
            D = DiffValue;

            lastTime = DateTime.UtcNow;
            _minOutput = MinOutput;
            _maxOutput = MaxOutput;
            Spann = MaxOutput - MinOutput;
		}

		public float GetOutput(float dt, float inputValue)
		{
            InputVariable = inputValue;
            float error = SetPoint - InputVariable;

            if (Circle360DegMode)
            {
                float errev = error - 360;
                if (errev*errev < error*error) error = errev;
            }

            float iError = integral + P * error * dt / I;

            if (iError < Spann && iError > 0)
                integral = iError;

            float correction = _minOutput + P * (error + D * (error - lastError) / dt) + integral;

            lastError = error;

            if (correction > _maxOutput)
                correction = _maxOutput;
            else if (correction < _minOutput)
                correction = _minOutput;

            return correction;
        }		
	}
}