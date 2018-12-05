using System;
using Microsoft.SPOT;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    /// <summary>
    /// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
    /// GNU General Public Licence
    /// </summary>
    /// <remarks>
    /// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    /// Modified 2012 by Nicolas Kruse
    /// </remarks>
    class MadgwickAHRS
    {
        /// <summary>AirSpeed in m/s</summary>
        public float InpAirSpeed = 0;

        /// <summary>Acceleration in m/s²</summary>
        public Vector3D InpAcc = new Vector3D();

        /// <summary>Angular rate in deg/s</summary>
        public Vector3D InpGyr = new Vector3D();

        public Vector3D InpMag = new Vector3D();

        /// <summary>Ascent/descent angle</summary>
        public float Pitch = 0;
        /// <summary>Roll angle</summary>
        public float Roll = 0;
        /// <summary>Attitude angle</summary>
        public float Yaw = 0;

        /// <summary>
        /// Gets or sets the algorithm gain beta.
        /// </summary>
        public float Beta = 0.1f;

        /// <summary>
        /// Gets or sets the Quaternion output.
        /// </summary>
        private float[] Quat = {0, 0, 0, 1};

        private float SamplePeriod;

        private static float piPerDeg = 0.017453292519f;
        private static float degPerPi = 57.29577951308f;

        /// <summary>
        /// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
        /// </summary>
        /// <param name="gx">
        /// Gyroscope x axis measurement in radians/s.
        /// </param>
        /// <param name="gy">
        /// Gyroscope y axis measurement in radians/s.
        /// </param>
        /// <param name="gz">
        /// Gyroscope z axis measurement in radians/s.
        /// </param>
        /// <param name="ax">
        /// Accelerometer x axis measurement in any calibrated units.
        /// </param>
        /// <param name="ay">
        /// Accelerometer y axis measurement in any calibrated units.
        /// </param>
        /// <param name="az">
        /// Accelerometer z axis measurement in any calibrated units.
        /// </param>
        /// <param name="mx">
        /// Magnetometer x axis measurement in any calibrated units.
        /// </param>
        /// <param name="my">
        /// Magnetometer y axis measurement in any calibrated units.
        /// </param>
        /// <param name="mz">
        /// Magnetometer z axis measurement in any calibrated units.
        /// </param>
        /// <remarks>
        /// Optimised for minimal arithmetic.
        /// Total ±: 160
        /// Total *: 172
        /// Total /: 5
        /// Total sqrt: 5
        /// </remarks> 
        public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
        {
            float q1 = Quat[0], q2 = Quat[1], q3 = Quat[2], q4 = Quat[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2f * q1;
            float _2q2 = 2f * q2;
            float _2q3 = 2f * q3;
            float _2q4 = 2f * q4;
            float _2q1q3 = 2f * q1 * q3;
            float _2q3q4 = 2f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = (float)MathEx.Sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = (float)MathEx.Sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2f * q1 * mx;
            _2q1my = 2f * q1 * my;
            _2q1mz = 2f * q1 * mz;
            _2q2mx = 2f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = (float)MathEx.Sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2f * _2bx;
            _4bz = 2f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = 1f / (float)MathEx.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * SamplePeriod;
            q2 += qDot2 * SamplePeriod;
            q3 += qDot3 * SamplePeriod;
            q4 += qDot4 * SamplePeriod;
            norm = 1f / (float)MathEx.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            Quat[0] = q1 * norm;
            Quat[1] = q2 * norm;
            Quat[2] = q3 * norm;
            Quat[3] = q4 * norm;
        }

        public void Calculate(float dt)
        {
            //Correcting centrifugal acceleration
            //CorrAccZ = InpAcc.Z - InpAirSpeed * InpGyr.X / degPerPi; //Check sign!

            if (dt < 0.5 && dt > 0)
            {
                SamplePeriod = dt;

                Update(InpGyr.X * piPerDeg, InpGyr.Y * piPerDeg, InpGyr.Z * piPerDeg, InpAcc.X, InpAcc.Y, InpAcc.Z, InpMag.X, InpMag.Y, InpMag.Z);

                float a01 = 2 * Quat[0] * Quat[1];
                float a02 = 2 * Quat[0] * Quat[2];
                float a03 = 2 * Quat[0] * Quat[3];
                float a11 = 2 * Quat[1] * Quat[1];
                float a12 = 2 * Quat[1] * Quat[2];
                float a13 = 2 * Quat[1] * Quat[3];
                float a22 = 2 * Quat[2] * Quat[2];
                float a23 = 2 * Quat[2] * Quat[3];
                float a33 = 2 * Quat[3] * Quat[3];

                Pitch = (float)MathEx.Asin(a01 + a23) * degPerPi;
                Roll =  -(float)MathEx.Atan2(a02 - a13, 1 - (a22 + a11)) * degPerPi;
                Yaw =   (float)MathEx.Atan2((a11 + a33) - 1, a12 - a03) * degPerPi;
            }
        }
    }
}
