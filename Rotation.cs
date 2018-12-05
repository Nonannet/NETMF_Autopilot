using System;
using Microsoft.SPOT;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    public class Vector3D
    {
        public float X;
        public float Y;
        public float Z;

        private static float degPerPi = 57.29577951308f;

        public static Vector3D xAxis = new Vector3D(1, 0, 0);
        public static Vector3D yAxis = new Vector3D(0, 1, 0);
        public static Vector3D zAxis = new Vector3D(0, 0, 1);

        public Vector3D()
        {
            this.X = 0;
            this.Y = 0;
            this.Z = 0;
        }

        public Vector3D(float x, float y, float z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public float Length
        {
            get { return (float)MathEx.Sqrt(X * X + Y * Y + Z * Z); }
        }

        public Vector3D Normated()
        {
            return new Vector3D(X / Length, Y / Length, Z / Length);
        }

        public static float VectorAngle(Vector3D Vector1, Vector3D Vector2)
        {
            return (float)MathEx.Acos(DotProduct(Vector1, Vector2) / Vector1.Length / Vector2.Length) * degPerPi;
        }

        public static float DotProduct(Vector3D Vector1, Vector3D Vector2)
        {
            return Vector1.X * Vector2.X + Vector1.Y * Vector2.Y + Vector1.Z * Vector2.Z;
        }

        public static Vector3D CrossProduct(Vector3D Vector1, Vector3D Vector2)
        {
            Vector3D result = new Vector3D();

            result.X = Vector1.Y * Vector2.Z - Vector1.Z * Vector2.Y;
            result.Y = Vector1.Z * Vector2.X - Vector1.X * Vector2.Z;
            result.Z = Vector1.X * Vector2.Y - Vector1.Y * Vector2.X;

            return result;
        }

        public static Vector3D operator *(Vector3D c1, Vector3D c2)
        {
            return CrossProduct(c1, c2);
        }
    }

    public class Quaternion
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        private static float degPerPi = 57.29577951308f;

        public Quaternion()
        {
            this.X = 0;
            this.Y = 0;
            this.Z = 0;
            this.W = 1;
        }

        public Quaternion(float x, float y, float z, float w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        public Quaternion(Vector3D RotationAxis, float Angle)
        {
            Angle = Angle / degPerPi / 2;

            float c = (float)MathEx.Cos(Angle);
            float s = (float)MathEx.Sin(Angle);

            if (RotationAxis.Length > float.Epsilon)
            {
                RotationAxis = RotationAxis.Normated();

                X = RotationAxis.X * s;
                Y = RotationAxis.Y * s;
                Z = RotationAxis.Z * s;
                W = c;
            }
            else
            {
                X = 0; Y = 0; Z = 0;
                W = 1;
            }
        }

        public static Quaternion GetFastQuaternion(Vector3D NormatedRotationAxis, float Angle)
        {
            Angle = Angle / degPerPi / 2f;
            float s = (float)MathEx.Sin(Angle);

            return new Quaternion(NormatedRotationAxis.X * s, NormatedRotationAxis.Y * s, NormatedRotationAxis.Z * s, (float)MathEx.Cos(Angle));
        }

        public void XAxisTurn(float Angle)
        {
            Angle = Angle / degPerPi / 2f;
            float s = (float)MathEx.Sin(Angle);
            float c = (float)MathEx.Cos(Angle);

            float tx = X; //s
            float ty = Y; //0
            float tz = Z; //0
            float tw = W; //c

            W = tw * c - tx * s;
            X = tw * s + tx * c;
            Y = ty * c + tz * s;
            Z = -ty * s + tz * c;
        }

        public void YAxisTurn(float Angle)
        {
            Angle = Angle / degPerPi / 2f;
            float s = (float)MathEx.Sin(Angle);
            float c = (float)MathEx.Cos(Angle);

            float tx = X; //0
            float ty = Y; //s
            float tz = Z; //0
            float tw = W; //c

            W = tw * c - ty * s;
            X = tx * c - tz * s;
            Y = tw * s + ty * c;
            Z = tx * s + tz * c;
        }

        public void ZAxisTurn(float Angle)
        {
            Angle = Angle / degPerPi / 2f;
            float s = (float)MathEx.Sin(Angle);
            float c = (float)MathEx.Cos(Angle);

            float tx = X; //0
            float ty = Y; //0
            float tz = Z; //s
            float tw = W; //c

            W = tw * c - tz * s;
            X = tx * c + ty * s;
            Y = -tx * s + ty * c;
            Z = tw * s + tz * c;
        }

        public float Angle
        {
            get { return (float)MathEx.Acos(W) * 2.0f * degPerPi; }
        }

        public static Quaternion operator *(Quaternion c1, Quaternion c2)
        {
            return hamiltonProduct(c1, c2);
        }

        private static Quaternion hamiltonProduct(Quaternion q1, Quaternion q2)
        {
            var result = new Quaternion();

            result.W = q1.W * q2.W - q1.X * q2.X - q1.Y * q2.Y - q1.Z * q2.Z;
            result.X = q1.W * q2.X + q1.X * q2.W + q1.Y * q2.Z - q1.Z * q2.Y;
            result.Y = q1.W * q2.Y - q1.X * q2.Z + q1.Y * q2.W + q1.Z * q2.X;
            result.Z = q1.W * q2.Z + q1.X * q2.Y - q1.Y * q2.X + q1.Z * q2.W;

            return result;
        }

        public void Normate()
        {
            float length = (float)MathEx.Sqrt(X * X + Y * Y + Z * Z);

            X = X / length;
            Y = Y / length;
            Z = Z / length;
        }

        public Quaternion Conjugate()
        {
            return new Quaternion(W, -X, -Y, -Z);
        }

        public Vector3D RotateVector(Vector3D Vector)
        {
            return QuaternionRotation(this, Vector);
        }

        private static Vector3D QuaternionRotation(Quaternion q, Vector3D v)
        {
            Vector3D result = new Vector3D();
            float a00 = q.W * q.W;
            float a01 = q.W * q.X;
            float a02 = q.W * q.Y;
            float a03 = q.W * q.Z;
            float a11 = q.X * q.X;
            float a12 = q.X * q.Y;
            float a13 = q.X * q.Z;
            float a22 = q.Y * q.Y;
            float a23 = q.Y * q.Z;
            float a33 = q.Z * q.Z;
            result.X = v.X * (a00 + a11 - a22 - a33) + 2 * (a12 * v.Y + a13 * v.Z + a02 * v.Z - a03 * v.Y);
            result.Y = v.Y * (a00 - a11 + a22 - a33) + 2 * (a12 * v.X + a23 * v.Z + a03 * v.X - a01 * v.Z);
            result.Z = v.Z * (a00 - a11 - a22 + a33) + 2 * (a13 * v.X + a23 * v.Y - a02 * v.X + a01 * v.Y);

            return result;
        }

        public Vector3D GetEulerAngles()
        {
            return GetEulerAnglesFromQuaternion(this);
        }

        private static Vector3D GetEulerAnglesFromQuaternion(Quaternion q)
        {
            Vector3D result = new Vector3D();
            float a01 = 2 * q.W * q.X;
            float a02 = 2 * q.W * q.Y;
            float a03 = 2 * q.W * q.Z;
            float a11 = 2 * q.X * q.X;
            float a12 = 2 * q.X * q.Y;
            float a13 = 2 * q.X * q.Z;
            float a22 = 2 * q.Y * q.Y;
            float a23 = 2 * q.Y * q.Z;
            float a33 = 2 * q.Z * q.Z;
            //result.Y = (float)MathEx.Atan2(a01 + a23, 1 - (a11 + a22)) * degPerPi;
            //result.X = (float)MathEx.Asin(a02 - a13) * degPerPi;
            //result.Z = (float)MathEx.Atan2(a03 + a12, 1 - (a22 + a33)) * degPerPi;

            result.Y = (float)MathEx.Atan2(a02 - a13, 1 - (a22 + a11)) * degPerPi;
            result.X = (float)MathEx.Asin(a01 + a23) * degPerPi;
            result.Z = (float)MathEx.Atan2(a03 - a12, 1 - (a11 + a33)) * degPerPi;

            return result;
        }
    }
}
