using System;
using System.Runtime.InteropServices;
using GHIElectronics.NETMF.System;
using GHIElectronics.NETMF.Hardware;

namespace PlaneOnBoardSoftware
{
    class ValueStorageClass
    {
        class Serializing
        {
            private static double Abs(double Number)
            {
                if (Number > 0)
                    return Number;
                else
                    return -Number;
            }

            public static void WriteBool(bool Value, ref byte[] ByteArray, ref int OffSet)
            {
                ByteArray[OffSet] = (byte)(Value ? 1 : 0);
                OffSet += 1;
            }

            public static bool ReadBool(ref byte[] ByteArray, ref int OffSet)
            {
                OffSet += 1;
                return ByteArray[OffSet-1] == 1;
            }

            public static void WriteByte(byte Number, ref byte[] ByteArray, ref int OffSet)
            {
                ByteArray[OffSet] = Number;
                OffSet += 1;
            }

            public static byte ReadByte(ref byte[] ByteArray, ref int OffSet)
            {
                OffSet += 1;
                return ByteArray[OffSet-1];
            }

            public static void WriteFloat(float Number, ref byte[] ByteArray, ref int OffSet)
            {
                int sigPos = (int)MathEx.Floor(MathEx.Log10(Abs(Number)));
                int intNumb = (int)(Number * MathEx.Pow(10, -sigPos + 5));

                if (intNumb < 0) intNumb += 0xFFFFFF;

                ByteArray[OffSet + 0] = (byte)(sigPos + 128);
                ByteArray[OffSet + 1] = (byte)(intNumb >> 16);
                ByteArray[OffSet + 2] = (byte)((intNumb >> 8) & 0xFF);
                ByteArray[OffSet + 3] = (byte)(intNumb & 0xFF);

                OffSet += 4;
            }

            public static float ReadFloat(ref byte[] ByteArray, ref int OffSet)
            {
                int sigPos = ByteArray[OffSet + 0] - 128;
                int intNumb = ByteArray[OffSet + 1];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 2];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 3];

                if (intNumb > 0x7FFFFF) intNumb -= 0xFFFFFF;

                OffSet += 4;
                return (float)(intNumb * MathEx.Pow(10, sigPos - 5));
            }

            public static void WriteDouble(double Number, ref byte[] ByteArray, ref int OffSet)
            {

                int sigPos = (int)MathEx.Floor(MathEx.Log10(Abs(Number)));
                long intNumb = (long)(Number * MathEx.Pow(10, -sigPos + 14));

                if (intNumb < 0) intNumb += 0xFFFFFFFFFFFFFF;

                ByteArray[OffSet + 0] = (byte)(sigPos + 128);
                ByteArray[OffSet + 7] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 6] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 5] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 4] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 3] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 2] = (byte)(intNumb & 0xFF);
                intNumb = (intNumb >> 8);
                ByteArray[OffSet + 1] = (byte)(intNumb);

                OffSet += 4;
            }

            public static double ReadDouble(ref byte[] ByteArray, ref int OffSet)
            {
                long sigPos = ByteArray[OffSet + 0] - 128;
                long intNumb = ByteArray[OffSet + 1];

                intNumb = (intNumb << 8) + ByteArray[OffSet + 2];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 3];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 4];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 5];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 6];
                intNumb = (intNumb << 8) + ByteArray[OffSet + 7];

                if (intNumb > 0x7FFFFFFFFFFFFF) intNumb -= 0xFFFFFFFFFFFFFF;

                OffSet += 4;
                return (intNumb * MathEx.Pow(10, sigPos - 14));
            }
        }

        private int StorageSize = 0;
        private int OffSet = 0;
        private byte[] buffer;

        public ValueStorageClass()
        {
            StorageSize = InternalFlashStorage.Size;
            OffSet = 0;
            buffer = new byte[StorageSize];
            InternalFlashStorage.Read(buffer);
        }

        public void WriteToFlash()
        {
            InternalFlashStorage.Write(buffer);
        }

        public void WriteBool(bool Value)
        {
            Serializing.WriteBool(Value, ref buffer, ref OffSet);
        }

        public void WriteFloat(float Value)
        {
            Serializing.WriteFloat(Value, ref buffer, ref OffSet);
        }

        public void WriteDouble(double Value)
        {
            Serializing.WriteDouble(Value, ref buffer, ref OffSet);
        }

        public bool ReadBool()
        {
            return Serializing.ReadBool(ref buffer, ref OffSet);
        }

        public float ReadFloat()
        {
            return Serializing.ReadFloat(ref buffer, ref OffSet);
        }

        public double ReadDouble()
        {
            return Serializing.ReadDouble(ref buffer, ref OffSet);
        }
    }
}
