using System;
using Microsoft.SPOT;
using GHIElectronics.NETMF.System;

namespace PlaneOnBoardSoftware
{
    class Navigation
    {
        const float R = 6378137; // meter
        const double pi180 = MathEx.PI / 180;
        const float Rpi180 = R * (float)pi180;

        public float Radius = 50; // meter
        public double Latitude = 0;
        public double Longitude = 0;
        public double DestLatitude = 0;
        public double DestLongitude = 0;

        public bool GpsHasFix = false;
        public double GpsLatitude = 0;
        public double GpsLongitude = 0;

        public float AirSpeed = 0;
        public float Heading = 0;
        public float Pitch = 0;
        public float TargetHeading = 0;
        public float Distance = 0; // meter

        public void Calculate(float dt)
        {
            double Lat1;
            double Lon1;
            double Lat2;
            double Lon2;
            float bearing;
            float rdRatio;
            double dLon;
            double CosLat2;
            double y;
            double x;


            //Calculate position
            if (GpsHasFix)
            {
                Latitude = GpsLatitude;
                Longitude = GpsLongitude;
            }
            else
            {
                Latitude += AirSpeed * dt * MathEx.Cos(Heading * pi180) / Rpi180;
                Longitude += AirSpeed * dt * MathEx.Sin(Heading * pi180) * MathEx.Cos(Latitude * pi180) / Rpi180;
            }


            //Calculate flight path
            Lat1 = Latitude * pi180;
            Lon1 = Longitude * pi180;
            Lat2 = DestLatitude * pi180;
            Lon2 = DestLongitude * pi180;
            dLon = Lon2 - Lon1;
            CosLat2 = MathEx.Cos(Lat2);
            y = MathEx.Sin(dLon) * CosLat2;
            x = MathEx.Cos(Lat1) * MathEx.Sin(Lat2) -
                       MathEx.Sin(Lat1) * CosLat2 * MathEx.Cos(dLon);
            bearing = (float)(MathEx.Atan2(y, x) / pi180);

            x = (Lon2 - Lon1) * MathEx.Cos((Lat1 + Lat2) / 2);
            y = (Lat2 - Lat1);
            Distance = (float)MathEx.Sqrt(x * x + y * y) * R;

            if (Distance > 1.0)
            {
                rdRatio = (Radius / Distance);
                TargetHeading = (bearing + 90 * (rdRatio * rdRatio) + 540) % 360 - 180;
            }
        }
    }
}
