using System;
using System.IO;
using System.Threading;

using Microsoft.SPOT;
using Microsoft.SPOT.IO;
using Microsoft.SPOT.Hardware;

using GHIElectronics.NETMF.FEZ;
using GHIElectronics.NETMF.IO;
using GHIElectronics.NETMF.Hardware;
using GHIElectronics.NETMF.Hardware.LowLevel;

using System.IO.Ports;
using System.Text;


namespace PlaneOnBoardSoftware
{
    public class Program
    {
        //static PersistentStorage sdPS;
        //static string rootDirectory;

        private static ExtendedWeakReference settingsReference;

        static ServoController Servo1 = new ServoController((PWM.Pin)FEZ_Pin.PWM.Di5, 90);
        static ServoController Servo2 = new ServoController((PWM.Pin)FEZ_Pin.PWM.Di6, 90);
        static LSM303DLM_Accelerometer Accelerometer = new LSM303DLM_Accelerometer();
        static LSM303DLM_Magnetometer Magnetometer = new LSM303DLM_Magnetometer();
        static L3G4200D_Gyro Gyroscope = new L3G4200D_Gyro();
        static BMP085_Barometer Barometer = new BMP085_Barometer();
        static DS18B20_Temperature Thermometer = new DS18B20_Temperature((Cpu.Pin)FEZ_Pin.Digital.Di9);
        static PitotTubeSpeedSensor PitotTube = new PitotTubeSpeedSensor(new AnalogIn((AnalogIn.Pin)FEZ_Pin.AnalogIn.An0));
        static BatteryMonitor PowerSource = new BatteryMonitor(new AnalogIn((AnalogIn.Pin)FEZ_Pin.AnalogIn.An7), new AnalogIn((AnalogIn.Pin)FEZ_Pin.AnalogIn.An6));
        static SkyTraqVenus6_GPS GpsRecever = new SkyTraqVenus6_GPS(new SerialPort("COM4"));
        static Linksprite_Camera Camera = new Linksprite_Camera(new SerialPort("COM3"));
        static RFM22b RadioModem = new RFM22b(SPI.SPI_module.SPI1, (Cpu.Pin)FEZ_Pin.Digital.UEXT5, (Cpu.Pin)FEZ_Pin.Digital.UEXT6, 869.545);
        static RFM22b.Channel StatusChannel = RadioModem.NewChannel(0, 32, 512);
        static RFM22b.Channel CmdChannel = RadioModem.NewChannel(1, 64, 128);
        static SdCardLogger Logger = new SdCardLogger(new SerialPort("COM1"));
        static MadgwickAHRS Imu = new MadgwickAHRS();
        static Navigation Navi = new Navigation();
        static PidController PitchCtr = new PidController(-45, 45, 5, 1, 0);
        static PidController RollCtr = new PidController(-45, 45, 5, 1, 0);
        static PidController YawCtr = new PidController(-45, 45, 1, 5, 0, true);
        static PidController SpeedCtr = new PidController(0, -60, 0.5f, 15, 0);

        static Igniter ParachuteRelese = new Igniter((Cpu.Pin)FEZ_Pin.Digital.Di3, 200);
        static Igniter BalloonRelese = new Igniter((Cpu.Pin)FEZ_Pin.Digital.Di4, 200);
        
        static Thread MainLoopThread;
        static Thread HelperLoopThread;
        static float dt;
        static bool ValueUnknown = false;
        static bool UseStatusChannel = true;

        static bool DisableServos = false;
        static float TargetAirSpeed = 8; //m/s
        
        public static void Main()
        {
            Debug.EnableGCMessages(false);

            //sdPS = new PersistentStorage("SD");
            //sdPS.MountFileSystem();
            //rootDirectory = VolumeInfo.GetVolumes()[0].RootDirectory;

            HelperLoopThread = Thread.CurrentThread;
            HelperLoopThread.Priority = ThreadPriority.Lowest;

            SetUpMainLoop();
            
            int pcount = 260000;

            byte[] Test = new byte[32];
            int PackLen = 1;

            //GHIElectronics.NETMF.Hardware.LowLevel.Watchdog.Enable(5000);
            LoadData();

            while(true)
            {
                pcount++;
                //Camera.SavePictureToSD(rootDirectory + @"\TestPicture_" +  pcount + ".jpg");
                Thread.Sleep(1000);


                GpsRecever.ReadGPSData();
                PowerSource.UpdateData();
                Thermometer.ReadTemperature();

                PitotTube.Temperature = Thermometer.Temperature2;
                PitotTube.BarometricPressure = Barometer.Pressure;

                CheckCml();
                GHIElectronics.NETMF.Hardware.LowLevel.Watchdog.ResetCounter();


                if (GpsRecever.Has3DFix && System.Math.Abs(DateTime.UtcNow.Subtract(GpsRecever.GpsTime).Seconds) > 10)
                {
                    Microsoft.SPOT.Time.TimeService.SetTimeZoneOffset(0);
                    Utility.SetLocalTime(GpsRecever.GpsTime);
                    Debug.Print("Set Time To: " + GpsRecever.GpsTime);
                    Microsoft.SPOT.Time.TimeService.SetTimeZoneOffset(120);
                }


                //Debug.Print(Imu.InpMag.X + "; "  + Imu.InpMag.Y + "; " + Imu.InpMag.Z);
                //Debug.Print("X/Y/Z: " + Imu.Pitch + "    /    " + Imu.Roll + "   /   " + Imu.Yaw + "  /  dt=" + dt);
                //Debug.Print("X/Y/Z: " + Imu.InpGyr.X + "    /    " + Imu.InpGyr.Y + "   /   " + Imu.InpGyr.Z + "  /  dt=" + dt);
                //Debug.Print("Voltage: " + PowerSource.GetVoltage() + "    Temperatur: " + Barometer.Temperature + " °C     Pressure: " + (Barometer.Pressure / 100f) +  " hPa" +  "   dt=" + dt);
                //Debug.Print("Latitude: " + GpsRecever.Latitude + "  Longitude: " + GpsRecever.Longitude + " Höhe: " + GpsRecever.Altitude + "   Speed: " + GpsRecever.Speed.Length + " m/s");
                //Debug.Print("vd: " + GpsRecever.VerticalDilution + "  hd: " + GpsRecever.HorizontalDilution + " fix: " + GpsRecever.Has3DFix + " Time: " + DateTime.Now + " UTC-Time: " + DateTime.UtcNow + " Voltage: " + PowerSource.GetVoltage() + Barometer.Temperature );
                //Debug.Print("RSSI: " + RadioModem.RSSI);
                //Debug.Print("Voltage: " + PowerSource.Voltage + "  Current: " + PowerSource.Current + "  Energie: " + PowerSource.Energy + "  Voltage: " + PowerSource.Charge);

                /*PackLen = 1;
                while (PackLen > 0)
                {
                    PackLen = StatusChannel.Recv(Test, 0, 32);

                    if (PackLen > 0)
                    {
                        try
                        {
                            string tmpCa = (new string(UTF8Encoding.UTF8.GetChars(Test))).Substring(0, PackLen);
                            //Debug.Print(tmpCa);
                            
                        }
                        catch
                        {
                            Debug.Print("Error");
                        }

                        
                    }
                }*/
                //"Latitude: " + GpsRecever.Latitude + "  Longitude: " + GpsRecever.Longitude + " Höhe: " + GpsRecever.Altitude + "   Speed: " + GpsRecever.Speed.Length + " m/s

                //Debug.Print("Voltage: " + PowerSource.Voltage + " Current: " + PowerSource.Current + " A  Charge: " + PowerSource.Charge + " mAh   Pressure: " + PitotTube.GetSpeed() + " kPa  GPS-Speed: " + GpsRecever.Speed.Length / 3.6f + " km/h  Bytes To Send: " + StatusChannel.BytesToSend + "   3D-Fix: " + GpsRecever.Has3DFix.ToString() + "   Druck:   " + (Barometer.Pressure / 100f) + " mbar   Temperature: " + Thermometer.Temperature1 + " °C  Temperature: " + Thermometer.Temperature2 + " °C");
                //Debug.Print("Druck:   " + (Barometer.Pressure / 100f) + " mbar   Temper.:" + Barometer.Temperature + " °C");

                if (GpsRecever.Has3DFix || true)
                {
                    Logger.Write(DateTime.UtcNow.ToString());
                    Logger.Write(";");
                    Logger.Write(Navi.Latitude);
                    Logger.Write(";");
                    Logger.Write(Navi.Longitude);
                    Logger.Write(";");
                    Logger.Write(GpsRecever.Altitude);
                    Logger.Write(";");
                    Logger.Write(GpsRecever.Speed.Length);
                    Logger.Write(";");
                    Logger.Write(Imu.Yaw);
                    Logger.Write(";");
                    Logger.Write(Imu.Roll);
                    Logger.Write(";");
                    Logger.Write(Imu.Pitch);
                    Logger.Write(";");
                    Logger.Write(PitotTube.GetSpeed());
                    Logger.Write(";");
                    Logger.Write(PowerSource.Voltage);
                    Logger.Write(";");
                    Logger.Write(PowerSource.Current);
                    Logger.Write(";");
                    Logger.Write(PowerSource.Energy);
                    Logger.Write(";");
                    Logger.Write(Barometer.Temperature);
                    Logger.Write(";");
                    Logger.Write((Barometer.Pressure / 100f));
                    Logger.Write(";");
                    Logger.Write(Thermometer.Temperature1);
                    Logger.Write(";");
                    Logger.Write(Thermometer.Temperature2);
                    Logger.Write(";");
                    Logger.Write(Servo1.Position);
                    Logger.Write(";");
                    Logger.Write(Servo2.Position);
                    Logger.Write(";");
                    Logger.Write(Navi.TargetHeading);
                    Logger.Write(";");
                    Logger.Write(Navi.Distance);
                    Logger.Write(";");
                    Logger.Write(GpsRecever.HorizontalDilution);
                    Logger.Write(";");
                    Logger.Write(GpsRecever.Has3DFix.ToString());
                    Logger.Write(";");
                    Logger.Write(Magnetometer.MagValue.X);
                    Logger.Write(";");
                    Logger.Write(Magnetometer.MagValue.Y);
                    Logger.Write(";");
                    Logger.Write(Magnetometer.MagValue.Z);
                    Logger.NewLine();
                }

                if (StatusChannel.BytesToSend == 0 && UseStatusChannel)
                {
                    /*byte[] tmp = Encoding.UTF8.GetBytes(DateTime.UtcNow + ";" + GpsRecever.Latitude + ";" + GpsRecever.Longitude + ";"
                        + GpsRecever.Altitude + ";" + GpsRecever.Speed.Length + ";" + Imu.Yaw +";" + PitotTube.GetSpeed()
                        + ";" + PowerSource.Voltage + ";" + PowerSource.Current + ";" + PowerSource.Energy + ";" + Barometer.Temperature + ";"
                        + (Barometer.Pressure / 100f) + ";" + Thermometer.Temperature1 + ";" + Thermometer.Temperature2 + ";"
                        + Servo1.Position + ";" + Servo2.Position + ";" + RadioModem.RSSI + ";" + RadioModem.LostPackets + "\r\n");
                    StatusChannel.Send(tmp, 0, tmp.Length);*/

                    StatusChannel.Send(DateTime.UtcNow + ";" + Navi.Latitude.ToString("f6") + ";" + Navi.Longitude.ToString("f6") + ";" +
                        GpsRecever.Altitude.ToString("f0") + ";" + GpsRecever.Speed.Length.ToString("f1") + ";" + Imu.Yaw.ToString("f1") + ";" + Imu.Roll.ToString("f1") + ";" + Imu.Pitch.ToString("f1") + ";" + PitotTube.GetSpeed().ToString("f1") + ";" +
                        PowerSource.Voltage.ToString("f2") + ";" + PowerSource.Current.ToString("f2") + ";" + PowerSource.Energy.ToString("f0") + ";" + Barometer.Temperature.ToString("f1") + ";" +
                        (Barometer.Pressure / 100f).ToString("f2") + ";" + Thermometer.Temperature1.ToString("f1") + ";" + Thermometer.Temperature2.ToString("f1") + ";" +
                        Servo1.Position.ToString("f0") + ";" + Servo2.Position.ToString("f0") + ";" + Navi.TargetHeading.ToString("f1") + ";" + Navi.Distance.ToString("f0") + ";" + GpsRecever.HorizontalDilution.ToString("f0") + ";" + GpsRecever.Has3DFix.ToString() +  ";" + RadioModem.RSSI + ";" + RadioModem.LostPackets + ";" + Magnetometer.MagValue.Length + "\r\n");
                }
            }
        }

        public static void CheckCml()
        {
            string comLine;
            string comName;
            string comValue;
            int i;

            while ((comLine = CmdChannel.RecvLine()).Length > 0)
            {
                i = comLine.IndexOf('=');
                if (i > 0)
                {
                    comName = comLine.Substring(0, i).Trim();
                    comValue = comLine.Substring(i + 1).Trim();

                    SetDevProperty(comName, comValue);
                    if (ValueUnknown)
                        CmdChannel.SendLine("err " + comName.Trim());
                    else
                        CmdChannel.SendLine("set " + comName + "=" + comValue);
                }
                else
                {
                    string tempRes = GetDevProperty(comLine.Trim());

                    if (ValueUnknown)
                        CmdChannel.SendLine("err " + comLine.Trim());
                    else
                        CmdChannel.SendLine("get " + comLine.Trim() + "=" + tempRes);
                }

                
            }
        }

        public static void SetUpMainLoop()
        {
            MainLoopThread = new Thread(MainLoop);
            MainLoopThread.Priority = ThreadPriority.Highest;
            MainLoopThread.Start();
            while (!MainLoopThread.IsAlive) ;
        }

        public static void MainLoop()
        {
            bool ledState = false;
            //int pressureCount = 0;
            OutputPort led = new OutputPort((Cpu.Pin)FEZ_Pin.Digital.LED, ledState);

            DateTime startTime = DateTime.Now;
            DateTime newTime;
            DateTime lastTimePoint = DateTime.Now;
            float AirSpeed;
            
            while (true)
            {
                newTime = DateTime.UtcNow;
                dt = newTime.Subtract(lastTimePoint).Ticks / (float)TimeSpan.TicksPerSecond;
                lastTimePoint = newTime;

                HelperLoopThread.Suspend();
                Accelerometer.RequesAcceleration();
                Magnetometer.RequestMagValue();
                Gyroscope.RequestAngularRate();
                Barometer.RequestSensorData();
                HelperLoopThread.Resume();
               
                AirSpeed = PitotTube.GetSpeed();

                Imu.InpAcc = Accelerometer.Acceleration;
                Imu.InpMag = Magnetometer.MagValue;
                Imu.InpGyr = Gyroscope.AngularRate;
                Imu.InpAirSpeed = AirSpeed;
                Imu.Calculate(dt);

                Debug.Print(Imu.InpMag.X + "; " + Imu.InpMag.Y + "; " + Imu.InpMag.Z + "; " + Imu.InpAcc.X + "; " + Imu.InpAcc.Y + "; " + Imu.InpAcc.Z);

                Navi.GpsHasFix = GpsRecever.Has3DFix;
                Navi.GpsLongitude = GpsRecever.Longitude;
                Navi.GpsLatitude = GpsRecever.Latitude;
                Navi.Heading = Imu.Yaw;
                Navi.Pitch = Imu.Pitch;
                Navi.AirSpeed = AirSpeed;
                Navi.Calculate(dt);

                YawCtr.SetPoint = Navi.TargetHeading;
                PitchCtr.SetPoint = SpeedCtr.GetOutput(dt, TargetAirSpeed);
                RollCtr.SetPoint = YawCtr.GetOutput(dt, Imu.Yaw);
                var pitchValue = PitchCtr.GetOutput(dt, Imu.Pitch);
                var rollValue = RollCtr.GetOutput(dt, Imu.Roll);

                if (!DisableServos)
                {
                    Servo1.Position = -rollValue - pitchValue;
                    Servo2.Position = -rollValue + pitchValue;
                }
                else
                {
                    Servo1.Position = 0;
                    Servo2.Position = 0;
                }

                ledState = !ledState;
                led.Write(ledState);

                Thread.Sleep(1);
            }
        }

        public static void SaveValues()
        {
            ValueStorageClass Flash = new ValueStorageClass();

            Flash.WriteFloat(1234f);
            Flash.WriteFloat(PitchCtr.P);
            Flash.WriteFloat(PitchCtr.I);
            Flash.WriteFloat(RollCtr.P);
            Flash.WriteFloat(RollCtr.I);
            Flash.WriteFloat(YawCtr.P);
            Flash.WriteFloat(YawCtr.I);
            Flash.WriteBool(DisableServos);
            Flash.WriteDouble(RadioModem.Frequency);
            Flash.WriteDouble(Navi.DestLatitude);
            Flash.WriteDouble(Navi.DestLongitude);
            Flash.WriteFloat(TargetAirSpeed);
            Flash.WriteFloat(Magnetometer.CalXoffSet);
            Flash.WriteFloat(Magnetometer.CalYoffSet);
            Flash.WriteFloat(Magnetometer.CalZoffSet);
            Flash.WriteFloat(Magnetometer.CalXscale);
            Flash.WriteFloat(Magnetometer.CalYscale);
            Flash.WriteFloat(Magnetometer.CalZscale);
            Flash.WriteFloat(Barometer.OffSetAltitude);

            Flash.WriteToFlash();
        }

        public static void LoadData()
        {
            ValueStorageClass Flash = new ValueStorageClass();

            if (Flash.ReadFloat() == 1234)
            {
                PitchCtr.P = Flash.ReadFloat();
                PitchCtr.I = Flash.ReadFloat();
                RollCtr.P = Flash.ReadFloat();
                RollCtr.I = Flash.ReadFloat();
                YawCtr.P = Flash.ReadFloat();
                YawCtr.I = Flash.ReadFloat(); ;
                DisableServos = Flash.ReadBool();
                RadioModem.Frequency = Flash.ReadDouble();
                Navi.DestLatitude = Flash.ReadDouble();
                Navi.DestLongitude = Flash.ReadDouble();
                TargetAirSpeed = Flash.ReadFloat();
                Magnetometer.CalXoffSet = Flash.ReadFloat();
                Magnetometer.CalYoffSet = Flash.ReadFloat();
                Magnetometer.CalZoffSet = Flash.ReadFloat();
                Magnetometer.CalXscale = Flash.ReadFloat();
                Magnetometer.CalYscale = Flash.ReadFloat();
                Magnetometer.CalZscale = Flash.ReadFloat();
                Barometer.OffSetAltitude = Flash.ReadFloat();
            }
        }

        public static void SetDevProperty(string Name, string StrValue)
        {
            double Value;
            
            try
            {
                Value = double.Parse(StrValue);
            }
            catch
            {
                Value = 0;
            }
            ValueUnknown = false;

            switch(Name)
            {
                case "PitchCtrP":
                    PitchCtr.P = (float)Value;
                    break;
                case "PitchCtrI":
                    PitchCtr.I = (float)Value;
                    break;
                case "RollCtrP":
                    RollCtr.P = (float)Value;
                    break;
                case "RollCtrI":
                    RollCtr.I = (float)Value;
                    break;
                case "YawCtrP":
                    YawCtr.P = (float)Value;
                    break;
                case "YawCtrI":
                    YawCtr.I = (float)Value;
                    break;
                case "DisableServos":
                    DisableServos = (Value == 1);
                    break;
                case "FireParachute":
                    if (Value > 0) ParachuteRelese.Fire();
                    break;
                case "ReleseBalloon":
                    if (Value > 0) BalloonRelese.Fire();
                    break;
                case "RadioFrequency":
                    RadioModem.Frequency = Value;
                    break;
                case "UseStatusChannel":
                    UseStatusChannel = (Value > 0);
                    break;
                case "DestLatitude":
                    Navi.DestLatitude = (float)Value;
                    break;
                case "DestLongitude":
                    Navi.DestLongitude = (float)Value;
                    break;
                case "TargetAirSpeed":
                    TargetAirSpeed = (float)Value;
                    break;
                case "CalibrationMode":
                    Magnetometer.CalibrationMode = (Value == 1);
                    break;
                case "Altitude":
                    Barometer.OffSetAltitude = (float)Value - Barometer.Altitude;
                    break;
                default:
                    ValueUnknown = true;
                    break;
            }

            if (!ValueUnknown) SaveValues();
        }

        public static string GetDevProperty(string Name)
        {
            string ret = "";

            ValueUnknown = false;
            switch (Name)
            {
                case "PitchCtrP":
                    ret = PitchCtr.P.ToString();
                    break;
                case "PitchCtrI":
                    ret = PitchCtr.I.ToString();
                    break;
                case "RollCtrP":
                    ret = RollCtr.P.ToString();
                    break;
                case "RollCtrI":
                    ret = RollCtr.I.ToString();
                    break;
                case "YawCtrP":
                    ret = YawCtr.P.ToString();
                    break;
                case "YawCtrI":
                    ret = YawCtr.I.ToString();
                    break;
                case "DisableServos":
                    ret = DisableServos ? "1" : "0";
                    break;
                case "RadioFrequency":
                    ret = RadioModem.Frequency.ToString();
                    break;
                case "UseStatusChannel":
                    ret = UseStatusChannel ? "1" : "0";
                    break;
                case "DestLatitude":
                    ret = Navi.DestLatitude.ToString();
                    break;
                case "DestLongitude":
                    ret = Navi.DestLongitude.ToString();
                    break;
                case "TargetAirSpeed":
                    ret = TargetAirSpeed.ToString();
                    break;
                case "CalibrationMode":
                    ret = Magnetometer.CalibrationMode ? "1" : "0";
                    break;
                case "Altitude":
                    ret = Barometer.Altitude.ToString();
                    break;
                default:
                    ValueUnknown = true;
                    break;
            }

            return ret;
        }
    }
}
