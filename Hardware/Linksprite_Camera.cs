using System;
using Microsoft.SPOT;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Collections;

using System.IO;
using Microsoft.SPOT.IO;


namespace PlaneOnBoardSoftware
{
    class Linksprite_Camera : IDisposable
    {
        static readonly byte[] RESET_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x26, 0x00 };
        static readonly byte[] RESET_COMMAND = new byte[] { 0x56, 0x00, 0x26, 0x00 };

        static readonly byte[] STOP_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x36, 0x00, 0x00 };
        static readonly byte[] STOP_COMMAND = new byte[] { 0x56, 0x00, 0x36, 0x01, 0x03 };

        static readonly byte[] SNAP_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x36, 0x00, 0x00 };
        static readonly byte[] SNAP_COMMAND = new byte[] { 0x56, 0x00, 0x36, 0x01, 0x00 };

        static readonly byte[] SIZE_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x34, 0x00, 0x04, 0x00, 0x00 };
        static readonly byte[] SIZE_COMMAND = new byte[] { 0x56, 0x00, 0x34, 0x01, 0x00 };

        static readonly byte[] GET_CHUNK_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x32, 0x00, 0x00 };

        //56 00 32 0C 00 0A 00 00 MH ML 00 00 KH KL XX XX:
        static readonly byte[] GET_CHUNK_COMMAND = new byte[] { 0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A, 0x00, 0x00, 255, 255, 0x00, 0x00, 255, 255, 0x00, 0x0A };

        static readonly byte[] SET_SIZE_160x120 = new byte[] { 0x56, 0x00, 0x31, 0x05, 0x04, 0x01, 0x00, 0x19, 0x22 };
        static readonly byte[] SET_SIZE_320x240 = new byte[] { 0x56, 0x00, 0x31, 0x05, 0x04, 0x01, 0x00, 0x19, 0x11 };
        static readonly byte[] SET_SIZE_640x480 = new byte[] { 0x56, 0x00, 0x31, 0x05, 0x04, 0x01, 0x00, 0x19, 0x00 };
        static readonly byte[] SET_SIZE_OK_RESPONSE = new byte[] { 0x76, 0, 0x31, 0 };

        static readonly byte[] SET_BAUDRATE_115200 = new byte[] { 0x56, 0x00, 0x24, 0x03, 0x01, 0x0D, 0xA6 };
        static readonly byte[] SET_BAUDRATE_57600 = new byte[] { 0x56, 0x00, 0x24, 0x03, 0x01, 0x1C, 0x4C };
        static readonly byte[] SET_BAUDRATE_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x24, 0x00, 0x00 };

        static readonly byte[] SET_COMPRESSION = new byte[] { 0x56, 0x00, 0x31, 0x05, 0x01, 0x10, 0x12, 0x04, 255 };
        static readonly byte[] SET_COMPRESSION_OK_RESPONSE = new byte[] { 0x76, 0x00, 0x31, 0x00, 0x00};

        SerialPort port;
        FileStream opendJpegFile;
        Thread WriteAsync;

        public delegate void ActionBytes(byte[] chunk);
        public delegate void Complete();

        const int IN_BUFFER_SIZE = 960;
        byte[] InBuffer = new byte[IN_BUFFER_SIZE];

        bool isActive = true;
        bool newWork = false;

        public Linksprite_Camera(SerialPort port)
        {
            port.BaudRate = 38400;
            port.Parity = Parity.None;
            port.DataBits = 8;
            port.StopBits = StopBits.One;

            WriteAsync = new Thread(startAsync);
            WriteAsync.Priority = ThreadPriority.Lowest;
            WriteAsync.Start();
            while (!WriteAsync.IsAlive) ;
            WriteAsync.Suspend();

            this.port = port;

            port.ReadTimeout = 250; //so read call doesn't block forever
            port.Open();

            //Reset();
            Debug.Print("Compr: " + SetCompression(1));
            Debug.Print("Size: " +SetPictureSize(SET_SIZE_640x480));
            Reset();
            

           //Debug.Print("Answ: " + SendAndLookFor(SET_BAUDRATE_115200, SET_BAUDRATE_OK_RESPONSE));

           //port.Close();
           //port.BaudRate = 115200;
           //port.Open();
        }

        public bool Reset()
        {
            bool sendAndLookFor = SendAndLookFor(RESET_COMMAND, RESET_OK_RESPONSE);

            //camera needs time after reset
            if (sendAndLookFor)
            {
                ReadAllRemaining();
                Thread.Sleep(3000);
            }

            return sendAndLookFor;
        }

        public bool SetPictureSize(byte[] sizeBytes)
        {
            bool sendAndLookFor = SendAndLookFor(sizeBytes, SET_SIZE_OK_RESPONSE);
            if (sendAndLookFor)
                ReadAllRemaining();

            return sendAndLookFor;
        }

        public bool SetCompression(byte compression)
        {
            SET_COMPRESSION[8] = compression;
            bool sendAndLookFor = SendAndLookFor(SET_COMPRESSION, SET_COMPRESSION_OK_RESPONSE);
            if (sendAndLookFor)
                ReadAllRemaining();

            return sendAndLookFor;
        }

        public bool Stop()
        {
            if (opendJpegFile != null) opendJpegFile.Close();
            ReadAllRemaining();
            return SendAndLookFor(STOP_COMMAND, STOP_OK_RESPONSE);
        }

        public void GetPicture(ActionBytes bytesAction)
        {
            Send(SNAP_COMMAND);
            if (LookFor(SNAP_OK_RESPONSE))
            {
                Send(SIZE_COMMAND);
                if (LookFor(SIZE_OK_RESPONSE))
                {
                    //MSB, LSB
                    var sizeBytesLength = Read(2);
                    int fileSize = (InBuffer[0] << 8) | InBuffer[1];

                    int startAddress = 0;
                    int bytesRead = 0;

                    GET_CHUNK_COMMAND[12] = MSB(IN_BUFFER_SIZE);
                    GET_CHUNK_COMMAND[13] = LSB(IN_BUFFER_SIZE);

                    bool endReached = false;
                    while (!endReached)
                    {
                        GET_CHUNK_COMMAND[8] = MSB(startAddress);
                        GET_CHUNK_COMMAND[9] = LSB(startAddress);

                        Send(GET_CHUNK_COMMAND);
                        if (LookFor(GET_CHUNK_OK_RESPONSE))
                        {
                            int chunkLength = 0;
                            do
                            {
                                //Thread.Sleep(10);

                                chunkLength = Read();

                                //ditch footer
                                Read(junkBuffer, GET_CHUNK_OK_RESPONSE.Length);

                                //publish byte data
                                if (chunkLength > 0)
                                {
                                    bytesRead += chunkLength;

                                    Debug.Print(bytesRead.ToString() + "  /  " + fileSize.ToString() + " (" + chunkLength.ToString() + ")");
                                    
                                    if (bytesRead >= fileSize)
                                    {
                                        endReached = true;

                                        chunkLength = FindEnd(chunkLength);
                                    }

                                    bytesAction(NewArray(chunkLength));
                                }

                                startAddress += chunkLength;

                            } while (!endReached && chunkLength > 0);
                        }
                    }
                }

                Stop();
            }
        }

        public void SavePictureToSD(string FilePath)
        {
            if (!newWork)
            {
                opendJpegFile = new FileStream(FilePath, FileMode.OpenOrCreate);

                newWork = true;
                WriteAsync.Resume();
            }
        }

        private void startAsync()
        {
            while (isActive)
            {
                if (newWork)
                {
                    GetPicture(SaveChuck);
                    newWork = false;
                    WriteAsync.Suspend();
                }
            }
        }

        private void SaveChuck(byte[] chunk)
        {
            opendJpegFile.Write(chunk, 0, chunk.Length);
        }

        private byte[] NewArray(int chunkLength)
        {
            //make new array for bytes event so receiver can consume
            //in sep thread without it changing during processing
            var chunk = new byte[chunkLength];
            Array.Copy(InBuffer, chunk, chunkLength);
            return chunk;
        }

        private int FindEnd(int chunkLength)
        {
            if (chunkLength >= 2)
            {
                bool foundEnd = false;

                for (int i = chunkLength - 1; i >= 2; i--)
                {
                    if (InBuffer[i - 1] == 0xFF &&
                    InBuffer[i - 0] == 0xD9
                    )
                    {
                        chunkLength = i + 1; //include end marker in output
                        foundEnd = true;
                        break;
                    }
                }

                if (!foundEnd)
                    Debug.Print("Invalid JPG data");
            }
            return chunkLength;
        }

        private static byte LSB(int num)
        {
            return (byte)(num & 0xFF);
        }

        private static byte MSB(int num)
        {
            return (byte)(num >> 8);
        }

        private bool SendAndLookFor(byte[] command, byte[] lookFor)
        {
            Send(command);

            return LookFor(lookFor);
        }

        byte[] junkBuffer = new byte[IN_BUFFER_SIZE];
        private void ReadAllRemaining()
        {
            int readCount = 0;

            do
            {
                readCount = Read(junkBuffer, IN_BUFFER_SIZE);
            } while (readCount != 0);
        }

        private bool LookFor(byte[] expectedResponse)
        {
            var inSize = Read(expectedResponse.Length);
            if (AreEqual(expectedResponse, inSize))
                return true;

            return false;
        }

        private int Read()
        {
            return Read(IN_BUFFER_SIZE);
        }

        private int Read(int bytes)
        {
            return Read(InBuffer, bytes);
        }

        private int Read(byte[] buffer, int bytes)
        {
            return port.Read(buffer, 0, bytes);
        }

        private void Send(byte[] command)
        {
            port.Write(command, 0, command.Length);
        }

        private bool AreEqual(byte[] left, int inSize)
        {
            if (left == null || left.Length != inSize)
                return false;

            for (int i = 0; i < left.Length; i++)
                if (left[i] != InBuffer[i])
                    return false;

            return true;
        }

        public void Dispose()
        {
            isActive = false;
            if (port != null)
                port.Dispose();
        }
    }
}