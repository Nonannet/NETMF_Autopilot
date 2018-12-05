//Nach dem c-Code von Ulrich Radig (http://www.ulrichradig.de)

using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System.Threading;

namespace PlaneOnBoardSoftware
{
    class RFM22b
    {
        public class CycleByteBuffer
        {
            private byte[] byteBuffer;

            private int rInd = 0;
            private int wInd = 0;
            private bool isFull = false;

            public CycleByteBuffer(int BufferSize)
            {
                byteBuffer = new Byte[BufferSize];
            }

            public int GetBytesToRead()
            {
                if (isFull && wInd == rInd)
                    return byteBuffer.Length;
                else
                    return (byteBuffer.Length + wInd - rInd) % byteBuffer.Length;
            }

            public int GetFreeByteCount()
            {
                if (!isFull && wInd == rInd)
                    return byteBuffer.Length;
                else
                    return (byteBuffer.Length + rInd - wInd) % byteBuffer.Length;
            }

            public int WriteBytes(byte[] Data, int OffSet, int Lenght)
            {
                int len = Lenght;
                int freeLen = GetFreeByteCount();

                if (len >= freeLen)
                {
                    len = freeLen;
                    isFull = true;
                }

                addToBuffer(Data, OffSet, byteBuffer, wInd, len);
                wInd = (wInd + len) % byteBuffer.Length;

                return len;
            }

            public int ReadBytes(byte[] Data, int OffSet, int Lenght)
            {
                return ReadBytes(Data, OffSet, Lenght, false);
            }

            public int ReadBytes(byte[] Data, int OffSet, int Lenght, bool KeepInBuffer)
            {
                int len = Lenght;
                int bytesLen = GetBytesToRead();

                if (len >= bytesLen)
                {
                    len = bytesLen;
                    if (!KeepInBuffer) isFull = false;
                }

                getFromBuffer(byteBuffer, rInd, Data, OffSet, len);
                if (!KeepInBuffer) rInd = (rInd + len) % byteBuffer.Length;

                return len;
            }

            public int GetNextLineBr()
            {
                int rPos = rInd;
                int lastrPos;
                int count = GetBytesToRead();

                for (int i = 0; i < count - 1; i++)
                {
                    lastrPos = rPos;
                    rPos++;
                    if (rPos == byteBuffer.Length) rPos = 0;
                    if (byteBuffer[lastrPos] == 13 && byteBuffer[rPos] == 10) return i;
                }

                return 0;
            }

            public int DropBytes(int ByteCount)
            {
                int bytesToRemove = GetBytesToRead();

                if (ByteCount < bytesToRemove)
                    bytesToRemove = ByteCount;
                else
                    isFull = false;

                rInd = (rInd + bytesToRemove) % byteBuffer.Length;
                return bytesToRemove;
            }

            public void ClearBuffer()
            {
                wInd = 0;
                rInd = 0;
                isFull = false;
            }

            private void addToBuffer(byte[] Data, int srcOffSet, byte[] Buffer, int destOffSet, int Lenght)
            {
                if (destOffSet + Lenght > Buffer.Length)
                {
                    int pLen = Buffer.Length - destOffSet;
                    Array.Copy(Data, srcOffSet, Buffer, destOffSet, pLen);
                    Array.Copy(Data, srcOffSet + pLen, Buffer, 0, Lenght - pLen);
                }
                else
                {
                    Array.Copy(Data, srcOffSet, Buffer, destOffSet, Lenght);
                }
            }

            private void getFromBuffer(byte[] Buffer, int srcOffSet, byte[] Data, int destOffSet, int Lenght)
            {
                if (srcOffSet + Lenght > Buffer.Length)
                {
                    int pLen = Buffer.Length - srcOffSet;
                    Array.Copy(Buffer, srcOffSet, Data, destOffSet, pLen);
                    Array.Copy(Buffer, 0, Data, destOffSet + pLen, Lenght - pLen);
                }
                else
                {
                    Array.Copy(Buffer, srcOffSet, Data, destOffSet, Lenght);
                }
            }
        }

        public class Channel
        {
            public byte chanID;
            public RFM22b rfMod;
            private CycleByteBuffer TxBuffer;
            private CycleByteBuffer RxBuffer;

            public byte Priority = 0;

            public int BytesToRead
            {
                get { return RxBuffer.GetBytesToRead(); }
            }

            public int BytesToSend
            {
                get { return TxBuffer.GetBytesToRead(); }
            }

            public Channel(RFM22b RFModul, byte ChannelID, int RxBufferSize, int TxBufferSize)
            {
                TxBuffer = new CycleByteBuffer(TxBufferSize);
                RxBuffer = new CycleByteBuffer(RxBufferSize);

                chanID = ChannelID;
                rfMod = RFModul;

                rfMod.addChannel(this, chanID);
            }

            public int Send(byte[] Data, int OffSet, int Lenght)
            {
                int retVal = TxBuffer.WriteBytes(Data, OffSet, Lenght);

                rfMod.sendNextPacket();

                return retVal;
            }

            public int Send(string Text)
            {
                byte[] txtData = System.Text.UTF8Encoding.UTF8.GetBytes(Text);
                int retVal = TxBuffer.WriteBytes(txtData, 0, txtData.Length);
                rfMod.sendNextPacket();
                return retVal;
            }

            public int SendLine(string Text)
            {
                return Send(Text + "\r\n");
            }

            public int Recv(byte[] Data, int OffSet, int Lenght)
            {
                return RxBuffer.ReadBytes(Data, OffSet, Lenght, false);
            }

            public string RecvLine()
            {
                int byteCount = RxBuffer.GetNextLineBr();
                byte[] Data = new byte[byteCount];

                if (byteCount > 0)
                {
                    RxBuffer.ReadBytes(Data, 0, byteCount);
                    RxBuffer.DropBytes(2);
                    return new string(System.Text.UTF8Encoding.UTF8.GetChars(Data));
                }
                else
                {
                    return "";
                }
            }

            public int AddBytesToRxBuffer(byte[] Data, int OffSet, int Lenght)
            {
                return RxBuffer.WriteBytes(Data, OffSet, Lenght);
            }

            public int GetBytesFromTxBuffer(byte[] Data, int OffSet, int Lenght)
            {
                return TxBuffer.ReadBytes(Data, OffSet, Lenght, true);
            }

            public void RemoveBytesFromTxBuffer(int ByteCount)
            {
                TxBuffer.DropBytes(ByteCount);
            }

            public void ClearTxBuffer()
            {
                TxBuffer.ClearBuffer();
                rfMod.CancelPacketFlag = true;
            }

            public void ClearRxBuffer()
            {
                RxBuffer.ClearBuffer();
            }
        }

        public enum ModemState
        {
            Init,
            Ready,
            Transmitting,
            Receving,
            Error
        };

        private enum PacketAttrib
        {
            Normal,
            ClearRecvBuffer
        };

        SPI.Configuration SpiConfig;
        SPI SpiBus;
        InterruptPort IRQn;
        Channel[] ChannelList = new Channel[8];
        Timer TimeOutTimer;

        public ModemState State = ModemState.Init;

        public int RSSI;
        public int LostPackets = 0;

        private int rxPackNr = 0;
        private int txPackNr = 0;

        private Channel lastTxChan;
        private int lastTxPackLen = 60;
        private int lastRxPackLen = 0;
        private bool CancelPacketFlag = false;
        private byte InterruptStatus1 = 0;

        private double _frequency = 869.545;

        public double Frequency
        {
            get
            {
                return _frequency;
            }
            set
            {
                rf22_setfreq((byte)((value - 860.0) * 3200));
            }
        }

        public RFM22b(SPI.SPI_module SpiMmodule, Cpu.Pin ChipSelectPort, Cpu.Pin IRQPort, double FrequInHz)
        {
            SpiConfig = new SPI.Configuration(ChipSelectPort, false, 0, 0, false, true, 1000, SpiMmodule);
            SpiBus = new SPI(SpiConfig);
            IRQn = new InterruptPort(IRQPort, false, Port.ResistorMode.Disabled, Port.InterruptMode.InterruptEdgeLow);

            IRQn.OnInterrupt += new NativeEventHandler(IRQn_OnInterrupt);

            rf22_init();
            rf22_setfreq((byte)((_frequency - 860.0) * 3200));

            rf22_rxmode();
            TimeOutTimer = new Timer(new TimerCallback(RecvTimeOut), null, 500, Timeout.Infinite);

            State = ModemState.Receving;
        }

        public Channel NewChannel(byte ChannelID, int RxBufferSize, int TxBufferSize)
        {
            return new Channel(this, ChannelID, RxBufferSize, TxBufferSize);
        }

        private void addChannel(Channel chanToAdd, byte chanID)
        {
            ChannelList[chanID] = chanToAdd;
        }

        private void sendNextPacket()
        {
            int hcIndex = -1;
            int prio = -1;

            //Debug.Print("--> Send State" + State + " <--");

            if (State == ModemState.Ready)
            {
                for (int i = 0; i < ChannelList.Length; i++)
                {
                    if (ChannelList[i] != null)
                    {
                        if (ChannelList[i].BytesToSend > 0 && ChannelList[i].Priority > prio)
                        {
                            hcIndex = i;
                            prio = ChannelList[i].Priority;
                        }
                    }
                }

                int byteCount;
                byte[] Buffer = new byte[64];

                Buffer[0] = (byte)rxPackNr;
                Buffer[1] = (byte)txPackNr;
                Buffer[2] = (byte)hcIndex;
                Buffer[3] = CancelPacketFlag ? (byte)PacketAttrib.ClearRecvBuffer : (byte)PacketAttrib.Normal;


                //Debug.Print("--> Send hcIndex" + hcIndex + " <--");

                if (hcIndex > -1)
                {
                    byteCount = ChannelList[hcIndex].GetBytesFromTxBuffer(Buffer, 4, lastTxPackLen);

                    //Debug.Print("** Send " +  byteCount + " **");

                    State = ModemState.Transmitting;
                    rf22_sendpacket(Buffer, 0, byteCount + 4);

                    lastTxChan = ChannelList[hcIndex];
                    lastTxPackLen = byteCount;
                }
                else if (lastRxPackLen > 4)
                {
                    //Debug.Print("** Send Empty **");
                    lastRxPackLen = 0;
                    State = ModemState.Transmitting;
                    rf22_sendpacket(Buffer, 0, 4);
                }
                else
                {
                    rf22_rxmode();
                }
            }
        }

        private void readPacket()
        {
            byte[] Buffer = new byte[64];
            int cInd;
            int packetLen;
            int newRxPacketNr;

            packetLen = rf22_getpacket(Buffer, 0);
            lastRxPackLen = packetLen;


            //Debug.Print("** Receve " + packetLen + " **");
            if (packetLen > 3)
            {


                newRxPacketNr = Buffer[1];

                cInd = Buffer[2]; //Channel ID


                if (Buffer[0] == txPackNr && lastTxChan != null)
                {
                    lastTxChan.RemoveBytesFromTxBuffer(lastTxPackLen);
                    lastTxPackLen = 60;
                    txPackNr++;
                    if (txPackNr > 255) txPackNr = 0;
                }

                if (cInd < ChannelList.Length)
                {
                    if (ChannelList[cInd] != null && rxPackNr != newRxPacketNr && packetLen > 4)
                    {
                        if (Buffer[3] == (byte)PacketAttrib.ClearRecvBuffer) ChannelList[cInd].ClearRxBuffer();
                        ChannelList[cInd].AddBytesToRxBuffer(Buffer, 4, packetLen - 4);
                        rxPackNr = newRxPacketNr;
                    }
                }
            }
        }

        private void RecvTimeOut(object nullObj)
        {
            if (State == ModemState.Receving)
            {
                State = ModemState.Ready;
                RSSI = -1;
                if (LostPackets == int.MaxValue) LostPackets = 0;
                LostPackets++;
                sendNextPacket();
            }
        }

        private void IRQn_OnInterrupt(uint port, uint state, DateTime time)
        {
            InterruptStatus1 = rf22_read(0x03);

            if (State == ModemState.Transmitting)
            {
                // All Data Transmitted
                rf22_write(0x07, 0x01);		//	switch to ready mode
                rf22_rxmode();
                TimeOutTimer.Dispose();
                TimeOutTimer = new Timer(new TimerCallback(RecvTimeOut), null, 500, Timeout.Infinite);
                State = ModemState.Receving;
            }
            else if (State == ModemState.Receving || State == ModemState.Ready)
            {
                RSSI = rf22_read(0x26);
                State = ModemState.Ready;
                readPacket();
                sendNextPacket();
            }
        }


        #region Hardware Commands

        private void rf22_write(byte addr, byte data)
        {
            byte[] addrArray = new byte[2];

            addrArray[0] = (byte)(128 | addr);
            addrArray[1] = data;

            SpiBus.Write(addrArray);
        }

        private byte rf22_read(byte addr)
        {
            byte[] ret = new byte[2];
            SpiBus.WriteRead(new byte[] { (byte)(addr & 127), 0xFF }, ret);
            return ret[1];
        }

        private void rf22_burstread(byte addr, byte[] data, int OffSet, int Lenght)
        {
            byte[] arg = new byte[Lenght + 1];

            arg[0] = (byte)(addr & 127);

            SpiBus.WriteRead(arg, 0, Lenght + 1, data, OffSet, Lenght, 1);
        }

        private void rf22_init()
        {
            Thread.Sleep(20);

            rf22_write(0x07, 0x80);		// software reset

            Thread.Sleep(20);

            rf22_write(0x05, 0x06);		// valid packed received and packet send interrupt on

            rf22_write(0x06, 0x00);		// all interrupts off
            rf22_write(0x07, 0x01);		// operating mode: ready mode
            rf22_write(0x09, 0x7f);		// xtal load capacitance
            rf22_write(0x0A, 0x02);		// uC CLK: 10MHz

            rf22_write(0x0b, 0xf2);		// GPIO0: TX_ANT - f2 
            rf22_write(0x0c, 0xf5);		// GPIO1: RX ANT - f5
            rf22_write(0x0d, 0x00);		// GPIO2: uC Clock out
            rf22_write(0x0e, 0x00);
            rf22_write(0x0f, 0x70);		// ADC Input: GND
            rf22_write(0x10, 0x00);		// ADC offset: 0
            rf22_write(0x12, 0x00);		// temp sensor calibration off
            rf22_write(0x13, 0x00);		// temp sensor offset: 0
            rf22_write(0x1d, 0x40);		// enable AFC
            rf22_write(0x1e, 0x0A);		// afc timing
            rf22_write(0x1f, 0x03);		// afc timing

            rf22_write(0x1C, 0x05);		// IF bandwidth
            rf22_write(0x20, 0x83);		// Clock Recovery Oversampling Rate
            rf22_write(0x21, 0xC0);		// Clock Recovery Offset 2
            rf22_write(0x22, 0x13);		// Clock Recovery Offset 1
            rf22_write(0x23, 0xA9);		// Clock Recovery Offset 0
            rf22_write(0x24, 0x00);		// Clock Recovery Timing Loop Gain 1
            rf22_write(0x25, 0x04);		// Clock Recovery Timing Loop Gain 0
            rf22_write(0x2A, 0x24);

            rf22_write(0x27, 0x10);		// RSSI Threashold: -120dB

            rf22_write(0x30, 0x8c);		// data access: RX/TX packet handling, enable crc: CCIT
            rf22_write(0x32, 0xff);		// header check enable
            rf22_write(0x33, 0x42);		// 2 word synchronisation
            rf22_write(0x34, 0x10);		// preamble length: 16 nibbles, = 64bits
            rf22_write(0x35, 0x30);		// preamble detection control: 6 nibbles = 24bits
            rf22_write(0x36, 0x2d);		// sync word 3
            rf22_write(0x37, 0xd4);		// sync word 2
            rf22_write(0x38, 0xAA);		// sync word 1
            rf22_write(0x39, 0xAA);		// sync word 0
            rf22_write(0x3a, 101);		// transmit header 3
            rf22_write(0x3b, 108);		// transmit header 2
            rf22_write(0x3c, 103);		// transmit header 1
            rf22_write(0x3d, 106);		// transmit header 0
            rf22_write(0x3e, 17);		// packet length
            rf22_write(0x3f, 101);		// check header 3
            rf22_write(0x40, 108);		// check header 2
            rf22_write(0x41, 103);		// check header 1
            rf22_write(0x42, 106);		// check header 0
            rf22_write(0x43, 0xff);		// header enable mask 3
            rf22_write(0x44, 0xff);		// header enable mask 2
            rf22_write(0x45, 0xff);		// header enable mask 1
            rf22_write(0x46, 0xff);		// header enable mask 0

            rf22_write(0x69, 0x60);		// AGC on
            rf22_write(0x6a, 0x0b);		// agc override 2

            //rf22_write(0x6d, 0x08);		// tx power: +1dBm
            rf22_write(0x6d, 0x0F);		// tx power: +17dBm (nicht 20dbm??)

            //baud rate: 2,39 kBit/s = val("&H13A9")/2^(16+5)*1000 kHz
            rf22_write(0x6E, 0x13);		// set baud high
            rf22_write(0x6F, 0xA9);		// set baud low

            rf22_write(0x70, 0x2C);		// modulation control 
            rf22_write(0x71, 0x22);		// modulation control 2: FIFO mode, OOK  //0x21 / 0x00

            rf22_write(0x72, 0x50);		// frequency deviation: 45kHz
            rf22_write(0x73, 0x00);		// offset: 0
            rf22_write(0x74, 0x00);		// offset: 0

            rf22_write(0x79, 0x0);		// frequency hopping off
            rf22_write(0x7a, 0x0);		// frequency hopping off

            rf22_write(0x75, 0x73);		// 860-880MHz range
            //rf22_write(0x75, 0x53);		// 430-440MHz range

            rf22_write(0x08, 0x00);		// clear fifo, disable multi packet
        }

        public void rf22_rxmode()
        {
            rf22_read(0x03);			// clear interrupt status
            rf22_read(0x04);			// clear interrupt status
            rf22_write(0x07, 0x01);		// to_ready_mode();

            rf22_write(0x07, 0x01);		// to_ready_mode();
            rf22_write(0x7e, 0x40);		// threshold for rx almost full, interrupt when 64 bytes received

            rf22_write(0x08, 0x03);		// clear RX fifo
            rf22_write(0x08, 0x00);     // clear RX fifo

            rf22_write(0x07, 0x05);		// RX on

            rf22_read(0x03);			// clear interrupt status
            rf22_read(0x04);			// clear interrupt status
        }


        private void rf22_setfreq(UInt16 freq)
        {
            rf22_write(0x76, (byte)((freq & 0xFF00) >> 8));
            rf22_write(0x77, (byte)(freq & 0x00FF));
        }

        private void rf22_sendpacket(byte[] data, int offset, int lenght)
        {
            //Debug.Print("---> " + lenght);

            byte i;
            byte size = (byte)lenght;

            if (size > 64) size = 64;

            rf22_write(0x07, 0x03);		// switch to ready mode
            rf22_read(0x03);			// clear interrupt status
            rf22_read(0x04);			// clear interrupt status

            rf22_write(0x08, 0x01);		// clear TX fifo
            rf22_write(0x08, 0x00);		// clear TX fifo

            //rf22_write(0x34, 32);		// premable length: 32 nibbles -> 128 Bits
            rf22_write(0x3e, size);		// packet length

            for (i = 0; i < size; i++)
            {
                rf22_write(0x7f, data[i + offset]);
            }

            rf22_write(0x07, 0x09);		// TX on
        }

        private int rf22_getpacket(byte[] data, int offset)
        {
            byte cnt;

            //if ((rf22_read(0x31) & 0x1A) > 0)		// receiving a packet
            //{
            if ((InterruptStatus1 & 2) > 0)	// packet received & not read  && ((rf22_read(0x02) & 32) == 0)
            {
                cnt = rf22_read(0x4B);		// packet length
                //Debug.Print(cnt + " " + data[cnt-2].ToString());

                /*for (int i = 0; i < cnt; i++)		// Daten (cnt - 2 für CRC)
                {
                    data[i + offset] = rf22_read(0x7f);
                }*/
                rf22_burstread(0x7f, data, offset, cnt);

                /*try
                {
                    byte[] Test = new byte[cnt - 4];
                    Array.Copy(data, 4, Test, 0, cnt - 4);

                    string tmpCa = (new string(System.Text.UTF8Encoding.UTF8.GetChars(Test)));
                    Debug.Print(tmpCa);
                }
                catch
                {
                    Debug.Print("Error");
                }*/

                return (cnt);
            }
            return 0;
            //}
            //return 0;
        }

        #endregion
    }
}