using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace LivoxPointCloudPlayer
{
    internal sealed class LivoxDevice
    {
        public int DeviceType;
        public string Serial;
        public string Ip;
        public int CommandPort;

        public string Model
        {
            get
            {
                if (DeviceType == 9) return "MID-360";
                if (DeviceType == 10) return "HAP Industrial";
                if (DeviceType == 15) return "HAP";
                return "TYPE_" + DeviceType.ToString(CultureInfo.InvariantCulture);
            }
        }

        public int HostPointPort { get { return DeviceType == 9 ? 56301 : 57000; } }
        public int LidarPointPort { get { return DeviceType == 9 ? 56300 : 57000; } }
        public int HostImuPort { get { return DeviceType == 9 ? 56401 : 58000; } }
        public int LidarImuPort { get { return DeviceType == 9 ? 56400 : 58000; } }
        public int HostCommandPort { get { return DeviceType == 9 ? 56101 : 56000; } }

        public override string ToString()
        {
            return Model + "  " + Ip + "  " + (string.IsNullOrEmpty(Serial) ? "(no serial)" : Serial);
        }
    }

    internal sealed class NetworkAdapterRow
    {
        public string Name;
        public string Ip;
        public override string ToString() { return Name + "  ·  " + Ip; }
    }

    internal sealed class LivoxWorkState
    {
        public byte TargetMode;
        public byte CurrentState;
        public byte PointSendState;
    }

    internal static class LivoxUdpProtocol
    {
        private const int HeaderSize = 24;

        public static List<NetworkAdapterRow> ListAdapters()
        {
            List<NetworkAdapterRow> rows = new List<NetworkAdapterRow>();
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (NetworkInterface adapter in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (adapter.OperationalStatus != OperationalStatus.Up || adapter.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;
                IPInterfaceProperties properties;
                try { properties = adapter.GetIPProperties(); }
                catch { continue; }
                foreach (UnicastIPAddressInformation address in properties.UnicastAddresses)
                {
                    if (address.Address.AddressFamily != AddressFamily.InterNetwork) continue;
                    string ip = address.Address.ToString();
                    if (ip.StartsWith("127.", StringComparison.Ordinal) || ip.StartsWith("169.254.", StringComparison.Ordinal) || !seen.Add(ip)) continue;
                    rows.Add(new NetworkAdapterRow { Name = adapter.Name, Ip = ip });
                }
            }
            rows.Sort(delegate(NetworkAdapterRow a, NetworkAdapterRow b)
            {
                bool ae = a.Name.IndexOf("ethernet", StringComparison.OrdinalIgnoreCase) >= 0 || a.Name.IndexOf("以太网", StringComparison.OrdinalIgnoreCase) >= 0;
                bool be = b.Name.IndexOf("ethernet", StringComparison.OrdinalIgnoreCase) >= 0 || b.Name.IndexOf("以太网", StringComparison.OrdinalIgnoreCase) >= 0;
                if (ae != be) return ae ? -1 : 1;
                int name = string.Compare(a.Name, b.Name, StringComparison.OrdinalIgnoreCase);
                return name != 0 ? name : string.Compare(a.Ip, b.Ip, StringComparison.OrdinalIgnoreCase);
            });
            return rows;
        }

        public static List<LivoxDevice> Discover(string hostIp, int timeoutMilliseconds)
        {
            IPAddress hostAddress = IPAddress.Parse(hostIp);
            List<LivoxDevice> devices = new List<LivoxDevice>();
            Dictionary<string, LivoxDevice> unique = new Dictionary<string, LivoxDevice>(StringComparer.OrdinalIgnoreCase);
            using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
            {
                socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, true);
                socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                socket.Bind(new IPEndPoint(hostAddress, 0));
                socket.ReceiveTimeout = 180;
                uint sequence = unchecked((uint)Environment.TickCount);
                byte[] request = MakeCommand(0x0000, new byte[0], sequence);
                List<byte> info = new List<byte>();
                PutUInt16(info, 2); PutUInt16(info, 0); PutUInt16(info, 0x8000); PutUInt16(info, 0x8001);
                byte[] infoRequest = MakeCommand(0x0101, info.ToArray(), sequence + 1);
                string subnetBroadcast = GetClassCBroadcast(hostAddress);
                DateTime deadline = DateTime.UtcNow.AddMilliseconds(Math.Max(200, timeoutMilliseconds));

                while (DateTime.UtcNow < deadline)
                {
                    socket.SendTo(request, new IPEndPoint(IPAddress.Broadcast, 56000));
                    SendInfo(socket, infoRequest, "255.255.255.255", 56000);
                    SendInfo(socket, infoRequest, "255.255.255.255", 56100);
                    if (subnetBroadcast != "255.255.255.255")
                    {
                        SendInfo(socket, infoRequest, subnetBroadcast, 56000);
                        SendInfo(socket, infoRequest, subnetBroadcast, 56100);
                    }
                    DateTime waitUntil = DateTime.UtcNow.AddMilliseconds(300);
                    if (waitUntil > deadline) waitUntil = deadline;
                    while (DateTime.UtcNow < waitUntil)
                    {
                        byte[] packet = new byte[2048];
                        EndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                        int length;
                        try { length = socket.ReceiveFrom(packet, ref sender); }
                        catch (SocketException ex)
                        {
                            if (ex.SocketErrorCode == SocketError.TimedOut || ex.SocketErrorCode == SocketError.WouldBlock) break;
                            throw;
                        }
                        ushort commandId;
                        byte[] payload;
                        if (!TryParseCommand(packet, length, out commandId, out payload)) continue;
                        IPEndPoint remote = (IPEndPoint)sender;
                        LivoxDevice device = null;
                        if (commandId == 0 && payload.Length >= 24 && payload[0] == 0)
                        {
                            string serial = ReadAscii(payload, 2, 16);
                            string lidarIp = new IPAddress(new byte[] { payload[18], payload[19], payload[20], payload[21] }).ToString();
                            if (lidarIp == "0.0.0.0") lidarIp = remote.Address.ToString();
                            device = new LivoxDevice
                            {
                                DeviceType = payload[1], Serial = serial, Ip = lidarIp,
                                CommandPort = ReadUInt16(payload, 22)
                            };
                        }
                        else if (commandId == 0x0101 && payload.Length >= 3 && payload[0] == 0)
                        {
                            Dictionary<ushort, byte[]> values = ReadKeyValues(payload, 3, ReadUInt16(payload, 1));
                            byte[] serialBytes;
                            if (!values.TryGetValue(0x8000, out serialBytes)) continue;
                            string serial = ReadAscii(serialBytes, 0, serialBytes.Length);
                            if (serial.Length == 0) continue;
                            byte[] productBytes;
                            string product = values.TryGetValue(0x8001, out productBytes) ? ReadAscii(productBytes, 0, productBytes.Length).ToUpperInvariant() : "";
                            int type;
                            if (product.Contains("HAP") || remote.Port == 56000) type = 10;
                            else if (product.Contains("MID-360") || product.Contains("MID360") || remote.Port == 56100) type = 9;
                            else continue;
                            device = new LivoxDevice { DeviceType = type, Serial = serial, Ip = remote.Address.ToString(), CommandPort = remote.Port };
                        }
                        if (device == null || (device.DeviceType != 9 && device.DeviceType != 10 && device.DeviceType != 15)) continue;
                        string key = string.IsNullOrEmpty(device.Serial) ? device.Ip : device.Serial;
                        unique[key] = device;
                    }
                }
            }
            foreach (LivoxDevice value in unique.Values) devices.Add(value);
            devices.Sort(delegate(LivoxDevice a, LivoxDevice b) { return string.Compare(a.Ip, b.Ip, StringComparison.OrdinalIgnoreCase); });
            return devices;
        }

        public static int ConfigureStream(LivoxDevice device, string hostIp, int timeoutMilliseconds)
        {
            List<byte> values = new List<byte>();
            AddKeyValue(values, 0x0000, new byte[] { 1 });
            List<byte> pointDestination = new List<byte>(IPAddress.Parse(hostIp).GetAddressBytes());
            PutUInt16(pointDestination, (ushort)device.HostPointPort); PutUInt16(pointDestination, (ushort)device.LidarPointPort);
            AddKeyValue(values, 0x0006, pointDestination.ToArray());
            List<byte> imuDestination = new List<byte>(IPAddress.Parse(hostIp).GetAddressBytes());
            PutUInt16(imuDestination, (ushort)device.HostImuPort); PutUInt16(imuDestination, (ushort)device.LidarImuPort);
            AddKeyValue(values, 0x0007, imuDestination.ToArray());
            AddKeyValue(values, 0x001A, new byte[] { 1 });
            AddKeyValue(values, 0x001C, new byte[] { 1 });
            SendParameters(device, hostIp, values, 5, timeoutMilliseconds, "点云配置");
            return device.HostPointPort;
        }

        public static void SetWorkMode(LivoxDevice device, string hostIp, bool sampling, int timeoutMilliseconds)
        {
            List<byte> values = new List<byte>();
            // HAP/Industrial HAP firmware enters real standby with mode 0x02
            // (current state transitions 0x07 -> 0x02). MID-360 uses Sleep 0x03.
            byte mode = sampling ? (byte)0x01 : (device.DeviceType == 10 || device.DeviceType == 15 ? (byte)0x02 : (byte)0x03);
            AddKeyValue(values, 0x001A, new byte[] { mode });
            SendParameters(device, hostIp, values, 1, timeoutMilliseconds, sampling ? "恢复正常工作" : "进入待机状态");
        }

        public static void SetPointSend(LivoxDevice device, string hostIp, bool enabled, int timeoutMilliseconds)
        {
            List<byte> values = new List<byte>();
            // Official Livox SDK2 uses 0x00 to enable and 0x01 to disable point sending.
            AddKeyValue(values, 0x0003, new byte[] { enabled ? (byte)0x00 : (byte)0x01 });
            SendParameters(device, hostIp, values, 1, timeoutMilliseconds, enabled ? "启用点云发送" : "禁用点云发送");
        }

        public static LivoxWorkState QueryWorkState(LivoxDevice device, string hostIp, int timeoutMilliseconds)
        {
            List<byte> body = new List<byte>();
            PutUInt16(body, 3); PutUInt16(body, 0);
            PutUInt16(body, 0x001A); PutUInt16(body, 0x8006); PutUInt16(body, 0x0003);
            byte[] request = MakeCommand(0x0101, body.ToArray(), unchecked((uint)Environment.TickCount));
            using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
            {
                socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                try { socket.Bind(new IPEndPoint(IPAddress.Parse(hostIp), device.HostCommandPort)); }
                catch (SocketException) { socket.Bind(new IPEndPoint(IPAddress.Parse(hostIp), 0)); }
                socket.ReceiveTimeout = Math.Max(250, timeoutMilliseconds);
                socket.SendTo(request, new IPEndPoint(IPAddress.Parse(device.Ip), device.CommandPort));
                DateTime deadline = DateTime.UtcNow.AddMilliseconds(Math.Max(250, timeoutMilliseconds));
                while (DateTime.UtcNow < deadline)
                {
                    byte[] packet = new byte[2048];
                    EndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                    int length;
                    try { length = socket.ReceiveFrom(packet, ref sender); }
                    catch (SocketException ex)
                    {
                        if (ex.SocketErrorCode == SocketError.TimedOut || ex.SocketErrorCode == SocketError.WouldBlock) break;
                        throw;
                    }
                    ushort commandId;
                    byte[] response;
                    if (!TryParseCommand(packet, length, out commandId, out response) || commandId != 0x0101) continue;
                    if (response.Length < 3 || response[0] != 0) throw new InvalidOperationException("雷达拒绝查询当前工作状态。");
                    Dictionary<ushort, byte[]> values = ReadKeyValues(response, 3, ReadUInt16(response, 1));
                    byte[] target, current, pointSend;
                    if (!values.TryGetValue(0x001A, out target) || target.Length == 0 ||
                        !values.TryGetValue(0x8006, out current) || current.Length == 0)
                        throw new InvalidOperationException("雷达没有返回完整的工作状态。");
                    values.TryGetValue(0x0003, out pointSend);
                    return new LivoxWorkState
                    {
                        TargetMode = target[0],
                        CurrentState = current[0],
                        PointSendState = pointSend != null && pointSend.Length > 0 ? pointSend[0] : (byte)0xFF
                    };
                }
            }
            throw new TimeoutException("查询雷达当前工作状态超时。");
        }

        private static void SendParameters(LivoxDevice device, string hostIp, List<byte> values, int valueCount, int timeoutMilliseconds, string operation)
        {
            List<byte> body = new List<byte>();
            PutUInt16(body, (ushort)valueCount); PutUInt16(body, 0); body.AddRange(values);
            byte[] request = MakeCommand(0x0100, body.ToArray(), unchecked((uint)Environment.TickCount));

            using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
            {
                socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                try { socket.Bind(new IPEndPoint(IPAddress.Parse(hostIp), device.HostCommandPort)); }
                catch (SocketException) { socket.Bind(new IPEndPoint(IPAddress.Parse(hostIp), 0)); }
                socket.ReceiveTimeout = Math.Max(250, timeoutMilliseconds);
                socket.SendTo(request, new IPEndPoint(IPAddress.Parse(device.Ip), device.CommandPort));
                DateTime deadline = DateTime.UtcNow.AddMilliseconds(Math.Max(250, timeoutMilliseconds));
                while (DateTime.UtcNow < deadline)
                {
                    byte[] packet = new byte[2048];
                    int length;
                    EndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                    try { length = socket.ReceiveFrom(packet, ref sender); }
                    catch (SocketException ex)
                    {
                        if (ex.SocketErrorCode == SocketError.TimedOut || ex.SocketErrorCode == SocketError.WouldBlock) break;
                        throw;
                    }
                    ushort commandId;
                    byte[] response;
                    if (!TryParseCommand(packet, length, out commandId, out response) || commandId != 0x0100) continue;
                    if (response.Length == 0 || response[0] != 0)
                    {
                        ushort errorKey = response.Length >= 3 ? ReadUInt16(response, 1) : (ushort)0;
                        throw new InvalidOperationException("雷达拒绝" + operation + "（错误键 0x" + errorKey.ToString("X4", CultureInfo.InvariantCulture) + "）。");
                    }
                    return;
                }
            }
            throw new TimeoutException("等待雷达确认“" + operation + "”超时。请检查网卡 IP、防火墙，并关闭占用 Livox UDP 端口的软件。");
        }

        public static PointCloudFrame DecodePointPacket(byte[] packet, int length, uint lidarId, out byte frameCounter)
        {
            frameCounter = 0;
            if (packet == null || length < 36) return null;
            int declaredLength = ReadUInt16(packet, 1);
            int dotCount = ReadUInt16(packet, 5);
            frameCounter = packet[9];
            int dataType = packet[10];
            int usable = declaredLength >= 36 ? Math.Min(length, declaredLength) : length;
            int itemSize = dataType == 1 ? 14 : dataType == 2 ? 8 : dataType == 3 ? 10 : 0;
            if (itemSize == 0) return null;
            int count = Math.Min(dotCount, Math.Max(0, usable - 36) / itemSize);
            List<float> vertices = new List<float>(count * 3);
            List<byte> reflectivity = new List<byte>(count);
            List<byte> tags = new List<byte>(count);
            List<uint> lidarIds = new List<uint>(count);
            for (int i = 0; i < count; i++)
            {
                int offset = 36 + i * itemSize;
                float x, y, z;
                byte intensity, tag;
                if (dataType == 1)
                {
                    x = BitConverter.ToInt32(packet, offset) * 0.001f;
                    y = BitConverter.ToInt32(packet, offset + 4) * 0.001f;
                    z = BitConverter.ToInt32(packet, offset + 8) * 0.001f;
                    intensity = packet[offset + 12]; tag = packet[offset + 13];
                }
                else if (dataType == 2)
                {
                    x = BitConverter.ToInt16(packet, offset) * 0.01f;
                    y = BitConverter.ToInt16(packet, offset + 2) * 0.01f;
                    z = BitConverter.ToInt16(packet, offset + 4) * 0.01f;
                    intensity = packet[offset + 6]; tag = packet[offset + 7];
                }
                else
                {
                    float radius = BitConverter.ToUInt32(packet, offset) * 0.001f;
                    double theta = ReadUInt16(packet, offset + 4) * 0.01 * Math.PI / 180.0;
                    double phi = ReadUInt16(packet, offset + 6) * 0.01 * Math.PI / 180.0;
                    double sinTheta = Math.Sin(theta);
                    x = (float)(radius * sinTheta * Math.Cos(phi));
                    y = (float)(radius * sinTheta * Math.Sin(phi));
                    z = (float)(radius * Math.Cos(theta));
                    intensity = packet[offset + 8]; tag = packet[offset + 9];
                }
                if ((x == 0f && y == 0f && z == 0f) || float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z)) continue;
                vertices.Add(x); vertices.Add(y); vertices.Add(z);
                reflectivity.Add(intensity); tags.Add(tag); lidarIds.Add(lidarId);
            }
            if (reflectivity.Count == 0) return null;
            return new PointCloudFrame(vertices.ToArray(), reflectivity.ToArray(), tags.ToArray(), lidarIds.ToArray());
        }

        public static uint LidarIdFromIp(string ip)
        {
            byte[] bytes = IPAddress.Parse(ip).GetAddressBytes();
            return ((uint)bytes[0] << 24) | ((uint)bytes[1] << 16) | ((uint)bytes[2] << 8) | bytes[3];
        }

        private static void SendInfo(Socket socket, byte[] request, string ip, int port)
        {
            try { socket.SendTo(request, new IPEndPoint(IPAddress.Parse(ip), port)); }
            catch (SocketException) { }
        }

        private static string GetClassCBroadcast(IPAddress address)
        {
            byte[] bytes = address.GetAddressBytes();
            if (bytes.Length != 4) return "255.255.255.255";
            bytes[3] = 255;
            return new IPAddress(bytes).ToString();
        }

        private static byte[] MakeCommand(ushort commandId, byte[] payload, uint sequence)
        {
            payload = payload ?? new byte[0];
            byte[] packet = new byte[HeaderSize + payload.Length];
            packet[0] = 0xAA;
            packet[1] = 0;
            WriteUInt16(packet, 2, (ushort)packet.Length);
            WriteUInt32(packet, 4, sequence);
            WriteUInt16(packet, 8, commandId);
            ushort headerCrc = Crc16(packet, 0, 18);
            WriteUInt16(packet, 18, headerCrc);
            uint payloadCrc = payload.Length == 0 ? Crc32(new byte[] { 0 }, 0, 1) : Crc32(payload, 0, payload.Length);
            WriteUInt32(packet, 20, payloadCrc);
            if (payload.Length > 0) Buffer.BlockCopy(payload, 0, packet, HeaderSize, payload.Length);
            return packet;
        }

        private static bool TryParseCommand(byte[] packet, int received, out ushort commandId, out byte[] payload)
        {
            commandId = 0; payload = new byte[0];
            if (received < HeaderSize || packet[0] != 0xAA) return false;
            int length = ReadUInt16(packet, 2);
            if (length < HeaderSize || length > received) return false;
            if (Crc16(packet, 0, 18) != ReadUInt16(packet, 18)) return false;
            int payloadLength = length - HeaderSize;
            byte[] crcPayload = payloadLength == 0 ? new byte[] { 0 } : new byte[payloadLength];
            if (payloadLength > 0) Buffer.BlockCopy(packet, HeaderSize, crcPayload, 0, payloadLength);
            if (Crc32(crcPayload, 0, crcPayload.Length) != ReadUInt32(packet, 20)) return false;
            commandId = ReadUInt16(packet, 8);
            payload = payloadLength == 0 ? new byte[0] : crcPayload;
            return true;
        }

        private static ushort Crc16(byte[] data, int offset, int count)
        {
            ushort crc = 0xFFFF;
            for (int i = 0; i < count; i++)
            {
                crc ^= (ushort)(data[offset + i] << 8);
                for (int bit = 0; bit < 8; bit++) crc = (ushort)((crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1);
            }
            return crc;
        }

        private static uint Crc32(byte[] data, int offset, int count)
        {
            uint crc = 0xFFFFFFFFU;
            for (int i = 0; i < count; i++)
            {
                crc ^= data[offset + i];
                for (int bit = 0; bit < 8; bit++) crc = (crc & 1U) != 0 ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
            }
            return crc ^ 0xFFFFFFFFU;
        }

        private static Dictionary<ushort, byte[]> ReadKeyValues(byte[] payload, int offset, int count)
        {
            Dictionary<ushort, byte[]> values = new Dictionary<ushort, byte[]>();
            for (int i = 0; i < count && offset + 4 <= payload.Length; i++)
            {
                ushort key = ReadUInt16(payload, offset);
                int length = ReadUInt16(payload, offset + 2);
                offset += 4;
                if (offset + length > payload.Length) break;
                byte[] value = new byte[length];
                Buffer.BlockCopy(payload, offset, value, 0, length);
                values[key] = value;
                offset += length;
            }
            return values;
        }

        private static void AddKeyValue(List<byte> output, ushort key, byte[] value)
        {
            PutUInt16(output, key); PutUInt16(output, (ushort)value.Length); output.AddRange(value);
        }

        private static string ReadAscii(byte[] data, int offset, int length)
        {
            int end = offset;
            int limit = Math.Min(data.Length, offset + length);
            while (end < limit && data[end] != 0) end++;
            return Encoding.ASCII.GetString(data, offset, end - offset).Trim();
        }

        private static ushort ReadUInt16(byte[] data, int offset) { return (ushort)(data[offset] | (data[offset + 1] << 8)); }
        private static uint ReadUInt32(byte[] data, int offset) { return (uint)(data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24)); }
        private static void WriteUInt16(byte[] data, int offset, ushort value) { data[offset] = (byte)value; data[offset + 1] = (byte)(value >> 8); }
        private static void WriteUInt32(byte[] data, int offset, uint value) { data[offset] = (byte)value; data[offset + 1] = (byte)(value >> 8); data[offset + 2] = (byte)(value >> 16); data[offset + 3] = (byte)(value >> 24); }
        private static void PutUInt16(List<byte> output, ushort value) { output.Add((byte)value); output.Add((byte)(value >> 8)); }
    }

    internal sealed class LivoxLiveReceiver : IDisposable
    {
        private readonly string hostIp;
        private readonly LivoxDevice device;
        private readonly object sync = new object();
        private readonly List<PointCloudFrame> packetFrames = new List<PointCloudFrame>();
        private Socket socket;
        private Thread thread;
        private volatile bool stopping;
        private PointCloudFrame latestFrame;
        private long latestSequence;
        private string lastError = "";
        private DateTime lastPacketAt = DateTime.MinValue;
        private DateTime lastPublishAt = DateTime.MinValue;

        private long packetCount;
        private long pointCount;
        public long PacketCount { get { return Interlocked.Read(ref packetCount); } }
        public long PointCount { get { return Interlocked.Read(ref pointCount); } }
        public string LastSender { get; private set; }
        public LivoxDevice Device { get { return device; } }
        public long LatestSequence { get { lock (sync) { return latestSequence; } } }
        public string LastError { get { lock (sync) { return lastError; } } }

        public LivoxLiveReceiver(string selectedHostIp, LivoxDevice selectedDevice)
        {
            hostIp = selectedHostIp;
            device = selectedDevice;
        }

        public void Start()
        {
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.ReceiveBufferSize = 4 * 1024 * 1024;
            socket.ReceiveTimeout = 120;
            socket.Bind(new IPEndPoint(IPAddress.Parse(hostIp), device.HostPointPort));
            thread = new Thread(ReceiveLoop);
            thread.IsBackground = true;
            thread.Name = "livox-live-points";
            thread.Start();
            try
            {
                LivoxUdpProtocol.ConfigureStream(device, hostIp, 2200);
                if (device.DeviceType == 10 || device.DeviceType == 15)
                    LivoxUdpProtocol.SetPointSend(device, hostIp, true, 2200);
            }
            catch { Dispose(); throw; }
        }

        public PointCloudFrame GetLatestFrame(out long sequence)
        {
            lock (sync) { sequence = latestSequence; return latestFrame; }
        }

        public void SetSampling(bool sampling)
        {
            if (sampling)
            {
                LivoxUdpProtocol.SetWorkMode(device, hostIp, true, 2200);
                if (device.DeviceType == 10 || device.DeviceType == 15)
                    LivoxUdpProtocol.SetPointSend(device, hostIp, true, 2200);
            }
            else
            {
                Exception pointSendError = null;
                Exception workModeError = null;
                if (device.DeviceType == 10 || device.DeviceType == 15)
                {
                    try { LivoxUdpProtocol.SetPointSend(device, hostIp, false, 2200); }
                    catch (Exception ex) { pointSendError = ex; }
                }
                try { LivoxUdpProtocol.SetWorkMode(device, hostIp, false, 2200); }
                catch (Exception ex) { workModeError = ex; }
                if (device.DeviceType == 9 && workModeError != null) throw workModeError;
                if (workModeError != null && pointSendError != null)
                    throw new InvalidOperationException("禁用点云发送和 Sleep 命令都失败：" + pointSendError.Message + "；" + workModeError.Message);
            }
        }

        public void ForceDisablePointSend()
        {
            LivoxUdpProtocol.SetPointSend(device, hostIp, false, 2200);
        }

        public bool WaitForPacketSilence(int quietMilliseconds, int timeoutMilliseconds)
        {
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(timeoutMilliseconds);
            DateTime quietSince = DateTime.UtcNow;
            long previous = PacketCount;
            while (DateTime.UtcNow < deadline)
            {
                Thread.Sleep(50);
                long current = PacketCount;
                if (current != previous)
                {
                    previous = current;
                    quietSince = DateTime.UtcNow;
                }
                else if ((DateTime.UtcNow - quietSince).TotalMilliseconds >= quietMilliseconds) return true;
            }
            return false;
        }

        public bool WaitForWorkMode(bool sampling, int timeoutMilliseconds, out LivoxWorkState finalState, Action<LivoxWorkState> progress)
        {
            byte expected = sampling ? (byte)0x01 : (device.DeviceType == 10 || device.DeviceType == 15 ? (byte)0x02 : (byte)0x03);
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(timeoutMilliseconds);
            finalState = null;
            while (DateTime.UtcNow < deadline)
            {
                try
                {
                    finalState = LivoxUdpProtocol.QueryWorkState(device, hostIp, 1200);
                    if (progress != null) progress(finalState);
                    if (finalState.TargetMode == expected && finalState.CurrentState == expected) return true;
                }
                catch (Exception) { /* A HAP may temporarily reject queries while its motor changes state. */ }
                Thread.Sleep(250);
            }
            return false;
        }

        public static string DescribeWorkState(byte state)
        {
            if (state == 1) return "正常工作";
            if (state == 2) return "待机";
            if (state == 3) return "休眠";
            if (state == 5) return "上电自检";
            if (state == 6) return "电机启动中";
            if (state == 7) return "电机停止中";
            if (state == 8) return "升级模式";
            return "状态 " + state.ToString(CultureInfo.InvariantCulture);
        }

        private void ReceiveLoop()
        {
            uint lidarId = LivoxUdpProtocol.LidarIdFromIp(device.Ip);
            byte[] buffer = new byte[65535];
            while (!stopping)
            {
                EndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                int length;
                try { length = socket.ReceiveFrom(buffer, ref sender); }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.TimedOut || ex.SocketErrorCode == SocketError.WouldBlock)
                    {
                        if (packetFrames.Count > 0 && (DateTime.UtcNow - lastPacketAt).TotalMilliseconds >= 80) PublishFrame();
                        continue;
                    }
                    if (!stopping) lock (sync) { lastError = ex.Message; }
                    break;
                }
                catch (ObjectDisposedException) { break; }
                byte packetCounter;
                PointCloudFrame decoded = LivoxUdpProtocol.DecodePointPacket(buffer, length, lidarId, out packetCounter);
                if (decoded == null) continue;
                Interlocked.Increment(ref packetCount);
                Interlocked.Add(ref pointCount, decoded.Count);
                LastSender = ((IPEndPoint)sender).Address.ToString();
                lastPacketAt = DateTime.UtcNow;
                packetFrames.Add(decoded);
                // Some SDK2 firmware keeps frame_counter unchanged while streaming.
                // Publish by time window, matching python_livox_viewer's 100 ms point hub,
                // so a healthy continuous stream can never remain invisible.
                if (lastPublishAt == DateTime.MinValue) lastPublishAt = lastPacketAt;
                if ((lastPacketAt - lastPublishAt).TotalMilliseconds >= 100) PublishFrame();
            }
            if (packetFrames.Count > 0) PublishFrame();
        }

        private void PublishFrame()
        {
            if (packetFrames.Count == 0) return;
            PointCloudFrame frame = packetFrames.Count == 1 ? packetFrames[0] : PointCloudFrame.Merge(packetFrames);
            packetFrames.Clear();
            lock (sync)
            {
                latestFrame = frame;
                latestSequence++;
            }
            lastPublishAt = DateTime.UtcNow;
        }

        public void Dispose()
        {
            stopping = true;
            if (socket != null)
            {
                try { socket.Close(); } catch { }
                socket = null;
            }
            if (thread != null && thread != Thread.CurrentThread)
            {
                try { thread.Join(1000); } catch { }
                thread = null;
            }
        }
    }

    internal sealed class LidarConnectionDialog : Form
    {
        private readonly ComboBox adapters = new ComboBox();
        private readonly Button scan = new Button();
        private readonly ListBox devices = new ListBox();
        private readonly Button connect = new Button();
        private readonly Label message = new Label();
        private bool scanning;

        public string SelectedHostIp { get; private set; }
        public LivoxDevice SelectedDevice { get; private set; }

        public LidarConnectionDialog()
        {
            Text = "连接 Livox 雷达";
            StartPosition = FormStartPosition.CenterParent;
            ClientSize = new System.Drawing.Size(620, 390);
            MinimumSize = new System.Drawing.Size(560, 350);
            BackColor = System.Drawing.Color.FromArgb(20, 22, 25);
            ForeColor = System.Drawing.Color.White;
            FormBorderStyle = FormBorderStyle.Sizable;

            Label adapterLabel = MakeLabel("本机有线网卡 / IPv4", 18, 18, 170, 24);
            adapters.DropDownStyle = ComboBoxStyle.DropDownList;
            adapters.SetBounds(18, 47, 470, 30);
            adapters.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            adapters.BackColor = System.Drawing.Color.FromArgb(16, 17, 19);
            adapters.ForeColor = System.Drawing.Color.White;

            scan.Text = "扫描雷达";
            scan.SetBounds(500, 46, 100, 31);
            scan.Anchor = AnchorStyles.Top | AnchorStyles.Right;
            StyleButton(scan);
            scan.Click += delegate { BeginScan(); };

            Label deviceLabel = MakeLabel("发现的设备", 18, 92, 120, 24);
            devices.SetBounds(18, 120, 582, 180);
            devices.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            devices.BackColor = System.Drawing.Color.FromArgb(7, 10, 14);
            devices.ForeColor = System.Drawing.Color.White;
            devices.Font = new System.Drawing.Font("Segoe UI", 10f);
            devices.DoubleClick += delegate { Confirm(); };
            devices.SelectedIndexChanged += delegate { connect.Enabled = devices.SelectedItem is LivoxDevice && !scanning; };

            message.SetBounds(18, 309, 430, 50);
            message.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            message.ForeColor = System.Drawing.Color.Gainsboro;
            message.Text = "请选择连接雷达的有线网卡，然后扫描。网卡 IPv4 需与雷达在同一网段。";

            connect.Text = "连接并播放";
            connect.SetBounds(470, 326, 130, 34);
            connect.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            connect.Enabled = false;
            StyleButton(connect);
            connect.Click += delegate { Confirm(); };

            Controls.AddRange(new Control[] { adapterLabel, adapters, scan, deviceLabel, devices, message, connect });
            Shown += delegate { LoadAdapters(); };
        }

        private void LoadAdapters()
        {
            adapters.Items.Clear();
            List<NetworkAdapterRow> rows = LivoxUdpProtocol.ListAdapters();
            for (int i = 0; i < rows.Count; i++) adapters.Items.Add(rows[i]);
            if (adapters.Items.Count > 0) adapters.SelectedIndex = 0;
            else
            {
                message.Text = "没有找到可用 IPv4 网卡。请连接网线并给网卡设置静态 IPv4。";
                scan.Enabled = false;
            }
        }

        private void BeginScan()
        {
            NetworkAdapterRow adapter = adapters.SelectedItem as NetworkAdapterRow;
            if (adapter == null || scanning) return;
            scanning = true;
            scan.Enabled = false;
            connect.Enabled = false;
            devices.Items.Clear();
            message.Text = "正在通过 " + adapter.Ip + " 扫描 Livox MID-360 / HAP…";
            ThreadPool.QueueUserWorkItem(delegate
            {
                List<LivoxDevice> found = null;
                Exception error = null;
                try { found = LivoxUdpProtocol.Discover(adapter.Ip, 1700); }
                catch (Exception ex) { error = ex; }
                if (!IsDisposed && IsHandleCreated)
                {
                    try
                    {
                        BeginInvoke((MethodInvoker)delegate
                        {
                            scanning = false;
                            scan.Enabled = true;
                            if (error != null) message.Text = "扫描失败：" + error.Message;
                            else if (found == null || found.Count == 0) message.Text = "没有发现雷达。请检查网段、防火墙，并关闭 Livox Viewer 2 或 ROS。";
                            else
                            {
                                for (int i = 0; i < found.Count; i++) devices.Items.Add(found[i]);
                                devices.SelectedIndex = 0;
                                message.Text = "发现 " + found.Count.ToString(CultureInfo.InvariantCulture) + " 台设备。选择后点击“连接并播放”。";
                            }
                        });
                    }
                    catch { }
                }
            });
        }

        private void Confirm()
        {
            NetworkAdapterRow adapter = adapters.SelectedItem as NetworkAdapterRow;
            LivoxDevice device = devices.SelectedItem as LivoxDevice;
            if (adapter == null || device == null || scanning) return;
            SelectedHostIp = adapter.Ip;
            SelectedDevice = device;
            DialogResult = DialogResult.OK;
            Close();
        }

        private static Label MakeLabel(string text, int x, int y, int width, int height)
        {
            Label label = new Label(); label.Text = text; label.SetBounds(x, y, width, height); return label;
        }

        private static void StyleButton(Button button)
        {
            button.FlatStyle = FlatStyle.Flat;
            button.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(80, 85, 92);
            button.BackColor = System.Drawing.Color.FromArgb(48, 52, 58);
            button.ForeColor = System.Drawing.Color.White;
        }
    }
}
