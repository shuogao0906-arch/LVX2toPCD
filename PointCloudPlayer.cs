using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;

namespace LivoxPointCloudPlayer
{
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            System.Diagnostics.Process currentProcess = System.Diagnostics.Process.GetCurrentProcess();
            System.Diagnostics.Process[] sameNameProcesses = System.Diagnostics.Process.GetProcessesByName(currentProcess.ProcessName);
            for (int i = 0; i < sameNameProcesses.Length; i++)
            {
                if (sameNameProcesses[i].Id != currentProcess.Id)
                {
                    MessageBox.Show("检测到旧版播放器仍在运行。请先关闭所有 Livox 点云播放器，再启动此版本，避免多个程序向雷达发送冲突命令。", "Livox 点云播放器", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
            }
            bool created;
            using (System.Threading.Mutex instanceMutex = new System.Threading.Mutex(true, "Local\\LivoxPointCloudPlayer.SingleInstance", out created))
            {
                if (!created)
                {
                    MessageBox.Show("播放器已经在运行。请关闭旧窗口后再启动，避免多个程序同时控制雷达。", "Livox 点云播放器", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
                Application.Run(new MainForm());
                GC.KeepAlive(instanceMutex);
            }
        }
    }

    internal sealed class PointCloudFrame
    {
        public float[] Vertices;
        public byte[] Intensities;
        public byte[] Tags;
        public uint[] LidarIds;

        public int Count { get { return Intensities == null ? 0 : Intensities.Length; } }

        public PointCloudFrame(float[] vertices, byte[] intensities, byte[] tags, uint[] lidarIds)
        {
            Vertices = vertices;
            Intensities = intensities;
            Tags = tags;
            LidarIds = lidarIds;
        }

        public static PointCloudFrame Merge(IList<PointCloudFrame> frames)
        {
            int count = 0;
            for (int i = 0; i < frames.Count; i++) count += frames[i].Count;
            float[] vertices = new float[count * 3];
            byte[] intensities = new byte[count];
            byte[] tags = new byte[count];
            uint[] lidarIds = new uint[count];
            int pointOffset = 0;
            for (int i = 0; i < frames.Count; i++)
            {
                PointCloudFrame frame = frames[i];
                Array.Copy(frame.Vertices, 0, vertices, pointOffset * 3, frame.Vertices.Length);
                Array.Copy(frame.Intensities, 0, intensities, pointOffset, frame.Count);
                Array.Copy(frame.Tags, 0, tags, pointOffset, frame.Count);
                Array.Copy(frame.LidarIds, 0, lidarIds, pointOffset, frame.Count);
                pointOffset += frame.Count;
            }
            return new PointCloudFrame(vertices, intensities, tags, lidarIds);
        }
    }

    internal abstract class FrameSource : IDisposable
    {
        public abstract int Count { get; }
        public abstract string DisplayName { get; }
        public abstract string BaseName { get; }
        public abstract long NativeIndex(int index);
        public abstract PointCloudFrame LoadFrame(int index);
        public virtual int SuggestedInterval { get { return 50; } }
        public virtual void Dispose() { }
    }

    internal sealed class Lvx2Source : FrameSource
    {
        private sealed class FrameEntry
        {
            public long Current;
            public long Next;
            public long Index;
        }

        private readonly string path;
        private readonly FileStream stream;
        private readonly List<FrameEntry> frames = new List<FrameEntry>();
        private readonly int interval;

        public Lvx2Source(string filePath)
        {
            path = filePath;
            stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1024 * 1024, FileOptions.RandomAccess);
            byte[] header = ReadExactly(stream, 29);
            string signature = Encoding.ASCII.GetString(header, 0, 16).TrimEnd('\0');
            if (!signature.StartsWith("livox_tech", StringComparison.OrdinalIgnoreCase) || BitConverter.ToUInt32(header, 20) != 0xAC0EA767U)
                throw new InvalidDataException("不是有效的 LVX2 文件。");

            uint rawInterval = BitConverter.ToUInt32(header, 24);
            interval = rawInterval >= 25 && rawInterval <= 500 ? (int)rawInterval : 50;
            int deviceCount = header[28];
            long offset = 29L + deviceCount * 63L;
            byte[] frameHeader = new byte[24];
            while (offset + 24 <= stream.Length)
            {
                stream.Position = offset;
                if (stream.Read(frameHeader, 0, 24) != 24) break;
                long current = BitConverter.ToInt64(frameHeader, 0);
                long next = BitConverter.ToInt64(frameHeader, 8);
                long index = BitConverter.ToInt64(frameHeader, 16);
                if (current != offset || next <= current || next > stream.Length) break;
                frames.Add(new FrameEntry { Current = current, Next = next, Index = index });
                offset = next;
            }
            if (frames.Count == 0) throw new InvalidDataException("LVX2 文件中没有找到点云帧。");
        }

        public override int Count { get { return frames.Count; } }
        public override string DisplayName { get { return Path.GetFileName(path); } }
        public override string BaseName { get { return Path.GetFileNameWithoutExtension(path); } }
        public override int SuggestedInterval { get { return interval; } }
        public override long NativeIndex(int index) { return frames[index].Index; }

        public override PointCloudFrame LoadFrame(int index)
        {
            FrameEntry entry = frames[index];
            long length64 = entry.Next - entry.Current;
            if (length64 <= 24 || length64 > int.MaxValue) throw new InvalidDataException("LVX2 帧长度异常。");
            stream.Position = entry.Current;
            byte[] buffer = ReadExactly(stream, (int)length64);
            List<float> vertices = new List<float>(65536);
            List<byte> intensities = new List<byte>(22000);
            List<byte> tags = new List<byte>(22000);
            List<uint> lidarIds = new List<uint>(22000);

            int offset = 24;
            while (offset + 27 <= buffer.Length)
            {
                uint lidarId = BitConverter.ToUInt32(buffer, offset + 1);
                int dataType = buffer[offset + 17];
                uint lengthValue = BitConverter.ToUInt32(buffer, offset + 18);
                if (lengthValue > int.MaxValue) break;
                int payloadLength = (int)lengthValue;
                int pointSize = dataType == 1 ? 14 : dataType == 2 ? 8 : 0;
                int dataOffset = offset + 27;
                if (pointSize == 0 || payloadLength < 0 || dataOffset + payloadLength > buffer.Length || payloadLength % pointSize != 0) break;
                int pointCount = payloadLength / pointSize;
                float scale = dataType == 1 ? 0.001f : 0.01f;
                for (int i = 0; i < pointCount; i++)
                {
                    int p = dataOffset + i * pointSize;
                    float x;
                    float y;
                    float z;
                    byte intensity;
                    byte tag;
                    if (dataType == 1)
                    {
                        x = BitConverter.ToInt32(buffer, p) * scale;
                        y = BitConverter.ToInt32(buffer, p + 4) * scale;
                        z = BitConverter.ToInt32(buffer, p + 8) * scale;
                        intensity = buffer[p + 12];
                        tag = buffer[p + 13];
                    }
                    else
                    {
                        x = BitConverter.ToInt16(buffer, p) * scale;
                        y = BitConverter.ToInt16(buffer, p + 2) * scale;
                        z = BitConverter.ToInt16(buffer, p + 4) * scale;
                        intensity = buffer[p + 6];
                        tag = buffer[p + 7];
                    }
                    if (x == 0f && y == 0f && z == 0f) continue;
                    vertices.Add(x);
                    vertices.Add(y);
                    vertices.Add(z);
                    intensities.Add(intensity);
                    tags.Add(tag);
                    lidarIds.Add(lidarId);
                }
                offset = dataOffset + payloadLength;
            }
            return new PointCloudFrame(vertices.ToArray(), intensities.ToArray(), tags.ToArray(), lidarIds.ToArray());
        }

        public override void Dispose()
        {
            stream.Dispose();
        }

        private static byte[] ReadExactly(Stream input, int count)
        {
            byte[] buffer = new byte[count];
            int offset = 0;
            while (offset < count)
            {
                int read = input.Read(buffer, offset, count - offset);
                if (read <= 0) throw new EndOfStreamException();
                offset += read;
            }
            return buffer;
        }
    }

    internal sealed class PcdSequenceSource : FrameSource
    {
        private readonly string[] files;
        private readonly string displayName;
        private readonly string baseName;

        public PcdSequenceSource(string[] inputFiles, string name)
        {
            files = inputFiles.OrderBy(delegate(string value) { return value; }, StringComparer.OrdinalIgnoreCase).ToArray();
            if (files.Length == 0) throw new InvalidDataException("没有找到 PCD 文件。");
            displayName = name;
            baseName = files.Length == 1 ? Path.GetFileNameWithoutExtension(files[0]) : new DirectoryInfo(Path.GetDirectoryName(files[0])).Name;
        }

        public override int Count { get { return files.Length; } }
        public override string DisplayName { get { return displayName; } }
        public override string BaseName { get { return baseName; } }
        public override long NativeIndex(int index)
        {
            long parsed;
            return long.TryParse(Path.GetFileNameWithoutExtension(files[index]), out parsed) ? parsed : index;
        }
        public override PointCloudFrame LoadFrame(int index) { return PcdReader.Read(files[index]); }
    }

    internal static class PcdReader
    {
        private sealed class FieldDefinition
        {
            public string Name;
            public int Size;
            public char Type;
            public int Count;
            public int Offset;
        }

        public static PointCloudFrame Read(string path)
        {
            using (FileStream stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                Dictionary<string, string[]> header = new Dictionary<string, string[]>(StringComparer.OrdinalIgnoreCase);
                string dataType = null;
                while (true)
                {
                    string line = ReadAsciiLine(stream);
                    if (line == null) throw new InvalidDataException("PCD 文件头不完整。");
                    line = line.Trim();
                    if (line.Length == 0 || line.StartsWith("#")) continue;
                    string[] parts = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length == 0) continue;
                    string key = parts[0].ToUpperInvariant();
                    string[] values = new string[parts.Length - 1];
                    Array.Copy(parts, 1, values, 0, values.Length);
                    header[key] = values;
                    if (key == "DATA")
                    {
                        dataType = values.Length > 0 ? values[0].ToLowerInvariant() : "";
                        break;
                    }
                }

                List<FieldDefinition> fields = BuildFields(header);
                int points = GetInt(header, "POINTS", GetInt(header, "WIDTH", 0) * Math.Max(1, GetInt(header, "HEIGHT", 1)));
                if (dataType == "binary") return ReadBinary(stream, fields, points);
                if (dataType == "ascii") return ReadAscii(stream, fields, points);
                throw new NotSupportedException("暂不支持 DATA " + dataType + " 的 PCD 文件。");
            }
        }

        private static List<FieldDefinition> BuildFields(Dictionary<string, string[]> header)
        {
            string[] names;
            if (!header.TryGetValue("FIELDS", out names) && !header.TryGetValue("FIELD", out names))
                throw new InvalidDataException("PCD 文件缺少 FIELDS。");
            string[] sizes = GetValues(header, "SIZE", names.Length, "4");
            string[] types = GetValues(header, "TYPE", names.Length, "F");
            string[] counts = GetValues(header, "COUNT", names.Length, "1");
            List<FieldDefinition> result = new List<FieldDefinition>();
            int offset = 0;
            for (int i = 0; i < names.Length; i++)
            {
                int size = int.Parse(sizes[i], CultureInfo.InvariantCulture);
                int count = int.Parse(counts[i], CultureInfo.InvariantCulture);
                result.Add(new FieldDefinition
                {
                    Name = names[i].ToLowerInvariant(),
                    Size = size,
                    Type = char.ToUpperInvariant(types[i][0]),
                    Count = count,
                    Offset = offset
                });
                offset += size * count;
            }
            return result;
        }

        private static PointCloudFrame ReadBinary(FileStream stream, List<FieldDefinition> fields, int points)
        {
            int stride = fields.Sum(delegate(FieldDefinition f) { return f.Size * f.Count; });
            if (stride <= 0 || points < 0) throw new InvalidDataException("PCD 点记录长度异常。");
            long remaining = stream.Length - stream.Position;
            int available = (int)Math.Min(points, remaining / stride);
            byte[] payload = new byte[available * stride];
            int totalRead = 0;
            while (totalRead < payload.Length)
            {
                int read = stream.Read(payload, totalRead, payload.Length - totalRead);
                if (read <= 0) break;
                totalRead += read;
            }
            int actual = totalRead / stride;
            List<float> vertices = new List<float>(actual * 3);
            List<byte> intensities = new List<byte>(actual);
            List<byte> tags = new List<byte>(actual);
            List<uint> lidarIds = new List<uint>(actual);
            FieldDefinition fx = Find(fields, "x");
            FieldDefinition fy = Find(fields, "y");
            FieldDefinition fz = Find(fields, "z");
            FieldDefinition fi = FindAny(fields, new string[] { "intensity", "reflectivity" });
            FieldDefinition ft = Find(fields, "tag");
            FieldDefinition fl = FindAny(fields, new string[] { "lidar_id", "lidarid", "lidar" });
            if (fx == null || fy == null || fz == null) throw new InvalidDataException("PCD 文件缺少 x/y/z 字段。");

            for (int i = 0; i < actual; i++)
            {
                int p = i * stride;
                float x = (float)ReadNumber(payload, p + fx.Offset, fx);
                float y = (float)ReadNumber(payload, p + fy.Offset, fy);
                float z = (float)ReadNumber(payload, p + fz.Offset, fz);
                if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z)) continue;
                if (x == 0f && y == 0f && z == 0f) continue;
                vertices.Add(x); vertices.Add(y); vertices.Add(z);
                intensities.Add(ToByte(fi == null ? 0 : ReadNumber(payload, p + fi.Offset, fi)));
                tags.Add(ToByte(ft == null ? 0 : ReadNumber(payload, p + ft.Offset, ft)));
                lidarIds.Add(fl == null ? 0U : ToUInt32(ReadNumber(payload, p + fl.Offset, fl)));
            }
            return new PointCloudFrame(vertices.ToArray(), intensities.ToArray(), tags.ToArray(), lidarIds.ToArray());
        }

        private static PointCloudFrame ReadAscii(FileStream stream, List<FieldDefinition> fields, int points)
        {
            List<float> vertices = new List<float>(Math.Max(points, 0) * 3);
            List<byte> intensities = new List<byte>(Math.Max(points, 0));
            List<byte> tags = new List<byte>(Math.Max(points, 0));
            List<uint> lidarIds = new List<uint>(Math.Max(points, 0));
            int ix = FieldIndex(fields, "x");
            int iy = FieldIndex(fields, "y");
            int iz = FieldIndex(fields, "z");
            int ii = FieldIndexAny(fields, new string[] { "intensity", "reflectivity" });
            int it = FieldIndex(fields, "tag");
            int il = FieldIndexAny(fields, new string[] { "lidar_id", "lidarid", "lidar" });
            if (ix < 0 || iy < 0 || iz < 0) throw new InvalidDataException("PCD 文件缺少 x/y/z 字段。");
            using (StreamReader reader = new StreamReader(stream, Encoding.ASCII, false, 65536, true))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    string[] values = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (values.Length < fields.Count) continue;
                    float x = ParseFloat(values[ix]);
                    float y = ParseFloat(values[iy]);
                    float z = ParseFloat(values[iz]);
                    if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z)) continue;
                    if (x == 0f && y == 0f && z == 0f) continue;
                    vertices.Add(x); vertices.Add(y); vertices.Add(z);
                    intensities.Add(ii < 0 ? (byte)0 : ToByte(ParseDouble(values[ii])));
                    tags.Add(it < 0 ? (byte)0 : ToByte(ParseDouble(values[it])));
                    lidarIds.Add(il < 0 ? 0U : ToUInt32(ParseDouble(values[il])));
                }
            }
            return new PointCloudFrame(vertices.ToArray(), intensities.ToArray(), tags.ToArray(), lidarIds.ToArray());
        }

        private static double ReadNumber(byte[] data, int offset, FieldDefinition field)
        {
            if (field.Type == 'F')
            {
                if (field.Size == 4) return BitConverter.ToSingle(data, offset);
                if (field.Size == 8) return BitConverter.ToDouble(data, offset);
            }
            if (field.Type == 'I')
            {
                if (field.Size == 1) return (sbyte)data[offset];
                if (field.Size == 2) return BitConverter.ToInt16(data, offset);
                if (field.Size == 4) return BitConverter.ToInt32(data, offset);
                if (field.Size == 8) return BitConverter.ToInt64(data, offset);
            }
            if (field.Type == 'U')
            {
                if (field.Size == 1) return data[offset];
                if (field.Size == 2) return BitConverter.ToUInt16(data, offset);
                if (field.Size == 4) return BitConverter.ToUInt32(data, offset);
                if (field.Size == 8) return BitConverter.ToUInt64(data, offset);
            }
            return 0;
        }

        private static string ReadAsciiLine(Stream stream)
        {
            List<byte> bytes = new List<byte>(128);
            while (true)
            {
                int value = stream.ReadByte();
                if (value < 0) return bytes.Count == 0 ? null : Encoding.ASCII.GetString(bytes.ToArray());
                if (value == '\n') break;
                if (value != '\r') bytes.Add((byte)value);
            }
            return Encoding.ASCII.GetString(bytes.ToArray());
        }

        private static string[] GetValues(Dictionary<string, string[]> header, string key, int count, string fallback)
        {
            string[] values;
            if (!header.TryGetValue(key, out values)) return Enumerable.Repeat(fallback, count).ToArray();
            if (values.Length != count) throw new InvalidDataException("PCD " + key + " 字段数量不匹配。");
            return values;
        }

        private static int GetInt(Dictionary<string, string[]> header, string key, int fallback)
        {
            string[] values;
            int result;
            return header.TryGetValue(key, out values) && values.Length > 0 && int.TryParse(values[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out result) ? result : fallback;
        }

        private static FieldDefinition Find(List<FieldDefinition> fields, string name)
        {
            return fields.FirstOrDefault(delegate(FieldDefinition field) { return field.Name == name; });
        }

        private static FieldDefinition FindAny(List<FieldDefinition> fields, string[] names)
        {
            for (int i = 0; i < names.Length; i++)
            {
                FieldDefinition value = Find(fields, names[i]);
                if (value != null) return value;
            }
            return null;
        }

        private static int FieldIndex(List<FieldDefinition> fields, string name)
        {
            for (int i = 0; i < fields.Count; i++) if (fields[i].Name == name) return i;
            return -1;
        }

        private static int FieldIndexAny(List<FieldDefinition> fields, string[] names)
        {
            for (int i = 0; i < names.Length; i++)
            {
                int index = FieldIndex(fields, names[i]);
                if (index >= 0) return index;
            }
            return -1;
        }

        private static float ParseFloat(string value) { return float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture); }
        private static double ParseDouble(string value) { return double.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture); }
        private static byte ToByte(double value) { return (byte)Math.Max(0, Math.Min(255, Math.Round(value))); }
        private static uint ToUInt32(double value) { return (uint)Math.Max(0, Math.Min(uint.MaxValue, Math.Round(value))); }
    }

    internal static class CloudColors
    {
        private static readonly Color[] ReflectivityStops = new Color[]
        {
            Color.FromArgb(59, 15, 112), Color.FromArgb(0, 51, 204), Color.FromArgb(0, 200, 255),
            Color.FromArgb(0, 200, 83), Color.FromArgb(255, 255, 0), Color.FromArgb(255, 0, 0)
        };

        public static byte[] Create(PointCloudFrame frame, string mode)
        {
            byte[] colors = new byte[frame.Count * 3];
            for (int i = 0; i < frame.Count; i++)
            {
                Color color;
                if (mode == "Reflectivity")
                {
                    color = MultiStop(frame.Intensities[i] / 255f, ReflectivityStops);
                }
                else if (mode == "Distance")
                {
                    int p = i * 3;
                    float x = frame.Vertices[p];
                    float y = frame.Vertices[p + 1];
                    float z = frame.Vertices[p + 2];
                    float distance = (float)Math.Sqrt(x * x + y * y + z * z);
                    color = MultiStop(Clamp01(distance / 4f), new Color[] { Color.LawnGreen, Color.Yellow, Color.Red });
                }
                else if (mode == "Elevation")
                {
                    color = Lerp(Color.White, Color.Magenta, Clamp01(frame.Vertices[i * 3 + 2]));
                }
                else if (mode == "LiDAR ID")
                {
                    color = IdColor(frame.LidarIds[i]);
                }
                else
                {
                    color = Color.FromArgb(235, 235, 235);
                }
                int c = i * 3;
                colors[c] = color.R;
                colors[c + 1] = color.G;
                colors[c + 2] = color.B;
            }
            return colors;
        }

        public static Color[] LegendStops(string mode)
        {
            if (mode == "Reflectivity") return ReflectivityStops;
            if (mode == "Distance") return new Color[] { Color.LawnGreen, Color.Yellow, Color.Red };
            if (mode == "Elevation") return new Color[] { Color.White, Color.Magenta };
            if (mode == "LiDAR ID") return new Color[] { Color.DeepSkyBlue, Color.LimeGreen, Color.Gold, Color.OrangeRed, Color.Magenta };
            return new Color[] { Color.FromArgb(235, 235, 235), Color.FromArgb(235, 235, 235) };
        }

        private static Color MultiStop(float t, Color[] stops)
        {
            t = Clamp01(t);
            float scaled = t * (stops.Length - 1);
            int index = Math.Min(stops.Length - 2, (int)Math.Floor(scaled));
            return Lerp(stops[index], stops[index + 1], scaled - index);
        }

        private static Color Lerp(Color a, Color b, float t)
        {
            t = Clamp01(t);
            return Color.FromArgb(
                (int)(a.R + (b.R - a.R) * t),
                (int)(a.G + (b.G - a.G) * t),
                (int)(a.B + (b.B - a.B) * t));
        }

        private static Color IdColor(uint id)
        {
            uint hash = id * 2654435761U;
            float hue = (hash % 360U) / 360f;
            return Hsv(hue, 0.82f, 1f);
        }

        private static Color Hsv(float h, float s, float v)
        {
            float scaled = h * 6f;
            int i = (int)Math.Floor(scaled) % 6;
            float f = scaled - (float)Math.Floor(scaled);
            float p = v * (1f - s);
            float q = v * (1f - f * s);
            float t = v * (1f - (1f - f) * s);
            float r = 0, g = 0, b = 0;
            if (i == 0) { r = v; g = t; b = p; }
            else if (i == 1) { r = q; g = v; b = p; }
            else if (i == 2) { r = p; g = v; b = t; }
            else if (i == 3) { r = p; g = q; b = v; }
            else if (i == 4) { r = t; g = p; b = v; }
            else { r = v; g = p; b = q; }
            return Color.FromArgb((int)(r * 255), (int)(g * 255), (int)(b * 255));
        }

        private static float Clamp01(float value) { return Math.Max(0f, Math.Min(1f, value)); }
    }

    internal sealed class LegendPanel : Panel
    {
        private string mode = "Reflectivity";

        public LegendPanel()
        {
            DoubleBuffered = true;
            BackColor = Color.FromArgb(6, 9, 13);
            ForeColor = Color.White;
            Width = 128;
        }

        public string Mode
        {
            get { return mode; }
            set { mode = value; Invalidate(); }
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            using (Font titleFont = new Font(Font.FontFamily, 10f, FontStyle.Bold | FontStyle.Italic))
            using (Brush white = new SolidBrush(Color.White))
            {
                string title = mode;
                if (mode == "Distance") title = "Distance(m)";
                else if (mode == "Elevation") title = "Elevation(m)";
                g.DrawString(title, titleFont, white, 8, 18);
                Rectangle bar = new Rectangle(18, 52, 24, Math.Max(120, Height - 115));
                Color[] source = CloudColors.LegendStops(mode);
                Color[] topToBottom = source.Reverse().ToArray();
                using (LinearGradientBrush gradient = new LinearGradientBrush(bar, topToBottom[0], topToBottom[topToBottom.Length - 1], LinearGradientMode.Vertical))
                {
                    ColorBlend blend = new ColorBlend(topToBottom.Length);
                    blend.Colors = topToBottom;
                    blend.Positions = Enumerable.Range(0, topToBottom.Length).Select(delegate(int i) { return i / (float)(topToBottom.Length - 1); }).ToArray();
                    gradient.InterpolationColors = blend;
                    g.FillRectangle(gradient, bar);
                }
                string[] ticks;
                if (mode == "Reflectivity") ticks = new string[] { "255", "204", "153", "102", "51", "0" };
                else if (mode == "Distance") ticks = new string[] { "4", "3.2", "2.4", "1.6", "0.8", "0" };
                else if (mode == "Elevation") ticks = new string[] { "1", "0.8", "0.6", "0.4", "0.2", "0" };
                else if (mode == "LiDAR ID") ticks = new string[] { "LiDAR", "IDs" };
                else ticks = new string[] { "Color" };
                for (int i = 0; i < ticks.Length; i++)
                {
                    float y = ticks.Length == 1 ? bar.Top + bar.Height / 2f : bar.Top + i * bar.Height / (float)(ticks.Length - 1);
                    g.DrawString(ticks[i], Font, white, 52, y - Font.Height / 2f);
                }
            }
        }
    }

    internal sealed class OpenGLPointCloudControl : Control
    {
        private IntPtr deviceContext = IntPtr.Zero;
        private IntPtr renderContext = IntPtr.Zero;
        private float[] vertices = new float[0];
        private byte[] colors = new byte[0];
        private float pointSize = 1f;
        private float centerX;
        private float centerY;
        private float centerZ = 1.4f;
        private float radius = 12.5f;
        private float distance = 32.5f;
        private float yaw = -45f;
        private float pitch = 25f;
        private float panX;
        private float panY;
        private Point lastMouse;
        private MouseButtons dragButton = MouseButtons.None;
        public OpenGLPointCloudControl()
        {
            SetStyle(ControlStyles.Opaque | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);
            BackColor = Color.FromArgb(6, 9, 13);
            TabStop = true;
        }

        public void SetCloud(float[] newVertices, byte[] newColors, bool resetView)
        {
            vertices = newVertices ?? new float[0];
            colors = newColors ?? new byte[0];
            if (resetView) ResetView();
            Invalidate();
        }

        public void SetColors(byte[] newColors)
        {
            colors = newColors ?? new byte[0];
            Invalidate();
        }

        public float PointSize
        {
            get { return pointSize; }
            set { pointSize = Math.Max(1f, value); Invalidate(); }
        }

        public void ResetView()
        {
            if (vertices.Length >= 3)
            {
                float minX = float.MaxValue, minY = float.MaxValue, minZ = float.MaxValue;
                float maxX = float.MinValue, maxY = float.MinValue, maxZ = float.MinValue;
                for (int i = 0; i < vertices.Length; i += 3)
                {
                    float x = vertices[i], y = vertices[i + 1], z = vertices[i + 2];
                    if (x < minX) minX = x; if (x > maxX) maxX = x;
                    if (y < minY) minY = y; if (y > maxY) maxY = y;
                    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
                }
                centerX = (minX + maxX) * 0.5f;
                centerY = (minY + maxY) * 0.5f;
                centerZ = (minZ + maxZ) * 0.5f;
                float dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
                radius = Math.Max(0.1f, (float)Math.Sqrt(dx * dx + dy * dy + dz * dz) * 0.5f);
            }
            else
            {
                centerX = centerY = 0f;
                centerZ = 1.4f;
                radius = 12.5f;
            }
            distance = radius * 2.6f;
            yaw = -45f;
            pitch = 25f;
            panX = panY = 0f;
            Invalidate();
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            deviceContext = GetDC(Handle);
            PIXELFORMATDESCRIPTOR pfd = new PIXELFORMATDESCRIPTOR();
            pfd.nSize = (ushort)Marshal.SizeOf(typeof(PIXELFORMATDESCRIPTOR));
            pfd.nVersion = 1;
            pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
            pfd.iPixelType = PFD_TYPE_RGBA;
            pfd.cColorBits = 32;
            pfd.cDepthBits = 24;
            pfd.iLayerType = PFD_MAIN_PLANE;
            int format = ChoosePixelFormat(deviceContext, ref pfd);
            if (format == 0 || !SetPixelFormat(deviceContext, format, ref pfd)) throw new InvalidOperationException("无法创建 OpenGL 像素格式。");
            renderContext = wglCreateContext(deviceContext);
            if (renderContext == IntPtr.Zero) throw new InvalidOperationException("无法创建 OpenGL 上下文。");
            wglMakeCurrent(deviceContext, renderContext);
            glClearColor(6f / 255f, 9f / 255f, 13f / 255f, 1f);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LEQUAL);
            glEnable(GL_FOG);
            glFogi(GL_FOG_MODE, (int)GL_EXP2);
            glFogf(GL_FOG_DENSITY, 0.006f);
            glFogfv(GL_FOG_COLOR, new float[] { 6f / 255f, 9f / 255f, 13f / 255f, 1f });
        }

        protected override void OnHandleDestroyed(EventArgs e)
        {
            if (renderContext != IntPtr.Zero)
            {
                wglMakeCurrent(deviceContext, renderContext);
                wglMakeCurrent(IntPtr.Zero, IntPtr.Zero);
                wglDeleteContext(renderContext);
                renderContext = IntPtr.Zero;
            }
            if (deviceContext != IntPtr.Zero)
            {
                ReleaseDC(Handle, deviceContext);
                deviceContext = IntPtr.Zero;
            }
            base.OnHandleDestroyed(e);
        }

        protected override void OnPaintBackground(PaintEventArgs pevent) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            if (renderContext == IntPtr.Zero || deviceContext == IntPtr.Zero) return;
            wglMakeCurrent(deviceContext, renderContext);
            int width = Math.Max(1, ClientSize.Width);
            int height = Math.Max(1, ClientSize.Height);
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            float nearPlane = Math.Max(0.01f, radius * 0.01f);
            float farPlane = Math.Max(100f, distance + radius * 20f);
            float aspect = width / (float)height;
            float f = 1f / (float)Math.Tan(52f * Math.PI / 360f);
            float[] projection = new float[]
            {
                f / aspect, 0, 0, 0,
                0, f, 0, 0,
                0, 0, (farPlane + nearPlane) / (nearPlane - farPlane), -1,
                0, 0, (2f * farPlane * nearPlane) / (nearPlane - farPlane), 0
            };
            glMatrixMode(GL_PROJECTION);
            glLoadMatrixf(projection);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glTranslatef(panX, panY, -distance);
            glRotatef(pitch, 1f, 0f, 0f);
            glRotatef(yaw, 0f, 0f, 1f);
            glTranslatef(-centerX, -centerY, -centerZ);

            DrawReferenceCanvas();

            int pointCount = Math.Min(vertices.Length / 3, colors.Length / 3);
            if (pointCount > 0)
            {
                GCHandle vertexHandle = GCHandle.Alloc(vertices, GCHandleType.Pinned);
                GCHandle colorHandle = GCHandle.Alloc(colors, GCHandleType.Pinned);
                try
                {
                    glEnableClientState(GL_VERTEX_ARRAY);
                    glEnableClientState(GL_COLOR_ARRAY);
                    glVertexPointer(3, GL_FLOAT, 0, vertexHandle.AddrOfPinnedObject());
                    glColorPointer(3, GL_UNSIGNED_BYTE, 0, colorHandle.AddrOfPinnedObject());
                    glPointSize(pointSize);
                    glDrawArrays(GL_POINTS, 0, pointCount);
                    glDisableClientState(GL_COLOR_ARRAY);
                    glDisableClientState(GL_VERTEX_ARRAY);
                }
                finally
                {
                    colorHandle.Free();
                    vertexHandle.Free();
                }
            }
            glFlush();
            SwapBuffers(deviceContext);
        }

        private void DrawReferenceCanvas()
        {
            glLineWidth(1f);
            glBegin(GL_LINES);
            for (int i = -20; i <= 20; i++)
            {
                if (i == 0) glColor3ub(39, 64, 85);
                else glColor3ub(19, 36, 49);
                float position = i * 2f;
                glVertex3f(position, -40f, 0f);
                glVertex3f(position, 40f, 0f);
                glVertex3f(-40f, position, 0f);
                glVertex3f(40f, position, 0f);
            }
            glEnd();

            glColor3ub(242, 169, 0);
            glBegin(GL_QUAD_STRIP);
            for (int i = 0; i <= 48; i++)
            {
                double angle = i * Math.PI * 2.0 / 48.0;
                float cosine = (float)Math.Cos(angle);
                float sine = (float)Math.Sin(angle);
                glVertex3f(cosine * 0.48f, sine * 0.48f, 0.015f);
                glVertex3f(cosine * 0.38f, sine * 0.38f, 0.015f);
            }
            glEnd();
            glLineWidth(1f);
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            base.OnMouseDown(e);
            Focus();
            dragButton = e.Button;
            lastMouse = e.Location;
            Capture = true;
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            base.OnMouseUp(e);
            dragButton = MouseButtons.None;
            Capture = false;
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            int dx = e.X - lastMouse.X;
            int dy = e.Y - lastMouse.Y;
            lastMouse = e.Location;
            if (dragButton == MouseButtons.Left)
            {
                yaw += dx * 0.20f;
                pitch += dy * 0.20f;
                Invalidate();
            }
            else if (dragButton == MouseButtons.Right)
            {
                float scale = distance / Math.Max(200f, ClientSize.Height);
                panX += dx * scale;
                panY -= dy * scale;
                Invalidate();
            }
        }

        protected override void OnMouseWheel(MouseEventArgs e)
        {
            base.OnMouseWheel(e);
            distance *= (float)Math.Exp(-e.Delta / 1200.0);
            distance = Math.Max(radius * 0.05f, Math.Min(radius * 100f, distance));
            Invalidate();
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct PIXELFORMATDESCRIPTOR
        {
            public ushort nSize; public ushort nVersion; public uint dwFlags;
            public byte iPixelType; public byte cColorBits; public byte cRedBits; public byte cRedShift;
            public byte cGreenBits; public byte cGreenShift; public byte cBlueBits; public byte cBlueShift;
            public byte cAlphaBits; public byte cAlphaShift; public byte cAccumBits; public byte cAccumRedBits;
            public byte cAccumGreenBits; public byte cAccumBlueBits; public byte cAccumAlphaBits; public byte cDepthBits;
            public byte cStencilBits; public byte cAuxBuffers; public byte iLayerType; public byte bReserved;
            public uint dwLayerMask; public uint dwVisibleMask; public uint dwDamageMask;
        }

        private const uint PFD_DOUBLEBUFFER = 0x00000001;
        private const uint PFD_DRAW_TO_WINDOW = 0x00000004;
        private const uint PFD_SUPPORT_OPENGL = 0x00000020;
        private const byte PFD_TYPE_RGBA = 0;
        private const byte PFD_MAIN_PLANE = 0;
        private const uint GL_COLOR_BUFFER_BIT = 0x00004000;
        private const uint GL_DEPTH_BUFFER_BIT = 0x00000100;
        private const uint GL_DEPTH_TEST = 0x0B71;
        private const uint GL_LEQUAL = 0x0203;
        private const uint GL_PROJECTION = 0x1701;
        private const uint GL_MODELVIEW = 0x1700;
        private const uint GL_VERTEX_ARRAY = 0x8074;
        private const uint GL_COLOR_ARRAY = 0x8076;
        private const uint GL_FLOAT = 0x1406;
        private const uint GL_UNSIGNED_BYTE = 0x1401;
        private const uint GL_POINTS = 0x0000;
        private const uint GL_LINES = 0x0001;
        private const uint GL_LINE_LOOP = 0x0002;
        private const uint GL_QUAD_STRIP = 0x0008;
        private const uint GL_FOG = 0x0B60;
        private const uint GL_FOG_MODE = 0x0B65;
        private const uint GL_FOG_DENSITY = 0x0B62;
        private const uint GL_FOG_COLOR = 0x0B66;
        private const uint GL_EXP2 = 0x0801;

        [DllImport("user32.dll")] private static extern IntPtr GetDC(IntPtr hWnd);
        [DllImport("user32.dll")] private static extern int ReleaseDC(IntPtr hWnd, IntPtr hDC);
        [DllImport("gdi32.dll")] private static extern int ChoosePixelFormat(IntPtr hdc, ref PIXELFORMATDESCRIPTOR pfd);
        [DllImport("gdi32.dll")] private static extern bool SetPixelFormat(IntPtr hdc, int format, ref PIXELFORMATDESCRIPTOR pfd);
        [DllImport("gdi32.dll")] private static extern bool SwapBuffers(IntPtr hdc);
        [DllImport("opengl32.dll")] private static extern IntPtr wglCreateContext(IntPtr hdc);
        [DllImport("opengl32.dll")] private static extern bool wglDeleteContext(IntPtr hglrc);
        [DllImport("opengl32.dll")] private static extern bool wglMakeCurrent(IntPtr hdc, IntPtr hglrc);
        [DllImport("opengl32.dll")] private static extern void glClearColor(float red, float green, float blue, float alpha);
        [DllImport("opengl32.dll")] private static extern void glClear(uint mask);
        [DllImport("opengl32.dll")] private static extern void glEnable(uint cap);
        [DllImport("opengl32.dll")] private static extern void glDisable(uint cap);
        [DllImport("opengl32.dll")] private static extern void glDepthFunc(uint func);
        [DllImport("opengl32.dll")] private static extern void glViewport(int x, int y, int width, int height);
        [DllImport("opengl32.dll")] private static extern void glMatrixMode(uint mode);
        [DllImport("opengl32.dll")] private static extern void glLoadMatrixf(float[] matrix);
        [DllImport("opengl32.dll")] private static extern void glLoadIdentity();
        [DllImport("opengl32.dll")] private static extern void glTranslatef(float x, float y, float z);
        [DllImport("opengl32.dll")] private static extern void glRotatef(float angle, float x, float y, float z);
        [DllImport("opengl32.dll")] private static extern void glEnableClientState(uint array);
        [DllImport("opengl32.dll")] private static extern void glDisableClientState(uint array);
        [DllImport("opengl32.dll")] private static extern void glVertexPointer(int size, uint type, int stride, IntPtr pointer);
        [DllImport("opengl32.dll")] private static extern void glColorPointer(int size, uint type, int stride, IntPtr pointer);
        [DllImport("opengl32.dll")] private static extern void glPointSize(float size);
        [DllImport("opengl32.dll")] private static extern void glDrawArrays(uint mode, int first, int count);
        [DllImport("opengl32.dll")] private static extern void glBegin(uint mode);
        [DllImport("opengl32.dll")] private static extern void glEnd();
        [DllImport("opengl32.dll")] private static extern void glColor3ub(byte red, byte green, byte blue);
        [DllImport("opengl32.dll")] private static extern void glVertex3f(float x, float y, float z);
        [DllImport("opengl32.dll")] private static extern void glLineWidth(float width);
        [DllImport("opengl32.dll")] private static extern void glFogi(uint pname, int param);
        [DllImport("opengl32.dll")] private static extern void glFogf(uint pname, float param);
        [DllImport("opengl32.dll")] private static extern void glFogfv(uint pname, float[] parameters);
        [DllImport("opengl32.dll")] private static extern void glFlush();
    }

    internal sealed class MainForm : Form
    {
        private enum ViewerMode
        {
            Start,
            Lvx2,
            PcdSingle,
            PcdFolder,
            Live
        }

        private readonly OpenGLPointCloudControl view = new OpenGLPointCloudControl();
        private readonly LegendPanel legend = new LegendPanel();
        private readonly TableLayoutPanel content = new TableLayoutPanel();
        private readonly Label status = new Label();
        private readonly ComboBox colorMode = new ComboBox();
        private readonly TrackBar interval = new TrackBar();
        private readonly Label intervalValue = new Label();
        private readonly Label colorLabel;
        private readonly Label intervalLabel;
        private readonly Timer timer = new Timer();
        private readonly Button openLvxButton;
        private readonly Button openPcdButton;
        private readonly Button openFolderButton;
        private readonly Button connectLidarButton;
        private readonly Button backButton;
        private readonly Button previousButton;
        private readonly Button playButton;
        private readonly Button pauseButton;
        private readonly Button nextButton;
        private readonly Button saveButton;
        private ViewerMode viewerMode = ViewerMode.Start;
        private FrameSource source;
        private int frameIndex = -1;
        private PointCloudFrame currentFrame;
        private PointCloudFrame displayedFrame;
        private readonly Dictionary<int, PointCloudFrame> cache = new Dictionary<int, PointCloudFrame>();
        private readonly List<int> cacheOrder = new List<int>();
        private LivoxLiveReceiver liveReceiver;
        private bool livePaused;
        private long displayedLiveSequence;
        private readonly List<PointCloudFrame> liveDisplayFrames = new List<PointCloudFrame>();
        private string liveBaseName = "livox_live";

        public MainForm()
        {
            Text = "Livox 点云播放器（原生桌面版）";
            StartPosition = FormStartPosition.CenterScreen;
            ClientSize = new Size(1400, 850);
            MinimumSize = new Size(980, 600);
            BackColor = Color.FromArgb(20, 22, 25);
            ForeColor = Color.White;
            KeyPreview = true;

            FlowLayoutPanel toolbar = new FlowLayoutPanel();
            toolbar.Dock = DockStyle.Top;
            toolbar.AutoSize = true;
            toolbar.WrapContents = true;
            toolbar.Padding = new Padding(8, 7, 8, 7);
            toolbar.BackColor = Color.FromArgb(31, 34, 39);

            openLvxButton = MakeButton("打开 LVX2", 100);
            openPcdButton = MakeButton("打开 PCD", 96);
            openFolderButton = MakeButton("打开 PCD 文件夹", 132);
            connectLidarButton = MakeButton("连接雷达", 96);
            backButton = MakeButton("返回", 68);
            previousButton = MakeButton("上一帧", 78);
            playButton = MakeButton("播放", 68);
            pauseButton = MakeButton("暂停", 68);
            nextButton = MakeButton("下一帧", 78);
            saveButton = MakeButton("保存当前帧 PCD", 132);

            openLvxButton.Click += delegate { OpenLvx2(); };
            openPcdButton.Click += delegate { OpenPcd(); };
            openFolderButton.Click += delegate { OpenPcdFolder(); };
            connectLidarButton.Click += delegate { ConnectLidar(); };
            backButton.Click += delegate { ReturnToStart(); };
            previousButton.Click += delegate { LoadFrame(frameIndex - 1, false); };
            playButton.Click += delegate { if (viewerMode == ViewerMode.Live) ResumeLivePlayback(); else StartPlayback(); };
            pauseButton.Click += delegate { if (viewerMode == ViewerMode.Live) PauseLivePlayback(); else StopPlayback(); };
            nextButton.Click += delegate { LoadFrame(frameIndex + 1, false); };
            saveButton.Click += delegate { SaveCurrentFrame(); };

            colorLabel = MakeLabel("Color", 42);
            colorMode.DropDownStyle = ComboBoxStyle.DropDownList;
            colorMode.Items.AddRange(new object[] { "Reflectivity", "Distance", "Solid Color", "Elevation", "LiDAR ID" });
            colorMode.SelectedIndex = 0;
            colorMode.Width = 118;
            colorMode.Margin = new Padding(2, 5, 10, 2);
            colorMode.BackColor = Color.FromArgb(16, 17, 19);
            colorMode.ForeColor = Color.White;
            colorMode.SelectedIndexChanged += delegate { Recolor(); };

            intervalLabel = MakeLabel("Interval", 58);
            interval.Minimum = 0;
            interval.Maximum = 200;
            interval.SmallChange = 5;
            interval.LargeChange = 20;
            interval.TickFrequency = 20;
            interval.Value = 50;
            interval.AutoSize = false;
            interval.Width = 160;
            interval.Height = 30;
            interval.Margin = new Padding(2, 2, 2, 2);
            interval.BackColor = Color.FromArgb(16, 17, 19);
            interval.ForeColor = Color.White;
            intervalValue.Text = "50 ms";
            intervalValue.Width = 55;
            intervalValue.Height = 30;
            intervalValue.TextAlign = ContentAlignment.MiddleLeft;
            intervalValue.Margin = new Padding(2, 2, 8, 2);
            intervalValue.ForeColor = Color.Gainsboro;
            interval.ValueChanged += delegate
            {
                timer.Interval = Math.Max(1, interval.Value);
                intervalValue.Text = interval.Value.ToString(CultureInfo.InvariantCulture) + " ms";
            };

            toolbar.Controls.AddRange(new Control[]
            {
                openLvxButton, openPcdButton, openFolderButton, connectLidarButton, backButton,
                previousButton, playButton, pauseButton, nextButton, saveButton,
                colorLabel, colorMode, intervalLabel, interval, intervalValue
            });

            status.Dock = DockStyle.Bottom;
            status.Height = 30;
            status.Padding = new Padding(10, 7, 4, 4);
            status.BackColor = Color.FromArgb(31, 34, 39);
            status.ForeColor = Color.Gainsboro;
            status.Text = "请选择 LVX2、PCD、PCD 帧文件夹，或连接 Livox 雷达。左键旋转，右键平移，滚轮缩放。";

            content.Dock = DockStyle.Fill;
            content.Margin = new Padding(0);
            content.Padding = new Padding(0);
            content.ColumnCount = 2;
            content.RowCount = 1;
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 128f));
            view.Dock = DockStyle.Fill;
            legend.Dock = DockStyle.Fill;
            content.Controls.Add(view, 0, 0);
            content.Controls.Add(legend, 1, 0);

            Controls.Add(content);
            Controls.Add(status);
            Controls.Add(toolbar);

            timer.Interval = 50;
            timer.Tick += delegate
            {
                if (source != null && viewerMode == ViewerMode.Lvx2) LoadFrame(frameIndex + 1, false);
                else if (viewerMode == ViewerMode.Live) PollLiveFrame();
            };
            FormClosed += delegate
            {
                if (source != null) source.Dispose();
                if (liveReceiver != null)
                {
                    try { StopLidarOutput(false); } catch { }
                    liveReceiver.Dispose();
                }
            };
            KeyDown += OnShortcut;
            UpdateButtons();
        }

        private static Button MakeButton(string text, int width)
        {
            Button button = new Button();
            button.Text = text;
            button.Width = width;
            button.Height = 30;
            button.Margin = new Padding(3, 2, 3, 2);
            button.FlatStyle = FlatStyle.Flat;
            button.FlatAppearance.BorderColor = Color.FromArgb(80, 85, 92);
            button.BackColor = Color.FromArgb(48, 52, 58);
            button.ForeColor = Color.White;
            return button;
        }

        private static Label MakeLabel(string text, int width)
        {
            Label label = new Label();
            label.Text = text;
            label.Width = width;
            label.Height = 30;
            label.TextAlign = ContentAlignment.MiddleRight;
            label.Margin = new Padding(3, 2, 1, 2);
            return label;
        }

        private void OpenLvx2()
        {
            using (OpenFileDialog dialog = new OpenFileDialog())
            {
                dialog.Filter = "Livox LVX2 (*.lvx2)|*.lvx2|All files (*.*)|*.*";
                dialog.Title = "选择 LVX2 文件";
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                RunBusy(delegate
                {
                    status.Text = "正在建立 LVX2 帧索引…";
                    SetSource(new Lvx2Source(dialog.FileName), ViewerMode.Lvx2);
                });
            }
        }

        private void OpenPcd()
        {
            using (OpenFileDialog dialog = new OpenFileDialog())
            {
                dialog.Filter = "Point Cloud Data (*.pcd)|*.pcd|All files (*.*)|*.*";
                dialog.Title = "选择 PCD 文件";
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                RunBusy(delegate { SetSource(new PcdSequenceSource(new string[] { dialog.FileName }, Path.GetFileName(dialog.FileName)), ViewerMode.PcdSingle); });
            }
        }

        private void OpenPcdFolder()
        {
            using (FolderBrowserDialog dialog = new FolderBrowserDialog())
            {
                dialog.Description = "选择包含连续 PCD 帧的文件夹";
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                string[] files = Directory.GetFiles(dialog.SelectedPath, "*.pcd", SearchOption.TopDirectoryOnly);
                if (files.Length == 0)
                {
                    MessageBox.Show(this, "该文件夹中没有 PCD 文件。", "点云播放器", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }
                RunBusy(delegate { SetSource(new PcdSequenceSource(files, new DirectoryInfo(dialog.SelectedPath).Name), ViewerMode.PcdFolder); });
            }
        }

        private void ConnectLidar()
        {
            using (LidarConnectionDialog dialog = new LidarConnectionDialog())
            {
                if (dialog.ShowDialog(this) != DialogResult.OK || dialog.SelectedDevice == null) return;
                RunBusy(delegate
                {
                    StopPlayback();
                    DisposeCurrentSources();
                    status.Text = "正在连接 " + dialog.SelectedDevice.Model + "（" + dialog.SelectedDevice.Ip + "）并启动点云…";
                    Application.DoEvents();
                    LivoxLiveReceiver receiver = new LivoxLiveReceiver(dialog.SelectedHostIp, dialog.SelectedDevice);
                    receiver.Start();
                    liveReceiver = receiver;
                    viewerMode = ViewerMode.Live;
                    livePaused = false;
                    displayedLiveSequence = 0;
                    liveDisplayFrames.Clear();
                    currentFrame = null;
                    displayedFrame = null;
                    frameIndex = -1;
                    liveBaseName = (string.IsNullOrEmpty(dialog.SelectedDevice.Serial) ? dialog.SelectedDevice.Model : dialog.SelectedDevice.Serial)
                        .Replace(' ', '_').Replace('-', '_') + "_live";
                    view.PointSize = 1f;
                    view.SetCloud(new float[0], new byte[0], true);
                    timer.Interval = Math.Max(1, interval.Value);
                    timer.Start();
                    status.Text = dialog.SelectedDevice + "  ·  已连接，等待点云数据…";
                    UpdateButtons();
                });
            }
        }

        private void SetSource(FrameSource newSource, ViewerMode newMode)
        {
            StopPlayback();
            DisposeCurrentSources();
            source = newSource;
            viewerMode = newMode;
            view.PointSize = source is Lvx2Source ? 1f : 1.5f;
            frameIndex = -1;
            currentFrame = null;
            displayedFrame = null;
            cache.Clear();
            cacheOrder.Clear();
            int suggested = Math.Max(interval.Minimum, Math.Min(interval.Maximum, source.SuggestedInterval));
            interval.Value = suggested;
            LoadFrame(0, true);
        }

        private void ReturnToStart()
        {
            if (viewerMode == ViewerMode.Live && liveReceiver != null)
            {
                livePaused = true;
                timer.Stop();
                UpdateButtons();
                Cursor previous = Cursor;
                try
                {
                    Cursor = Cursors.WaitCursor;
                    status.Text = "正在通知雷达停止采样…";
                    Application.DoEvents();
                    bool silent = StopLidarOutput(true);
                    if (!silent)
                        MessageBox.Show(this, "待机命令已发送，但没有查询确认到待机状态，或 UDP 点包仍在增加。", "未确认雷达待机", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(this, ex.Message + "\n\n程序仍将断开雷达连接。", "停止雷达失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
                finally { Cursor = previous; }
            }
            StopPlayback();
            DisposeCurrentSources();
            viewerMode = ViewerMode.Start;
            frameIndex = -1;
            currentFrame = null;
            displayedFrame = null;
            cache.Clear();
            cacheOrder.Clear();
            liveDisplayFrames.Clear();
            displayedLiveSequence = 0;
            livePaused = false;
            view.SetCloud(new float[0], new byte[0], true);
            status.Text = "请选择 LVX2、PCD、PCD 帧文件夹，或连接 Livox 雷达。左键旋转，右键平移，滚轮缩放。";
            UpdateButtons();
        }

        private void DisposeCurrentSources()
        {
            if (source != null) source.Dispose();
            source = null;
            if (liveReceiver != null) liveReceiver.Dispose();
            liveReceiver = null;
        }

        private void PollLiveFrame()
        {
            if (viewerMode != ViewerMode.Live || liveReceiver == null || livePaused) return;
            long sequence;
            PointCloudFrame latest = liveReceiver.GetLatestFrame(out sequence);
            if (latest == null)
            {
                status.Text = string.Format(CultureInfo.InvariantCulture,
                    "{0}  ·  已连接  ·  {1:N0} packets  ·  {2:N0} points  ·  {3}",
                    liveReceiver.Device, liveReceiver.PacketCount, liveReceiver.PointCount,
                    liveReceiver.PacketCount == 0
                        ? "尚未收到 UDP 点包，请检查网卡 IP、防火墙和端口占用"
                        : "正在生成首个画面");
                if (!string.IsNullOrEmpty(liveReceiver.LastError)) status.Text += "  ·  " + liveReceiver.LastError;
                return;
            }
            if (sequence == displayedLiveSequence) return;
            bool resetView = displayedLiveSequence == 0;
            displayedLiveSequence = sequence;
            currentFrame = latest;
            liveDisplayFrames.Add(latest);
            while (liveDisplayFrames.Count > 2) liveDisplayFrames.RemoveAt(0);
            displayedFrame = PointCloudFrame.Merge(liveDisplayFrames);
            view.SetCloud(displayedFrame.Vertices, CloudColors.Create(displayedFrame, (string)colorMode.SelectedItem), resetView);
            status.Text = string.Format(CultureInfo.InvariantCulture,
                "{0}  ·  LIVE frame {1}  ·  {2:N0} points  ·  2-frame accumulation  ·  {3:N0} packets  ·  {4}",
                liveReceiver.Device, sequence, currentFrame.Count, liveReceiver.PacketCount,
                string.IsNullOrEmpty(liveReceiver.LastSender) ? "waiting" : liveReceiver.LastSender);
            if (!string.IsNullOrEmpty(liveReceiver.LastError)) status.Text += "  ·  " + liveReceiver.LastError;
            UpdateButtons();
        }

        private PointCloudFrame GetFrame(int index)
        {
            PointCloudFrame frame;
            if (cache.TryGetValue(index, out frame)) return frame;
            frame = source.LoadFrame(index);
            cache[index] = frame;
            cacheOrder.Add(index);
            while (cacheOrder.Count > 12)
            {
                int old = cacheOrder[0];
                cacheOrder.RemoveAt(0);
                if (old != index) cache.Remove(old);
            }
            return frame;
        }

        private void LoadFrame(int requestedIndex, bool resetView)
        {
            if (source == null || source.Count == 0) return;
            int target = ((requestedIndex % source.Count) + source.Count) % source.Count;
            try
            {
                currentFrame = GetFrame(target);
                List<PointCloudFrame> frames = new List<PointCloudFrame>();
                int start = Math.Max(0, target - 1);
                for (int i = start; i <= target; i++) frames.Add(GetFrame(i));
                displayedFrame = PointCloudFrame.Merge(frames);
                frameIndex = target;
                byte[] colors = CloudColors.Create(displayedFrame, (string)colorMode.SelectedItem);
                view.SetCloud(displayedFrame.Vertices, colors, resetView);
                int lidarCount = new HashSet<uint>(displayedFrame.LidarIds).Count;
                status.Text = string.Format(CultureInfo.InvariantCulture,
                    "{0}  ·  frame {1}/{2}  ·  {3:N0} points  ·  {4}-frame accumulation  ·  {5} LiDAR",
                    source.DisplayName, target + 1, source.Count, currentFrame.Count, frames.Count, lidarCount);
                UpdateButtons();
            }
            catch (Exception ex)
            {
                StopPlayback();
                MessageBox.Show(this, ex.Message, "读取点云失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void Recolor()
        {
            string mode = colorMode.SelectedItem == null ? "Reflectivity" : colorMode.SelectedItem.ToString();
            legend.Mode = mode;
            if (displayedFrame != null) view.SetColors(CloudColors.Create(displayedFrame, mode));
            UpdateLegendVisibility();
        }

        private void UpdateLegendVisibility()
        {
            string mode = colorMode.SelectedItem == null ? "Reflectivity" : colorMode.SelectedItem.ToString();
            bool showLegend = viewerMode != ViewerMode.Start && mode != "Solid Color";
            legend.Visible = showLegend;
            if (content.ColumnStyles.Count > 1) content.ColumnStyles[1].Width = showLegend ? 128f : 0f;
        }

        private void StartPlayback()
        {
            if (viewerMode == ViewerMode.Live)
            {
                ResumeLivePlayback();
                return;
            }
            if (source == null || viewerMode != ViewerMode.Lvx2) return;
            timer.Interval = Math.Max(1, interval.Value);
            timer.Start();
            UpdateButtons();
        }

        private void StopPlayback()
        {
            if (viewerMode == ViewerMode.Live) livePaused = true;
            timer.Stop();
            UpdateButtons();
        }

        private void PauseLivePlayback()
        {
            if (viewerMode != ViewerMode.Live || liveReceiver == null || !timer.Enabled) return;
            RunBusy(delegate
            {
                status.Text = "正在停止雷达采样…";
                Application.DoEvents();
                livePaused = true;
                timer.Stop();
                bool silent = StopLidarOutput(true);
                if (silent)
                    status.Text = liveReceiver.Device + "  ·  雷达已停止采样，UDP 点包已停止  ·  当前画面已保留，可保存 PCD";
                else
                {
                    status.Text = liveReceiver.Device + "  ·  警告：没有确认到雷达待机状态";
                    MessageBox.Show(this, "没有查询确认到待机状态，或仍检测到雷达点包。", "未确认雷达待机", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
                UpdateButtons();
            });
        }

        private bool StopLidarOutput(bool verifySilence)
        {
            if (liveReceiver == null) return true;
            liveReceiver.SetSampling(false);
            if (!verifySilence) return true;
            LivoxWorkState finalState;
            bool standby = liveReceiver.WaitForWorkMode(false, 12000, out finalState, delegate(LivoxWorkState state)
            {
                status.Text = liveReceiver.Device + "  ·  目标：待机  ·  当前：" + LivoxLiveReceiver.DescribeWorkState(state.CurrentState);
                Application.DoEvents();
            });
            bool silent = liveReceiver.WaitForPacketSilence(400, 1800);
            if (!silent)
            {
                try { liveReceiver.ForceDisablePointSend(); } catch { }
                silent = liveReceiver.WaitForPacketSilence(400, 1200);
            }
            return standby && silent;
        }

        private void ResumeLivePlayback()
        {
            if (viewerMode != ViewerMode.Live || liveReceiver == null || timer.Enabled) return;
            RunBusy(delegate
            {
                status.Text = "正在恢复雷达采样…";
                Application.DoEvents();
                liveReceiver.SetSampling(true);
                LivoxWorkState finalState;
                bool normal = liveReceiver.WaitForWorkMode(true, 12000, out finalState, delegate(LivoxWorkState state)
                {
                    status.Text = liveReceiver.Device + "  ·  目标：正常工作  ·  当前：" + LivoxLiveReceiver.DescribeWorkState(state.CurrentState);
                    Application.DoEvents();
                });
                if (!normal) throw new InvalidOperationException("雷达没有在规定时间内恢复到正常工作状态。");
                livePaused = false;
                timer.Interval = Math.Max(1, interval.Value);
                timer.Start();
                PollLiveFrame();
                UpdateButtons();
            });
        }

        private void UpdateButtons()
        {
            bool live = viewerMode == ViewerMode.Live;
            bool ready = live ? liveReceiver != null && currentFrame != null : source != null && frameIndex >= 0;
            bool loaded = viewerMode != ViewerMode.Start;
            bool lvx2 = viewerMode == ViewerMode.Lvx2;
            bool folder = viewerMode == ViewerMode.PcdFolder;

            openLvxButton.Visible = !loaded;
            openPcdButton.Visible = !loaded;
            openFolderButton.Visible = !loaded;
            connectLidarButton.Visible = !loaded;
            backButton.Visible = loaded;
            playButton.Visible = lvx2 || live;
            pauseButton.Visible = lvx2 || live;
            saveButton.Visible = lvx2 || live;
            previousButton.Visible = folder;
            nextButton.Visible = folder;
            colorLabel.Visible = loaded;
            colorMode.Visible = loaded;
            intervalLabel.Visible = loaded;
            interval.Visible = loaded;
            intervalValue.Visible = loaded;
            UpdateLegendVisibility();

            previousButton.Enabled = ready && folder;
            nextButton.Enabled = ready && folder;
            playButton.Enabled = (live ? liveReceiver != null : ready) && (lvx2 || live) && !timer.Enabled;
            pauseButton.Enabled = (live ? liveReceiver != null : ready) && (lvx2 || live) && timer.Enabled;
            saveButton.Enabled = ready && (lvx2 || live) && currentFrame != null;
        }

        private void SaveCurrentFrame()
        {
            bool live = viewerMode == ViewerMode.Live;
            if (currentFrame == null || (!live && (source == null || frameIndex < 0 || viewerMode != ViewerMode.Lvx2))) return;
            PointCloudFrame frameToSave = currentFrame;
            try
            {
                string programDirectory = Path.GetDirectoryName(typeof(MainForm).Assembly.Location);
                string saveDirectory = Path.Combine(programDirectory, "save");
                Directory.CreateDirectory(saveDirectory);
                string fileName = live
                    ? liveBaseName + "_frame_" + displayedLiveSequence.ToString("D6", CultureInfo.InvariantCulture) + "_" + DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture) + ".pcd"
                    : source.BaseName + "_frame_" + source.NativeIndex(frameIndex).ToString("D6", CultureInfo.InvariantCulture) + ".pcd";
                string path = Path.Combine(saveDirectory, fileName);
                int suffix = 1;
                while (File.Exists(path))
                {
                    path = Path.Combine(saveDirectory, Path.GetFileNameWithoutExtension(fileName) + "_" + suffix.ToString("D2", CultureInfo.InvariantCulture) + ".pcd");
                    suffix++;
                }
                WritePcd(path, frameToSave);
                status.Text = path + "  ·  " + frameToSave.Count.ToString("N0") + " points  ·  已保存当前原始帧";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "保存 PCD 失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private static void WritePcd(string path, PointCloudFrame frame)
        {
            string header =
                "# .PCD v0.7 - Point Cloud Data file format\n" +
                "VERSION 0.7\n" +
                "FIELDS x y z intensity tag lidar_id\n" +
                "SIZE 4 4 4 4 4 4\n" +
                "TYPE F F F F U U\n" +
                "COUNT 1 1 1 1 1 1\n" +
                "WIDTH " + frame.Count.ToString(CultureInfo.InvariantCulture) + "\n" +
                "HEIGHT 1\n" +
                "VIEWPOINT 0 0 0 1 0 0 0\n" +
                "POINTS " + frame.Count.ToString(CultureInfo.InvariantCulture) + "\n" +
                "DATA binary\n";
            using (FileStream stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream, Encoding.ASCII))
            {
                writer.Write(Encoding.ASCII.GetBytes(header));
                for (int i = 0; i < frame.Count; i++)
                {
                    int p = i * 3;
                    writer.Write(frame.Vertices[p]);
                    writer.Write(frame.Vertices[p + 1]);
                    writer.Write(frame.Vertices[p + 2]);
                    writer.Write((float)frame.Intensities[i]);
                    writer.Write((uint)frame.Tags[i]);
                    writer.Write(frame.LidarIds[i]);
                }
            }
        }

        private void OnShortcut(object sender, KeyEventArgs e)
        {
            if (viewerMode == ViewerMode.PcdFolder && e.KeyCode == Keys.Left) { LoadFrame(frameIndex - 1, false); e.Handled = true; }
            else if (viewerMode == ViewerMode.PcdFolder && e.KeyCode == Keys.Right) { LoadFrame(frameIndex + 1, false); e.Handled = true; }
            else if ((viewerMode == ViewerMode.Lvx2 || viewerMode == ViewerMode.Live) && e.KeyCode == Keys.Space)
            {
                if (viewerMode == ViewerMode.Live)
                {
                    if (timer.Enabled) PauseLivePlayback(); else ResumeLivePlayback();
                }
                else if (timer.Enabled) StopPlayback(); else StartPlayback();
                e.Handled = true;
            }
        }

        private void RunBusy(Action action)
        {
            Cursor previous = Cursor;
            try
            {
                Cursor = Cursors.WaitCursor;
                Application.DoEvents();
                action();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "点云播放器", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                Cursor = previous;
            }
        }
    }
}
