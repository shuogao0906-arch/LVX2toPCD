# LVX2toPCD

Windows 原生 Livox 点云播放器与 LVX2/PCD 工具。无需浏览器、Python 运行环境或额外 SDK DLL。

## 功能

- 播放 `.lvx2` 文件，固定 2 帧累积。
- 将整个 `.lvx2` 文件逐帧转换为 PCD，每一帧生成一个文件。
- 查看单个 PCD 或连续 PCD 帧文件夹。
- 保存 LVX2 当前原始帧为二进制 PCD。
- 发现并连接 Livox MID-360、HAP、HAP Industrial，实时显示点云。
- 实时画面可暂停并将当前原始帧保存为 PCD。
- PCD 字段：`x y z intensity tag lidar_id`。
- Reflectivity、Distance、Solid Color、Elevation、LiDAR ID 着色。
- 原生 OpenGL 三维画布：Z 轴朝上、透视相机、网格、轨道旋转、平移和缩放。
- 自动在程序目录下创建 `save` 文件夹保存当前帧。

打开 LVX2 后点击“转换为 PCD”，程序会在 LVX2 文件旁创建 `<文件名>_pcd_frames` 文件夹。转换结果采用六位帧号命名（例如 `000000.pcd`），保留 `x y z intensity tag lidar_id` 字段；进度窗口可随时取消转换。

## 下载运行

直接下载：

- `LivoxPointCloudPlayer-latest.exe`
- 或完整压缩包 `LivoxPointCloudPlayer-Windows.zip`

Windows 10/11 双击 EXE 即可运行。

## 实时雷达

1. 用网线连接雷达。
2. 将有线网卡设置为与雷达相同网段的静态 IPv4。
3. 点击“连接雷达”，选择网卡并扫描设备。
4. 选择设备后点击“连接并播放”。

暂停和返回会切换雷达到待机状态：

- HAP/HAP Industrial：目标模式 `0x02`，查询确认当前状态为 `0x02`。
- MID-360：目标模式 `0x03`。
- HAP 同时使用 `kKeyPointSendEn` 禁用点云发送。

播放器采用单实例运行，避免多个程序同时控制同一雷达。

## 从源码构建

在 Windows PowerShell 中运行：

```powershell
.\build.ps1
```

构建依赖 Windows 自带的 .NET Framework、Windows Forms 和 OpenGL 1.1。

## 操作

- 左键拖动：旋转
- 右键拖动：平移
- 滚轮：缩放
- `Space`：播放/暂停
- PCD 文件夹模式下 `←` / `→`：上一帧/下一帧

详细说明见 `使用说明.txt`。
