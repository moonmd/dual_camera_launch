Testing vl42 in ROS2 Ubuntu 24.04.2 (kilted) on Pi5 with dual MIPI CSI-2 cameras
# dual_camera_launch

ROS2 launch package running two OV5647 cameras on Pi 5 RP1

The resolution is set to 1920x1080 @15fps in `launch/dual_cameras.launch.py`

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/moonmd/dual_camera_launch.git
cd ~/ros2_ws
colcon build --packages-select dual_camera_launch
```

## Usage

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch dual_camera_launch dual_cameras.launch.py
```

GUI Preview 
```bash
ros2 run rqt_image_view rqt_image_view
```

## Topics

| Topic                    | Type                     | Description                    |
|--------------------------|--------------------------|--------------------------------|
| `/camera0/image_raw`     | `sensor_msgs/Image`      | Raw image stream from camera 0 |
| `/camera0/camera_info`   | `sensor_msgs/CameraInfo` | Intrinsics for camera 0        |
| `/camera1/image_raw`     | `sensor_msgs/Image`      | Raw image stream from camera 1 |
| `/camera1/camera_info`   | `sensor_msgs/CameraInfo` | Intrinsics for camera 1        |

```bash
ros2 topic list
```
```text
/camera0/camera_info
/camera0/image_raw
/camera0/image_raw/compressed
/camera1/camera_info
/camera1/image_raw
/camera1/image_raw/compressed
/parameter_events
/rosout
```
## Troubleshooting

Check v4l2 on rpi5: 
```bash
v4l2-ctl --list-devices
```
```text
pispbe (platform:1000880000.pisp_be):
	/dev/video20
	/dev/video21
	/dev/video22
	/dev/video23
	/dev/video24
	/dev/video25
	/dev/video26
	/dev/video27
	/dev/video28
	/dev/video29
	/dev/video30
	/dev/video31
	/dev/video32
	/dev/video33
	/dev/video34
	/dev/video35
	/dev/video36
	/dev/video37
	/dev/media1
	/dev/media4

rp1-cfe (platform:1f00110000.csi):
	/dev/video8
	/dev/video9
	/dev/video10
	/dev/video11
	/dev/video12
	/dev/video13
	/dev/video14
	/dev/video15
	/dev/media0

rp1-cfe (platform:1f00128000.csi):
	/dev/video0
	/dev/video1
	/dev/video2
	/dev/video3
	/dev/video4
	/dev/video5
	/dev/video6
	/dev/video7
	/dev/media2

rpivid (platform:rpivid):
	/dev/video19
	/dev/media3
```
```bash
v4l2-ctl --device=/dev/video0 --all
```
```text
Driver Info:
	Driver name      : rp1-cfe
	Card type        : rp1-cfe
	Bus info         : platform:1f00128000.csi
	Driver version   : 6.8.12
	Capabilities     : 0xaca00001
		Video Capture
		Metadata Capture
		Metadata Output
		I/O MC
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x24a00001
		Video Capture
		Metadata Capture
		I/O MC
		Streaming
		Extended Pix Format
Media Driver Info:
	Driver name      : rp1-cfe
	Model            : rp1-cfe
	Serial           : 
	Bus info         : platform:1f00128000.csi
	Media version    : 6.8.12
	Hardware revision: 0x00114666 (1132134)
	Driver version   : 6.8.12
Interface Info:
	ID               : 0x03000014
	Type             : V4L Video
Entity Info:
	ID               : 0x00000012 (18)
	Name             : rp1-cfe-csi2_ch0
	Function         : V4L2 I/O
	Pad 0x01000013   : 0: Sink, Must Connect
	  Link 0x02000034: from remote pad 0x1000006 of entity 'csi2' (Video Interface Bridge): Data
Priority: 2
Video input : 0 (rp1-cfe-csi2_ch0: ok)
Format Video Capture:
	Width/Height      : 640/480
	Pixel Format      : 'pRAA' (10-bit Bayer RGRG/GBGB Packed)
	Field             : None
	Bytes per Line    : 800
	Size Image        : 384000
	Colorspace        : Raw
	Transfer Function : None
	YCbCr/HSV Encoding: ITU-R 601
	Quantization      : Full Range
	Flags             : 
Format Metadata Capture:
	Sample Format   : 'SENS' (Sensor Ancillary Metadata)
	Buffer Size     : 16384
```	
```bash
v4l2-ctl --device=/dev/video8 --all
```
```text
Driver Info:
	Driver name      : rp1-cfe
	Card type        : rp1-cfe
	Bus info         : platform:1f00110000.csi
	Driver version   : 6.8.12
	Capabilities     : 0xaca00001
		Video Capture
		Metadata Capture
		Metadata Output
		I/O MC
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x24a00001
		Video Capture
		Metadata Capture
		I/O MC
		Streaming
		Extended Pix Format
Media Driver Info:
	Driver name      : rp1-cfe
	Model            : rp1-cfe
	Serial           : 
	Bus info         : platform:1f00110000.csi
	Media version    : 6.8.12
	Hardware revision: 0x00114666 (1132134)
	Driver version   : 6.8.12
Interface Info:
	ID               : 0x03000014
	Type             : V4L Video
Entity Info:
	ID               : 0x00000012 (18)
	Name             : rp1-cfe-csi2_ch0
	Function         : V4L2 I/O
	Pad 0x01000013   : 0: Sink, Must Connect
	  Link 0x02000034: from remote pad 0x1000006 of entity 'csi2' (Video Interface Bridge): Data
Priority: 2
Video input : 0 (rp1-cfe-csi2_ch0: ok)
Format Video Capture:
	Width/Height      : 640/480
	Pixel Format      : 'pRAA' (10-bit Bayer RGRG/GBGB Packed)
	Field             : None
	Bytes per Line    : 800
	Size Image        : 384000
	Colorspace        : Raw
	Transfer Function : None
	YCbCr/HSV Encoding: ITU-R 601
	Quantization      : Full Range
	Flags             : 
Format Metadata Capture:
	Sample Format   : 'SENS' (Sensor Ancillary Metadata)
	Buffer Size     : 16384
```

### Fixing Ubuntu 24.04.2 drivers:

Check for these problematic packages: 
```bash
dpkg -l | grep libcamera
```
```text
ii  gstreamer1.0-libcamera:arm64                      0.2.0-3fakesync1build6                   arm64        complex camera support library (GStreamer plugin)
ii  libcamera-tools                                   0.2.0-3fakesync1build6                   arm64        complex camera support library (tools)
ii  libcamera0.2:arm64                                0.2.0-3fakesync1build6                   arm64        complex camera support library
ii  ros-kilted-camera-ros                             0.4.0-2noble.20250607.023522             arm64        node for libcamera supported cameras (V4L2, Raspberry Pi Camera Modules)
ii  ros-kilted-libcamera                              0.5.0-4noble.20250430.201904             arm64        An open source camera stack and framework for Linux, Android, and ChromeOS
```

### Purge and Rebuild

```bash
sudo apt purge libcamera* ros-kilted-libcamera gstreamer1.0-libcamera
sudo apt autoremove
sudo apt install python3-ply
```

```bash
mkdir -p ~/src
cd ~/src
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/pisp
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

### Potenetial Issues

Group:
```bash
sudo usermod -aG video $USER
newgrp video
```

DMA Heap Device:
```bash
ls /dev/dma_heap
```

If you do not see: `cma  linux,cma  system`
```bash
sudo ln -s /dev/dma_heap/linux,cma /dev/dma_heap/cma
```

Fixing DMA Permissions
```bash
sudo chown root:video /dev/dma_heap/*
sudo chmod 0660 /dev/dma_heap/*
```

Setup udev for a permanent fix:
```bash
/etc/udev/rules.d/99-dma-heap.rules
```
```bash
KERNEL=="dma_heap", NAME="dma_heap/%k", GROUP="video", MODE="0660"
```

reload:
```bash
sudo udevadm control --reload
sudo udevadm trigger
```

