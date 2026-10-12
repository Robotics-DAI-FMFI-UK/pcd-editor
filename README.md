
# OpenGL  + ImGUI visualiser

This project focuses on establishing display utilities for various data types related to Lidar and SLAM.

## Setup

This project depends on OpenGL v2.x, GLFW and ImGUI.

### Prerequisites

1. Install `OpenGL` and its components
	```bash
	sudo apt install libglu1-mesa libglu1-mesa-dev libgl1-mesa-dev libglfw3-dev libglm-dev
	```

2. Download `ImGUI` into project folder
	```bash
	git clone --depth=1 https://github.com/ocornut/imgui.git external/imgui
	```

3. Install Point cloud library (PCL) system-wide
	```bash
	sudo apt install libpcl-dev libeigen3-dev
	```

4. Download `Native File Dialogs` into project folder and install `zenity`
	```bash
	git clone --depth=1 https://github.com/mlabbe/nativefiledialog.git external/nfd
	sudo apt install zenity
	```
	***note:** in VS Code on Linux, for `zenity` to work, the environment variable `GTK_PATH` must be unset (VSCode sets it and it breaks `zenity`)*

5. Add `i9.pcd` (downloaded from [unitree wiki](https://dai.fmph.uniba.sk/projects/unitree/i9.pcd)) to the project root
	```bash
	wget https://dai.fmph.uniba.sk/projects/unitree/i9.pcd
	```

### Compiling the project

With the `makefile` setup, to compile the project is just

```bash
make
```

which creates `./build` directory containing `*.o` files.

### Running the project

To run the application, execute the binary at project folder root

```bash
./main
```

### Controls

Press `C` to toggle map-pose selection mode. While it is enabled, mouse camera
controls are disabled and the overlay shows the original PCD map `X` and `Y`
coordinates under the cursor.

Drag the left mouse button from a desired location toward the robot's desired
forward direction. The viewer draws an arrow and, on release, freezes the
selected position, yaw and quaternion in `q_x q_y q_z q_w` order. The
quaternion is expressed in the Unitree/map convention used by `places.txt`:
`(cos(yaw/2), sin(yaw/2), 0, 0)` in `q_x q_y q_z q_w` order. This combines
the selected map yaw with the fixed 180-degree X-axis body-frame rotation.
On release, a valid pose is copied to the clipboard as one ready-to-paste
`x y z q_x q_y q_z q_w` line and printed to the console as a fallback. The
generated pose uses `z = 0`, the nominal robot/lidar pose origin in the map;
the mouse intersection itself lies on the physical floor roughly one lidar
mounting height below that origin. Click once to clear the selection and return
to live coordinate inspection; press `C` to restore normal camera controls.
