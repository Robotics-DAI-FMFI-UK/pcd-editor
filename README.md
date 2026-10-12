
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

5. Download `stduuid` into project folder
	```bash
	git clone --depth=1 https://github.com/mariusbancila/stduuid external/stduuid
	```

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
