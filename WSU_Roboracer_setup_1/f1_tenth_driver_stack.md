---
sort: 3
---

# F1 Tenth Driver Stack

## Create a workspace
```bash
cd $HOME
mkdir -p f1tenth_ws/src
```

## move to workspace

```bash
cd f1tenth_ws
colcon build
```

## clone the repo

```bash
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
```


## update submodule

```bash
cd f1tenth_system
git submodule update --init --force --remote
```

# Change Branches to Humble

```bash
git switch humble-devel
cd teleop_tools
git switch humble-devel
cd ..
cd vesc
git switch humble

```
## Install dependencies

```
cd $HOME/f1tenth_ws
sudo rosdep init
```

```
rosdep update
```

## Install Missing Dependencies

Ensure the missing `asio_cmake_module` dependency is installed. Use `rosdep` to install all necessary dependencies for the `vesc_driver` package:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

### **2. Check Your ROS 2 Installation**

Verify that the required ROS 2 packages (`io_context` and `asio_cmake_module`) are installed:

```bash
sudo apt update && sudo apt install ros-humble-io-context ros-humble-asio-cmake-module
```

## Build

```
colcon build --symlink-install
source ~/.bashrc
```