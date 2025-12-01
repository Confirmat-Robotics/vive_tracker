# vive_ros2

ROS 2 package that streams pose data from a Vive Tracker socket server and republishes it as standard ROS 2 messages.

## Prerequisites
- ROS 2 Humble (or later) with `rclpy`, `nav_msgs`, and `tf2_ros` available
- `colcon` build tool and Python 3.8+
- Python dependencies: `pydantic`, `scipy`
- Windows PowerShell (commands below) or your preferred shell sourced with ROS 2

## Install System Dependencies
From the workspace root (this folder), resolve ROS 2 dependencies:

```powershell
rosdep install --from-paths . --ignore-src -y
```

If `rosdep` cannot install Python packages automatically, install them manually, for example:

```powershell
pip install pydantic scipy
```

## Build the Package
1. Navigate to the workspace root (`vive_ros2`).
2. Build with `colcon`:

```powershell
colcon build --packages-select vive_ros2
```

3. Source the overlay so ROS 2 can discover the new executables:

```powershell
call install\setup.bat
```

(Use `source install/local_setup.bash` on Linux shells.)

## Run the Nodes
- Start the streaming node:

```powershell
ros2 run vive_ros2 vive_tracker_node --ros-args -p host_ip:=192.168.50.171 -p tracker_name:=T_1
```

- Run the standalone client for debugging or recording:

```powershell
ros2 run vive_ros2 vive_tracker_client -- --debug true
```

Parameters published by the node:
- `host_ip`/`host_port`: address of the Vive tracker UDP server
- `tracker_name`: name of the tracker to request
- `topic`: override for the Odometry topic (defaults to `<tracker_name>/odom`)
- `link_name`/`child_link_name`: frame IDs used in the Odometry message

## Development Tips
- Use `colcon test --packages-select vive_ros2` (when tests are added).
- Format and lint Python with `ruff`/`black` if you introduce them.
- Update `setup.py` and `package.xml` whenever you add new runtime dependencies.

## Repository Layout
```
vive_ros2/
├── README.md
├── package.xml
├── CMakeLists.txt
├── setup.py
├── vive_ros2/            # Python package with nodes and helpers
├── scripts/              # Thin wrappers for direct execution
└── utils/                # Analysis scripts and notebooks
```
