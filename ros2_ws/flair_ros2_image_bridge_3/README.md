# FLAIR -> ROS2: streaming the vertical camera

Protocol (intentionally minimal): FLAIR connects over TCP to the ROS2 node,
sends `width` + `height` once (2x uint32), then loops sending raw BGR
frames (`width*height*3` bytes, no per-frame header). No velocity/pose is
sent, image only (per the need expressed in the project meeting).

CircleFollower's simulated vertical camera (core2-64/x4): 320x240, BGR
format (3 bytes/pixel), verified in `SimuX4.cpp`/`SimulatedCamera.cpp`.

See `flair_to_ros2_tutorial.md` for the full step-by-step guide, including
the two build pitfalls below and a troubleshooting table.

## 1. FLAIR side (~/flair/flair-src)

Copy these 2 new files into `demos/CircleFollower/uav/src/`:
- `ImageStreamer.h`
- `ImageStreamer.cpp`

Replace these 4 existing files with the ones provided (they just add the
`-s/--streaming` CLI option and the `ImageStreamer` creation):
- `demos/CircleFollower/uav/src/CircleFollower.h`
- `demos/CircleFollower/uav/src/CircleFollower.cpp`
- `demos/CircleFollower/uav/src/main.cpp`
- `demos/CircleFollower/uav/CMakeLists.txt`

Then rebuild **and install**: both steps are required, `flairrun` executes
the installed binary, not the one under `flair-build/`:

```bash
flairmake all
cd $FLAIR_ROOT/flair-build/build_core2_64
make install
```

## 2. ROS2 side (`flair_image_bridge` folder)

Copy the `flair_image_bridge/` folder into `~/ros2_ws/src/`, then:

```bash
cd ~/ros2_ws
colcon build --packages-select flair_image_bridge
source install/setup.bash
```

Required dependencies (normally already present with a ROS2 desktop
install): `rclpy`, `sensor_msgs`, `cv_bridge`, `python3-numpy`.

## 3. Run (order matters)

The ROS2 node must be listening **before** FLAIR tries to connect;
`ImageStreamer` blocks on connect otherwise (a failed connect is only a
warning and does not stop the demo, but no images will stream until FLAIR
is relaunched with the node already up).

Terminal 1:
```bash
ros2 run flair_image_bridge image_receiver_node
```

Terminal 2 (once the ROS2 node is running):
```bash
flairrun CircleFollower
```
(the `-s` option defaults to `127.0.0.1`, FLAIR and ROS2 on the same
machine; otherwise pass `-s <ros2_machine_ip>` in the drone's launch `.sh`
if needed).

## 4. Verify

```bash
ros2 topic hz /flair/image_raw
ros2 run rqt_image_view rqt_image_view   # click refresh, then select /flair/image_raw
```

## 5. Use cases (demos)

Two optional consumer nodes are included in `flair_image_bridge` (the base
pipeline above works whether or not they are running):

```bash
# dense optical flow, drawn as motion arrows, on /flair/optical_flow_view
ros2 run flair_image_bridge optical_flow_demo_node

# records incoming frames + a manifest.csv under ~/flair_datasets/run_<date>/
ros2 run flair_image_bridge dataset_recorder_node
```

See `flair_to_ros2_tutorial.md` §7 for details on each.