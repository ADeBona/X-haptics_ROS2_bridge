# kinova_haptic_teleop

ROS 2 Humble package for a wearable pneumatic haptic interface (chi-Haptics)
that renders simulated Kinova arm torque as skin-stretch pressure via an
Arduino-driven pneumatic pad.

## Nodes
- `fake_torque_pub`: keyboard-driven simulated F/T sensor, publishes Float32
  torque (-5.0 to +5.0 Nm) on `/kinova/sim_torque`.
- `kinova_haptic_bridge`: subscribes to `/kinova/sim_torque`, maps |torque|
  to a 0-50 kPa pressure target, streams it to the Arduino over serial.

## Build & Run

    cd docker
    docker compose up -d --build
    docker exec -it kinova_haptic_humble bash

    cd /ros2_ws
    colcon build
    source install/setup.bash

    # Terminal 1
    ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args -p serial_port:=/dev/ttyACM0

    # Terminal 2 (new docker exec into same container)
    ros2 run kinova_haptic_teleop fake_torque_pub