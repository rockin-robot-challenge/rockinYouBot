rostopic pub -r 10 /rockin_manipulation/rockin_arm_cartesian_control/cartesian_velocity_command geometry_msgs/TwistStamped '{header: {frame_id: /base_link}, twist: {linear: {z: -0.05}}}'
