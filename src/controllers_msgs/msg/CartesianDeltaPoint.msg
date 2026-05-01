# One Cartesian delta sample for Cartesian delta control.
#
# translation is [dx, dy, dz] in meters.
# rotation is [droll, dpitch, dyaw] in radians and is accumulated into the
# Servo pose target around x/y/z in command_frame.
# gripper is forwarded as the raw DH gripper target position.

geometry_msgs/Vector3 translation
geometry_msgs/Vector3 rotation
float64 gripper
