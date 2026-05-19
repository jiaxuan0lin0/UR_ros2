# One TCP delta sample using UR rotvec policy semantics.
#
# translation is [dx, dy, dz] in meters in command_frame.
# rotvec is the axis-angle rotation delta [drx, dry, drz] in radians in
# command_frame, applied as R_next = exp(rotvec) * R_current.
# gripper is forwarded as the raw DH gripper target position.

geometry_msgs/Vector3 translation
geometry_msgs/Vector3 rotvec
float64 gripper
