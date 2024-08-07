std_msgs/Header header

# The desired attitude/rotation of the desired body frame w.r.t. the inertial
# frame
geometry_msgs/Quaternion reference

# The current attitude
geometry_msgs/Quaternion feedback

# Rotation of the body frame w.r.t. the desired body frame
geometry_msgs/Quaternion attitude_error

# The 3-component rotational/angular error
geometry_msgs/Vector3 error

# The feedforward angular velocity 
geometry_msgs/Vector3 rate_feedforward

# Current output of the attitude controller
geometry_msgs/Vector3 output
