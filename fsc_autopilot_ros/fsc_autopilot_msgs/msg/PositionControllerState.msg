std_msgs/Header header

# The desired position/velocity
geometry_msgs/Point position_reference
geometry_msgs/Vector3 velocity_reference

# The current vehicle absolute position/velocity/acceleration
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

# Absolute position error
geometry_msgs/Vector3 position_error

# Absolute velocity error
geometry_msgs/Vector3 velocity_error

# Current output of the position controller
geometry_msgs/Vector3 output
