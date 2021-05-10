# An occupancy belief map with frame and scale
Header header

# Distance between voxel centers
float32 scale
geometry_msgs/Point origin

# Occupancy probability
# For binary thresholding, use:
# 0.0 <= x <  0.5 --> 0
# 0.5 <= x <= 1.0 --> 1
# For ternary thresholding (free, unknown, occupied), use:
# 0.0     <= x <  0.33... --> 0
# 0.33... <= x <  0.66... --> 0.5
# 0.66... <= x <= 1.0     --> 1
std_msgs/Float32MultiArray occupancy

std_msgs/ColorRGBA color
bool has_color
