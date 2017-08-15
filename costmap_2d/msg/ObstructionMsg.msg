# The frame in which the point is expressed
string frame_id
# The center of the obstruction
float32 x
float32 y

# Time it was first seen and last time it was seen
time last_sighting_time
time first_sighting_time

# Indicate if the obstruction has been cleared
bool cleared

# Indicate if the obstruction has been updated
bool updated

# The effective radius (how much the costmap is affected by this obstruction)
float32 effective_radius
# Max cost within the kernel
uint8 max_cost

# The type of the obstruction
uint8 type
uint8 STATIC = 0
uint8 DYNAMIC = 1
uint8 PSEUDOSTATIC = 2
