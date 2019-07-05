# imu_driver
An imu driver

topic /imu_bosch/data
data:
+ orientation.w
+ orientation.x
+ orientation.y
+ orientation.z
+ linear_acceleration.x (m/s^2)
+ linear_acceleration.y
+ linear_acceleration.z
+ angular_velocity.x (rad/s)
+ angular_velocity.y
+ angular_velocity.z

topic /imu_bosch/raw
data:
+ linear_acceleration.x (m/s^2)
+ linear_acceleration.y
+ linear_acceleration.z
+ angular_velocity.x (rad/s)
+ angular_velocity.y
+ angular_velocity.z
