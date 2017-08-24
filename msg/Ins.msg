Header header
float64 measurement_time

uint32 INVALID=0
uint32 CONVERGING=1
uint32 GOOD=2

uint32 type
PointLLH position
Point3D euler_angles
Point3D linear_velocity
Point3D angular_velocity
Point3D linear_acceleration

float32[] position_covariance
float32[] euler_angles_covariance
float32[] linear_velocity_covariance
float32[] angular_velocity_covariance
float32[] linear_acceleration_covariance
