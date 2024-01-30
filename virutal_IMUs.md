# Virtual IMUs in PyBullet

Jan 30th 2024, Written by Felipe Galindo
</br>Note: This is a work in progress</br>
## Introduction

PyBullet is a physics simulator that can be used to simulate robots in a virtual physics environment. It is a very useful tool for testing and developing algorithms for robotics.

The Spot mini mini project implements a virtual [BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/overview) IMU in PyBullet that is used to modulate the Bezier curves that characterize the robot's gait. Using this IMU, the robot can tell if it is out of balance and adjust its gait such that it can recover its balance and maintain the desired trajectory and robot pose.

The same algorithm can be used in the real robot, but the IMU is replaced by a real IMU. Spot mini mini has proved that is is possible to train a ML model in a simulated robot and then transfer the model to the real robot with minimal modifications. 

For a Quadruped Robot to have the capability to transverse rough terrain, ideally, it should have the ability accurately estimate the pose of the robot, the movement of the robot relative to the environment, and the shape of the surface that the robot is walking on. This information can be used to plan the robot's trajectory and gait.

A singular IMU like the [BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/overview), which was used in Spot mini mini, is not enough to estimate the pose of the robot. However, it can be used to estimate the movement of the robot relative to the ground. This information can be used to modulate the robot's gait and trajectory to maintain balance and recover from falls, but we do not have a good way to estimate the *real* pose of the robot other than the angle positions that are being sent to the servos. 


The *real* angle of the servos is never the same as the *desired* angle of the servos. Lag, play in the gears of the servos, misalignments, bending of the legs, and other factors cause the *real* angle to be different from the *desired* angle. 

The solution to this problem ofter is to include an encoder in the joints of the robot, that way you can accurately measure the *real* angle of the servos(or whatever motor used in the joints). However, this solution is not ideal for a quadruped robot. The quadruped robot has 12 joints, and each joint would need an encoder. This would add a lot of weight to the robot, and it would be very expensive.

For example, the [Stanford Pupper V2](https://pupper-independent-study.readthedocs.io/en/latest/) uses 12x DJI [m2006](https://store.dji.com/product/rm-m2006-p36-brushless-motor?vid=40031) & [c610](https://store.dji.com/product/rm-c610-brushless-dc-motor-speed-control?vid=40021) brusheless motors with encoders. This motor+ESC combo with encoder costs about $100 per pair, or about $1200 for the 12 motors. This is a lot of money for a hobbyist, and not a very good value since motors *without* encoders, but similar specs, can be found for $50 each or $600 for the 12 motors.

Without encoders in the joints, as would be the case if we used servos, we need to find a way to estimate the *real* angle of the servos. 

For this, we can take advantage of the development of MEMS IMUs, oriented towards consumer electronics like smartphones and tablets, these IMUs are very cheap, and they are very accurate. 
Companies like Google and Apple have taken advantage of these IMUs to develop algorithms that can, among other things, determine if you are on a car accident, if you are walking, running, or biking, taking the bus. Estimate the location of the phone *without* continuous GPS signal. Etc, etc.

Micro Electronic Micro Systems (MEMS) IMUs like the MPU6050, MPU9250, BNO055, [BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/overview), etc all work in a similar way. They have a 3-axis accelerometer, a 3-axis gyroscope, a optional 3-axis magnetometer, and a optional 'motion-coprocessor'([like the Apple motion coprocessors](https://en.wikipedia.org/wiki/Apple_motion_coprocessors)).

The accelerometer measures the acceleration of the IMU in the 3 axis, the gyroscope measures the angular velocity of the IMU in the 3 axis, and the magnetometer measures the magnetic field in the 3 axis.

The MPU6050 does not have a magnetometer or a motion coprocessor, but it is still possible to perform sensor fusion of the accelerometer and gyroscope to estimate the orientation of the IMU, or better yet, the data from 4 MPU6050s one each on the tips of the legs, one in the middle of the body a BMI270 IMU in the front of the body and a MPU-9250 IMU in the back of the body can be fused to estimate the pose of the robot.

The MPU9250 and BN085 both have additional 3-axis magnetometers, and the BNO085 has a motion coprocessor that performs sensor fusion of the accelerometer, gyroscope, and magnetometer. 

The highlight of this project is to take the ML model that was to trained to receive the data from the IMU in the center of the robot and increase the dimensionality of the input data to include the data from an additional 6 IMUs, 4 on the tips of the legs, one in the front of the body, and one in the back of the body.

### The Virtual IMU in spot mini mini

The virtual IMU in spot mini mini is implemented in the `spot_mini_mini.py` file. The IMU is implemented as a class that is initialized with the `pybullet` client and the `spot_mini_mini` robot object. The IMU class has a `get_data()` method that returns the data from the IMU. The data from the IMU is the acceleration, angular velocity, and orientation of the IMU in the world frame.

### Real vs Virtual IMU