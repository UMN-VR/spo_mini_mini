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
The virtual IMU in spot mini mini is implemented in the `spot.py` file. The function `GetObservation` is called every `step` of the simulation. The function returns the data from the IMU in the center of the robot. The data from the IMU is used to modulate the Bezier curves that characterize the robot's gait.

`spot.py`:616-710 
```python
    def GetObservation(self):
        """Get the observations of minitaur.
        It includes the angles, velocities, torques and the orientation of the base.
        Returns:
          The observation list. observation[0:8] are motor angles. observation[8:16]
          are motor velocities, observation[16:24] are motor torques.
          observation[24:28] is the orientation of the base, in quaternion form.
          NOTE: DIVERGES FROM STOCK MINITAUR ENV. WILL LEAVE ORIGINAL COMMENTED
          For my purpose, the observation space includes Roll and Pitch, as well as
          acceleration and gyroscopic rate along the x,y,z axes. All of this
          information can be collected from an onboard IMU. The reward function
          will contain a hidden velocity reward (fwd, bwd) which cannot be measured
          and so is not included. For spinning, the gyroscopic z rate will be used
          as the (explicit) velocity reward.
          This version operates without motor torques, angles and velocities. Erwin
          Coumans' paper suggests a sparse observation space leads to higher reward

          # NOTE: use True version for perfect data, or other for realistic data
        """
        observation = []
        # GETTING TWIST IN BODY FRAME
        pos = self.GetBasePosition()
        orn = self.GetBaseOrientation()
        roll, pitch, yaw = self._pybullet_client.getEulerFromQuaternion(
            [orn[0], orn[1], orn[2], orn[3]])
        # rpy = LA.RPY(roll, pitch, yaw)
        # R, _ = LA.TransToRp(rpy)
        # T_wb = LA.RpToTrans(R, np.array([pos[0], pos[1], pos[2]]))
        # T_bw = LA.TransInv(T_wb)
        # Adj_Tbw = LA.Adjoint(T_bw)

        # Get Linear and Angular Twist in WORLD FRAME
        lin_twist, ang_twist = self.GetBaseTwist()

        lin_twist = np.array([lin_twist[0], lin_twist[1], lin_twist[2]])
        ang_twist = np.array([ang_twist[0], ang_twist[1], ang_twist[2]])

        # Vw = np.concatenate((ang_twist, lin_twist))
        # Vb = np.dot(Adj_Tbw, Vw)

        # roll, pitch, _ = self._pybullet_client.getEulerFromQuaternion(
        #     [orn[0], orn[1], orn[2], orn[3]])

        # # Get linear accelerations
        # lin_twist = -Vb[3:]
        # ang_twist = Vb[:3]
        lin_acc = lin_twist - self.prev_lin_twist
        if lin_acc.all() == 0.0:
            lin_acc = self.prev_lin_acc
        self.prev_lin_acc = lin_acc
        # print("LIN TWIST: ", lin_twist)
        self.prev_lin_twist = lin_twist
        self.prev_ang_twist = ang_twist

        # Get Contacts
        CONTACT = list(self._pybullet_client.getContactPoints(self.quadruped))

        FLC = 0
        FRC = 0
        BLC = 0
        BRC = 0

        if len(CONTACT) > 0:
            for i in range(len(CONTACT)):
                Contact_Link_Index = CONTACT[i][3]
                if Contact_Link_Index == self._foot_id_list[0]:
                    FLC = 1
                    # print("FL CONTACT")
                if Contact_Link_Index == self._foot_id_list[1]:
                    FRC = 1
                    # print("FR CONTACT")
                if Contact_Link_Index == self._foot_id_list[2]:
                    BLC = 1
                    # print("BL CONTACT")
                if Contact_Link_Index == self._foot_id_list[3]:
                    BRC = 1
                    # print("BR CONTACT")
        # order: roll, pitch, gyro(x,y,z), acc(x, y, z)
        observation.append(roll)
        observation.append(pitch)
        observation.extend(list(ang_twist))
        observation.extend(list(lin_acc))
        # Control Input
        # observation.append(self.StepLength)
        # observation.append(self.StepVelocity)
        # observation.append(self.LateralFraction)
        # observation.append(self.YawRate)
        observation.extend(self.LegPhases)
        if self.contacts:
            observation.append(FLC)
            observation.append(FRC)
            observation.append(BLC)
            observation.append(BRC)
        # print("CONTACTS: {}  {}  {}  {}".format(FLC, FRC, BLC, BRC))
        return observation

```
Two functions are called by `GetObservation` to get the linear and angular twist of the robot. 


```python
orn = self.GetBaseOrientation()

```

```python
roll, pitch, yaw = self._pybullet_client.getEulerFromQuaternion([orn[0], orn[1], orn[2], orn[3]])
```


### Real vs Virtual IMU