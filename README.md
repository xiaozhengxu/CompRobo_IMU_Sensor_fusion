# CompRobo_IMU_Sensor_fusion
This is our final project for Computational Robotics class to incorporate a razor IMU sensor to improve the neato's wheel odometry. 

Link to website:
https://xiaozhengxu.github.io/CompRobo_IMU_Sensor_fusion/

# Phase 1

This week our goal was to read IMU data from the arduino, pass it through the pi and publish the data as an IMU message on ROS. 

First, we learned about the neato’s software structure, as shown in the diagram below.

![System Diagram](media/system.png)

The IMU has an ATmega 328P microcontroller, which is flashed with the Arduino bootloader. This communicates with the raspi through an FTDI USB-UART converter. On the raspi there is code to read from the arduino through serial and send the data through a web socket to the laptop. 

We were in charge of writing the bridge that takes the data sent from the raspi and publishes it to ROS. To do that, we referenced and used the code on https://github.com/KristofRobot/razor_imu_9dof as a starting point. The imu_node.py in this repo read the imu data directly from serial and published it to ROS. What we had to change was to read from a web socket instead. 

In that process, first we learned about UDP and TCP communication protocols. We decided to establish a connection between the raspi and laptop with TCP and then use the IP of the laptop address to send the data packets in UDP. The advantage of UDP should have been that it sends the entire data in one pack, but that wasn’t the case for us, so we had to write our own receive all function. 
After some more editing the code we were able to publish the IMU data to ROS! 

![IMU to ROS](media/imu_visulization_2.gif)

We also changed imu_node.py to follow an object oriented code architecture. 

In order to get odometry and IMU data at the same time, we are required to run a bunch of ROS nodes all at the same time, a perfect use case for launch files! Our launch file takes a “host” argument which we pass to the launch structure from the neato node to establish the standard connection to the robot. We also start up our IMU node, which established the connection between the raspi and IMU, then begins publishing data. We briefly experimented with also starting a Robot Pose EKF node, but decided that the point of this project is to learn about Kalman filters, not just use one that is already built. 

We had some issues running both the neato node and IMU node at the same time, which resulted in the serial communication to both the neato and IMU failing. We’re currently resolving this by plugging in the IMU after we have started up the neato. 

Now that we can access both Odometry data from the neato’s wheels and imu data, we are ready to start implementing an extended Kalman filter! 
