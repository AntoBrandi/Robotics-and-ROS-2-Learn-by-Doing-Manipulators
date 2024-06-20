# Robotics and ROS 2 - Learn by Doing! Manipulators
[![LinkedIn][linkedin-shield]][linkedin-url]
[![Udemy][udemy-shield]][udemy-url]


<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="images/cover_manipulators.png" alt="Cover Manipulators">
</p>


<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Course](#about-the-course)
* [Other Courses](#other-courses)
   * [Self Driving and ROS 2 - Learn by Doing! Odometry & Control](#odometry-control)
   * [Self Driving and ROS 2 - Learn by Doing! Map & Localization](#map-localization)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Usage](#usage)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)


<!-- ABOUT THE COURSE -->
## About the Course
This repository contain the material used in the course **Robotics and ROS 2 - Learn by Doing! Manipulators** that is currently available on the following platforms:

* [Udemy](https://www.udemy.com/course/robotics-and-ros-learn-by-doing-manipulators/?couponCode=LEARNBYDOING)

In this course I'll guid you through the creation of a real robotic arm that you can control with your voice using the Amazon Alexa voice assistant.
Some of the concepts that are covered in this course are

* Gazebo Simulation
* Robot Kinematics
* ROS 2 Basics
* ros2_control Library
* MoveIt! 2
* Using Arduino with ROS 2
* Interface Alexa with ROS 2

Furthermore, all the laboratory classes in which we are going to develop the actual Software of our mobile robot are available both in **Pyhton** and in **C++** to let you have the freedom of choosing the programming language you like the most or become proficient in both!


<!-- OTHER COURSES -->
### Other Courses
If you find this course interesting and you are passionate about robotics in general (not limited to manipulators), then you definitely have to take a look at my outher courses!

#### Self Driving and ROS 2 - Learn by Doing! Odometry & Control
<br />
<p align="center">
   <img src="images/cover_odometry_control.png" alt="Cover Odometry Control">
</p>

If you are passionate about Self-Driving and you want to make a real robot Autonomously Navigate, then this course is for you! 
Apart from explaining in detail all the functionalities and the logic of **ROS 2**, the latest version of the Robot Operating System, it covers some key concepts of Autonomous Navigation such as

* Sensor Fusion
* Kalman Filter
* Probability Theory
* Robot Kinematics
* Odometry
* Robot Localization
* Control

Looks funny? Check it out on the following platforms:
* [Udemy](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/?referralCode=50BCC4E84DB2DB09BFB3)

#### Self Driving and ROS 2 - Learn by Doing! Map & Localization
<br />
<p align="center">
   <img src="images/cover_map_localization.png" alt="Cover Map Localization">
</p>
Have you ever developed a mapping and a localization algorithm for your robot?
Do you want to know more about SLAM (Simultaneous Localization and Mapping) and how to use it to enable your robot to create a nice and accurate map of the environment using a 2D LiDAR sensor?

Then this course will teach you exaclty that, with many more topics:
* Robot Localization
* Map Representations
* Mapping
* SLAM
* Obstacle Avoidance
* Speed and Separation monitoring
* Using LiDAR Sensors

enroll on the following platforms:
<br />
<p align="center">
   <img src="images/cover_map_localization.png" alt="Cover Map & Localization">
</p>

* [Udemy](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-map-localization/?referralCode=8FC4AC725C57F7A93F79)

#### ROS 1 Nostalgic?

Do you want to master Self-Driving or Manipulation using **ROS**, the first version of the Robot Operating System? 

Despite many companies already started switching to ROS 2, knowing both ROS 1 and ROS 2 will position you at the forefront of this demand, making you an attractive candidate for a wide range of roles.

Here you can access the same courses, where will be created the same robots, implementing the same functionalities in ROS 1

* [Self Driving and ROS - Learn by Doing! Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-learn-by-doing-odometry-control/?referralCode=5B857932D7C6FE9D014D)

* [Robotics and ROS - Learn by Doing! Manipulators](https://www.udemy.com/course/robotics-and-ros-learn-by-doing-manipulators/?referralCode=6EDAA8501C5E3CCEE526)


<!-- GETTING STARTED -->
## Getting Started
You can decide whether to build the real robot or just have fun with the simulated one. The course can be followed either way, most of the lessons and most of the code will work the same in the simulation as in the real robot

### Prerequisites
You don't need any prior knowledge of ROS 2 or of Robotics, I'll explain all the concepts as they came out and as they are needed to implement new functionalities to our robot.
A basic knowledge of programming, either using **C++** or **Python** is required as this is not a Programming course and so I'll not dwell too much on basic Programming concepts.

To prepare your PC you need:
* Install Ubuntu 22.04 on PC or in Virtual Machine
Download the ISO [Ubuntu 22.04](https://ubuntu.com/download/desktop) for your PC
* Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 22.04
* Install ROS 2 missing libraries. Some libraries that are used in this project are not in the standard ROS package. Install them with:
```sh
sudo apt-get update && sudo apt-get install -y \
     ros-humble-joint-state-publisher-gui \
     ros-humble-gazebo-ros \
     ros-humble-xacro \
     ros-humble-ros2-control \
     ros-humble-moveit \
     ros-humble-ros2-controller \
     ros-humble-gazebo-ros2-control 
```
* Install VS Code and Arduino IDE on your PC in order to build and load the Arduino code on the device
* Install Python and C++ additional libraries
```sh
sudo apt-get update && sudo apt-get install -y \
     libserial-dev \
     python3-pip
```

```sh
pip install pyserial
```


### Installation

1. Clone the repo
```sh
git clone https://github.com/AntoBrandi/Robotics-and-ROS-2-Learn-by-Doing-Manipulators.git
```
2. Build the ROS 2 workspace
```sh
cd ~/Robotics-and-ROS-2-Learn-by-Doing-Manipulators/Section9-Build/arduinobot_ws
```
```sh
colcon build
```
3. Source the project
```sh
. install/setup.bash
```

<!-- USAGE EXAMPLES -->
## Usage

To launch the ROS 2 **Simulated robot**
```sh
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

To launch the **Real robot**, connect the Arduino to the PC and upload the code in the [folder](https://github.com/AntoBrandi/Robotics-and-ROS-2-Learn-by-Doing-Manipulators/blob/humble/Section9_Build/arduinobot_ws/src/arduinobot_firmware/firmware/robot_control/robot_control.inol) on the Arduino controller.
Then launch the real robot
```sh
ros2 launch arduinobot_bringup real_robot.launch.py
```

To launch the interface with Alexa download [ngrok](https://ngrok.com/download) and create an [account](https://dashboard.ngrok.com/signup) then setup ngrok with your key
```sh
./ngrok authtoken <YOUR-KEY>
```
Then start the ngrok web server with
```sh
./ngrok http 5000
```
Copy the link that provides ngrok and paste it in the section Endpoint of your Alexa Developer account


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request


<!-- LICENSE -->
## License

Distributed under the Apache 2.0 License. See `LICENSE` for more information.


<!-- CONTACT -->
## Contact

Antonio Brandi - [LinkedIn]([linkedin-url]) - antonio.brandi@outlook.it

My Projects: [https://github.com/AntoBrandi](https://github.com/AntoBrandi)


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Arduino 3D-Printed Robotic Arm](https://create.arduino.cc/projecthub/mircemk/arduino-3d-printed-robotic-arm-e824d8?ref=search&ref_id=robot%20arm&offset=86)
* [EEZYbotARM](https://www.thingiverse.com/thing:1015238)


<!-- MARKDOWN LINKS & IMAGES -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/antonio-brandi-512166bb/
[udemy-shield]: https://img.shields.io/badge/-Udemy-black.svg?style=flat-square&logo=udemy&colorB=555
[udemy-url]: https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/?referralCode=4B27D2CF97C1E099DD4C
