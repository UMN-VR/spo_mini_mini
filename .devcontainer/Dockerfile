# FROM osrf/ros:melodic-desktop-full
FROM ros:melodic-perception

# Install P
# compile python from source - avoid unsupported library problems 
RUN apt update -y && sudo apt upgrade -y && \
    apt-get install -y wget build-essential checkinstall mesa-utils libreadline-gplv2-dev  libncursesw5-dev  libssl-dev  libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev && \
    cd /usr/src && \
    sudo wget https://www.python.org/ftp/python/3.10.0/Python-3.10.0.tgz && \
    sudo tar xzf Python-3.10.0.tgz && \
    cd Python-3.10.0 && \
    sudo ./configure --enable-optimizations && \
    sudo make altinstall


# install pip
RUN sudo wget https://bootstrap.pypa.io/get-pip.py && \
    sudo python3.10 get-pip.py && \
    sudo rm get-pip.py

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc



# set $DISPLAY env variable to ':1'
RUN echo "export DISPLAY=:1" >> ~/.bashrc

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create a catkin workspace

RUN mkdir -p ~/catkin_ws/src

# install joint_state_publisher_gui

RUN sudo apt install -y ros-melodic-joint-state-publisher-gui

#RUN source /home/vscode/catkin_ws/src/spot_mini_env/bin/activate
# source the virtual environment 
RUN echo "source ~/catkin_ws/src/spot_mini_env/bin/activate" >> ~/.bashrc




# Install dependencies for i2c and the Adafruit libraries
RUN sudo apt-get update && sudo apt-get install -y i2c-tools python3-pip libi2c-dev python3-smbus

# Install Adafruit libraries for CircuitPython
RUN sudo python3.10 -m pip install adafruit-circuitpython-servokit

# Install libgpiod2 for GPIO access
#RUN sudo apt-get install -y libgpiod2
RUN sudo apt-get install -y libgpiod1

# Install Python bindings for libgpiod
RUN  sudo python3.10 -m pip install gpiod

# Install ROS Joy
RUN sudo apt-get install -y ros-melodic-joy

# Install foxglove bridge
RUN sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y

# Install ROS Melodic xacro
RUN sudo apt-get install ros-melodic-xacro -y

RUN sudo apt-get install ros-melodic-robot-state-publisher -y

# install MPU6050 library
RUN sudo apt-get install -y ros-melodic-imu-filter-madgwick
# Install 
RUN sudo python3.10 -m pip install mpu6050-raspberrypi smbus 

RUN sudo python3.10 -m pip install adafruit-circuitpython-tca9548a

RUN sudo python3.10 -m pip install pyYAML

RUN sudo python3.10 -m pip install flask_cors

RUN sudo python3.10 -m pip install mpu6050-raspberrypi smbus