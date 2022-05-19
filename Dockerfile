ARG robot

FROM therobotcooperative/${robot}

#Copy the repo to the image
COPY . /error-monitor-ros-gazebo

#Install dependencies to run teleop
RUN apt-get update \
    && apt-get install -y --allow-unauthenticated ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control
#TODO remove after pull request accepted

#create test_pkg
RUN cd /ros_ws/src \
    && catkin_create_pkg test_pkg std_msgs rospy \
    && cd .. \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && eval "catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
#TODO build method is dependent on robots ${BUILD_COMMAND}

#install python 3.8
RUN cd .. \
    && apt-get update \
    && wget https://www.python.org/ftp/python/3.8.0/Python-3.8.0.tgz \
    && tar -xf Python-3.8.0.tgz \
    && cd Python-3.8.0 \
    && ./configure \
    && sudo make altinstall

#Install necessary python packages
RUN apt-get update \
    && python3.8 -m pip install --upgrade setuptools \
    && python3.8 -m pip install -U Jinja2 \
    && python3.8 -m pip install \
    requests \
    ply \
    && python -m pip install colorama
