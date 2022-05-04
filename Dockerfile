FROM ros:noetic

SHELL ["/bin/bash","-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-ply \
    python3-jinja2 \
    wget \
    git \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \
    && python3 -m pip install requests \
    && pip install colorama \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make \
    && cd $CATKIN_WS/src \
    && catkin_create_pkg test_pkg std_msgs rospy \
    && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && cd .. \
    && catkin_make

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /usr/local/bin/ros_catkin_entrypoint.sh" >> /root/.bashrc

COPY . /

COPY ros_catkin_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint.sh
RUN chmod +x /usr/local/bin/ros_catkin_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint.sh"]
CMD ["bash"]
