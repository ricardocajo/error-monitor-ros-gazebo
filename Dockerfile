FROM therobotcooperative/turtlebot3

WORKDIR /

COPY . /sim_monitor_compiler

#Install python3.8
RUN apt-get update \
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

WORKDIR /ros_ws/src

#Create ros package to run tests
RUN catkin_create_pkg test_pkg std_msgs rospy \
    && cd .. \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && eval "catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

WORKDIR /
