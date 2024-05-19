FROM osrf/ros:noetic-desktop-full
RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list

#-----------------------------
# Environment Variables
#-----------------------------
ENV LC_ALL=C.UTF-8
ENV export LANG=C.UTF-8
# no need input key
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]


#install common pkg
#-----------------------------
RUN apt-get -y update
RUN apt-get -y install git ssh python3-pip wget net-tools vim curl make build-essential lsb-release cmake


#-----------------------------
#install tmc-ros pkg
#-----------------------------

RUN echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list
RUN echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add -
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -
RUN wget https://packages.osrfoundation.org/gazebo.key -O - |  apt-key add -

RUN mkdir -p /etc/apt/auth.conf.d
RUN echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf
RUN apt-get update -y
RUN apt-get install -y ros-noetic-tmc-desktop-full


RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
RUN cd /catkin_ws/src && . /opt/ros/noetic/setup.sh && catkin_init_workspace

COPY ./follow_human /catkin_ws/src/follow_human
COPY ./hsr_tools /catkin_ws/src/hsr_tools
COPY ./hardware /catkin_ws/src/hardware
COPY ./navigation /catkin_ws/src/navigation

RUN cd /catkin_ws && . /opt/ros/noetic/setup.sh && catkin_make
RUN echo "source ./catkin_ws/devel/setup.bash --extend" >> .bashrc

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /catkin_ws
