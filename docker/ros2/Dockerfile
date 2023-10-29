FROM ubuntu:22.04 AS ros2-base

RUN apt update
RUN apt install -y locales && \
      locale-gen en_US en_US.UTF-8 && \
      update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata
RUN apt install -y ros-dev-tools ros-iron-ros-base

WORKDIR /ros2_ws
RUN chsh -s /bin/bash
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.rosrc
RUN echo "[ -f \"/ros2_ws/install/local_setup.bash\" ] && source /ros2_ws/install/local_setup.bash" >> /root/.rosrc
RUN echo "source /root/.rosrc" >> /root/.bashrc
ENV PATH=$PATH:/opt/ros/iron/bin/

# Enable below to test the talker/listener demo
# RUN apt install -y ros-iron-demo-nodes-cpp
# Then, in two terminals run:
# $ ros2 run demo_nodes_cpp talker
# $ ros2 run demo_nodes_cpp listener

# Spin forever
CMD tail -f /dev/null


# For desktop, just install additional packages on top of base
FROM ros2-base as ros2-desktop

RUN apt install -y ros-dev-tools ros-iron-desktop


FROM ros2-base as ros2-base-and-micro-ros

# Setup micro-ros in a seperate build ws to still allow volumes for ros2_ws
RUN apt-get install -y python3-pip
RUN git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git /micro_ros_ws/src/micro_ros_setup
RUN /bin/bash -c "cd /micro_ros_ws &&  \
    rosdep init &&  \
    rosdep update &&  \
    source /opt/ros/iron/setup.bash && rosdep install --from-paths src --ignore-src -y &&  \
    colcon build"
RUN /bin/bash -c "cd /micro_ros_ws &&  \
    source /opt/ros/iron/setup.bash && source /micro_ros_ws/install/local_setup.bash &&  \
    ros2 run micro_ros_setup create_agent_ws.sh"
RUN /bin/bash -c "cd /micro_ros_ws &&  \
    source /opt/ros/iron/setup.bash && source /micro_ros_ws/install/local_setup.bash &&  \
    ros2 run micro_ros_setup build_agent.sh"
RUN echo "source /micro_ros_ws/install/local_setup.bash" >> /root/.rosrc


FROM ros2-base-and-micro-ros as ros2-desktop-and-micro-ros

RUN apt install -y ros-dev-tools ros-iron-desktop


FROM ros2-base-and-micro-ros as ros2-base-and-micro-ros-and-platformio

ENV PATH=$PATH:/root/scripts/:/root/.platformio/penv/bin/
COPY ./scripts/ensure-platformio.sh /root/scripts/ensure-platformio.sh
RUN chmod +x /root/scripts/ensure-platformio.sh && echo "ensure-platformio.sh" >> /root/.rosrc
