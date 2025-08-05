FROM arm64v8/ros:humble

ENV DEPENDENCIES_WS=/home/ros/dependencies_ws
ENV WS=/home/ros/ws
ENV USER_GID=1000
ENV USER_UID=1000
ENV USERNAME=ros
ENV ROS_DISTRO=humble

RUN groupadd -g ${USER_GID} ${USERNAME}
USER root
ENV DEBIAN_FRONTEND=noninteractive

RUN apt autoremove -y && \
    #sudo rm -rf /etc/apt/sources.list.d/ros2.list && \
    #curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    #echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt-get install -y --fix-broken && \
    apt install -y python3-dev openssl && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*


RUN useradd -s /bin/bash -m -c "Dogzilla ROS User" ${USERNAME} -p "" -u ${USER_UID} -g ${USER_GID} -G sudo -G dialout -G video
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd

WORKDIR /home/${USERNAME}

USER ${USERNAME}
RUN sudo chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

RUN sudo apt update && \
    sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-ros2launch ros-${ROS_DISTRO}-ros2run && \
    sudo apt install -y ros-${ROS_DISTRO}-rqt-tf-tree ros-${ROS_DISTRO}-ros2bag && \
    sudo apt install -y ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-rclpy && \
    sudo apt install -y rapidjson-dev libopencv-dev libeigen3-dev libyaml-cpp-dev && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt update && \
    sudo apt install ros-humble-foxglove-bridge -y && \
    sudo apt install -y ros-${ROS_DISTRO}-slam-toolbox && \
    sudo apt install -y ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-navigation2 && \
    sudo apt install tmux emacs htop wget curl nano python3-pip -y && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN sudo pip3 install paho-mqtt utm xacro pyserial

# Building dependencies #
COPY --chown=ros:ros . ${DEPENDENCIES_WS}/src
WORKDIR ${DEPENDENCIES_WS}
#RUN cd src/dogzilla_code/yahboom_color_identify_interfaces && touch COLCON_IGNORE
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    sudo apt update && \
    sudo apt dist-upgrade -y && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} --skip-keys "roslib gazebo-ros gazebo-plugins" && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN cd src/yahboom_color_identify_interfaces && touch COLCON_IGNORE
RUN cd src/yahboom_publish && touch COLCON_IGNORE

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython3_EXECUTABLE=/usr/bin/python3

RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
RUN sed -i 's/font/pure/g' ${HOME}/.bashrc

RUN echo "if [ -f $WS/install/setup.bash ]; then source $WS/install/setup.bash; fi" >> ${HOME}/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ${HOME}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ${HOME}/.bashrc
