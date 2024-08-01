FROM osrf/ros:humble-desktop-full

ENV DISPLAY=host.docker.internal:0.0

RUN \
    apt-get update \
    && apt-get -y install \
    nano \
    ros-humble-gazebo-ros-pkgs \ 
    python3-pip \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install setuptools==58.2.0

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN \
    groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN \
    apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

WORKDIR /ros2_ws/

RUN \
    echo "\nsource /opt/ros/humble/setup.bash\nsource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\nsource /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

#CMD ["ros2", "launch", "robot_arm_visualization", "robot_state_publisher_sim.launch.py"]
CMD ["bash"]

USER root

# Docker Build Command: docker build -t gheatherington/arm_visualization:{VERSION} .
# Docker Run Command: docker run -it --rm --user ros --name ros2_ws --hostname ros2_docker -v $PWD\:/ros2_ws/src/robot_arm_visualization/ gheatherington/arm_visualization:{VERSION}