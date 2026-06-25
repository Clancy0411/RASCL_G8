FROM docker.io/ros:jazzy

ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    git \
    htop \
    less \
    libserial-dev \
    nano \
    neovim \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-ament-cmake-cppcheck \
    ros-jazzy-ament-cmake-cpplint \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-controller-manager \
    ros-jazzy-forward-command-controller \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    tree \
    vim \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages pysoem==1.1.12

RUN git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git /opt/SOEM \
  && mkdir -p /opt/SOEM/build \
  && cd /opt/SOEM/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
  && cmake --build . -- -j$(nproc) \
  && cmake --build . --target install \
  && ldconfig

RUN printf "source /opt/ros/jazzy/setup.bash\n" >> /root/.bashrc \
  && printf "alias rosbuild='cd /root/ws && colcon build --symlink-install'\n" >> /root/.bashrc \
  && printf "alias rossetup='cd /root/ws && source install/local_setup.bash && ros2 daemon start'\n" >> /root/.bashrc \
  && printf "alias rosclean='cd /root/ws && rm -rf build install log'\n" >> /root/.bashrc \
  && printf "echo 'rascl-container: use rosbuild, rossetup, rosclean'\n" >> /root/.bashrc \
  && printf "PS1='\\[\\e[32m\\]rascl-container\\[\\e[0m\\]:\\[\\e[34m\\]\\w\\[\\e[0m\\]$ '\n" >> /root/.bashrc

WORKDIR /root/ws

CMD ["/bin/bash"]
