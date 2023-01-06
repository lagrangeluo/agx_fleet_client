FROM ros:noetic

#安装相关依赖
RUN sudo apt update \
    && sudo apt install -y \
    git wget qt5-default \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions 

#添加环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc