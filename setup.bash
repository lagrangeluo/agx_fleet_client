#!bin/bash

source /opt/ros/noetic/setup.bash

sudo apt-get -y install python3-pip \
&& sudo pip install rosdepc \
&& rosdepc init \
&& rosdepc update \
    && rosdepc install \
    --from-paths src \
    --ignore-src \
    --rosdistro noetic \
    -yr