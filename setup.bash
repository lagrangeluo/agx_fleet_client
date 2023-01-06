source /opt/ros/noetic/setup.bash

rosdep update \
    && rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro noetic \
    -yr