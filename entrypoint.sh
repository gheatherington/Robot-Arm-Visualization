#~/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

cd /ros2_ws/
colcon build --symlink-install

source /ros2_ws/install/setup.bash

echo "Provided arguments: $@"

exec $@