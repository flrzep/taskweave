#!/bin/bash
# Source ROS 2 and the taskweave install, then run the command given to the container.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/taskweave/install/setup.bash
exec "$@"
