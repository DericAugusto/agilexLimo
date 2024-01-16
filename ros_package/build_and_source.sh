#!/bin/bash

colcon build --symlink-install
echo
echo "Build completed successfully."
source install/setup.bash
source ~/.bashrc
echo "Sourcing completed successfully."

