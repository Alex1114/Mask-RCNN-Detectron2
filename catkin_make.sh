#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash

catkin_make -C /home/arg/Mask-RCNN-Detectron2/catkin_ws -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

source /home/arg/Mask-RCNN-Detectron2/environment.sh
