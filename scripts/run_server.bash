#!/bin/env bash

source /catkin_ws/devel/setup.bash
roscd nav_rest_api
python3 -m swagger_server
