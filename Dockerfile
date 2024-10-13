FROM ros:noetic-ros-base-focal

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
    git \
    python3-pip \
    ros-noetic-move-base-msgs \
    ros-noetic-geometry-msgs \
  && rm -rf /var/lib/apt/lists/*

# Create the ROS 1 package
WORKDIR /catkin_ws/src/
RUN mkdir nav_rest_api
COPY ./launch ./nav_rest_api/launch
COPY ./CMakeLists.txt ./nav_rest_api/CMakeLists.txt
COPY ./package.xml ./nav_rest_api/package.xml
COPY ./requirements.txt ./nav_rest_api/requirements.txt
WORKDIR /catkin_ws/src/nav_rest_api/scripts
COPY ./scripts/run_server.bash ./run_server.bash

WORKDIR /catkin_ws/src/
RUN git clone -b devel https://github.com/cardboardcode/autodock.git --depth 1 --single-branch
RUN rm -r autodock/autodock_examples
RUN rm -r autodock/autodock_sim
RUN rm -r autodock/docs
RUN rm -r autodock/.github
RUN rm -r autodock/scripts

WORKDIR /catkin_ws/src/nav_rest_api
RUN pip3 install --no-cache-dir -r requirements.txt

WORKDIR /catkin_ws/
RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
  && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && catkin_make

RUN sed -i '$isource "/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

EXPOSE 8080

WORKDIR /catkin_ws/src/
COPY ./swagger_server ./nav_rest_api/swagger_server

ENTRYPOINT ["/ros_entrypoint.sh"]

