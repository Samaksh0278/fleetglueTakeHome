# Stage 1: ROS environment
FROM ros:humble-ros-base AS ros_build

# Install necessary packages for ROS and action server
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    curl \
    npm \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/

# Install dependencies and build ROS packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select fleetglue

# Stage 2: Next.js and final runtime
FROM ros:humble-ros-base

# Set up workspace and copy ROS packages
WORKDIR /ros2_ws
COPY --from=ros_build /ros2_ws /ros2_ws

# Install Next.js dependencies
RUN apt-get update && apt-get install -y npm && \
    rm -rf /var/lib/apt/lists/*

# Copy only the Next.js app to a separate directory
WORKDIR /nextjs_api
COPY ./src/fleetglue_cpp/nextjs_api /nextjs_api

# Install Next.js dependencies
RUN npm install

# Source ROS and set environment variables for ROS 2 and Next.js
ENV ROS_WS=/ros2_ws
ENV NEXTJS_DIR=/ros2_ws/src/fleetglue_cpp/nextjs_api
ENV ROS_PACKAGE_PATH=$ROS_WS/src:$ROS_PACKAGE_PATH

# Define entrypoint to source the ROS setup and Next.js server
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
