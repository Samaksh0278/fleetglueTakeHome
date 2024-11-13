# Base image with ROS
FROM ros:humble-ros-base

# Install necessary packages for ROS, Next.js, and libcurl
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    curl \
    libcurl4-openssl-dev \
    libjsoncpp-dev && \
    curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Set up ROS workspace
WORKDIR /ros2_ws

# Copy ROS and Next.js source files into the container
COPY ./src/fleetglue_cpp /ros2_ws/src/fleetglue_cpp

# Verify the contents of fleetglue_cpp directory
RUN ls /ros2_ws/src/fleetglue_cpp

# Clean previous build and install folders to avoid cache issues
RUN rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log

# Build ROS packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select fleetglue_cpp

# Set up Next.js application
WORKDIR /ros2_ws/src/fleetglue_cpp/nextjs_api
RUN npm install && npm run build

# Set environment variables for ROS
ENV ROS_WS=/ros2_ws
ENV ROS_PACKAGE_PATH=$ROS_WS/src:$ROS_PACKAGE_PATH

# Define entrypoint to source ROS setup
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Expose Next.js port
EXPOSE 3000
