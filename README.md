# FLEETGLUE Takehome Assignment

This project demonstrates the integration of a REST API with ROS 2 nodes, where JSON data from the API is processed into ROS actions and communicated between an Action Client and an Action Server. This project uses Docker to containerize both the ROS nodes and the REST API server, simplifying deployment.

## Project Overview

### Functionality
1. **REST API**:
   - **POST** `/mission`: Accepts a JSON payload to store mission data.
   - **GET** `/mission`: Retrieves the latest mission data provided via the POST endpoint.

2. **ROS Nodes**:
   - **Node 1 (RN1)**: 
     - Regularly calls the GET API endpoint to check for new mission data.
     - Processes the JSON data from the REST API and converts it into a custom ROS action.
     - Hosts an Action Client to send the action to Node 2.
   - **Node 2 (RN2)**:
     - Hosts an Action Server.
     - Prints the ROS action data when received via the Execute callback.

3. **Extra Functionality**:
   - Custom ROS messages to define the structure of the action data.
   - A ROS launch file to start all components.

---

## Repository Structure
repository {
  src {
    my_ros_package {
      action {
        MissionAction # Custom ROS action definition
      }
      launch {
        missionLaunch # Launch file to start API and ROS nodes
      }
      src {
        rn1Node # RN1 - Action Client and API interaction
        rn2Node # RN2 - Action Server
      }
      packageXml # ROS 2 package metadata
      cmakeLists # ROS 2 build configuration
    }
  }
  nextjs_api {
    pages {
      api {
        missionJs # REST API endpoints (GET and POST)
      }
    }
    packageJson # Node.js dependencies
  }
  dockerfile # Docker build instructions
  dockerCompose # Multi-container Docker setup
  entrypointSh # Entry script for starting ROS and API
  readmeMd # Documentation and usage instructions
}


---

## REST API Description

The REST API is implemented using Next.js in `nextjs_api/pages/api/mission.js`.

1. **POST /mission**: Accepts a JSON payload for the mission data. Stores this data for future retrieval.
    - **Expected JSON format**:
      ```json
      {
          "mission_data": "Sample mission data"
      }
      ```

2. **GET /mission**: Retrieves the latest mission data from the POST endpoint.
    - **Response format**:
      ```json
      {
          "mission_data": "Sample mission data"
      }
      ```

## ROS Nodes

### Node 1 (RN1): Action Client and API Interaction
- Runs a loop to call the GET endpoint every second, checking for updated mission data.
- Processes the JSON mission data from the API and converts it into a custom ROS action.
- Sends the action to Node 2 using the ROS Action Client.

### Node 2 (RN2): Action Server
- Hosts an Action Server, which accepts the action sent by RN1.
- Simply prints the ROS action data when received through the Execute callback.

### Custom ROS Messages
- **Mission.action**: Defines the structure of the ROS action data that is sent from RN1 to RN2.
  - **Goal**: Contains the mission data.
  - **Result**: (Empty for now, customizable based on future needs).
  - **Feedback**: (Empty for now, customizable as needed).

### Launch File
The `mission_launch.py` launch file is located in `src/my_ros_package/launch/`. It automates the startup of:
- The Next.js API server.
- ROS Node 1 (RN1) and ROS Node 2 (RN2).

---

## Docker Setup

The project is Dockerized for easy deployment of both the ROS nodes and the REST API.

### Docker Files

1. **Dockerfile**: Defines the Docker image, including ROS 2, Node.js, and dependencies.
2. **docker-compose.yml**: Manages multi-container services (ROS nodes and the API server).

### Environment Variables

In the Dockerfile, environment variables are set to define paths for the ROS workspace and the API:

```dockerfile
ENV ROS_WS=/ros2_ws
ENV NEXTJS_DIR=/ros2_ws/src/my_ros_package/nextjs_api
ENV ROS_PACKAGE_PATH=$ROS_WS/src:$ROS_PACKAGE_PATH
---

## Acknowledgments

I would like to thank Fleetglue for this oppurtunity, all feedback is welcome!

---
