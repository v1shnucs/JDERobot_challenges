# ROS2 Navigation Challenge Solution

This project demonstrates basic ROS2 concepts and navigation capabilities using ROS2 Dashing.

## Part 1: Basic ROS2 Implementation

### 1a. Publisher-Subscriber Demo
- Created a basic publisher-subscriber system
- Publisher sends "Hello! ROS2 is fun" message
- Subscriber receives and displays the message
- Built using colcon build system

Files:
- `src/publisher.cpp`: Publishes the message
- `src/subscriber.cpp`: Receives and displays the message

### 1b. Robot Setup
- Implemented using TurtleSim as the robot platform
- Provides visual feedback of robot movement
- Demonstrates basic robot control capabilities

## Part 2: Navigation Implementation

### Waypoint Navigation
- Implemented autonomous navigation through three waypoints:
  1. (8.0, 5.0) - Move right
  2. (8.0, 8.0) - Move up
  3. (5.5, 5.5) - Return to center

Files:
- `turtle_nav.py`: Main navigation implementation
  * Position tracking
  * Velocity control
  * Waypoint sequencing

### Key Features
- Smooth velocity control
- Goal-seeking behavior
- Autonomous navigation
- Sequential waypoint following

## How to Run

1. Build the workspace:
```bash
colcon build
```

2. Run publisher-subscriber demo:
```bash
# Terminal 1
source install/setup.bash
ros2 run ros2_demo publisher

# Terminal 2
source install/setup.bash
ros2 run ros2_demo subscriber
```

3. Run navigation demo:
```bash
# Terminal 1
source /opt/ros/dashing/setup.bash
ros2 run turtlesim turtlesim_node

# Terminal 2
source /opt/ros/dashing/setup.bash
python3 turtle_nav.py
```

## Implementation Details

The solution demonstrates:
- ROS2 node creation and management
- Topic-based communication
- Robot control and navigation
- Waypoint-based autonomous movement

### Navigation Algorithm
- Uses proportional control for angular velocity
- Linear velocity controlled based on distance to goal
- Smooth transitions between waypoints
- Goal tolerance checking

## Dependencies
- ROS2 Dashing
- TurtleSim package
- Python 3
- C++ compiler