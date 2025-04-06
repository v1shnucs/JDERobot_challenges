#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        
        # Publishers and subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.update_pose,
            10)
        
        # Robot pose
        self.pose = Pose()
        
        # Waypoints (in TurtleSim coordinates)
        self.waypoints = [
            {'x': 8.0, 'y': 5.0},    # Right
            {'x': 8.0, 'y': 8.0},    # Up
            {'x': 5.5, 'y': 5.5}     # Back to center
        ]
        self.current_waypoint = 0
        
        # Control parameters
        self.distance_tolerance = 0.1
        
        # Timer
        self.timer = self.create_timer(0.1, self.move_to_waypoint)
        
    def update_pose(self, data):
        """Update turtle's pose"""
        self.pose = data
        self.get_logger().info(f'Position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, theta={self.pose.theta:.2f}')
    
    def euclidean_distance(self, goal_point):
        """Calculate distance to goal"""
        return sqrt(pow((goal_point['x'] - self.pose.x), 2) + pow((goal_point['y'] - self.pose.y), 2))
    
    def steering_angle(self, goal_point):
        """Calculate angle to goal"""
        return atan2(goal_point['y'] - self.pose.y, goal_point['x'] - self.pose.x)
    
    def angular_difference(self, goal_point):
        """Calculate difference in angle"""
        angle = self.steering_angle(goal_point) - self.pose.theta
        # Normalize angle to [-pi, pi]
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
    
    def move_to_waypoint(self):
        """Move turtle towards current waypoint"""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)
            return
            
        goal = self.waypoints[self.current_waypoint]
        distance = self.euclidean_distance(goal)
        
        if distance < self.distance_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}')
            self.current_waypoint += 1
            return
            
        # Create velocity message
        vel_msg = Twist()
        
        # Angular velocity
        angle_diff = self.angular_difference(goal)
        vel_msg.angular.z = 1.5 * angle_diff
        
        # Linear velocity
        if abs(angle_diff) < 0.1:
            vel_msg.linear.x = min(1.0, 0.5 * distance)
        else:
            vel_msg.linear.x = 0.0
            
        # Publish velocity commands
        self.velocity_publisher.publish(vel_msg)

def main():
    rclpy.init()
    navigator = TurtleNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation stopped by user')
    finally:
        # Stop the turtle
        stop_msg = Twist()
        navigator.velocity_publisher.publish(stop_msg)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()