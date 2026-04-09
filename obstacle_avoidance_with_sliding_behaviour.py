"""
Drone Obstacle Avoidance Node with Sliding Maneuver
====================================================
This node implements a state machine for autonomous drone control with obstacle avoidance.
States: INIT → FLYING → SLIDING → LANDING
When an obstacle is detected, the drone performs a sliding maneuver (moving laterally) 
until the obstacle is cleared, then resumes flight.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Float32
from collections import deque
import time
import math
import signal
import subprocess

class DroneWithSlidingAvoidance(Node):
    """
    Autonomous drone controller with obstacle avoidance using a sliding maneuver strategy.
    
    State Machine:
    - INIT: Waiting for sensors to initialize
    - FLYING: Normal flight toward goal, monitoring for obstacles
    - SLIDING: Lateral avoidance maneuver when obstacle detected
    - LANDING: Final descent to ground
    """
    def __init__(self):
        super().__init__('drone_sliding_node')

        # ─── State Machine ────────────────────────────────────────────────
        self.flight_state = 'INIT'  # Current flight state (INIT → FLYING → SLIDING → LANDING)
        self.slide_start_time = None  # Timestamp when sliding maneuver began

        # ─── Obstacle Detection Thresholds ────────────────────────────────
        self.enter_threshold = 0.7  # Distance (m) below which obstacle is detected
        self.exit_threshold = 1.0   # Distance (m) above which obstacle is considered cleared
        self.min_slide_duration = 3.0  # Minimum time (s) to spend in sliding before checking exit
        self.safe_distance = 0.5  # Target safe distance from obstacles (for reference)
        
        # ─── Distance Filtering ────────────────────────────────────────────
        # Rolling buffer for averaging distance readings to reduce noise
        self.distance_buffer = deque(maxlen=10)

        # ─── Sensor Data ──────────────────────────────────────────────────
        self.current_pose = None  # Latest drone position and orientation from vision
        self.current_distance = None  # Latest obstacle distance measurement

        # ─── Subscriptions ────────────────────────────────────────────────
        # Subscribe to drone position/orientation from vision system
        self.create_subscription(PoseStamped, '/drone2/mavros/vision_pose/pose', self.pose_callback, 10)
        # Subscribe to obstacle distance sensor
        self.create_subscription(Float32, '/drone2/obstacle_distance', self.distance_callback, 10)

        # ─── Publisher ────────────────────────────────────────────────────
        # Publish target setpoints for position control
        self.set_point_pub = self.create_publisher(PoseStamped, '/drone2/mavros/setpoint_position/local', 1)

        # ─── Service Clients (MAVROS Interface) ───────────────────────────
        # Services for drone control via MAVROS (MAVLink ROS interface)
        self.set_mode_client = self.create_client(SetMode, '/drone2/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/drone2/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/drone2/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/drone2/mavros/cmd/land')

        # ─── Service Request Objects ──────────────────────────────────────
        # Pre-allocated request objects for efficiency (reused across calls)
        self.set_mode_req = SetMode.Request()
        self.arm_req = CommandBool.Request()
        self.takeoff_req = CommandTOL.Request()  # Renamed from to_req for clarity
        self.land_req = CommandTOL.Request()

    def pose_callback(self, msg):
        """Store latest drone pose from vision system."""
        self.current_pose = msg

    def distance_callback(self, msg):
        """Store latest obstacle distance measurement."""
        self.current_distance = msg.data

    def control_loop(self):
        """Main control loop that routes to appropriate state handler."""
        if self.flight_state == 'FLYING':
            self.fly_state_handler()
        elif self.flight_state == 'SLIDING':
            self.slide_state_handler()

    def fly_state_handler(self):
        """
        FLYING State: Navigate toward goal while monitoring for obstacles.
        Transitions to SLIDING state if obstacle is detected.
        """
        # Buffer distance readings for noise reduction
        if self.current_distance is not None and self.current_distance > 0:
            self.distance_buffer.append(self.current_distance)

        # Check for obstacle once buffer is full (reduces false positives)
        if len(self.distance_buffer) == self.distance_buffer.maxlen:
            avg_d = sum(self.distance_buffer) / len(self.distance_buffer)
            self.get_logger().info(f'[FLYING] avg dist = {avg_d:.2f} m')
            if avg_d < self.enter_threshold:
                # Obstacle detected - initiate sliding maneuver
                self.get_logger().warn('Obstacle detected → switching to SLIDING')
                self.slide_start_time = self.get_clock().now()
                self.distance_buffer.clear()  # Reset buffer for exit condition checking
                self.flight_state = 'SLIDING'
                return

        # Fly toward goal (2.0, 0.0) at constant altitude
        if self.current_pose:
            z = self.current_pose.pose.position.z
        else:
            z = 0.5  # Default altitude if pose not available yet
        self.fly_to_point(2.0, 0.0, z)

    def slide_state_handler(self):
        """
        SLIDING State: Perform lateral avoidance maneuver.
        Drone moves diagonally (+x, -y) to slide around obstacle.
        Transitions back to FLYING once obstacle is cleared.
        """
        # Calculate elapsed time since sliding began
        now = self.get_clock().now()
        elapsed = (now - self.slide_start_time).nanoseconds * 1e-9

        if self.current_pose:
            target = PoseStamped()
            target.header.stamp = now.to_msg()
            target.header.frame_id = self.current_pose.header.frame_id
            
            # Sliding vectors: +x (forward), -y (right), fixed z (altitude)
            target.pose.position.x = self.current_pose.pose.position.x + 0.4
            target.pose.position.y = self.current_pose.pose.position.y - 0.4
            target.pose.position.z = 0.5
            target.pose.orientation = self.current_pose.pose.orientation
            self.set_point_pub.publish(target)

        # Check exit conditions: minimum slide duration elapsed and buffer filled with new data
        if elapsed > self.min_slide_duration:
            if self.current_distance is not None and self.current_distance > 0:
                self.distance_buffer.append(self.current_distance)
            
            # Once buffer is full, check if obstacle is cleared
            if len(self.distance_buffer) == self.distance_buffer.maxlen:
                avg_d = sum(self.distance_buffer) / len(self.distance_buffer)
                self.get_logger().info(f'[SLIDING] avg dist = {avg_d:.2f} m')
                if avg_d > self.exit_threshold:
                    # Obstacle cleared - return to normal flight
                    self.get_logger().info('Obstacle cleared → switching back to FLYING')
                    self.distance_buffer.clear()
                    self.flight_state = 'FLYING'

    # ────────────────────────────────────────────────────────────────────────
    # MAVROS (MAVLink ROS Bridge) Helper Functions
    # ────────────────────────────────────────────────────────────────────────
    def call_service(self, client, req, name: str):
        """
        Generic service call wrapper with waiting and error handling.
        
        Args:
            client: Service client object
            req: Service request object
            name: Service name (for logging)
        
        Returns:
            Service response object
        """
        # Wait for service to be available (1s timeout per attempt)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name} service…')
        
        # Call service asynchronously and wait for result
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def set_mode(self, mode: str = 'GUIDED'):
        """
        Set flight mode (e.g., GUIDED for autonomous waypoint following).
        
        Args:
            mode: Flight mode string (GUIDED, AUTO, STABILIZE, etc.)
        
        Returns:
            Service response with success status
        """
        self.set_mode_req.custom_mode = mode
        resp = self.call_service(self.set_mode_client,
                                 self.set_mode_req,
                                 'set_mode')
        self.get_logger().info(
            f'Mode → {mode}' if getattr(resp, 'mode_sent', False)
            else f'Failed to set mode → {mode}')
        return resp

    def arm(self):
        """
        Arm the drone (enable motors for flight).
        
        Returns:
            Service response with success status
        """
        self.arm_req.value = True
        resp = self.call_service(self.arm_client,
                                 self.arm_req,
                                 'arming')
        self.get_logger().info(
            'Armed' if getattr(resp, 'success', False)
            else 'Arming failed')
        return resp

    def take_off(self, altitude: float = 0.5):
        """
        Initiate takeoff to specified altitude.
        
        Args:
            altitude: Target altitude in meters
        
        Returns:
            Service response with success status
        """
        self.takeoff_req.altitude  = altitude
        self.takeoff_req.latitude  = 0.0
        self.takeoff_req.longitude = 0.0
        self.takeoff_req.min_pitch = 0.0
        self.takeoff_req.yaw       = 0.0

        resp = self.call_service(self.takeoff_client,
                                 self.takeoff_req,
                                 'takeoff')
        self.get_logger().info(
            f'Takeoff to {altitude} m OK' if getattr(resp, 'success', False)
            else 'Takeoff failed')
        return resp

    def land(self):
        """
        Initiate landing sequence (descend to ground).
        
        Returns:
            Service response with success status
        """
        self.land_req.altitude  = 0.0
        self.land_req.latitude  = 0.0
        self.land_req.longitude = 0.0
        self.land_req.min_pitch = 0.0
        self.land_req.yaw       = 0.0

        resp = self.call_service(self.land_client,
                                 self.land_req,
                                 'land')
        self.get_logger().info(
            'Landed' if getattr(resp, 'success', False)
            else 'Landing failed')
        return resp

    def fly_to_point(self, x: float, y: float, z: float):
        """
        Publish target setpoint for position control.
        Used by autopilot to navigate to goal position.
        
        Args:
            x, y, z: Target position in meters (map frame)
        """
        p = PoseStamped()
        p.header.stamp    = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        # Keep drone level (no roll/pitch)
        p.pose.orientation.w = 1.0
        self.set_point_pub.publish(p)


def main(args=None):
    """
    Main entry point for the drone obstacle avoidance node.
    
    Sequence:
    1. Initialize ROS2 and node
    2. Wait for vision pose data
    3. Set flight mode to GUIDED
    4. Arm the drone
    5. Takeoff to 0.5m altitude
    6. Start rosbag recording for data collection
    7. Begin autonomous flight with obstacle avoidance
    8. Land when reaching target position
    """
    rclpy.init(args=args)
    node = DroneWithSlidingAvoidance()

    # ──────────────────────────────────────────────────────────────────────
    # Step 1: Wait for pose data from vision system
    # ──────────────────────────────────────────────────────────────────────
    node.get_logger().info('Waiting for vision pose data…')
    while rclpy.ok() and node.current_pose is None:
        node.get_logger().info('Waiting for vision pose…')
        rclpy.spin_once(node, timeout_sec=1.0)

    node.get_logger().info('Vision pose received. Proceeding with initialization.')

    # ──────────────────────────────────────────────────────────────────────
    # Step 2-5: Set mode, arm, and takeoff
    # ──────────────────────────────────────────────────────────────────────
    # Set to GUIDED mode for autonomous waypoint following
    resp = node.set_mode('GUIDED')
    if not getattr(resp, 'mode_sent', False):
        node.get_logger().error('Failed to set GUIDED mode. Aborting.')
        return
    time.sleep(0.5)

    # Arm the drone to enable motors
    resp = node.arm()
    if not getattr(resp, 'success', False):
        node.get_logger().error('Failed to arm drone. Aborting.')
        return
    time.sleep(3.0)

    # Takeoff to 0.5m altitude
    resp = node.take_off(0.5)
    if not getattr(resp, 'success', False):
        node.get_logger().error('Takeoff failed. Aborting.')
        return
    
    # Wait for drone to reach takeoff altitude
    time.sleep(10.0)
    node.get_logger().info('Takeoff complete. Starting data collection and autonomous flight.')

    # ──────────────────────────────────────────────────────────────────────
    # Step 6: Start rosbag recording for data logging
    # ──────────────────────────────────────────────────────────────────────
    rosbag_proc = subprocess.Popen([
        'ros2', 'bag', 'record',
        '-o', 'obstacle_avoidance_slide',
        '/drone2/obstacle_distance',
        '/drone2/mavros/vision_pose/pose',
        '/drone2/vicon/drone1_obstacle/drone1_obstacle'
    ])
    
    # ──────────────────────────────────────────────────────────────────────
    # Step 7-8: Autonomous flight loop with landing detection
    # ──────────────────────────────────────────────────────────────────────
    node.flight_state = 'FLYING'
    landing_target = (2.0, 0.0, 0.5)  # Target position to reach before landing
    landing_thresh = 0.2  # Distance threshold to trigger landing (meters)
    control_loop_dt = 0.05  # Control loop rate limiting: 50ms (20Hz)

    try:
        while rclpy.ok() and node.flight_state != 'LANDING':
            # Execute state machine update
            node.control_loop()
            
            # Check landing condition only during normal flight
            cp = node.current_pose
            if node.flight_state == 'FLYING' and cp:
                # Calculate distance to landing target
                dx = cp.pose.position.x - landing_target[0]
                dy = cp.pose.position.y - landing_target[1]
                dz = cp.pose.position.z - landing_target[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Trigger landing when close enough to target
                if dist < landing_thresh:
                    node.get_logger().info('Reached landing point → landing now')
                    node.land()
                    node.flight_state = 'LANDING'
            
            # Rate limiting to prevent CPU spinning (20Hz control loop)
            rclpy.spin_once(node, timeout_sec=control_loop_dt)
            
    finally:
        # ────────────────────────────────────────────────────────────────
        # Cleanup: Stop recording and shutdown
        # ────────────────────────────────────────────────────────────────
        node.get_logger().info('Stopping rosbag recording…')
        rosbag_proc.send_signal(signal.SIGINT)
        rosbag_proc.wait()
        
        node.get_logger().info('Shutting down node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()