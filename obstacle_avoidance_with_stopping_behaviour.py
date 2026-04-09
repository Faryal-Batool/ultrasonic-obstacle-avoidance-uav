"""
Drone Obstacle Avoidance with Ultrasonic Sensor
================================================
This node implements autonomous drone control with collision detection.
When an obstacle is detected, the drone hovers for 3 seconds then lands.

State Machine: INIT → FLYING → STOPPING → LANDED
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Float32
from collections import deque
import time
import math
import signal
import subprocess

class DroneWithUltrasonic(Node):
    """
    Autonomous drone controller with ultrasonic obstacle detection.
    
    Behavior:
    - INIT: Waiting for sensors and initialization
    - FLYING: Navigate toward goal while monitoring distance
    - STOPPING: Hold position when obstacle detected, then land after pause
    - LANDED: Mission complete
    """
    def __init__(self):
        super().__init__('drone_ultrasonic_node')

        # ─── State Machine ───────────────────────────────────────────────────
        self.flight_state    = 'INIT'     # Current state: INIT → FLYING → STOPPING → LANDED
        self.stop_time       = None       # Timestamp when obstacle triggered stopping
        self.pause_duration  = 3.0        # Hover duration (seconds) before landing

        # ─── Sensor Data ─────────────────────────────────────────────────────
        self.current_pose     = None      # Latest pose from vision system (PoseStamped)
        self.current_distance = None      # Latest distance measurement (float, meters)
        self.distance_buffer  = deque(maxlen=10)  # Rolling buffer for averaging distance readings
        self.safe_distance    = 0.5       # Minimum safe distance threshold (meters)

        # ─── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            '/drone2/mavros/vision_pose/pose',
            self.pose_callback,
            10)
        self.create_subscription(
            Float32,
            '/drone2/obstacle_distance',
            self.distance_callback,
            10)

        # ─── Publisher ───────────────────────────────────────────────────────
        self.set_point_pub = self.create_publisher(
            PoseStamped,
            '/drone2/mavros/setpoint_position/local',
            1)

        # ─── MAVROS service clients ──────────────────────────────────────────
        self.set_mode_client = self.create_client(
            SetMode,
            '/drone2/mavros/set_mode')
        self.arm_client      = self.create_client(
            CommandBool,
            '/drone2/mavros/cmd/arming')
        self.takeoff_client  = self.create_client(
            CommandTOL,
            '/drone2/mavros/cmd/takeoff')
        self.land_client     = self.create_client(
            CommandTOL,
            '/drone2/mavros/cmd/land')

        # ─── Prebuilt request msgs ──────────────────────────────────────────
        self.set_mode_req = SetMode.Request()
        self.arm_req      = CommandBool.Request()
        self.takeoff_req  = CommandTOL.Request()
        self.land_req     = CommandTOL.Request()

    # ─── Callbacks ─────────────────────────────────────────────────────────
    def pose_callback(self, msg: PoseStamped):
        """Store latest drone position and orientation from vision system."""
        self.current_pose = msg

    def distance_callback(self, msg: Float32):
        """Store latest distance measurement from ultrasonic sensor."""
        self.current_distance = msg.data

    # ─── State Machine Handlers ───────────────────────────────────────────
    def control_loop(self):
        """Route to appropriate state handler based on current flight state."""
        if self.flight_state == 'FLYING':
            self.fly_state_handler()
        elif self.flight_state == 'STOPPING':
            self.stop_state_handler()
        # INIT and LANDED states don't require active control

    def fly_state_handler(self):
        """
        FLYING State: Navigate toward goal while monitoring for obstacles.
        Transitions to STOPPING state if obstacle is detected.
        """
        # Buffer distance readings for noise reduction via averaging
        if self.current_distance is not None and self.current_distance > 0:
            self.distance_buffer.append(self.current_distance)

        # Once buffer is full with valid readings, compute average
        if len(self.distance_buffer) == self.distance_buffer.maxlen:
            avg_d = sum(self.distance_buffer) / len(self.distance_buffer)
            self.get_logger().info(f'[FLYING] avg dist = {avg_d:.2f} m')
            
            # Check if obstacle is too close
            if avg_d < self.safe_distance:
                self.get_logger().warn('Obstacle too close → STOPPING')
                self.stop_time = self.get_clock().now()
                self.distance_buffer.clear()  # Reset buffer for fresh readings during stop phase
                self.flight_state = 'STOPPING'
                return

        # Continue flying toward goal position (2.0, 0.0, 1.0)
        self.fly_to_point(2.0, 0.0, 1.0)

    def stop_state_handler(self):
        """
        STOPPING State: Hold position at current altitude.
        After pause_duration expires, transition to LANDED and initiate descent.
        """
        # Hover at current position by republishing current pose as setpoint
        if self.current_pose:
            hover = PoseStamped()
            hover.header.stamp    = self.get_clock().now().to_msg()
            hover.header.frame_id = self.current_pose.header.frame_id
            hover.pose            = self.current_pose.pose  # Maintain current position and orientation
            self.set_point_pub.publish(hover)
        else:
            self.get_logger().warn('No pose available for hovering!')

        # Check if minimum pause duration has elapsed
        if (self.get_clock().now() - self.stop_time) > Duration(seconds=self.pause_duration):
            self.get_logger().info(f'[STOPPING] hovered for {self.pause_duration}s → initiating landing')
            self.land()  # Send land command
            self.flight_state = 'LANDED'  # Transition to final state
            self.get_logger().info('Mission complete')

    # ─── MAVROS Service Helpers ───────────────────────────────────────────
    def call_service(self, client, req, name: str):
        """
        Generic service call wrapper with waiting and error handling.
        
        Args:
            client: Service client object
            req: Service request object
            name: Service name (for logging purposes)
        
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

    def take_off(self, altitude: float = 1.0):
        """
        Initiate takeoff to specified altitude.
        
        NOTE: The TAKEOFF service works indoors without GPS; many firmware versions support it.
        
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
        # Keep drone level (no roll/pitch by setting quaternion identity)
        p.pose.orientation.w = 1.0
        self.set_point_pub.publish(p)


def main(args=None):
    """
    Main entry point for obstacle avoidance drone control.
    
    Sequence:
    1. Initialize ROS2 and node
    2. Wait for vision pose data
    3. Set flight mode to GUIDED
    4. Arm the drone
    5. Takeoff to 0.5m altitude
    6. Start rosbag recording
    7. Execute autonomous flight with obstacle detection
    8. Land or stop when mission complete
    """
    rclpy.init(args=args)
    node = DroneWithUltrasonic()

    # ─── Step 1-2: Wait for Vision Pose ───────────────────────────────────
    node.get_logger().info('Waiting for vision pose data…')
    while rclpy.ok() and node.current_pose is None:
        node.get_logger().info('Waiting for vision pose…')
        rclpy.spin_once(node, timeout_sec=1.0)
    node.get_logger().info('Vision pose received. Proceeding with initialization.')

    # ─── Step 3-5: Set Mode, Arm, and Takeoff ─────────────────────────────
    node.get_logger().info('INIT: Setting mode, arming, and taking off…')
    
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
    node.get_logger().info('Takeoff complete. Proceeding to data collection phase.')

    # ─── Step 6: Start Rosbag Recording ───────────────────────────────────
    node.get_logger().info('Starting rosbag recording…')
    rosbag_proc = subprocess.Popen([
        'ros2', 'bag', 'record',
        '-o', 'obstacle_avoidance_test_5',
        '/drone2/obstacle_distance',
        '/drone2/mavros/vision_pose/pose',
        '/drone2/vicon/drone1_obstacle/drone1_obstacle'
    ])
    node.get_logger().info('Rosbag recording started.')

    # ─── Step 7: Autonomous Flight Loop with Obstacle Detection ───────────
    node.get_logger().info('Entering autonomous flight loop')
    node.flight_state = 'FLYING'  # Begin in FLYING state
    landing_target = (2.0, 0.0, 1.0)  # Target waypoint for example mission
    landing_thresh = 0.2  # Distance threshold to trigger landing (meters)
    control_loop_dt = 0.05  # Control loop rate limiting: 50ms (20Hz)

    try:
        while rclpy.ok() and node.flight_state != 'LANDED':
            # Execute state machine (FLYING/STOPPING handlers)
            node.control_loop()

            # Check if we've reached landing target (only check during FLYING state)
            cp = node.current_pose
            if node.flight_state == 'FLYING' and cp is not None:
                # Calculate Euclidean distance to landing target
                dx = cp.pose.position.x - landing_target[0]
                dy = cp.pose.position.y - landing_target[1]
                dz = cp.pose.position.z - landing_target[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Trigger landing when close enough to target
                if dist < landing_thresh:
                    node.get_logger().info('Reached landing point → initiating landing')
                    node.land()
                    node.flight_state = 'LANDED'
                    break
            
            # Rate limiting to prevent CPU spinning (20Hz control loop)
            rclpy.spin_once(node, timeout_sec=control_loop_dt)

        # Ensure landing was requested (safety check)
        if node.flight_state != 'LANDED':
            node.get_logger().warn('Exited loop without explicit landing—sending LAND command as safety measure')
            node.land()
        else:
            node.get_logger().info('Mission succeeded: loop ended in LANDED state')

    finally:
        # ─── Step 8: Stop Rosbag Recording ────────────────────────────────────
        node.get_logger().info('Stopping rosbag recording…')
        # Check if rosbag process is still running before sending signal
        if rosbag_proc.poll() is None:
            rosbag_proc.send_signal(signal.SIGINT)  # Graceful shutdown signal
            rosbag_proc.wait()  # Wait for process to terminate
        node.get_logger().info('Rosbag recording stopped.')

        # ─── Cleanup: Shutdown Node and ROS2 ──────────────────────────────────
        node.get_logger().info('Shutting down node and ROS2…')
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Shutdown complete')

if __name__ == '__main__':
    main()