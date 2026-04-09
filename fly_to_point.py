import rclpy
import socket
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import sqrt



class DroneClient(Node):
    def __init__(self):
        super().__init__('drone_command_client')

        self.current_pose = None

        # while self.current_pose is None:
        #     self.get_logger().info("Waiting for pose...")
        #     time.sleep(0.1)
        # self.pose_sub = self.create_subscription(
        #     PoseStamped,
        #     '/drone2/mavros/vision_pose/pose',
        #     self.pose_callback,
        #     1
        # )
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone2/mavros/vision_pose/pose',
            self.pose_callback,
            qos_profile
)

        self.set_point_pub = self.create_publisher(PoseStamped, '/drone2/mavros/setpoint_position/local', 1)
        self.set_mode_client = self.create_client(SetMode, '/drone2/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/drone2/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/drone2/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/drone2/mavros/cmd/land')

        self.set_mode_req = SetMode.Request()
        self.arm_req = CommandBool.Request()
        self.takeoff_req = CommandTOL.Request()
        self.land_req = CommandTOL.Request()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def set_mode(self):
        self.set_mode_req.custom_mode = 'GUIDED'
        future = self.set_mode_client.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, future)

    def arm(self, timeout_sec=5.0):
        self.arm_req.value = True

        start_time = time.time()
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Arming service unavailable. Timeout.")
                return False

        future = self.arm_client.call_async(self.arm_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone armed successfully.")
            return True
        else:
            self.get_logger().error("Failed to arm drone.")
            return False


    def take_off(self):
        self.takeoff_req.altitude = 1.0
        self.takeoff_req.latitude = 0.0
        self.takeoff_req.longitude = 0.0
        self.takeoff_req.min_pitch = 0.0
        self.takeoff_req.yaw = 0.0
        future = self.takeoff_client.call_async(self.takeoff_req)
        rclpy.spin_until_future_complete(self, future)

        # Wait until drone reaches at least 0.8m altitude
        for _ in range(100):
            if self.current_pose and self.current_pose.z > 0.8:
                return True
            time.sleep(0.1)
        return False

    def fly_to_point(self, x, y, z, tol=0.3):
        target = PoseStamped()
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)

        for _ in range(50):
            self.set_point_pub.publish(target)
            time.sleep(0.1)

        # Wait until drone reaches close to target
        for _ in range(100):
            if self.current_pose:
                dist = sqrt(
                    (self.current_pose.x - float(x))**2 +
                    (self.current_pose.y - float(y))**2 +
                    (self.current_pose.z - float(z))**2
                )
                if dist < tol:
                    return True
            time.sleep(0.1)
        return False

    def land(self):
        self.land_req.altitude = 0.0
        self.land_req.latitude = 0.0
        self.land_req.longitude = 0.0
        self.land_req.min_pitch = 0.0
        self.land_req.yaw = 0.0
        future = self.land_client.call_async(self.land_req)
        rclpy.spin_until_future_complete(self, future)

        # Wait until drone lands (z < 0.2)
        for _ in range(100):
            if self.current_pose and self.current_pose.z < 0.2:
                return True
            time.sleep(0.1)
        return False


def main(args=None):
    rclpy.init(args=args)
    drone = DroneClient()

     # Start spinning the ROS node in a background thread
    # threading.Thread(target=rclpy.spin, args=(drone,), daemon=True).start()

    try:
        HOST = '192.168.50.48'  # Replace with GS IP
        PORT = 9090
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            client.connect((HOST, PORT))
            client.sendall(b"drone2")
            print("[Drone2] Connected to Ground Station")

            drone.set_mode()
            time.sleep(1.0)

            while True:
                cmd = client.recv(1024).decode()
                print(f"[drone2] Received command: {cmd}")

                if cmd == "ARM":
                    success = drone.arm()
                    time.sleep(3)
                    client.sendall(b"ARMED" if success else b"ARM_FAILED")

                elif cmd == "TAKEOFF":
                    success = drone.take_off()
                    client.sendall(b"TAKEN_OFF" if success else b"FAILED_TAKEOFF")

                elif cmd.startswith("GOTO"):
                    _, x, y, z = cmd.split()
                    success = drone.fly_to_point(x, y, z)
                    client.sendall(b"READY_FOR_NEXT" if success else b"GOTO_FAILED")

                elif cmd == "LAND":
                    success = drone.land()
                    client.sendall(b"LANDED" if success else b"LANDING_FAILED")
                    break

    except KeyboardInterrupt:
        print("\n[drone2] Interrupted by user.")

    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
