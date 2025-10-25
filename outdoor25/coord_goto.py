import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleGlobalPosition, VehicleCommand
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MoveThroughGPSWaypoints(Node):
    def __init__(self):
        super().__init__('move_through_gps_waypoints')
        self.get_logger().info('move_through_gps_waypoints node started')

        # === Waypoints list (lat, lon, alt) ===
        self.waypoints = [
            [18.9430934, -98.1864723, 40.0],
            [18.942020609, -98.186548134, 40.0],
            [18.942021623, -98.189672295, 40.0],
            [18.942557966, -98.189672154, 40.0],
            [18.942557028, -98.186548089, 40.0],
            [18.943093448, -98.186548044, 40.0],
            [18.943094310, -98.189672013, 40.0],
            [18.943630653, -98.189671872, 40.0],
            [18.943629867, -98.186548000, 40.0],
            [18.944166287, -98.186547955, 40.0],
            [18.944166996, -98.189671731, 40.0],
            [18.943745790, -98.189910418, 40.0],
            [18.944514420, -98.186892891, 40.0],
            [18.943996268, -98.186747027, 40.0],
            [18.943227687, -98.189764669, 40.0],
            [18.942709584, -98.189618920, 40.0],
            [18.943478116, -98.186601164, 40.0],
            [18.942959963, -98.186455302, 40.0],
            [18.942191482, -98.189473173, 40.0],
            [18.941673379, -98.189327426, 40.0],
            [18.942441811, -98.186309440, 40.0],
            [18.9430934, -98.1864723, 40.0]
        ]
        self.current_wp_index = 0

        # === Control parameters ===
        self.max_speed = 10.0
        self.min_distance = 10.0
        self.kp = 0.8
        self.kd = 1.5

        # === Internal state ===
        self.home_set = False
        self.current_local = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]

        # === QoS ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.cmd_vehicle_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # === Subscription ===
        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_profile
        )

        # === Timer ===
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.timer.cancel()

    # -------------------------------
    # GPS callback
    # -------------------------------
    def gps_callback(self, msg: VehicleGlobalPosition):
        if not self.home_set:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.home_set = True
            self.get_logger().info(
                f"Home set at lat={self.home_lat}, lon={self.home_lon}, alt={self.home_alt:.2f}"
            )

            self.arm_and_offboard()
            self.timer.reset()

        self.current_local = self.gps_to_local(msg.lat, msg.lon, msg.alt)

    # -------------------------------
    # GPS to local NED
    # -------------------------------
    def gps_to_local(self, lat, lon, alt):
        R = 6378137.0
        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)
        x = d_lat * R
        y = d_lon * R * math.cos(math.radians((self.home_lat + lat) / 2))
        z = alt - self.home_alt
        return [x, y, -z]  # NED convention

    # -------------------------------
    # Arm and set OFFBOARD
    # -------------------------------
    def arm_and_offboard(self):
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.source_system = 1
        arm_cmd.source_component = 1
        arm_cmd.from_external = True
        self.cmd_vehicle_pub.publish(arm_cmd)
        self.get_logger().info("Sent ARM command")

        mode_cmd = VehicleCommand()
        mode_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        mode_cmd.param1 = 1.0
        mode_cmd.param2 = 6.0  # OFFBOARD
        mode_cmd.target_system = 1
        mode_cmd.target_component = 1
        mode_cmd.source_system = 1
        mode_cmd.source_component = 1
        mode_cmd.from_external = True
        self.cmd_vehicle_pub.publish(mode_cmd)
        self.get_logger().info("Sent OFFBOARD command")

    # -------------------------------
    # Control loop
    # -------------------------------
    def control_loop(self):
        if not self.home_set or self.current_wp_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_wp_index]
        target_local = self.gps_to_local(target[0], target[1], target[2])

        error = [
            target_local[0] - self.current_local[0],
            target_local[1] - self.current_local[1],
            target_local[2] - self.current_local[2]
        ]
        dist = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)

        if dist < self.min_distance:
            self.get_logger().info(f"Reached waypoint {self.current_wp_index + 1}/{len(self.waypoints)} ✅")
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached! 🚀 Landing or holding position.")
                self.publish_twist(0.0, 0.0, 0.0, 0.0)
                self.timer.cancel()
            return

        # Derivative term
        dt = self.timer_period
        d_error = [
            (error[0] - self.prev_error[0]) / dt,
            (error[1] - self.prev_error[1]) / dt,
            (error[2] - self.prev_error[2]) / dt
        ]
        self.prev_error = error

        # PD control (3D)
        vy = -(self.kp * error[0] + self.kd * d_error[0])
        vx = -(self.kp * error[1] + self.kd * d_error[1])
        vz = -(self.kp * error[2] + self.kd * d_error[2])

        # Limit velocity
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale
            vz *= scale

        self.publish_twist(vx, vy, vz, 0.0)
        self.get_logger().info(
            f"[WP {self.current_wp_index+1}/{len(self.waypoints)}] "
            f"cmd_vel -> vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, dist={dist:.2f}"
        )

    # -------------------------------
    # Publish Twist
    # -------------------------------
    def publish_twist(self, vx, vy, vz, yaw_rate):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.z = yaw_rate
        self.cmd_pub.publish(twist)


# -------------------------------
# Main
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MoveThroughGPSWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()