import rclpy
from rclpy.node import Node

# Messages and services used
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn
import math

SPEED = 1.0


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Save user preference
        self.stop_at_top = True

        # Publisher to command turtle velocity
        self.pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # Subscriber to get the turtle's pose
        self.sub = self.create_subscription(Pose, "/turtle2/pose", self.turtle_callback, 10)

        # Kill turtle1 and spawn turtle2
        self.kill_and_spawn()

    def kill_and_spawn(self):
        client_kill = self.create_client(Kill, "kill")
        client_spawn = self.create_client(Spawn, "spawn")

        # Wait for /kill service
        while not client_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill service...")

        req_kill = Kill.Request()
        req_kill.name = "turtle1"
        future_kill = client_kill.call_async(req_kill)
        rclpy.spin_until_future_complete(self, future_kill)

        # Wait for /spawn service
        while not client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")

        req_spawn = Spawn.Request()
        req_spawn.x = 2.0
        req_spawn.y = 1.0
        req_spawn.theta = 0.0
        req_spawn.name = "turtle2"
        future_spawn = client_spawn.call_async(req_spawn)
        rclpy.spin_until_future_complete(self, future_spawn)

        self.get_logger().info("Spawned turtle2 at (2.0, 1.0, 0.0)")

    def turtle_callback(self, msg: Pose):
        my_vel = Twist()
        self.get_logger().info(f"Angle: {msg.theta}")
        # If user selected "stop at top" and turtle reaches the top edge
        if self.stop_at_top and msg.y > 10.5:
            my_vel.linear.x = 0.0
            my_vel.angular.z = 0.0
            self.get_logger().info("Reached top edge. Turtle stopped.")
        
        #if msg.x > 9.0 and abs(abs(msg.theta) - math.pi) < 0.1:
        # If turtle goes beyond right boundary, turn in a circular arc
        elif msg.x > 9.0 and abs(msg.theta) < (math.pi - 0.05):
            my_vel.linear.x = SPEED
            my_vel.angular.z = SPEED 

        # If turtle goes beyond left boundary, turn in the opposite circular arc
        elif msg.x < 1.5 and abs(msg.theta) > (0.05):
            my_vel.linear.x = SPEED
            my_vel.angular.z = -SPEED

        # Otherwise move straight along x
        else:
            my_vel.linear.x = SPEED
            my_vel.angular.z = 0.0

        # Publish velocity command
        self.pub.publish(my_vel)


def main(args=None):
    rclpy.init(args=args)

    node = TurtleController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


