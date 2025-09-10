i
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

from  assignment_msg.msg import TurtleInfo  # <-- Custom msg

import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # --- Publisher for turtle1 and turtle2 cmd_vel ---
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # --- Publisher for custom TurtleInfo msg ---
        self.info_pub = self.create_publisher(TurtleInfo, '/turtle2/info', 10)

        # --- Client to spawn turtle2 ---
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Request to create turtle2
        request = Spawn.Request()
        request.x = 5.0
        request.y = 5.0
        request.theta = 0.0
        request.name = 'turtle2'
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.spawn_callback)

        # --- Subscriber to get turtle2 pose ---
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)
        self.turtle2_pose = None

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle spawned: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle2: {e}")

    def pose_callback(self, msg: Pose):
        """Save turtle2 pose and publish custom message."""
        self.turtle2_pose = msg
        info_msg = TurtleInfo()
        info_msg.name = "turtle2"
        info_msg.x = msg.x
        info_msg.y = msg.y
        info_msg.theta = msg.theta
        self.info_pub.publish(info_msg)

    def run_cli(self):
        """Simple textual interface to send commands."""
        while rclpy.ok():
            # Ask user for turtle
            turtle = input("Select turtle (turtle1 / turtle2): ").strip()
            if turtle not in ["turtle1", "turtle2"]:
                print("Invalid turtle. Try again.")
                continue

            # Ask for velocities
            try:
                lin = float(input("Linear velocity (x): "))
                ang = float(input("Angular velocity (z): "))
            except ValueError:
                print("Invalid number. Try again.")
                continue

            # Create Twist msg
            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang

            # Publish command for 1 second
            start = time.time()
            while time.time() - start < 1.0:
                if turtle == "turtle1":
                    self.pub_turtle1.publish(twist)
                else:
                    self.pub_turtle2.publish(twist)
                time.sleep(0.1)

            # Stop the turtle
            stop = Twist()
            if turtle == "t1":
                self.pub_turtle1.publish(stop)
            else:
                self.pub_turtle2.publish(stop)

            print(f"{turtle} moved with v={lin}, w={ang} for 1s and stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        node.run_cli()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

