import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion


class TFListener(Node):

    def __init__(self):
        super().__init__("tf2_listener")
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1, self.get_transform)

    def get_transform(self):
        try:
            result = self.buffer_.lookup_transform("base_link", "bottle_link", rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = result.transform
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f"平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}")
        except Exception as e:
            self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")


def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()
