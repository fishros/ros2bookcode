import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class LearnLifeCycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecyclenode')
        self.timer_period = 0
        self.timer_ = None
        self.get_logger().info(f'{self.get_name()}:已创建')

    def timer_callback(self):
        self.get_logger().info('定时器打印进行中...')

    def on_configure(self, state):
        self.timer_period = 1.0  # 设置定时器周期
        self.get_logger().info('on_configure():配置周期 timer_period')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('on_activate():处理激活指令，创建定时器')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.destroy_timer(self.timer_) # 销毁定时器
        self.get_logger().info('on_deactivate():处理失活指令停止定时器')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.timer_ = None
        self.timer_period = 0
        self.get_logger().info('on_cleanup():处理清理指令')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        #  定时器未销毁则销毁
        if self.timer_: self.destroy_timer(self.timer_)
        self.get_logger().info('on_shutdown():处理关闭指令')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        # 直接调用父类处理
        return super().on_error(state)

def main():
    rclpy.init()
    node = LearnLifeCycleNode()
    rclpy.spin(node)
    rclpy.shutdown()