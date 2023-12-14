import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue() # 创建队列，存放小说
        # 创建话题发布者，发布小说
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        self.timer_ = self.create_timer(5, self.timer_callback) # 创建定时器

    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info(f'下载完成：{url}')
        for line in response.text.splitlines(): # 按行分割，放入队列
            self.novels_queue_.put(line)

    def timer_callback(self):
        if self.novels_queue_.qsize() > 0: # 队列中有数据，取出发布一行
            msg = String() # 实例化一个消息
            msg.data = self.novels_queue_.get() # 对消息结构体进行赋值
            self.novel_publisher_.publish(msg) # 发布消息
            self.get_logger().info(f'发布了一行小说：{msg.data}') 

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download_novel('http://localhost:8000/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()
