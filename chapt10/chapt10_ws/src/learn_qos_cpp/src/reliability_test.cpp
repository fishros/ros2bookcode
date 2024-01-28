#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomPublisherSubscriber : public rclcpp::Node
{
public:
    OdomPublisherSubscriber() : Node("odom_publisher_subscriber")
    {
        rclcpp::QoS qos_profile(10);                                       // 队列深度为10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);   // 可靠性策略
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // 持久性策略
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);             // 历史记录策略
        qos_profile.deadline(rclcpp::Duration(1, 0));                      // 截止时间为1秒

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", qos_profile);

        // // 创建发布者并设置QoS为sensor
        // odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        //     "odom", rclcpp::SensorDataQoS());

        // 创建订阅者（默认QoS配置）队列深度设置为 5
        // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "odom", 5,
        //     [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        //       (void)msg;
        //       RCLCPP_INFO(this->get_logger(), "收到里程计消息");
        //     });

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                (void)msg;
                RCLCPP_INFO(this->get_logger(), "收到里程计消息");
            });

        // 创建一个1秒的定时器，并指定回调函数
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                         { odom_publisher_->publish(nav_msgs::msg::Odometry()); });
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto odom_node = std::make_shared<OdomPublisherSubscriber>();
    rclcpp::spin(odom_node);
    rclcpp::shutdown();
    return 0;
}