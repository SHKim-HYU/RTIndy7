#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <alchemy/task.h>
#include <alchemy/timer.h>

class RTNode : public rclcpp::Node
{
public:
    RTNode() : Node("rt_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        rt_task_create(&task_, "RT_Task", 0, 99, T_JOINABLE);
        rt_task_start(&task_, &RTNode::run, this);
    }

private:
    static void run(void *arg)
    {
        auto node = static_cast<RTNode*>(arg);
        rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(1e6)); // 1ms period (1e6 ns)
        while (rclcpp::ok())
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello from RT thread!";
            node->publisher_->publish(message);
            rt_task_wait_period(NULL); // Wait for the next period
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    RT_TASK task_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTNode>());
    rclcpp::shutdown();
    return 0;
}
