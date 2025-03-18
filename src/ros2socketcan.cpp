// Original Author: Philipp Wuestenberg <philipp.wuestenberg@tu-berlin.de>
// Maintainer: Jonathan Blixti <blixt013@umn.edu>
// Last Updated: November 2023

// Import header file
#include "ros2socketcan.h"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ros2socketcan::ros2socketcan() : Node("ros2socketcan"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    std::string can_socket = this->declare_parameter<std::string>("can_interface", "can0");

    // Obtener el nivel de log desde los parámetros (por defecto INFO)
    log_level_ = this->declare_parameter<int>("log_level", RCUTILS_LOG_SEVERITY_INFO);
    

    // Establecer el nivel de log
    rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), log_level_);
    
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(this->get_logger(), "Error setting logger level!");
    }

    RCLCPP_INFO(this->get_logger(), "CAN_INTERFACE: %s", can_socket.c_str());

    
    topicname_receive << "CAN/" << can_socket << "/receive";
    topicname_transmit << "CAN/" << can_socket << "/transmit";

    rclcpp::executors::MultiThreadedExecutor exec;
    
    publisher_ = this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(), 10);
    subscription_ = this->create_subscription<can_msgs::msg::Frame>(
        topicname_transmit.str(), 100, std::bind(&ros2socketcan::CanPublisher, this, _1));

    strcpy(ifr.ifr_name, can_socket.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
    }

    stream.assign(natsock);

    RCLCPP_INFO(this->get_logger(), "ROS 2 to CAN-Bus topic: %s", subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "CAN-Bus to ROS 2 topic: %s", publisher_->get_topic_name());

    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
                           std::bind(&ros2socketcan::CanListener, this, std::ref(rec_frame), std::ref(stream)));
    signals.async_wait(std::bind(&ros2socketcan::stop, this));

    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    RCLCPP_DEBUG(this->get_logger(), "Thread started");
    bt.detach();

}
void ros2socketcan::stop()
{
    RCLCPP_INFO(this->get_logger(), "End of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

ros2socketcan::~ros2socketcan()
{
    RCLCPP_INFO(this->get_logger(), "End of Publisher Thread. \n");
}

void ros2socketcan::CanSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;

    frame1.can_id = msg.id;

    if (msg.is_extended == true)
    {
        frame1.can_id = frame1.can_id + CAN_EFF_FLAG;
    }

    if (msg.is_error == true)
    {
        frame1.can_id = frame1.can_id + CAN_ERR_FLAG;
    }

    if (msg.is_rtr == true)
    {
        frame1.can_id = frame1.can_id + CAN_RTR_FLAG;
    }

    frame1.can_dlc = msg.dlc;

    for (int i = 0; i < (int)frame1.can_dlc; i++)
    {
        frame1.data[i] = msg.data[i];
    }
    std::stringstream out;
    out << std::string("S | ") << std::to_string(frame1.can_id) << std::string("| ");
    for (int j = 0; j < (int)frame1.can_dlc; j++)
    {
        out << std::to_string(frame1.data[j]) << std::string(" ");
    }
    out << std::endl;
    RCLCPP_DEBUG(this->get_logger(), out.str().c_str());
    stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)), std::bind(&ros2socketcan::CanSendConfirm, this));
}

// Publish messages to the CAN bus
void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{
    can_msgs::msg::Frame msg1;
    msg1.id = msg->id;
    msg1.dlc = msg->dlc;
    msg1.is_extended = msg->is_extended;
    msg1.is_rtr = msg->is_rtr;
    msg1.is_error = msg->is_error;
    msg1.data = msg->data;

    CanSend(msg1);
}

void ros2socketcan::CanSendConfirm(void)
{
    RCLCPP_DEBUG(this->get_logger(), "Message sent");
}

// Listen for CAN messages
void ros2socketcan::CanListener(struct can_frame &rec_frame, boost::asio::posix::basic_stream_descriptor<> &stream)
{
    can_msgs::msg::Frame frame;

    std::stringstream s;

    frame.id = rec_frame.can_id;
    frame.dlc = int(rec_frame.can_dlc);

    s << std::string("R | ") << std::to_string(rec_frame.can_id) << std::string(" | ");
    for (int i = 0; i < rec_frame.can_dlc; i++)
    {
        frame.data[i] = rec_frame.data[i];
        s << std::to_string(rec_frame.data[i]);
    }
    current_frame = frame;
    s << " | ";

    for (int j = 0; j < (int)rec_frame.can_dlc; j++)
    {
        s << std::to_string(rec_frame.data[j]) << " ";
    }
    s << std::endl;
    
    
    RCLCPP_DEBUG(this->get_logger(), s.str().c_str());
    
    RCLCPP_INFO(this->get_logger(), "Log level set to: %d", log_level_);
    
    publisher_->publish(frame);

    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)), std::bind(&ros2socketcan::CanListener, this, std::ref(rec_frame), std::ref(stream)));
}

// Main method of the node
int main(int argc, char *argv[])
{
    std::cout << programdescr << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2socketcan>();
    rclcpp::spin(node);
    // Free up any resources being used by the node
    rclcpp::shutdown();
    return 0;
}
