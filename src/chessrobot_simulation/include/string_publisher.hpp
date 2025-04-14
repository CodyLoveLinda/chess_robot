// string_publisher.hpp
#ifndef STRING_PUBLISHER_HPP
#define STRING_PUBLISHER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @class StringPublisher
 * @brief ROS node class for publishing string messages
 */
class StringPublisher {
public:
    /**
     * @brief Constructor initializes ROS publisher
     * @param topic_name Name of the topic to publish (default: "/gripper_control_topic")
     * @param queue_size Publisher queue size (default: 10)
     */
    explicit StringPublisher(const std::string& topic_name = "/gripper_control_topic", 
                           int queue_size = 10) 
    {
        pub_ = nh_.advertise<std_msgs::String>(topic_name, queue_size);
    }

    /**
     * @brief Publishes a string message
     * @param message The string content to publish
     */
    void publishMessage(const std::string& message) {
        std_msgs::String msg;
        msg.data = message;
        pub_.publish(msg);
        ROS_INFO("Published: %s", msg.data.c_str());
    }

private:
    ros::NodeHandle nh_;      ///< ROS node handle
    ros::Publisher pub_;      ///< ROS publisher object
};

#endif // STRING_PUBLISHER_HPP

