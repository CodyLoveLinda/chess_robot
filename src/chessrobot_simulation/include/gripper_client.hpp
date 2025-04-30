#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <chess_robot_service/gripper.h>
#include <string>

class StringServiceClient
{
public:
    StringServiceClient()
    {
        ros::NodeHandle n;
        client = n.serviceClient<chess_robot_service::gripper>("string_service");
    }

    bool callService(const std::string &command)
    {
        chess_robot_service::gripper srv;
        srv.request.command = command;

        if (client.call(srv))
        {
            ROS_INFO("gripper service Feedback: %d", srv.response.feedback);
            return srv.response.feedback;
        }
        else
        {
            ROS_ERROR("Failed to call service string_service");
            return false;
        }
    }

private:
    ros::ServiceClient client;
};

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "string_service_client");

    StringServiceClient client;
    std::string request_data = "Hello, ROS!";

    if (client.callService(request_data))
    {
        ROS_INFO("Service call succeeded.");
    }
    else
    {
        ROS_ERROR("Service call failed.");
    }

    return 0;
}*/
