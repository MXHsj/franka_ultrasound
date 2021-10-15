#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

void target_pose_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    std::vector<double> tar_pose = msg->data;
    std::cout << "{ ";
    for (int i = 0; i < tar_pose.size(); i++)
    {
        std::cout << tar_pose[i] << " ";
    }
    std::cout << "}" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_data_receiver");
    ros::NodeHandle nh_;
    ros::Subscriber tar_pose = nh_.subscribe("cmd_pos", 10, target_pose_callback);
    ros::Rate loop_rate(60);
    while (nh_.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}