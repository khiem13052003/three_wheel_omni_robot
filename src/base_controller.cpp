#include <ros/ros.h>

// Ð?nh nghia m?t class BaseController co b?n
class BaseController
{
public:
    BaseController(ros::NodeHandle& nh)
    {
        ROS_INFO("BaseController dã du?c kh?i t?o.");
        // T?i dây b?n có th? kh?i t?o publisher, subscriber, ho?c controller hardware
    }

    void spin()
    {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok())
        {
            ROS_INFO("BaseController dang ch?y...");
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    BaseController controller(nh);
    controller.spin();

    return 0;
}

