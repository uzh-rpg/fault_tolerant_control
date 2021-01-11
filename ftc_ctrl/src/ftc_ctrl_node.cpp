#include <ros/ros.h>
#include "ftc_ctrl/ftc_ctrl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ftc_ctrl");
    ftc::NDICtrl NDI_ctrl;
    ros::spin();
    
    return 0;
}
