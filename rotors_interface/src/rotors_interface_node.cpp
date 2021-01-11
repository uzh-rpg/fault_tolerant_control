#include "rotors_interface/rotors_interface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotors_interface");
    rotors_interface::RotorSInterface rotors_interface;

    ros::spin();

    return 0;
}
