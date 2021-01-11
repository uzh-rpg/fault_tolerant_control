#include "ftc_state_est/state_est_CF.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_est");
    state_est_CF::StateEstCF state_est_cf;

    ros::spin();

    return 0;
}
