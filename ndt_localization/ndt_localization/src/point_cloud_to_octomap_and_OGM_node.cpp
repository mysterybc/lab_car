#include "ndt_localization/point_cloud_octomap_and_OGM.h"

//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "point_cloud_to_octomap_and_OGM_node");

    ros::NodeHandle nh; 
    ros::NodeHandle nh_p("~");

    MapGeneration mapgeneration(nh);

    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}
