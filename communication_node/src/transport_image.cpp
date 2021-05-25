#include "ros/ros.h"
#include "sensor_msgs/Image.h"
//c++
#include "string"

ros::Subscriber image_sub;
ros::Publisher image_pub;
int pub_frequence;

void ImageBC(const sensor_msgs::ImageConstPtr& image){
    static int image_count{0};
    int image_gap = 30 / pub_frequence;
    if( (++image_count) == image_gap){
        image_pub.publish(image);
        image_count = 0;
    }
}


int main(int argc,char** argv){
    ros::init(argc,argv,"transport_image");
    ros::NodeHandle nh;

    //get param
    std::string image_topic;
    nh.param("image_topic",image_topic,std::string("/camera/fisheye1/image_raw"));
    nh.param("image_frequence",pub_frequence,15); //只接受5 10 15
    if(30%pub_frequence != 0){
        ROS_WARN("image transport node set pub frequence wrong!! Reset to 15");
    }

    image_sub = nh.subscribe(image_topic,1,&ImageBC);
    image_pub = nh.advertise<sensor_msgs::Image>(image_topic + "/transported",1);

    ros::spin();
    return 0;
}