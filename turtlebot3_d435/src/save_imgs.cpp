#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

int i = 1;
float BASELINE = 0.05;
float FOCAL_LENGTH = 696;

// NEED TO DEAL WITH TEXTURELESS REGIONS / RANDOM PATTEN
void convertImg(const ImageConstPtr &msg, string img_type, int index)
{
    std_msgs::Header header = msg->header;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        string path = "/home/xfan/tmp/" + to_string(index) + "_" + img_type + ".png";
        cv_ptr = cv_bridge::toCvCopy(msg);
        imwrite(path, cv_ptr->image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void callback(const ImageConstPtr &ir1, const ImageConstPtr &ir2, const ImageConstPtr &dep, const ImageConstPtr &raw_disp)
{
    cout << i << endl;
    // i++;
    cout << "Processing frame " << i << endl;
    convertImg(ir1, "left", i);
    convertImg(ir2, "right", i);
    convertImg(dep, "gt", i);
    convertImg(raw_disp, "raw_disp", i);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_d435_data");
    ros::NodeHandle nh;
    message_filters::Subscriber<Image> ir1_sub(nh, "/camera/infra1/image_raw", 1);
    message_filters::Subscriber<Image> ir2_sub(nh, "/camera/infra2/image_raw", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<Image> raw_disp_sub(nh, "/raw_disp", 1);

    typedef sync_policies::ApproximateTime<Image, Image, Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ir1_sub, ir2_sub, dep_sub, raw_disp_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
    ros::spin();

    return 0;
}