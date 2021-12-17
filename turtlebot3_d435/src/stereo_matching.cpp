#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;
using namespace std_msgs;

typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
const int min_disp = 0;
const int num_disp = 192;
const int block_sz = 11;
const int p1 = 8 * block_sz * block_sz;
const int p2 = 32 * block_sz * block_sz;
const int lr_thres = 1;
const int pre_filter_cap = 15;
const int uniqueness_ratio = 15;
const int speckle_sindow = 150;
const int speckle_range = 1;
const int sgbm_mode = StereoSGBM::MODE_SGBM;

// BASELINE 0.05
// FOCAL LENGTH 696

class RawDispGenerator
{
    // example https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
    // and https://answers.ros.org/question/172772/writing-a-c-class-with-message_filters-member/
public:
    RawDispGenerator() : sync(MySyncPolicy(100), ir1_sub, ir2_sub)
    {
        disp_publisher = nh.advertise<Image>("/raw_disp", 100);
        stereo_matcher = StereoBM::create(num_disp, block_sz);
        // stereo_matcher = StereoSGBM::create(min_disp, num_disp, block_sz, p1, p2, lr_thres, pre_filter_cap, uniqueness_ratio, speckle_sindow, speckle_range, sgbm_mode);
        ir1_sub.subscribe(nh, "/camera/infra1/image_raw", 100);
        ir2_sub.subscribe(nh, "/camera/infra2/image_raw", 100);
        sync.registerCallback(boost::bind(&RawDispGenerator::callback, this, _1, _2));
    }
    ~RawDispGenerator() {}

private:
    ros::NodeHandle nh;
    ros::Publisher disp_publisher;
    Ptr<StereoBM> stereo_matcher;
    // Ptr<StereoSGBM> stereo_matcher;
    message_filters::Subscriber<Image> ir1_sub;
    message_filters::Subscriber<Image> ir2_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cv_bridge::CvImagePtr convertImg(const ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        static cv_bridge::CvImagePtr old_cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg);
            old_cv_ptr = cv_ptr;
            return cv_ptr;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return old_cv_ptr;
        }
    }

    void callback(const ImageConstPtr &ir1, const ImageConstPtr &ir2)
    {
        CvImagePtr ir1_ptr = RawDispGenerator::convertImg(ir1);
        CvImagePtr ir2_ptr = RawDispGenerator::convertImg(ir2);
        CvImage disp_image_bridge;
        Image disp_img;

        Mat ir1_img = ir1_ptr->image;
        Mat ir2_img = ir2_ptr->image;
        Mat init_disp;
        Mat f_disp;
        stereo_matcher->compute(ir1_img, ir2_img, init_disp);
        init_disp.convertTo(f_disp, CV_32FC1, 1.0f / 16.0f);

        Header header = ir1_ptr->header;
        disp_image_bridge = CvImage(header, image_encodings::TYPE_32FC1, f_disp);
        disp_image_bridge.toImageMsg(disp_img);
        RawDispGenerator::disp_publisher.publish(disp_img);

        // in case we want to visualize the disparity map

        // Mat disp_display;
        // f_disp.convertTo(disp_display, CV_8U);
        // namedWindow("disp", WINDOW_AUTOSIZE);
        // imshow("disp", disp_display);
        // waitKey(0);
        // destroyAllWindows();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "raw_disp_generator");
    RawDispGenerator generator;
    ros::spin();

    return 0;
}