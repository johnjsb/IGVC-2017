/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tiff.h>

using namespace cv;

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage);

TEST(imageTest, angleStraight){
    String filename = "/home/robyncastro/IGVC-2017/src/decision/imageTests/testStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(0, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, angleLeft){
    String filename = "/home/robyncastro/IGVC-2017/src/decision/imageTests/testLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(-34, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, angleRight){
    String filename = "/home/robyncastro/IGVC-2017/src/decision/imageTests/testNoisyRightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(40, VisionDecision::getDesiredAngle(300, testImageScan));
}

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage){
    sensor_msgs::Image img_msg; // >> message to be sent
    MatIterator_<uchar> it, end;

    img_msg.height           = cvMatImage.rows;
    img_msg.width            = cvMatImage.cols;
    img_msg.encoding         = "8UC1";
    img_msg.is_bigendian     = false;
    img_msg.step             = (int) cvMatImage.step;

    for (it = cvMatImage.begin<uchar>(), end = cvMatImage.end<uchar>(); it != end; it++){
        if((int) *it == 0)
            img_msg.data.push_back(0);
        else
            img_msg.data.push_back(255);
    }


    return img_msg;
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
