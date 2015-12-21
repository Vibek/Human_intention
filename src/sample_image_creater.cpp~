#include <cstdio>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ostream>

class sample_image_creater{
public:
    sample_image_creater(bool modus):
    _it(nh)
    {
    cuttingbox=modus;
    std::cout<< "Reached the Constructor"<< std::endl;
    working = false;
    cv::namedWindow("BoxTrackbar",CV_WINDOW_NORMAL);
    if(cuttingbox){
        img_sub = _it.subscribe("/camera/rgb/image_rect_color", 1, &sample_image_creater::imageCB, this);
        xpos=0;
        ypos=0;
        boxsize=400;
        cv::createTrackbar("Xpos","BoxTrackbar",&xpos,640);
        cv::createTrackbar("Ypos","BoxTrackbar",&ypos,480);
        cv::createTrackbar("Squaresize","BoxTrackbar",&boxsize,480);

    }
    else{
        detection_sub = _it.subscribe("/object_detection/object",1, &sample_image_creater::detectCB,this);
    }

    waste=0;
    samplenumber=0;
    saving=1;
    cv::createTrackbar("Done","BoxTrackbar",&waste,1,Box_picked,this);
    cv::createTrackbar("Save?","BoxTrackbar",&saving,1);
    cv::imshow("BoxTrackbar",1);

    cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE);

    std::cout<< "Done the Constructor"<< std::endl;
    }
    void detectCB(const sensor_msgs::ImageConstPtr& img_msg){
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::cout<< "Reached the convertion"<< std::endl;

        cv::medianBlur(cv_ptr->image, image, 9);
        cv::cvtColor(image,cropped, CV_BGR2HSV);

        std::cout<< "Done the convertion"<< std::endl;
        cv::imshow("Display window",cropped);
        working=true;

        Box_picked(1,this);

    }

    void imageCB(const sensor_msgs::ImageConstPtr& img_msg){

        std::cout<< "Reached the callback"<< std::endl;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::cout<< "Reached the convertion"<< std::endl;

        cv::medianBlur(cv_ptr->image, image, 9);
        cv::cvtColor(image,image, CV_BGR2HSV);

        std::cout<< "Done the convertion"<< std::endl;
        cv::imshow("Display window",image);

        working=true;

    }
    void cut_box(){
        if(ypos+boxsize>480){
            boxsize=480-ypos;
        }
        if(xpos+boxsize>640){
            xpos=640-boxsize;
        }
        cv::Rect myROI(xpos, ypos, boxsize, boxsize);



        cv::Mat croppedRef(image, myROI);

        std::cout<< "Cropped completed"<< std::endl;
        croppedRef.copyTo(cropped);

        std::cout<< "Copy Completed"<< std::endl;
        cv::imshow("Display window",cropped);
    }
    static void Box_picked(int newValue ,void* object){
        sample_image_creater* myClass = (sample_image_creater*) object;
        if(myClass->saving==1){
            cv::Mat result;

            myClass->reshape_image(myClass->cropped,result);
            std::stringstream ss;
            ss<< myClass->samplenumber<<"sample.ppm";
            while(!myClass->exists_test3(ss.str())){
                std::cout<<"File allready exists";
                myClass->samplenumber++;
                ss.str(std::string());
                ss<< "sample_images/"<< myClass->samplenumber<<"sample.ppm";
            }

            std::cout<< "Try to write file"<< std::endl;
            cv::imwrite(ss.str(),result);
            myClass->samplenumber++;
            myClass->working=false;

            std::cout<< "Creating file completed"<< std::endl;
            return ;
        }
        myClass->working=false;
    }

    inline bool exists_test3 (const std::string& name) {
      struct stat buffer;
      return (stat (name.c_str(), &buffer) != 0);
    }
    void reshape_image(cv::Mat& src, cv::Mat& dst ){
        cv::Size dsize = cv::Size(sample_size_x,sample_size_y);
        cv::resize(src,dst,dsize,cv::INTER_AREA);
        //imshow("Display Window",dst);
    }

    bool working;
    int saving;
    static const int sample_size_x = 100;
    static const int sample_size_y = 100;
    bool cuttingbox;
    cv::Mat cropped, image;
    int xpos,ypos,boxsize, waste,samplenumber;
    ros::NodeHandle nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber img_sub,detection_sub;
private:



};


int main(int argc, char** argv){

    std::cout<< "Reached the Main"<< std::endl;
    ros::init(argc, argv, "sample_image_creator");
    bool cuttingbox = false;
    sample_image_creater sic(cuttingbox);
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        std::cout<< "After Spin"<< std::endl;
        cv::waitKey(1);
        while(sic.working){
            if(cuttingbox){
                std::cout<< "Try to cut Box"<< std::endl;
                sic.cut_box();
            }
            cv::waitKey(1);
            rate.sleep();

        }
        rate.sleep();

    }
}
