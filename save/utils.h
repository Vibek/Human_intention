#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>
#include <vector>


void drawBoundingBox(cv::Mat image, std::vector<cv::Point2f> bb);
std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints);

void newWindow(const std::string &name, cv::Mat *img);
void destroyWindows();

void drawBoundingBox(cv::Mat image, std::vector<cv::Point2f> bb)
{
    for(unsigned i = 0; i < bb.size() - 1; i++) {
        cv::line(image, bb[i], bb[i + 1], cv::Scalar(0, 255, 255), 2);
    }
    cv::line(image, bb[bb.size() - 1], bb[0], cv::Scalar(0, 255, 255), 2);
}

std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}


std::vector<std::string> windows;
void newWindow(const std::string &name, cv::Mat *img = 0)
{
    cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
    if (img)
    {
        cv::imshow(name, *img);
    }
    windows.push_back(name);
}
void destroyWindows()
{
    for (int k = 0; k < (int)windows.size(); ++k)
    {
        cv::destroyWindow(windows[k]);
    }
    windows.clear();
}

#endif // UTILS_H
