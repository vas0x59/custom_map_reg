#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include "math.h"
#include "markers.h"
// class Marker{
// };
// struct Marker {
//     int id;
//     std::vector<cv::Point2f> points;
// };
using namespace cv;

#ifndef _ARUCOMARKERS_H_
#define _ARUCOMARKERS_H_

struct ArucoMarkerMap
{
    int id;
    float size;
    float z_rotation;
    Point3f point;
};
// struct ArucoMarkerMapPoints{

// }

class ArucoMarkersDetector
{
public:
    ArucoMarkersDetector(cv::Ptr<cv::aruco::Dictionary> dictionary);
    void addMarker(int id, float size, Point3f point, float z_rotation = 0);
    void genBoard();
    // void drawMarkers();
    Mat drawBoard(cv::Size size);
    Mat drawMarker(int id);
    void drawViz(Mat viz);
    
    bool detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints);
    bool detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints, cv::Mat &outImage);
    void loadMap(std::string);
private:
    std::vector<int> _ids_to_detect;
    std::vector<ArucoMarkerMap> _markers_map;
    cv::Ptr<cv::aruco::Board> _board;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    std::vector<std::vector<cv::Point2f>> _corners;
    void _getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
                                       InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints);
    
    // std::vector<ArucoMark
    // std::vector<>
};
#endif