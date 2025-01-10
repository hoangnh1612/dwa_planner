#include"obstacles.h"
#include"cubic_spline.h"
#include"rrtstar.h"
#include<iostream>

int main()
{
    // read .png map by opencv 
    // cv::Mat map = cv::imread("../map/map.png", cv::IMREAD_COLOR);
    // std::cout<<"Rows:"<<map.rows<<" Cols:"<<map.cols<<"Size: "<<map.size[0]<<std::endl;
    // int c= map.at<cv::Vec3b>(127, 336)[0];
    // for (int i = 200;i<400;i++)
    // {
    //     for (int j = 100;j<400;j++)
    //     {
    //         map.at<cv::Vec3b>(i, j)[0] = 0;
    //         map.at<cv::Vec3b>(i, j)[1] = 0;
    //         map.at<cv::Vec3b>(i, j)[2] = 255;
    //     }
    // }
    // render map 

    /**
     * @brief test obstacle 
     */
    // ObstacleList obs_list;
    // obs_list.initialization(map, 10, 0.1, {0, 0, 0});
    // while(1)
    // {
    // obs_list.updateObstaclePosition();
    // cv::Mat obs_map = obs_list.obstacleVisualization();
    // cv::imshow("Obstacle", obs_map);
    // cv::waitKey(5);
    // }





    /**
     * @brief test cubic spline 
     */
    // Waypoints waypoints;
    // waypoints.push_back({0, 0});
    // waypoints.push_back({1, 2});
    // waypoints.push_back({2, 2});
    // waypoints.push_back({3, 5});
    // waypoints.push_back({4, 4});
    // waypoints.push_back({5, 4});
    // waypoints.push_back({6, 7});
    // waypoints.push_back({7, 1});
    // waypoints.push_back({8, 8});
    // waypoints.push_back({9, 9});
    // waypoints.push_back({10, 5});
    // CubicSpline2D cubic_spline;
    // cubic_spline.initialization(waypoints, 0.01, 0.1);
    // ReferencePath path = cubic_spline.computeCubicPath();
    // for (int i = 0; i < path.size(); i++)
    // {
    //     std::cout<<"X: "<<path[i].x<<" Y: "<<path[i].y<<std::endl;
    // }
    // //visualise point and path in opencv
    // cv::Mat path_map = cv::Mat::zeros(1000, 1000, CV_8UC3);
    // for (int i = 0; i < path.size(); i++)
    // {
    //     Point2DPixel point = convertMeterToPixel(path[i], 10, 10, 1000);
    //     path_map.at<cv::Vec3b>(point.y, point.x)[0] = 255;
    //     path_map.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    //     path_map.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    //     for (int i = 0;i<waypoints.size();i++)
    //     {
    //         Point2DPixel point = convertMeterToPixel(waypoints[i], 10, 10, 1000);
    //         // path_map.at<cv::Vec3b>(point.y, point.x)[0] = 255;
    //         // path_map.at<cv::Vec3b>(point.y, point.x)[1] = 0;
    //         // path_map.at<cv::Vec3b>(point.y, point.x)[2] = 0;
    //         cv::circle(path_map, cv::Point(point.x, point.y), 3, cv::Scalar(0, 0, 255), -1);
    //     }
    // }
    // cv::imshow("Path", path_map);
    // cv::waitKey(0);


    /**
     * @brief test rrtstar
     */

    cv::Mat path_map = cv::imread("../map/map.png", cv::IMREAD_COLOR);
    RRTSTAR rrtstar;
    rrtstar.initilaize(path_map, {0, 0, 0},{10, -10, 0}, {0, 0, 0}, 0.5);
    rrtstar.implementAlgorithm();
    std::vector<Node*> path = rrtstar.path;
    for (int i = 0; i < path.size() ; i++)
    {
        std::cout<<"X: "<<path[i]->pose.position.x<<" Y: "<<path[i]->pose.position.y<<std::endl;
    }

    for (int i = 0; i < path.size(); i++) {
        Point2DPixel point = convertMeterToPixel(path[i]->pose.position, rrtstar.gain_x, rrtstar.gain_y, path_map.rows);
        
        // Simple bounds check before drawing
        if (point.x >= 0 && point.x < path_map.cols && point.y >= 0 && point.y < path_map.rows) {
            // Draw white points
            std::cout<<"draw"<<std::endl;
            // path_map.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(0, 0, 255);
            cv::circle(path_map, cv::Point(point.x, point.y), 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    cv::imshow("Path", path_map);
    cv::waitKey(0);







}