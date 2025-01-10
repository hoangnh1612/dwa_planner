#include"obstacles.h"
#include<iostream>

int main()
{
    // read .png map by opencv 
    cv::Mat map = cv::imread("../map/map.png", cv::IMREAD_COLOR);
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
    ObstacleList obs_list;
    obs_list.initialization(map, 10, 0.1, {0, 0, 0});
    while(1)
    {
    obs_list.updateObstaclePosition();
    cv::Mat obs_map = obs_list.obstacleVisualization();
    cv::imshow("Obstacle", obs_map);
    cv::waitKey(5);
    }

}