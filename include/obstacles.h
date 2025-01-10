#ifndef DWA_OBSACTLES_H
#define DWA_OBSACTLES_H
#include "ultis.h"
  /**
    * @brief Default constructor of the abstract obstacle class
    */
class Obstacle
{
public:
    /**
     * @brief Position of the obstacle
     */
    Point2D position;
    /**
     * @brief Velocity of the obstacle
     */
    Velocity vel;
    /**
     * @brief Params
     */
    double width, length, radius, predictTime, dt;
    /**
     * @brief Default constructor of the abstract obstacle class
     * @param Point2D: position
     * @param Velocity: velocity
     * @param double: radius
     * @param double: predict time
     * @param double: delta time
     */
    Obstacle(Point2D, Velocity, double, double, double);
    /**
     * @brief Default destructor of the abstract obstacle class
     */
    ~Obstacle();
    /**
     * @brief Update position of the obstacle
     * @param cv::Mat: map
     * @param double: gain x
     * @param double: gain y
     * @param int: height
     * @param BGR: color map
     */
    void updatePosition(const cv::Mat, const double, const double, const int, const BGR);
    /**
     * @brief Predict the state of obstacles
     */
    std::vector<Point2D> predictObstacleState();
    /**
     * @brief Check collision with wall
     * @param Point2D: point
     * @param cv::Mat: map
     * @param double: gain x
     * @param double: gain y
     * @param int: height
     * @param BGR: color map
     * @return true if collide with wall (bsc we read a map from an image)
     */
    bool checkCollideWithWall(const Point2D, const cv::Mat, const double, const double, const int, const BGR);


class ObstacleList
{
private:
	cv::Mat map;
	int height;
	double gain_x, gain_y, predict_time, dt;
	BGR color_map;
public:
    /**
     * @brief List of obs
     */
	std::vector<Obstacle> obstacles;
    /**
     * @brief Default constructor
     */
	ObstacleList();
    /**
     * @brief Initialization
     * @brief For simulation, it init 11 moving obstacles
     * @param cv::Mat: map
     * @param double: predict time
     * @param double: delta time
     * @param BGR: color
     */
	void initialization(cv::Mat, const double, const double, const BGR);
    /**
     * @brief Default destructor
     */
	~ObstacleList();
    /**
     * @brief For simulation
     * @return list of obs if in observation range
     */
	std::vector<Obstacle> getObstacleInObservation(const Pose2D, const double);
    /**
     * @brief For simulation
     * @return Update obs pos
     */
	void updateObstaclePosition();
    /**
     * @brief Visualize obstacles to map 
     */
	cv::Mat obstacleVisualization();
};
};