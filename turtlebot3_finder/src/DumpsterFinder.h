#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/LaserScan.h"

#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>

class DumpsterFinder
{
    public:

        constexpr static double MAX_LINVEL_SPEED = 0.22; /// m/sn
        constexpr static double MAX_ROTVEL_SPEED = 1.82; // rad/sn

        constexpr static double STOP_SPEED = 0.0;
        constexpr static double FORWARD_SPEED = 0.22;
        constexpr static double RIGHT_ROTATE_SPEED = -0.5;
        constexpr static double LEFT_ROTATE_SPEED = 0.5;
        constexpr static float MIN_DIST_FROM_OBSTACLE = 0.3; // in meter

        constexpr static float TARGET_DIST_FROM_OBSTACLE = 1.0; // in meter

        DumpsterFinder();
        virtual ~DumpsterFinder();
        void start(); 

    private:
        ros::NodeHandle node;
        ros::Publisher commandPub; // Publisher to the robot's velocity command topic
        ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
        ros::Subscriber DeptSub; // Subscriber to the robot's laser scan topic
        // Subscriber to the robot's depth camera
        image_transport::ImageTransport imageTransport;
	    image_transport::Subscriber imageSubscriber;
        // Processing depth camera data
        sensor_msgs::PointCloud2 input_pointcloud;
        sensor_msgs::PointCloud out_pointcloud;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void dephtCallback(const sensor_msgs::PointCloud2::ConstPtr& PointCloud2);

        // returns the min range value between the starting and stopping angles
        // from the received scanner ranges data.
        double getMinRangeValue(double startAngle, double stopAngle, sensor_msgs::LaserScan ranges);

        void move(double LinVel, double RotVel);// Send a velocity command to move the robot to forward
        void moveForward();// Send a velocity command to move the robot to forward
        void moveRight();// Send a velocity command to turn the robot to the right
        void moveLeft();// Send a velocity command to turn the robot to the left
        void Stop();// Send a velocity command to stop robot

        double RotVel;
        bool moving; // Indicates whether the robot should continue moving forward
        bool movingForward; // Indicates whether the robot should continue moving forward
        bool movingRight; // Indicates whether the robot should rotating right
        bool movingLeft; // Indicates whether the robot should rotating left
        bool stopping; // Indicates whether the robot should stopping
        bool stoppingDone; // Indicates whether the robot should continue moving



};