#include "DumpsterFinder.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <math.h> 
#include "list"


static const std::string OPENCV_WINDOW = "Image window";
double dumspterCenterAngle;
double dumpsterCenterX = 0;
double dumpsterCenterZ = 0;

double error_sum = 0;
double error_prev = 0;
double Kp = 1.1;
double Ki = 0.005;
double Kd = 0.5;

DumpsterFinder::DumpsterFinder() : imageTransport(node) 
{
    // init params
    moving = true;
    movingForward = false;
    movingRight = false;
    movingLeft = false;
    stopping = false;
    stoppingDone = false;

    RotVel = 0;

    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    imageSubscriber = imageTransport.subscribe("/camera/rgb/image_raw", 1,&DumpsterFinder::imageCallback, this);
    
    laserSub = node.subscribe("scan", 1, &DumpsterFinder::scanCallback, this);

    DeptSub = node.subscribe("/camera/depth/points", 1, &DumpsterFinder::dephtCallback,this);

    // Create a display window
	cv::namedWindow(OPENCV_WINDOW);
}

DumpsterFinder::~DumpsterFinder() 
{
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}

int cnt = 0;
std::list<cv::Vec3i> Wheel;

// Send a velocity command to move the robot to forward
void DumpsterFinder::moveForward() {
    geometry_msgs::Twist msg; 
    msg.linear.x = FORWARD_SPEED;
    msg.angular.z = RotVel;
    commandPub.publish(msg);
}

// Send a velocity command to move the robot to forward
void DumpsterFinder::move(double LinVel, double RotVel) {
    geometry_msgs::Twist msg; 
    msg.linear.x = LinVel;
    msg.angular.z = RotVel;
    commandPub.publish(msg);
}

// Send a velocity command to turn the robot to the right
void DumpsterFinder::moveRight() {
    geometry_msgs::Twist msg;
    msg.linear.x = STOP_SPEED;
    msg.angular.z = RIGHT_ROTATE_SPEED;
    commandPub.publish(msg);
}

// Send a velocity command to turn the robot to the left
void DumpsterFinder::moveLeft() {
    geometry_msgs::Twist msg; 
    msg.linear.x = STOP_SPEED;
    msg.angular.z = LEFT_ROTATE_SPEED;
    commandPub.publish(msg);
}

// Send a velocity command to stop robot
void DumpsterFinder::Stop() {
    geometry_msgs::Twist msg; 
    msg.linear.x = STOP_SPEED;
    msg.angular.z = STOP_SPEED;
    commandPub.publish(msg);
}

void DumpsterFinder::dephtCallback(const sensor_msgs::PointCloud2::ConstPtr& depht)
{

    sensor_msgs::PointCloud2 data = *depht;
    int h = data.height;
    int w = data.width;

    sensor_msgs::convertPointCloud2ToPointCloud(data, out_pointcloud);
    int pointSize = out_pointcloud.points.size();
    
    size_t WheelCount = Wheel.size();
    ROS_INFO(" WheelCount = %d", WheelCount);

    bool isValid = false;
    if(moving == true)
    {
        if(WheelCount == 4.0)
        {
            double sumX = 0;
            double sumZ = 0;


            for( cv::Vec3i c : Wheel)
            {
                int cx = c[0];
                int cy = c[1];

                int i = cy * w + cx;

                double point_x = out_pointcloud.points[i].x;
                double point_y = out_pointcloud.points[i].y;
                double point_z = out_pointcloud.points[i].z;

                sumX += point_x;
                sumZ += point_z;
                
                // check if the wheel found in the image has valid distance information
                if(isnan(point_x) == true || isnan(point_z) == true)
                {
                    isValid = false;
                    break;
                }

                if(point_z < 2.0)
                {
                    isValid = true;
                }
            }
    
            if(isValid == true)
            {
                // Find the center of the dumpster and its center angle with respect to the robot's frame on the 2D plane
                dumpsterCenterX = sumZ / 4.0;
                dumpsterCenterZ = -sumX / 4.0;
                dumspterCenterAngle =  atan2 (dumpsterCenterZ,dumpsterCenterX) ;
                // then start the maneuver to get the robot to the front of the dumpster
                moving = false;
                movingLeft = true;
                Stop();
            }

        }

    }
}


void DumpsterFinder::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// convert the ROS image message to a CvImage
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
    cv::Mat Ori_image = cv_ptr->image;

    // new ROI 
    int xoffset = 150;
    int yoffset = 300;
    int img_width = 1700;
    int img_height = 300;

    // crop the image with new ROI
    cv::Rect crop_region(xoffset, yoffset, img_width, img_height);
    cv::Mat image = Ori_image(crop_region);

    cv::Mat grayImg;
    cv::cvtColor(image , grayImg, CV_BGR2GRAY);
    std::vector<cv::Vec3f>circles;
    cv::HoughCircles(grayImg, circles, cv::HOUGH_GRADIENT,1,20,50,20,15,40);

    // Check the candidate wheels and find the wheel with the largest radius 
    // from those whose centers are less than a certain value to each other.
    bool canAdd;
    Wheel.clear();
    for(size_t i = 0; i<circles.size(); i++)
    {
        cv::Vec3i c_ref = circles[i];
        canAdd = true;
        
        for(size_t j = 0; j<circles.size(); j++)
        {
            if(j == i)
            {
                continue;
            }

            cv::Vec3i c = circles[j];

            double dist = sqrt((c_ref[0] - c[0]) * (c_ref[0] - c[0]) + (c_ref[1] - c[1]) * (c_ref[1] - c[1]));

            if(dist < 100)
            {
                if(c_ref[2] <= c[2])
                {
                    canAdd = false;
                    break;
                }
            }

        }

        c_ref[0] = c_ref[0] + xoffset;
        c_ref[1] = c_ref[1] + yoffset;
        if(canAdd == true)
            Wheel.push_back(c_ref);
    }

    // draw the wheels with their indexes on the image
    int cnt =0;
    std::to_string(cnt);
    for( cv::Vec3i c : Wheel)
    {
        cv::putText(Ori_image, 
            std::to_string(cnt),
            cv::Point(c[0],c[1]), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1); // Anti-alias (Optional)

        cv::circle(Ori_image, cv::Point(c[0], c[1]), c[2], cv::Scalar(0, 0, 255), 3, 8);
        cv::circle(Ori_image, cv::Point(c[0], c[1]), 2, cv::Scalar(0, 255, 0), 3, 8);
        cnt++;
    }

	// Update the GUI window
	cv::imshow(OPENCV_WINDOW, image);
	cv::waitKey(3);
}



void DumpsterFinder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    // Robot moves along the wall 
    // keeps its vertical distance from the wall at a certain value.
    // This control is done by pid control.
    if(moving == true)
    {
        double startAngle = -80 * M_PI / 180.0;
        double stopAngle = 90 * M_PI / 180.0;
        double CurrentMin = getMinRangeValue(startAngle, stopAngle, *scan);

        double error = TARGET_DIST_FROM_OBSTACLE - CurrentMin;
        double error_diff = error - error_prev;
        error_sum += error;

        RotVel = Kp * error + Ki * error_sum + Kd * error_diff; 
        
        // Max rotvel of wafflebot is 1.82rad/sn
        if(RotVel > MAX_ROTVEL_SPEED)
            RotVel = MAX_ROTVEL_SPEED;

        if(RotVel < -MAX_ROTVEL_SPEED)
            RotVel = -MAX_ROTVEL_SPEED;
        
        // PID anti-windup prevention
        if(error_sum > 2.0)
            error_sum = 2.0;

        if(error_sum < -2.0)
            error_sum = -2.0;
        

        error_prev = error;

        // LinVel is restricted to RotVel to make turning maneuvers easier.
        //double LinVel = MAX_LINVEL_SPEED - abs(RotVel) * MAX_LINVEL_SPEED / 2 / MAX_ROTVEL_SPEED;

        double LinVel = MAX_LINVEL_SPEED;
        move(LinVel, RotVel);

    }


}



void DumpsterFinder::start()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");
    bool setTime = false;
    ros::Time start;
    ros::Time stop;
    double MovingTime;
    while (ros::ok()) 
    {
         if(movingForward)
        {
            ROS_INFO("movingForward");
            if(setTime == false)
            {
                start = ros::Time::now();
                setTime = true;
                //Stop();
                
                double TargetDist = 2 * M_PI * dumpsterCenterX / 4.0;
                MovingTime = abs(TargetDist / FORWARD_SPEED);
                RotVel = - FORWARD_SPEED / dumpsterCenterX;
                ROS_INFO("MovingTime = %0.2f", MovingTime);
                moveForward();
            }
            else
            {
                //Stop();
                stop = ros::Time::now();
                if((stop - start).toSec() >= MovingTime)
                {
                    Stop();
                    movingForward = false;
                    setTime = false;
                    movingRight = true;

                }
            }
        }
        else if(movingLeft)
        {
            ROS_INFO("movingLeft");
            if(setTime == false)
            {
                start = ros::Time::now();
                setTime = true;
                //Stop();
                
                double TargetAngle = dumspterCenterAngle + M_PI/2.0;
                MovingTime = TargetAngle / LEFT_ROTATE_SPEED;
                ROS_INFO("MovingTime = %0.2f", MovingTime);
                moveLeft();
            }
            else
            {
                //Stop();
                stop = ros::Time::now();
                if((stop - start).toSec() >= MovingTime)
                {
                    Stop();
                    movingLeft = false;
                    setTime = false;
                    movingForward = true;
                }
            }
        }
        else if(movingRight)
        {
            ROS_INFO("movingRight");
            if(setTime == false)
            {
                start = ros::Time::now();
                setTime = true;
                //Stop();
                
                double TargetAngle = M_PI/2.0;
                MovingTime = abs(TargetAngle / LEFT_ROTATE_SPEED);
                ROS_INFO("MovingTime = %0.2f", MovingTime);
                moveRight();
            }
            else
            {
                //Stop();
                stop = ros::Time::now();
                if((stop - start).toSec() >= MovingTime)
                {
                    Stop();
                    movingRight = false;
                    setTime = false;
                }
            }
        }
        
        ros::spinOnce(); 
        rate.sleep();
    }

}

// Returns the min range value between the starting and stopping angles
// from the received scanner ranges data.
double DumpsterFinder::getMinRangeValue(double startAngle, double stopAngle, sensor_msgs::LaserScan scan)
{
    // Wrap the angle between [0,2pi]
    if(startAngle < 0)
    {
        startAngle += 2*M_PI;
    }

    if(stopAngle < 0)
    {
        stopAngle += 2*M_PI;
    }

    // Find angle indexes
    int startIndex = ceil((startAngle - scan.angle_min) / scan.angle_increment);
    int stopIndex = ceil((stopAngle - scan.angle_min) / scan.angle_increment);
    int rangeSize = scan.ranges.size();

    // Calculate index count
    int indexCount = stopIndex - startIndex;

    // start index may be greater than stop idex
    if(startIndex > stopIndex)
        indexCount += rangeSize;

    float incrAngle = scan.angle_increment;
    float currentMinRange = scan.range_max;
    float currentMinAngle = 0;
    float rangeVal;
    int currentIndex;
    
    for(int i = 0; i<indexCount; i++)
    {
        // The mode can be taken because range data is circular.
        currentIndex = (i + startIndex) % rangeSize;

        // Find min range value
        rangeVal = scan.ranges[currentIndex];
        if(rangeVal < currentMinRange)
        {
            currentMinRange = rangeVal;
            currentMinAngle = (currentIndex) * incrAngle;
        }
    }

    // Wrap the angle between [0,2pi]
    if(currentMinAngle > 2*M_PI)
        currentMinAngle -= 2*M_PI;

    if(currentMinAngle < 0)
        currentMinAngle += 2*M_PI;

    currentMinAngle = (currentMinAngle) * 180 / M_PI;
    //ROS_INFO("startIndex= %d, stopIndex = %d, indexRange = %d, rangeSize = %d", startIndex, stopIndex, indexRange, rangeSize);
    //ROS_INFO("currentMinRange = %0.02f, currentMinAngle = %0.02f", currentMinRange, currentMinAngle);

    return currentMinRange;

}