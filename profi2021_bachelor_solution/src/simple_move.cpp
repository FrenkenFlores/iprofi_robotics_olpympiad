#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <hector_uav_msgs/EnableMotors.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Camera from Quadrator";


class SimpleMover {

	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber image_sub;
	ros::ServiceClient motor_client;
	ros::Rate rate = ros::Rate(30);
	
	cv::Mat dst, cdst, cdstP;
	cv_bridge::CvImagePtr cv_ptr;

	double altitude_desired;
	float average_x_value;
	float average_y_value;

  public:

	SimpleMover() : average_x_value(0), average_y_value(0) {
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		image_sub = nh.subscribe("/cam_1/camera/image", 1, &SimpleMover::camera_cb, this);
		motor_client = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

		if (!nh.getParam("/profi2021_bachelor_solution/altitude_desired", altitude_desired)) {
			ROS_ERROR("Failed to get param '/profi2021_bachelor_solution/altitude_desired'");
		}

		cv::namedWindow(OPENCV_WINDOW);

		ros::Duration(1).sleep();       // требуется для инициализации времени
	}                                   // при слишком быстром старте узла


	~SimpleMover() {
		cv::destroyWindow(OPENCV_WINDOW);
	}


	void detect_line(cv_bridge::CvImagePtr &cv_ptr) {

		float x_average_HoughLines = 0;
		float x_average_HoughLinesP = 0;
		this->average_x_value = 0;
		this->average_y_value = 0;
		this->average_y_value = cdst.size().height / 2;
		Canny(cv_ptr->image, dst, 50, 200, 3);
		cvtColor(dst, cdst, COLOR_GRAY2BGR);
    	cdstP = cdst.clone();
		std::vector<Vec2f> lines;
		HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 );
		for( size_t i = 0; i < lines.size(); i++ )
		{
			float rho = lines[i][0], theta = lines[i][1];
			cv::Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			x_average_HoughLines = ((pt1.x + pt2.x) / 2) + x_average_HoughLines;
			cv::line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
		}
		
		vector<Vec4i> linesP;
		HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
		for( size_t i = 0; i < linesP.size(); i++ )
		{
			Vec4i l = linesP[i];
			x_average_HoughLinesP = ((l[0] + l[2]) / 2) + x_average_HoughLinesP;
			line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
		}
		this->average_x_value = ((x_average_HoughLines / lines.size()) + (x_average_HoughLinesP / linesP.size())) / 2;
		cv::circle( cdst, cv::Point(this->average_x_value, this->average_y_value), 10, CV_RGB(0,255,0));
		cv::circle( cdstP, cv::Point(this->average_x_value, this->average_y_value), 10, CV_RGB(0,0,255));
	}

	void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			detect_line(cv_ptr);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		show_image(cv_ptr);
    	imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    	imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
	}


	void show_image(const cv_bridge::CvImagePtr cv_ptr) {
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
	}


	void enable_motors() {
		ros::service::waitForService("/enable_motors");
		hector_uav_msgs::EnableMotors srv;
		srv.request.enable = true;
		if (!motor_client.call(srv)) {
			ROS_ERROR("Failed to call service enable_motors");
		}
	}

	void take_off() {

		enable_motors();

		double cur_time = ros::Time::now().toSec();
		double end_time = cur_time + 3.0;
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.z = 1.0;

		while ( nh.ok() && (cur_time < end_time)) {
			cmd_vel_pub.publish(twist_msg);
			ros::spinOnce();
			rate.sleep();
			cur_time = ros::Time::now().toSec();
		}
	}


	void spin() {

		take_off();

		double start_time = ros::Time::now().toSec();
		while (nh.ok()) {
			geometry_msgs::Twist twist_msg;

			double delta_time = ros::Time::now().toSec() - start_time;
			twist_msg.linear.z = 0.05 * cos(1.2 * delta_time);
			twist_msg.linear.y = 1 * (twist_msg.linear.y - this->average_y_value) + 0.05 * sin(0.6 * delta_time);
			// twist_msg.linear.y = 0.05 * sin(0.6 * delta_time);
			twist_msg.linear.x = 1 * (twist_msg.linear.x - this->average_x_value) + 0.05 * sin(0.6 * delta_time);
			// twist_msg.linear.x = 0.05 * sin(0.6 * delta_time);
			cmd_vel_pub.publish(twist_msg);

			ros::spinOnce();
			rate.sleep();
		}
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_mover");

	SimpleMover simpleMover;
	simpleMover.spin();

  return 0;
}
