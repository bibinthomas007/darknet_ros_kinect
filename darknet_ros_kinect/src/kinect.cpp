#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>

class Darknet_Kinect {
public:
	Darknet_Kinect();
	~Darknet_Kinect();
	cv::Mat color;
	cv::Mat depth;
	cv::Mat view_depth;

private:
	ros::NodeHandle n;
	ros::Publisher kinect_image;
	ros::Publisher pose_estimator_3d;
	ros::Subscriber point_cloud_data;
	ros::Subscriber pose_estimator_2d;

	double hypot_3d(double x, double y, double z) {
		return sqrt(x * x + y * y + z * z);
	}

	void point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {

		int height = (int)input->height;
		int width = (int)input->width;
		double x, y, z;

		color = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
		depth = cv::Mat::zeros(cv::Size(width, height), CV_64F);

		//pcl変換
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*input, pcl_pc2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

		//展開
		for (int w = 0; w < width; ++w) {
			for (int h = 0; h < height; ++h) {
				color.at<cv::Vec3b>(h, w)[0] = (int)temp_cloud->points[width * h + w].b;
				color.at<cv::Vec3b>(h, w)[1] = (int)temp_cloud->points[width * h + w].g;
				color.at<cv::Vec3b>(h, w)[2] = (int)temp_cloud->points[width * h + w].r;
				x = temp_cloud->points[width * h + w].x;
				y = temp_cloud->points[width * h + w].y;
				z = temp_cloud->points[width * h + w].z;
				depth.at<double>(h, w) = z;
			}
		}

		depth.convertTo(view_depth, CV_8UC1, 255.0 / 4.0);

		cv::namedWindow("depth", CV_WINDOW_NORMAL);
		cv::imshow("depth", view_depth);

		cv::namedWindow("color", CV_WINDOW_NORMAL);
		cv::imshow("color", color);

		cv::waitKey(1);

		kinect_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", color));

	}

	void boundingboxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & data) {
		for (auto box : data->bounding_boxes) {
			if (box.probability < 0.3) continue;

			
		}
	}



};

Darknet_Kinect::Darknet_Kinect() {
	kinect_image = n.advertise<sensor_msgs::Image>("/tf_pose/kinect_image", 1);
	point_cloud_data = n.subscribe("/camera/depth_registered/points", 1, &Darknet_Kinect::point_cloud_data_callback, this);
}

Darknet_Kinect::~Darknet_Kinect() {
	printf("Shutdown class of 'Darknet_Kinect'\n");
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "tfpose_ros_kinect");

	Darknet_Kinect kinect;

	ros::spin();

}