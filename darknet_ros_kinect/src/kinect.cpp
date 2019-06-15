#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <opencv2/highgui.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_kinect/ObjectPositions.h>
#include <darknet_ros_kinect/ObjectPosition.h>

class Darknet_Kinect {
public:
	Darknet_Kinect();
	~Darknet_Kinect();
	cv::Mat color;
	cv::Mat depth;
	cv::Mat view_depth;
	pcl::PointCloud<pcl::PointXYZRGB> pc;

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

		pc = *temp_cloud;

		//展開
		for (int w = 0; w < width; ++w) {
			for (int h = 0; h < height; ++h) {
				auto point = temp_cloud->points[width * h + w];
				color.at<cv::Vec3b>(h, w)[0] = (int)point.b;
				color.at<cv::Vec3b>(h, w)[1] = (int)point.g;
				color.at<cv::Vec3b>(h, w)[2] = (int)point.r;
				x = point.x;
				y = point.y;
				z = point.z;
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

	void boundingboxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &data) {

		darknet_ros_kinect::ObjectPositions object_positions;

		object_positions.header = data->header;
		object_positions.image_header = data->image_header;


		for (auto box : data->bounding_boxes) {
			if (box.probability < 0.3) continue;

			/*
			cv::Point3d min_cost_point;
			double min_cost = DBL_MAX;
			double cost, z;

			for (int y = box.ymin; y < box.ymax; ++y) {
				for (int x = box.xmin; x < box.xmax; ++x) {
					z = depth.at<double>(y, x);
					if (std::isnan(z)) continue;
					cost = hypot_3d(cv::Point3d(x, box.ymin, 0), cv::Point3d(x, y, z));
					if (cost < min_cost) {
						cost = min_cost;
						min_cost_point.x = x;
						min_cost_point.y = y;
						min_cost_point.z = z;
					}
				}
			}
			cv::circle(color, cv::Point(min_cost_point.x, min_cost_point.y), 10, cv::Scalar(0, 200, 0), -1, CV_AA);
			*/

			int i = 1, image_x, image_y;
			cv::Point3d result;
			darknet_ros_kinect::ObjectPosition object_position;

			object_position.probability = box.probability;
			object_position.Class = box.Class;
			image_x = (box.xmax + box.xmin) / 2;
			image_y = (box.ymax + box.ymin) / 2;

			//auto point = pc.points[color.cols * image_y + image_x];
			//object_position.x = point.x;
			//object_position.y = point.y;
			//printf("%f\n", object_position.x );

			while (search_around(i++, cv::Point(image_x , image_y), color.cols, &result));
			object_position.x = result.x;
			object_position.y = result.y;
			object_position.z = result.z;

			cv::circle(color, cv::Point(image_x , image_y), 10, cv::Scalar(0, 200, 0), -1, CV_AA);
			cv::putText(color, std::to_string(object_position.z), cv::Point(image_x , image_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);

			object_positions.object_positions.push_back(object_position);

		}
		cv::imshow("objectW_result", color);
		cv::waitKey(1);

		object_position_data.publish(object_positions);
	}

private:
	ros::NodeHandle n;
	ros::Publisher kinect_image;
	ros::Publisher object_position_data;
	ros::Subscriber point_cloud_data;
	ros::Subscriber bounding_boxes_data;

	double hypot_3d(cv::Point3d first, cv::Point3d end) {
		double X = end.x - first.x;
		double Y = end.y - first.y;
		double Z = end.z - first.z;
		return sqrt(X * X + Y * Y + Z * Z);
	}

	bool isnan_point3d(cv::Point3d point) {
		bool flag = false;
		if (std::isnan(point.x)) flag = true;
		if (std::isnan(point.y)) flag = true;
		if (std::isnan(point.z)) flag = true;
		return flag;
	}

	bool isinf_point3d(cv::Point3d point) {
		bool flag = false;
		if (std::isinf(point.x)) flag = true;
		if (std::isinf(point.y)) flag = true;
		if (std::isinf(point.z)) flag = true;
		return flag;
	}

	cv::Point3d get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data, int width, cv::Point image) {
		double z = depth.at<double>(image.y, image.x);
		if (!(z >= 0.4 && z <= 4.0)) return cv::Point3d(0.0, 0.0, 0.0);
		auto point = data->points[width * image.y + image.x];
		return cv::Point3d(point.x, point.y, point.z);
	}

	bool search_around(int area, cv::Point image_center, int width, cv::Point3d *result) {
		//areaは奇数に限る
		if (area % 2 == 0) return true;
		if (area == 1) {
			*result = get_real_point_data(&pc, width, cv::Point(image_center.x, image_center.y));
			if (result->z == 0.0) return true;
			return false;
		}

		cv::Point min, max;
		cv::Point3d sum, tmp;
		double count = 0;

		min.x = image_center.x - (int)(area / 2);
		min.y = image_center.y - (int)(area / 2);
		max.x = image_center.x + (int)(area / 2);
		max.y = image_center.y + (int)(area / 2);

		for (int y = min.y; y < max.y; ++y) {
			for (int x = min.x; x < max.x; ++x) {
				tmp = get_real_point_data(&pc, width, cv::Point(x, y));
				if (tmp.z == 0.0) continue;
				sum.x += tmp.x;
				sum.y += tmp.y;
				sum.z += tmp.z;
				count++;
			}
		}

		if (count == 0) return true;

		result->x = sum.x / count;
		result->y = sum.y / count;
		result->z = sum.z / count;

		return false;
	}

};

Darknet_Kinect::Darknet_Kinect() {
	kinect_image = n.advertise<sensor_msgs::Image>("/darknet/rgb/image_raw", 1);
	object_position_data = n.advertise<darknet_ros_kinect::ObjectPositions>("/darknet_ros/object_position", 1);
	point_cloud_data = n.subscribe("/camera/depth_registered/points", 1, &Darknet_Kinect::point_cloud_data_callback, this);
	bounding_boxes_data = n.subscribe("/darknet_ros/bounding_boxes", 1, &Darknet_Kinect::boundingboxes_callback, this);

}

Darknet_Kinect::~Darknet_Kinect() {
	printf("Shutdown class of 'Darknet_Kinect'\n");
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Darknet_Kinect");

	Darknet_Kinect kinect;

	ros::spin();

}