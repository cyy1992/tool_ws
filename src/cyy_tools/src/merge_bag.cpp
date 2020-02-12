#include "stdio.h"
// #include "g2otypes.h"
#include "glog/logging.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"
#include "fstream"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include "Thirdparty/sophus/sophus/so3.hpp"
// #include "Thirdparty/sophus/sophus/se3.hpp"

#include "poslvx/INS.h"
#include "poslvx/INSRMS.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace std;
using namespace Eigen;
int main(int argc, char **argv){
// 	google::InitGoogleLogging(argv[0]);
// 	FLAGS_colorlogtostderr=true;
// 	FLAGS_alsologtostderr = true;
// 	google::InstallFailureSignalHandler();
		
    ros::init(argc, argv, "merge_bag");
    ros::start();
	ros::NodeHandle pnh("~");	
	
	string bagname1, bag_with_imu_name, bag_merge_name;
	bool getbagname1=pnh.getParam("bag1", bagname1);
	bool getbagimu=pnh.getParam("bag2_with_imu", bag_with_imu_name);
	bool getbagmerge=pnh.getParam("bag_merge", bag_merge_name);
	
	if(!(getbagimu&&getbagname1&&getbagmerge)){
		cout<<"can get bag name"<<endl;
	}
	
	string left_image_name = "/stereo_grey/left/image_raw";
	string right_image_name = "/stereo_grey/right/image_raw";
	string imu_msg_name = "/xsens/imu_data";
	
	std::vector<string> topics1;
// 	topics1.push_back("/stereo_grey/left/image_raw");
// 	topics1.push_back("/stereo_grey/right/image_raw");
	topics1.push_back("/ins/data");
	topics1.push_back("/lidar_center/velodyne_points");
	topics1.push_back("/xsens/imu_data");
		
	std::vector<string> topics2;
	topics2.push_back("/ins/data");
	topics2.push_back("/ins/rms");
// 	topics2.push_back(imu_msg_name);
// 	topics2.push_back(left_image_name);
// 	topics2.push_back(right_image_name);
	topics2.push_back("/lidar_center/velodyne_points");

	rosbag::Bag new_bag_merge, bag1, bag_with_imu;
	new_bag_merge.open(bag_merge_name, rosbag::bagmode::Write);
	bag1.open(bagname1, rosbag::bagmode::Read);
	bag_with_imu.open(bag_with_imu_name, rosbag::bagmode::Read);
	
	rosbag::View view1(bag1, rosbag::TopicQuery(topics1));
	rosbag::View view_with_imu(bag_with_imu, rosbag::TopicQuery(topics2));
	
	cout<<"----bags opened!----" << endl;
	
	foreach(rosbag::MessageInstance const m, view1){
		sensor_msgs::ImageConstPtr im_msg_left = m.instantiate<sensor_msgs::Image>();
		if((im_msg_left!=nullptr) && (m.getTopic() == left_image_name)){
			new_bag_merge.write(left_image_name, im_msg_left->header.stamp, *im_msg_left);
		}else if((im_msg_left!=nullptr) && (m.getTopic() == right_image_name)){
			new_bag_merge.write(right_image_name, im_msg_left->header.stamp, *im_msg_left);
		}

		
		sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
		if((imu_msg!=nullptr) && m.getTopic()==imu_msg_name ){
			new_bag_merge.write(imu_msg_name, imu_msg->header.stamp, *imu_msg);
		}
		
		sensor_msgs::PointCloud2Ptr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
		if((lidar_msg!=nullptr) && (m.getTopic()=="/lidar_center/velodyne_points")){
			new_bag_merge.write("/lidar_center/velodyne_points", lidar_msg->header.stamp, *lidar_msg);

		}
		
		poslvx::INSConstPtr gps_msg = m.instantiate<poslvx::INS>();
		if((gps_msg!=nullptr) && (m.getTopic()=="/ins/data")){
			poslvx::INS new_gps = *gps_msg;
			new_gps.header.stamp.fromSec(gps_msg->header.stamp.toSec()-6.500);
			new_bag_merge.write("/ins/data", new_gps.header.stamp, new_gps);
// 			cout << m.getTopic() <<" in ins" << endl;
		}
		
    poslvx::INSRMSConstPtr rms_msg = m.instantiate<poslvx::INSRMS>();
    if((rms_msg!=nullptr) && (m.getTopic()=="/ins/rms")){
      poslvx::INSRMS new_rms = *rms_msg;
      new_rms.header.stamp.fromSec(rms_msg->header.stamp.toSec()-6.50);
      new_bag_merge.write("/ins/rms", new_rms.header.stamp, new_rms);
    }
		
	}
	

	
	cout << "add one bag done!" <<endl;
	foreach(rosbag::MessageInstance const m, view_with_imu){
		poslvx::INSConstPtr gps_msg = m.instantiate<poslvx::INS>();
		if((gps_msg!=nullptr) && (m.getTopic()=="/ins/data")){
			poslvx::INS new_gps = *gps_msg;
			new_gps.header.stamp.fromSec(gps_msg->header.stamp.toSec()-6.50);
			new_bag_merge.write("/ins/data", new_gps.header.stamp, new_gps);
		}
		
		poslvx::INSRMSConstPtr  rms_msg= m.instantiate<poslvx::INSRMS>();
		if((rms_msg!=nullptr) && (m.getTopic()=="/ins/rms")){
			poslvx::INSRMS new_rms = *rms_msg;
			new_rms.header.stamp.fromSec(rms_msg->header.stamp.toSec()-6.50);
			new_bag_merge.write("/ins/rms", new_rms.header.stamp, new_rms);
		}
		
		sensor_msgs::PointCloud2Ptr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
		if((lidar_msg!=nullptr) && (m.getTopic()=="/lidar_center/velodyne_points")){
			new_bag_merge.write("/lidar_center/velodyne_points", lidar_msg->header.stamp, *lidar_msg);
		}
		
		sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
		if((imu_msg!=nullptr) &&(m.getTopic()==imu_msg_name)){
			new_bag_merge.write(imu_msg_name, imu_msg->header.stamp, *imu_msg);
		}
		
		sensor_msgs::ImageConstPtr im_msg_left = m.instantiate<sensor_msgs::Image>();
		if((im_msg_left!=nullptr) && (m.getTopic() == left_image_name)){
			new_bag_merge.write(left_image_name, im_msg_left->header.stamp, *im_msg_left);
		}else if((im_msg_left!=nullptr) && (m.getTopic() == right_image_name)){
			new_bag_merge.write(right_image_name, im_msg_left->header.stamp, *im_msg_left);
		}

	}
	cout << "done!" <<endl;
	new_bag_merge.close();
	bag1.close();
  
	bag_with_imu.close();
	
}