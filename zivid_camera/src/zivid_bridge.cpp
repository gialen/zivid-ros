#include <zivid_bridge.h>


zivid_bridge::zivid_bridge()
{
  ros::NodeHandle n;

  ROS_INFO("Starting zivid_bridge.cpp");

  flag_pc2_ = false;
  flag_img_ = false;

  // Subscibers
  points_sub_             = n.subscribe("/zivid_camera/points/xyzrgb", 1, &zivid_bridge::on_points, this);
  image_color_sub_        = n.subscribe("/zivid_camera/color/image_color", 1, &zivid_bridge::on_image_color, this);
  // Publishers
  points_pub_ 		        = n.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
  image_color_pub_ 	      = n.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
  // Servers
  server_camera_capture_  = n.advertiseService("cameraRGBD", &zivid_bridge::callback_camera_data, this);
  // Clients
  load_settings_          = n.serviceClient<zivid_camera::LoadSettingsFromFile>("/zivid_camera/load_settings_from_file");
}

zivid_bridge::~zivid_bridge(){}


void zivid_bridge::on_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("PointCloud received");
  msg_pc2_  = *msg;
  points_pub_.publish(msg_pc2_);
  flag_pc2_ = true;
}

void zivid_bridge::on_image_color(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("2D color image received");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr   = cv_bridge::toCvCopy(msg, "rgb8");                                            // Transforming image encoding to rgb8
  msg_img_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_ptr->image).toImageMsg();
  image_color_pub_.publish(*msg_img_);
  flag_img_ = true;
}

bool zivid_bridge::callback_camera_data(rpwc_msgs::PCloudImageReq::Request &req, rpwc_msgs::PCloudImageReq::Response &res)
{
  zivid_camera::LoadSettingsFromFile file;
  //file.request.file_path = req.file_path;
  
  file.request.file_path = ros::package::getPath("zivid_samples") + "/test10sec.yml";
  if (load_settings_.call(file)) ROS_INFO("Loading image settings");
  else ROS_ERROR("Failed to call service load_settings_from_file");  

  capture();

  while (!flag_pc2_ || !flag_img_)
  {
    usleep(1000);
    ros::spinOnce();
  }

  flag_pc2_ = false;
  flag_img_ = false;

  //res.data.pointCloud = msg_pc2_;
  //res.data.image = *msg_img_;
  ROS_INFO("finito");
  return true;
}

void zivid_bridge::capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}