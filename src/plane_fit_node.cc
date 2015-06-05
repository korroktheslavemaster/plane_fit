#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "plane_fit.h"
#include "depth.h"

#include "plane_fit/PlaneFit.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"

#include <assert.h>
#include <Eigen/Geometry>

cv::Point ImCoordToPoint(const ImCoord &ic) {
    assert(ic[2] != 0);
    return cv::Point(ic[0]/ic[2], ic[1]/ic[2]);
}

cv::Point SpaceCoordToPoint(const SpaceCoord &sc, const Intrinsics &in) {
    return ImCoordToPoint(in * sc);
}

cv::Mat currentDepthImage;
cv::Mat currentColorImage;
sensor_msgs::CameraInfo currentCameraInfo;
bool has_camera_info_ = false;

namespace Constants {
  // should make these changeable through ros params/service/topics
  float noise = 0.02;
  int step = 8;
  float tolerance = 0.01;
  int ransac_iters = 400;
  float min_distance = 0.25;
  float max_distance = 1.0;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    currentDepthImage = cv_bridge::toCvCopy(msg, "16UC1")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
  }
}
void colorImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    currentColorImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

// Callback for camera info
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    currentCameraInfo = (*camera_info);
    has_camera_info_ = true;
}

bool plane_fitting(plane_fit::PlaneFit::Request &req,
         plane_fit::PlaneFit::Response &res)
{
  using namespace Constants;
  ROS_INFO("request: received");
  if (!has_camera_info_) {
    ROS_INFO("did not have camera_info ready!");
    return false;
  }
  SpaceCoord plane_centroid;
  Intrinsics intrinsics;
  intrinsics << currentCameraInfo.K[0], 0., currentCameraInfo.K[2], 0.,
                0., currentCameraInfo.K[4], currentCameraInfo.K[5], 0.,
                0., 0., 1., 0.;

  SpaceCoord plane = PlaneFitDepthImage(
          currentDepthImage,
          intrinsics,
          step,
          ransac_iters,
          tolerance,
          min_distance,
          max_distance,
          plane_centroid);

  res.p0 = plane[0];
  res.p1 = plane[1];
  res.p2 = plane[2];
  res.p3 = plane[3];

  

  // find the rotation matrix for the normal of the plane
  Eigen::Vector3d normal(plane[0], plane[1], plane[2]);
  normal = normal / sqrt(normal.dot(normal));
  Eigen::Vector3d unit_std(0, 0, 1);
  // need quaternion representation
  Eigen::Quaterniond R_q;
  R_q.setFromTwoVectors(unit_std, normal);
  
  // find the translation matrix for the centroid
  Pose T;
  T.setIdentity();
  T.translation() = Eigen::Vector3d(plane_centroid[0]/plane_centroid[3],
      plane_centroid[1]/plane_centroid[3], 
      plane_centroid[2]/plane_centroid[3]);

  // bounding box halflens
  double x_half_len = 1.8/2.;
  double y_half_len = .75/2.;
  // write everything into response
  {
    Eigen::Vector3d t = T.translation();
    res.pose.position.x = t.x();
    res.pose.position.y = t.y();
    res.pose.position.z = t.z();
    
    res.pose.orientation.x = R_q.x();
    res.pose.orientation.y = R_q.y();
    res.pose.orientation.z = R_q.z();
    res.pose.orientation.w = R_q.w();

    res.bb_min.x = -x_half_len;
    res.bb_min.y = -y_half_len;
    res.bb_min.z = 0;
    res.bb_max.x = x_half_len;
    res.bb_max.y = y_half_len;
    res.bb_max.z = 0;
  }
  
  ROS_INFO("sending back response (plane): [%lf] [%lf] [%lf] [%lf]", 
    res.p0, res.p1, res.p2, res.p3);
  // debugging purpose: show the plane on an rgb image.
  cv::Mat displayImage = currentColorImage.clone();
  cv::resize(displayImage, displayImage, currentDepthImage.size());
  
  // draw a circle at centroid
  ImCoord centroid = intrinsics * plane_centroid;
  cv::circle(displayImage, cv::Point(centroid[0]/centroid[2], 
      centroid[1]/centroid[2]), 10, cv::Scalar(255,0,0));
  // make some lines on the x, y, z directions to get an idea of orientation
  // around the centroid
  SpaceCoord xpoint = plane_centroid; xpoint[0] += 0.2*xpoint[3];
  SpaceCoord ypoint = plane_centroid; ypoint[1] += 0.2*ypoint[3];
  SpaceCoord zpoint = plane_centroid; zpoint[2] += 0.2*zpoint[3];

  cv::line(displayImage, SpaceCoordToPoint(plane_centroid, intrinsics), 
      SpaceCoordToPoint(xpoint, intrinsics), cv::Scalar(255,0,0));


  cv::line(displayImage, SpaceCoordToPoint(plane_centroid, intrinsics), 
      SpaceCoordToPoint(ypoint, intrinsics), cv::Scalar(0,255,0));


  cv::line(displayImage, SpaceCoordToPoint(plane_centroid, intrinsics), 
      SpaceCoordToPoint(zpoint, intrinsics), cv::Scalar(0,0,255));

  // final transform matrix
  Pose R(R_q);
  Pose final_transform = T * R;

  // now generate corners of the cuboid, and draw them...
  // first just try drawing a rectangle on the plane, centred on centroid
  
  SpaceCoord p1(x_half_len, y_half_len, 0, 1);
  SpaceCoord p2(-x_half_len, y_half_len, 0, 1);
  SpaceCoord p3(-x_half_len, -y_half_len, 0, 1);
  SpaceCoord p4(x_half_len, -y_half_len, 0, 1);

  {
    cv::Point p1_ = SpaceCoordToPoint(final_transform * p1, intrinsics);
    cv::Point p2_ = SpaceCoordToPoint(final_transform * p2, intrinsics);
    cv::Point p3_ = SpaceCoordToPoint(final_transform * p3, intrinsics);
    cv::Point p4_ = SpaceCoordToPoint(final_transform * p4, intrinsics);
    cv::Scalar color(255, 200, 0);
    cv::line(displayImage, p1_, p2_, color);
    cv::line(displayImage, p2_, p3_, color);
    cv::line(displayImage, p3_, p4_, color);
    cv::line(displayImage, p4_, p1_, color);
  }
  {
    // draw another rectangle with z displacement
    p1.z() += 0.2; p2.z() += 0.2; p3.z() += 0.2; p4.z() += 0.2;

    cv::Point p1_ = SpaceCoordToPoint(final_transform * p1, intrinsics);
    cv::Point p2_ = SpaceCoordToPoint(final_transform * p2, intrinsics);
    cv::Point p3_ = SpaceCoordToPoint(final_transform * p3, intrinsics);
    cv::Point p4_ = SpaceCoordToPoint(final_transform * p4, intrinsics);
    cv::Scalar color(255, 200, 0);
    cv::line(displayImage, p1_, p2_, color);
    cv::line(displayImage, p2_, p3_, color);
    cv::line(displayImage, p3_, p4_, color);
    cv::line(displayImage, p4_, p1_, color);
  }
  {
    // draw connecting lines between rectangles
    SpaceCoord p1_orig = p1; p1_orig.z() -= 0.2;
    SpaceCoord p2_orig = p2; p2_orig.z() -= 0.2;
    SpaceCoord p3_orig = p3; p3_orig.z() -= 0.2;
    SpaceCoord p4_orig = p4; p4_orig.z() -= 0.2;

    cv::Point p1_ = SpaceCoordToPoint(final_transform * p1, intrinsics);
    cv::Point p2_ = SpaceCoordToPoint(final_transform * p2, intrinsics);
    cv::Point p3_ = SpaceCoordToPoint(final_transform * p3, intrinsics);
    cv::Point p4_ = SpaceCoordToPoint(final_transform * p4, intrinsics);

    cv::Point p1_orig_ = SpaceCoordToPoint(final_transform * p1_orig, intrinsics);
    cv::Point p2_orig_ = SpaceCoordToPoint(final_transform * p2_orig, intrinsics);
    cv::Point p3_orig_ = SpaceCoordToPoint(final_transform * p3_orig, intrinsics);
    cv::Point p4_orig_ = SpaceCoordToPoint(final_transform * p4_orig, intrinsics);

    cv::Scalar color(255, 200, 0);
    cv::line(displayImage, p1_, p1_orig_, color);
    cv::line(displayImage, p2_, p2_orig_, color);
    cv::line(displayImage, p3_, p3_orig_, color);
    cv::line(displayImage, p4_, p4_orig_, color);

  }
  float plane_norm = sqrt(plane[0] * plane[0] +
                    plane[1] * plane[1] +
                    plane[2] * plane[2]);
  image_color inlier_color(0,0,255);
  
  for(size_t j =0; j < currentDepthImage.rows; j+=step){
    for(size_t k = 0; k < currentDepthImage.cols; k+=step){
      dep raw_depth = currentDepthImage.at<dep>(j,k);
      if(!raw_depth){
        continue;
      }
      cm z = RawToMeters(raw_depth);
      ImCoord pt = ImPoint(k,j);
      SpaceCoord position = ProjectToSpace(pt, z, intrinsics);
      cm distance = fabs(position.dot(plane) / plane_norm);
      if(distance < tolerance){
        displayImage.at<image_color>(j,k) = inlier_color;
      }
    }
  }
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::imshow("view", displayImage);
  cv::waitKey(0);
  cv::destroyWindow("view");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport depth_it(nh);
  // rename the image topic before using
  image_transport::Subscriber depth_sub = 
    depth_it.subscribe("depth_image", 1, depthImageCallback);  
  
  image_transport::ImageTransport color_it(nh);
  // rename the image topic before using
  image_transport::Subscriber color_sub = 
    color_it.subscribe("color_image", 1, colorImageCallback);

  // camera_info topic, rename this too
  ros::Subscriber info_subscriber = nh.subscribe("camera_info", 10, 
    &cameraInfoCallback);
  ros::ServiceServer service = nh.advertiseService("plane_fit", plane_fitting);
  ROS_INFO("Ready to fit planes.");

  ros::spin();
  return 0;
}