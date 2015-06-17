#include <stdio.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "detect_table.h"
#include "plane_fit.h"
#include "units.h"
#include "depth.h"
#include <Eigen/Geometry>

cv::Point ImCoordToPoint(const ImCoord &ic) {
  assert(ic[2] != 0);
  return cv::Point(ic[0]/ic[2], ic[1]/ic[2]);
}

cv::Point SpaceCoordToPoint(const SpaceCoord &sc, const Intrinsics &in) {
  return ImCoordToPoint(in * sc);
}


int main(int argc, char const *argv[])
{
  int step = 8;
  float tolerance = 0.01;
  int ransac_iters = 400;
  float min_distance = 0.25;
  float max_distance = 1.0;

  Intrinsics intrinsics = ReadIntrinsicsFromFile(
    "/data/rock/cameras/kinect2/kinect2.yml");

  cv::Mat depth = cv::imread(
    "/data/depth-images/depth_remap/depth_frame0000.png",
    CV_LOAD_IMAGE_UNCHANGED);
  SpaceCoord plane_centroid;
  vector<SpaceCoord> plane_points;
  SpaceCoord plane = PlaneFitDepthImage(
    depth,
    intrinsics,
    step,
    ransac_iters,
    tolerance,
    min_distance,
    max_distance,
    plane_centroid,
    &plane_points);

  cout << "Plane " << 0 << " "
  << plane[0] << " "
  << plane[1] << " "
  << plane[2] << " "
  << plane[3] << endl;

  float plane_norm = sqrt(plane[0] * plane[0] +
    plane[1] * plane[1] +
    plane[2] * plane[2]);

  image_color inlier_color(0,0,255);
  cv::Mat image = cv::imread("/data/rgb-images/rgb_frame0000.png");
  for(size_t j =0; j < depth.rows; j+=step){
    for(size_t k = 0; k < depth.cols; k+=step){
      dep raw_depth = depth.at<dep>(j,k);
      if(!raw_depth){
        continue;
      }
      cm z = RawToMeters(raw_depth);
      ImCoord pt = ImPoint(k,j);
      SpaceCoord position = ProjectToSpace(pt, z, intrinsics);
      cm distance = fabs(position.dot(plane) / plane_norm);
      if(distance < tolerance){
        image.at<image_color>(j,k) = inlier_color;
      }
    }
  }
  double x_half_len = 1.8/2.;
  double y_half_len = .75/2.;

  // find the rotation matrix for the normal of the plane
  Eigen::Vector3d normal(plane[0], plane[1], plane[2]);
  normal = normal / sqrt(normal.dot(normal));
  Eigen::Vector3d unit_std(0, 0, -1);
  // need quaternion representation
  Eigen::Quaterniond R_q;
  R_q.setFromTwoVectors(unit_std, normal);
  
  // find the translation matrix for the centroid
  Pose T;
  T.setIdentity();
  T.translation() = Eigen::Vector3d(plane_centroid[0]/plane_centroid[3],
    plane_centroid[1]/plane_centroid[3], 
    plane_centroid[2]/plane_centroid[3]);

  // final transform matrix
  Pose R(R_q);
  Pose final_transform = T * R;

  final_transform = getBestTable(final_transform, plane_points, x_half_len, 
    y_half_len);

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
      cv::line(image, p1_, p2_, color);
      cv::line(image, p2_, p3_, color);
      cv::line(image, p3_, p4_, color);
      cv::line(image, p4_, p1_, color);
  }

  // draw the center
  {
    SpaceCoord centre(0,0,0,1);
    cv::Point centre_ = SpaceCoordToPoint(final_transform * centre, intrinsics);
    cv::Scalar color(255, 0, 200);
    cv::circle(image, centre_, 10, color);    
  }
  string output_path = "/data/planefit-images/table_frame0000.png";
  cv::imwrite(output_path, image);
  return 0;
}
