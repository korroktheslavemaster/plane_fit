#ifndef __ROCK_PLANE_FIT__
#define __ROCK_PLANE_FIT__

#include <units.h>
#include <camera.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

SpaceCoord PlaneFitDepthImage(cv::Mat depth_image,
                              Intrinsics intrinsics,
                              int step,
                              int iterations,
                              cm inlier_distance,
                              cm min_distance,
                              cm max_distance,
                              SpaceCoord& centroid);

SpaceCoord RansacPlaneFit(vector<SpaceCoord> point_cloud,
                          int iterations,
                          cm inlier_distance,
                          cm min_distance,
                          cm max_distance,
                          SpaceCoord& centroid);

SpaceCoord PlaneFit(vector<SpaceCoord> point_cloud);

#endif
