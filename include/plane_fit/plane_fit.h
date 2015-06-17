#ifndef __ROCK_PLANE_FIT__
#define __ROCK_PLANE_FIT__

#include <units.h>
#include <camera.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <vector>

SpaceCoord PlaneFitDepthImage(cv::Mat depth_image,
                              Intrinsics intrinsics,
                              int step,
                              int iterations,
                              cm inlier_distance,
                              cm min_distance,
                              cm max_distance,
                              SpaceCoord& centroid,
                              std::vector<SpaceCoord>* plane_points = NULL);

// plane_points: if non NULL, copies over the 3d points detected to lie on 
// the plane of the table to plane_points.
SpaceCoord RansacPlaneFit(vector<SpaceCoord> point_cloud,
                          int iterations,
                          cm inlier_distance,
                          cm min_distance,
                          cm max_distance,
                          SpaceCoord& centroid,
                          std::vector<SpaceCoord>* plane_points = NULL);

SpaceCoord PlaneFit(std::vector<SpaceCoord> point_cloud);

#endif
