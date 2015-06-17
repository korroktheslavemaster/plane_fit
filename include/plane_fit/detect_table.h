#ifndef _DETECT_TABLE_H_
#define _DETECT_TABLE_H_ 
#include <units.h>
#include <camera.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <vector>

Pose getBestTable(const Pose base_pose, const vector<SpaceCoord> &
  plane_points, double x_half_len, double y_half_len);

Pose getBestTableOpt(const Pose base_pose, const vector<SpaceCoord> &
  plane_points, double x_half_len, double y_half_len);
#endif