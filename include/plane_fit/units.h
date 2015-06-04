#ifndef __ROCK_UNITS_H__
#define __ROCK_UNITS_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cv.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/unordered_set.hpp>
#include "units_forward.h"

/*
This file sets the types used for various units via typedefs.
This is a halfstep towards generality that lets me avoid writing templates.
*/

#define RNG_ boost::mt19937
#define SMALL_LIMIT 0.000000001

// The 'color' type is for colors.
const int INTEGRAL_IMAGE_TYPE = CV_32S;
//typedef cv::Vec3d integral_color;
//typedef double integral_color_channel;
typedef cv::Vec3i integral_color;
typedef int integral_color_channel;

typedef cv::Vec3b image_color;
typedef uchar image_color_channel;

// The following typedefs standardize the types used for points/vectors in
// space, points/vectors in image coordinates, poses and camera intrinsics.
// M Koval notes that it is possible to use 3d values for this
// and then use specific functions to do either point or vector multiply
// might be worth switchign at some point.
typedef Eigen::Matrix<cm, 4, 1> SpaceCoord;
typedef Eigen::Matrix<px, 3, 1> ImCoord;

//typedef Eigen::Matrix<cm, 3, 1> TruncSpaceCoord;
//TruncSpaceCoord TruncateSpaceCoordinate(SpaceCoord);

typedef Eigen::Transform<cm, 3, Eigen::Affine> Pose;

// This is used a few places, so I'm putting it here
typedef boost::unordered_set<int> int_set;

// These functions are a simple way to create a new point or vector
// in space or in image coordinates.  The distinction between point and vector
// lies in the homogenous coordinate.  A point represents a position in space
// and has a homogenous coordinate of 1, a vector represents a direction with
// magnitude and has a homogenous coordinate of 0.  Practically this means that
// the translation of a transformation will affect a point, but not a vector.
static inline SpaceCoord SpacePoint(cm x, cm y, cm z){
    return SpaceCoord(x, y, z, 1);
}

static inline SpaceCoord SpaceVec(cm x, cm y, cm z){
    return SpaceCoord(x, y, z, 0);
}

static inline ImCoord ImPoint(px u, px v){
    return ImCoord(u, v, 1);
}

static inline ImCoord ImVec(px u, px v){
    return ImCoord(u, v, 0);
}

class Barycentric{
public:
    float r;
    float s;
    float t;
    
    Barycentric();
    Barycentric(float, float, float);
    Barycentric(float, float);
    Barycentric(ImCoord pt, ImCoord a, ImCoord b, ImCoord c);
};

#endif
