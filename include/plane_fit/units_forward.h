#ifndef __ROCK_UNITS_FORWARD_H__
#define __ROCK_UNITS_FORWARD_H__

// The 'cm' type is for data measured in physical space in meters (I know).
typedef double cm;

// The 'px' type is for data measured in screen/image space in pixels.
typedef double px;

// The 'rad' type is for angles in radians.
typedef double rad;

// The 'dep' type is for kinect depth values.
typedef unsigned short dep;

class Barycentric;

namespace Eigen{
    template<typename, int, int, int, int, int> class Matrix;
    template<typename, int, int, int> class Transform;
};

typedef Eigen::Matrix<cm, 4, 1, 0, 4, 1> SpaceCoord;
typedef Eigen::Matrix<px, 3, 1, 0, 3, 1> ImCoord;
typedef Eigen::Transform<cm, 3, 2, 0> Pose;
typedef Eigen::Matrix<cm, 3, 4, 0, 3, 4> Intrinsics;
#endif
