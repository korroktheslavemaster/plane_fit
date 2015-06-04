#ifndef __ROCK_CAMERA_H__
#define __ROCK_CAMERA_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <units.h>

using namespace std;

// Camera intrinsics map centimeters to pixels.   Since these are two different
// types, the one with the greatest precision should be used here.  At the
// moment, they're both the same, but I imagine that spatial units (centimeters)
// are more likely to be converted to doubles at some point than pixels, so I
// will use cm for now.
typedef Eigen::Matrix<cm, 3, 4> Intrinsics;

ImCoord ProjectToImage(SpaceCoord, Intrinsics);

SpaceCoord ProjectToSpace(ImCoord pt, cm z, Intrinsics k);

void ImHomogenize(ImCoord& pt);

Intrinsics ReadIntrinsicsFromFile(string file_path);

void PrintIntrinsics(Intrinsics);

px IntrinsicsPxPerCm(Intrinsics);

#endif
