#ifndef __ROCK_DEPTH_H__
#define __ROCK_DEPTH_H__

#include "units.h"
#include "camera.h"

int CompareDepthPositions(ImCoord a,
                          ImCoord b,
                          cm z_a,
                          cm z_b,
                          cv::Mat depth_image);

int CompareDepthPatch(SpaceCoord position,
                      Intrinsics intrinsics,
                      cv::Mat depth_image,
                      cm depth_tolerance,
                      cm patch_size);

int CompareImCoordToDepthImage(ImCoord position,
                               cm expected_depth,
                               Intrinsics intrinsics,
                               cv::Mat depth_image,
                               cm depth_tolerance);

int CompareMToDepth(cm expected_depth,
                    unsigned short measured_depth,
                    cm depth_tolerance);

cm RawToMeters(dep depth);

vector<SpaceCoord> SpaceCoordsFromDepth(vector<ImCoord> pts,
                                        cv::Mat depth_image,
                                        vector<cm> offsets,
                                        Intrinsics k,
                                        cm clip = 0.25);

//void InitDepToCm();

#endif
