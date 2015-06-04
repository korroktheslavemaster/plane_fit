#include "depth.h"

using namespace std;

//cm dep_to_cm[2048];

int CompareDepthPositions(ImCoord proj_a,
                          ImCoord proj_b,
                          cm z_a,
                          cm z_b,
                          cv::Mat depth_image){
    int row = (int)proj_a[1];
    int col = (int)proj_a[0];
    if(row >= depth_image.rows || col >= depth_image.cols || row<0 || col<0){
        return -1;
    }
    dep a_depth = depth_image.at<dep>(row, col);
    if(a_depth == 0){
        return -1;
    }
    
    row = (int)proj_b[1];
    col = (int)proj_b[0];
    if(row >= depth_image.rows || col >= depth_image.cols || row<0 || col<0){
        return -1;
    }
    dep b_depth = depth_image.at<dep>(row, col);
    if(b_depth == 0){
        return -1;
    }
    
    if(a_depth < b_depth){
        if(z_a < z_b){
            return 1;
        }
        else{
            return 0;
        }
    }
    else{
        if(z_a > z_b){
            return 1;
        }
        else{
            return 0;
        }
    }
}

int CompareDepthPatch(SpaceCoord position,
                      Intrinsics intrinsics,
                      cv::Mat depth_image,
                      cm depth_tolerance,
                      cm patch_size){
    
    //cout << "CPU\n";
    ImCoord projected_position = ProjectToImage(position, intrinsics);
    //cout << "CPU " << projected_position[0] << ", " << projected_position[1] << endl;
    
    //position[3] = 0;
    //cm expected_depth = position.norm();
    cm expected_depth = position[2];
    
    px x_offset = intrinsics(0,0)*patch_size/expected_depth;
    px y_offset = intrinsics(1,1)*patch_size/expected_depth;
    //cout << "CPU " << x_offset << " " << y_offset << endl;
    
    for(int xx = -1; xx < 2; xx += 2){
        for(int yy = -1; yy < 2; yy += 2){
            ImCoord corner_position = projected_position;
            corner_position[0] += xx * x_offset;
            corner_position[1] += yy * y_offset;
            int corner_response = CompareImCoordToDepthImage(corner_position,
                                                             expected_depth,
                                                             intrinsics,
                                                             depth_image,
                                                             depth_tolerance);
            if(corner_response == 0){
                return corner_response;
            }
        }
    }
    
    int center_response = CompareImCoordToDepthImage(projected_position,
                                                     expected_depth,
                                                     intrinsics,
                                                     depth_image,
                                                     depth_tolerance);
    
    
    return center_response;
}

int CompareImCoordToDepthImage(ImCoord position,
                               cm expected_depth,
                               Intrinsics intrinsics,
                               cv::Mat depth_image,
                               cm depth_tolerance){
    
    //ImCoord projected_position = ProjectToImage(position, intrinsics);
    int row = (int)position[1];
    int col = (int)position[0];
    if(row >= depth_image.rows || col >= depth_image.cols || row<0 || col<0){
        return 2;
    }
    
    //cout << position[0] << ", " << position[1] << ": ";
    
    //position[3] = 0.;
    //cm expected_depth = position.norm();
    return CompareMToDepth(expected_depth,
                           depth_image.at<dep>(row, col),
                           depth_tolerance);
}

int CompareMToDepth(cm expected_depth,
                     dep measured_depth,
                     cm depth_tolerance){
    
    cm real_depth = RawToMeters(measured_depth);
    //cout << " " << measured_depth << endl;
    //0.1236 * tan(measured_depth / 2842.5 + 1.1863);
    
    if(fabs(expected_depth - real_depth) < depth_tolerance){
        return 0;
    }
    else if(real_depth < expected_depth){
        return -1;
    }
    else{
        return 1;
    }
}

cm RawToMeters(dep depth){
    //assert(depth < 2048);
    return (cm)depth/1000.;
}
/*
void InitDepToCm(){
    
    //cm k1 = 1.1863;
    //cm k2 = 2842.5;
    //cm k3 = 0.1236;
    //for(size_t i = 0 ; i < 2048; ++i){
    //    dep_to_cm[i] = k3 * tanf(i/k2 + k1);
    //}
    for(size_t i = 0; i < 2048; ++i){
        dep_to_cm[i] = 1/(-0.00307 * i + 3.33);
    }
}
*/

vector<SpaceCoord> SpaceCoordsFromDepth(vector<ImCoord> pts, 
                                        cv::Mat depth_image, 
                                        vector<cm> offsets, 
                                        Intrinsics k,
                                        cm clip){
    vector<SpaceCoord> result;
    for(size_t i = 0; i < pts.size(); ++i){
        dep raw_z = depth_image.at<dep>((int)pts[i][1], (int)pts[i][0]);
        if(raw_z == 0){
            /*
            if(interpolate){
                int s = 0;
                vector<ImCoord> offsets;
                offsets.push_back(ImVec(-1,-1));
                offsets.push_back(ImVec(-1, 1));
                offsets.push_back(ImVec( 1, 1));
                offsets.push_back(ImVec( 1,-1));
                
                while(raw_z == 0){
                    s += 1;
                    
                    for(size_t
                    dep neighbor_z = 
                }
            }
            else{*/
                //cout << "||||||||||||||||||||||||||||||||||ZERO|||||||" << endl;
                //result.push_back(SpacePoint(0,0,0));
                continue;
            //}
        }
        cm z = RawToMeters(raw_z);
        for(size_t j = 0; j < offsets.size(); ++j){
            //cout << "Offset " << offsets[j] << endl;
            SpaceCoord pt = ProjectToSpace(pts[i], z+offsets[j], k);
            if(pt[2] > clip){
                result.push_back(pt);
            }
        }
    }
    
    return result;
}
