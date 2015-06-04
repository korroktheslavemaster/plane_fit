#include "plane_fit.h"
#include "depth.h"
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv){
    

    
    //plane_pose.setIdentity();
    
    string model_name = "pop_tarts";
    float noise = 0.02;
    int step = 8;
    float tolerance = 0.01;
    int ransac_iters = 400;
    float min_distance = 0.25;
    float max_distance = 1.0;
    
    
    int start_image = 0;
    
    Intrinsics intrinsics = ReadIntrinsicsFromFile(
        "/data/rock/cameras/kinect2/kinect2.yml");
    
    cv::Mat depth = cv::imread(
        "/data/depth-images/depth_remap/depth_frame0000.png",
        CV_LOAD_IMAGE_UNCHANGED);
    SpaceCoord plane_centroid;
    SpaceCoord plane = PlaneFitDepthImage(
            depth,
            intrinsics,
            step,
            ransac_iters,
            tolerance,
            min_distance,
            max_distance,
            plane_centroid);
    
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
    
    string output_path = "/data/planefit-images/planefit_fram0000.png";
    cv::imwrite(output_path, image);
    
    /*
    vector<SpaceCoord> plane_points;
    for(int i = 0; i < 20; ++i){
        for(int j = 0; j < 20; ++j){
            float r = (float)rand() / RAND_MAX * 2 * noise - noise;
            SpaceCoord plane_point = plane_pose * SpacePoint(i,j,r);
            plane_points.push_back(plane_point);
            //cout << plane_point << endl << endl;
        }
    }
    
    SpaceCoord plane = RansacPlaneFit(plane_points, 100, 0.01);
    
    PrintPose(plane_pose);
    std::cout
        << std::endl
        << plane[0] << "," << plane[1] << "," << plane[2] << "," << plane[3]
        << std::endl;
    */
    return 0;
};
