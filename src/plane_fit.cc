#include <plane_fit.h>
#include <units.h>
#include <camera.h>
#include <depth.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <assert.h>

SpaceCoord PlaneFitDepthImage(cv::Mat depth_image,
                              Intrinsics intrinsics,
                              int step,
                              int iterations,
                              cm inlier_distance,
                              cm min_distance,
                              cm max_distance,
                              SpaceCoord& centroid){
    
    vector<SpaceCoord> point_cloud;
    for(int i = 0; i < depth_image.rows; i+=step){
        for(int j = 0; j < depth_image.cols; j+=step){
            dep raw_depth = depth_image.at<dep>(i,j);
            if(raw_depth == 0){
                continue;
            }
            cm depth = RawToMeters(raw_depth);
            ImCoord screen_pt = ImPoint(j,i);
            point_cloud.push_back(ProjectToSpace(screen_pt,depth,intrinsics));
        }
    }
    
    return RansacPlaneFit(point_cloud, iterations, inlier_distance,
                          min_distance, max_distance, centroid);
}

SpaceCoord RansacPlaneFit(vector<SpaceCoord> point_cloud,
                          int iterations,
                          cm inlier_distance,
                          cm min_distance,
                          cm max_distance,
                          SpaceCoord& centroid){
    
    
    SpaceCoord best_plane;
    vector<int> best_inliers;
    
    Eigen::Matrix<cm, 3, 1> n_ones;
    n_ones[0] = -1.;
    n_ones[1] = -1.;
    n_ones[2] = -1.;
    for(int i = 0; i < iterations; ++i){
        vector<SpaceCoord> plane_candidate;
        int idx_a = (float)rand() / RAND_MAX * point_cloud.size();
        plane_candidate.push_back(point_cloud[idx_a]);
        int idx_b = (float)rand() / RAND_MAX * point_cloud.size();
        plane_candidate.push_back(point_cloud[idx_b]);
        int idx_c = (float)rand() / RAND_MAX * point_cloud.size();
        plane_candidate.push_back(point_cloud[idx_c]);
        
        SpaceCoord plane = PlaneFit(plane_candidate);
        cm plane_norm = sqrt(plane[0] * plane[0] +
                             plane[1] * plane[1] +
                             plane[2] * plane[2]);
        cm plane_distance = fabs(plane[3]/plane_norm);
        if(plane_distance < min_distance ||
           plane_distance > max_distance){
            continue;
        }
        
        vector<int> inliers;
        for(int j = 0; j < point_cloud.size(); ++j){
            cm distance = fabs(point_cloud[j].dot(plane) / plane_norm);
            if(distance < inlier_distance){
                inliers.push_back(j);
            }
        }
        //cout << inliers.size() << " " << best_inliers.size() << endl;
        if(inliers.size() > best_inliers.size()){
            best_inliers = inliers;
            best_plane = plane;
        }
    }
    
    vector<SpaceCoord> best_points;
    centroid = SpacePoint(0,0,0);
    for(int i = 0; i < best_inliers.size(); ++i){
        best_points.push_back(point_cloud[best_inliers[i]]);
        centroid += point_cloud[best_inliers[i]];
    }
    centroid /= best_inliers.size();
    
    best_plane = PlaneFit(best_points);
    
    return best_plane;
}

SpaceCoord PlaneFit(vector<SpaceCoord> point_cloud){
    assert(point_cloud.size() > 2);
    Eigen::MatrixXd plane_fit(point_cloud.size(), 4);
    for(int i = 0; i < point_cloud.size(); ++i){
        plane_fit(i,0) = point_cloud[i][0];
        plane_fit(i,1) = point_cloud[i][1];
        plane_fit(i,2) = point_cloud[i][2];
        plane_fit(i,3) = point_cloud[i][3];
    }
    
    /*
    cout << "PF:" << endl;
    cout << plane_fit << endl;
    */
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(plane_fit,
                                          Eigen::ComputeFullV);
    /*
    cout << "V:" << endl;
    cout << svd.matrixV() << endl << endl;
    */
    SpaceCoord plane;
    plane[0] = svd.matrixV()(0,3);
    plane[1] = svd.matrixV()(1,3);
    plane[2] = svd.matrixV()(2,3);
    plane[3] = svd.matrixV()(3,3);
    
    return plane;
}
