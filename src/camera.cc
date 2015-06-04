#include <camera.h>
#include <fstream>
#include "yaml-cpp/yaml.h"

ImCoord ProjectToImage(SpaceCoord pt, Intrinsics k){
    //SpatialHomogenize(pt);
    ImCoord result = k * pt;
    ImHomogenize(result);
    return result;
}

SpaceCoord ProjectToSpace(ImCoord pt, cm z, Intrinsics k){
    cm x = (pt[0] - k(0,2))*z/k(0,0);
    cm y = (pt[1] - k(1,2))*z/k(1,1);
    return SpacePoint(x,y,z);
}

void ImHomogenize(ImCoord& pt){
    pt[0] /= pt[2];
    pt[1] /= pt[2];
    pt[2] = 1.0;
}

Intrinsics ReadIntrinsicsFromFile(string file_path){
    YAML::Node doc = YAML::LoadFile(file_path.c_str());
    Intrinsics result;
    
    for(YAML::const_iterator field = doc.begin(); field!=doc.end(); ++field){
        string key = field->first.as<string>();
        if(key == "k"){
            const YAML::Node& value = field->second;
            YAML::const_iterator it = value.begin();
            result(0,0) = it->as<double>(); ++it;
            result(1,1) = it->as<double>(); ++it;
            result(0,2) = it->as<double>(); ++it;
            result(1,2) = it->as<double>(); ++it;
            result(0,1) = 0.;
            result(0,3) = 0.;
            result(1,0) = 0.;
            result(1,3) = 0.;
            result(2,0) = 0.;
            result(2,1) = 0.;
            result(2,2) = 1.;
            result(2,3) = 0.;
        }
    }
    
    return result;
}

void PrintIntrinsics(Intrinsics I){
    cout << I(0,0) << " " << I(0,1) << " " << I(0,2) << " " << I(0,3) << " ";
    cout << endl;
    cout << I(1,0) << " " << I(1,1) << " " << I(1,2) << " " << I(1,3) << " ";
    cout << endl;
    cout << I(2,0) << " " << I(2,1) << " " << I(2,2) << " " << I(2,3) << " ";
    cout << endl;
}

px IntrinsicsPxPerCm(Intrinsics intrinsics){
    return (abs(intrinsics(0,0)) + abs(intrinsics(1,1)))*0.5;
}
