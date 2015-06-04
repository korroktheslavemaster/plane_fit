#include <units.h>

//TruncSpaceCoord TruncateSpaceCoordinate(SpaceCoord in){
//    return TruncSpaceCoord(in[0], in[1], in[2]);
//}

Barycentric::Barycentric() {}

Barycentric::Barycentric(float a, float b, float c) : r(a), s(b), t(c) {}

Barycentric::Barycentric(float a, float b) : r(a), s(b), t(1.-a-b) {
    if(r + s > 1.){
        r = 1. - r;
        s = 1. - s;
        t = 1. - r - s;
    }
}

Barycentric::Barycentric(ImCoord pt, ImCoord a, ImCoord b, ImCoord c){
    float den = ( (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1]) );
    //assert(fabs(den) > SMALL_LIMIT);

    r =  ( (b[1]-c[1])*(pt[0]-c[0]) + (c[0]-b[0])*(pt[1]-c[1]) )/den;
    s =  ( (c[1]-a[1])*(pt[0]-c[0]) + (a[0]-c[0])*(pt[1]-c[1]) )/den;
    t = 1 - r - s;
}
