//
//  Ray.cpp
//  my_ray_tracer
//
//  Created by Alexandre on 14/04/2015.
//  Copyright (c) 2015 Alexandre. All rights reserved.
//

#include "Ray.h"
#include "tiny_obj_loader.h"


Vec3f IntersectionRayonTriangle (Vec3f o, Vec3f w,Vec3f p0,Vec3f p1,Vec3f p2){
    
    Vec3f e0 = p1-p0;
    Vec3f e1 = p2-p0;
    Vec3f n = normalize(cross(e0,e1));
    Vec3f q = cross(w,e1);
    float a = dot(e0,q);
    if( dot(n,w) >= 0 || std::abs(a) < 0.0000001 ){
        return nullptr;
    }
    Vec3f s = (o - p0)/a;
    Vec3f r = cross(s,e0);
    float b0 = dot(s,q);
    float b1 = dot(r,w);
    float b2 = 1 - b0 - b1;
    if( b0 < 0 || b1 < 0 || b2 < 0){
        return nullptr;
    }
    float t = dot(e1,r);
    if(t>=0){
        return  Vec3f(b0,b1,b2);
    }
    return nullptr;
};

Vec3f raySceneIntersection(std::vector<tinyobj::shape_t> shapes){
    for (int i=0; i<shapes.size(); i++) {
        for (int j=0; j<shapes[i].mesh.positions.size()/3; j++) {
            //
        }
        //float p0 = shapes[i].mesh[j];
        //float p1 = shapes[i].mesh[j+1];
        //float p2 = shapes[i].mesh[j+2];
    
    }
    //TODO : pour une direction, calculer toutes les intersections
    //Si dist(camPos,intersection)<distMin le sauvegarder
    //return distMin
    Vec3f toto = Vec3f{0,0,0};
    return toto;
};

bool rayBBoxIntersection(const BBox& box,const Ray& r, float t0, float t1){
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
    std::vector<Vec3f> bounds;
    bounds.push_back(box.coin);
    bounds.push_back(box.coin+Vec3f(box.xL,box.yL,box.zL));
        if (r.direction[0] >= 0) {
            tmin = (bounds[0][0] - r.origin[0]) / r.direction[0];
            tmax = (bounds[1][0] - r.origin[0]) / r.direction[0];
        }
        else {
            tmin = (bounds[1][0] - r.origin[0]) / r.direction[0];
            tmax = (bounds[0][0] - r.origin[0]) / r.direction[0];
        }
        if (r.direction[1] >= 0) {
            tymin = (bounds[0][1] - r.origin[1]) / r.direction[1];
            tymax = (bounds[1][1] - r.origin[1]) / r.direction[1];
        }
        else {
            tymin = (bounds[1][1] - r.origin[1]) / r.direction[1];
            tymax = (bounds[0][1] - r.origin[1]) / r.direction[1];
        }
        if ( (tmin > tymax) || (tymin > tmax) )
            return false;
        if (tymin > tmin)
        tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        if (r.direction[2] >= 0) {
            tzmin = (bounds[0][2] - r.origin[2]) / r.direction[2];
            tzmax = (bounds[1][2] - r.origin[2]) / r.direction[2];
        }
        else {
            tzmin = (bounds[1][2] - r.origin[2]) / r.direction[2];
            tzmax = (bounds[0][2] - r.origin[2]) / r.direction[2];
        }
        if ( (tmin > tzmax) || (tzmin > tmax) )
            return false;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        return ( (tmin < t1) && (tmax > t0) );
    
}



float evaluateResponse(Vec3f intersection){
    return 0;
};



