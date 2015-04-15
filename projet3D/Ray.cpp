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

float evaluateResponse(Vec3f intersection){
    return 0;
};



