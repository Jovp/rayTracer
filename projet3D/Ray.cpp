//
//  Ray.cpp
//  my_ray_tracer
//
//  Created by Alexandre on 14/04/2015.
//  Copyright (c) 2015 Alexandre. All rights reserved.
//

#include "Ray.h"



void IntersectionRayonTriangle (const Vec3f & o,const Vec3f & w,const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d){
    
    Vec3f e0 = p1-p0;
    Vec3f e1 = p2-p0;
    Vec3f n = normalize(cross(e0,e1));
    Vec3f q = cross(w,e1);
    float a = dot(e0,q);
    if( dot(n,w) >= 0 || std::abs(a) < 0.0000001 ){
        //return nullptr;
    }
    Vec3f s = (o - p0)/a;
    Vec3f r = cross(s,e0);
    float b0 = dot(s,q);
    float b1 = dot(r,w);
    float b2 = 1 - b0 - b1;
    if( b0 < 0 || b1 < 0 || b2 < 0){
        //return nullptr;
    }
    float t = dot(e1,r);
    if(t>=0){
        //return  Vec3f(b0,b1,b2);
        b = Vec3f(b0, b1, b2);
        d = t;
    }
    // return nullptr;
};

Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, const Vec3f & o,const Vec3f & w){
    float distMin = INFINITY;
    Vec3f intersection;
    for (int i=0; i<shapes.size(); i++) {                           //Pour chaque shape
        for (int j=0; j<shapes[i].mesh.indices.size()/9; j++) {     //Pour chaque triangle
            Vec3f p0, p1, p2;
            p0[0] = shapes[i].mesh.positions[(j*9)];
            p0[1] = shapes[i].mesh.positions[(j*9)+1];
            p0[2] = shapes[i].mesh.positions[(j*9)+2];
            
            p1[0] = shapes[i].mesh.positions[(j*9)+3];
            p1[1] = shapes[i].mesh.positions[(j*9)+4];
            p1[2] = shapes[i].mesh.positions[(j*9)+5];
            
            p2[0] = shapes[i].mesh.positions[(j*9)+6];
            p2[1] = shapes[i].mesh.positions[(j*9)+7];
            p2[2] = shapes[i].mesh.positions[(j*9)+8];
            Vec3f b;
            float d;
            IntersectionRayonTriangle (o, w, p0, p1, p2, b, d);
            if (dist(o, b)<distMin) {
                distMin = dist(o, b);
                intersection = b;
            }
        }
    }
    return intersection;
};

float evaluateResponse(Vec3f intersection){
    return 0;
};



