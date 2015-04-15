//
//  Ray.h
//  my_ray_tracer
//
//  Created by Alexandre on 14/04/2015.
//  Copyright (c) 2015 Alexandre. All rights reserved.
//

#ifndef __my_ray_tracer__Ray__
#define __my_ray_tracer__Ray__

#include <stdio.h>
#include "Vec3.h"
#include "tiny_obj_loader.h"
#include "BBox.h"
#include "kdTree.h"

class Ray {
public:
    Vec3f origin;
    Vec3f direction;
    
    Ray(Vec3f _origin, Vec3f _direction)
    : origin(_origin), direction(_direction){}
    
    void rayTriangleIntersection (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d);
    Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes);
    
    bool rayBBoxIntersection(const BBox& box,const float& t0,const float& t1);
    void raySceneIntersectionKdTree(const kdTree& tree);
    
    float evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, Vec3f intersection);
    
};

#endif /* defined(__my_ray_tracer__Ray__) */
