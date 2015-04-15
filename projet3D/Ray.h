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

class Ray {
public:
    Vec3f origin;
    Vec3f direction;

    Ray(Vec3f _origin, Vec3f _direction)
        : origin(_origin), direction(_direction){}
    
    void IntersectionRayonTriangle (const Vec3f & o,const Vec3f & w,const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d);
    Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, const Vec3f & o,const Vec3f & w);
    float evaluateResponse(Vec3f intersection);
    
};

#endif /* defined(__my_ray_tracer__Ray__) */
