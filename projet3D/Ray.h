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
#include <float.h>

class Ray {
public:
    Vec3f origin;
    Vec3f direction;
    
    Ray(Vec3f _origin, Vec3f _direction)
    : origin(_origin), direction(_direction){}
    
    void rayTriangleIntersection (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d);
<<<<<<< HEAD
    Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, Triangle & t, float & d);
    bool rayBBoxIntersection(const BBox& box,const float& t0,const float& t1);
    void raySceneIntersectionKdTree(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes,Triangle& triIntersect,Vec3f& b, float& t);
    Vec3f evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t);
=======
    Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes,  Triangle & t);
    
    bool rayBBoxIntersection(const BBox& box,const float& t0,const float& t1);
    void raySceneIntersectionKdTree(const kdTree& tree);
    
    Vec3f evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t,  Vec3f lightPos);
>>>>>>> origin/Alex3
    
};

#endif /* defined(__my_ray_tracer__Ray__) */
