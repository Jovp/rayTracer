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
    : origin(_origin), direction(normalize(_direction)){}
    
    void rayTriangleIntersection (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d);
    void rayTriangleIntersectionInverted (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d);

    void raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, Triangle & t, float& d, Vec3f& intersection);
    bool rayBBoxIntersection(const BBox& box,const float& t0,const float& t1);
    
    void raySceneIntersectionKdTree(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes,Triangle& triIntersect,Vec3f& b, float& t,const bool& inverted);
    // Cette fonction permet de tester si on rencontre un triangle plus près que la lampe pas besoin de parcourir tout l'arbre.
    void raySceneIntersectionKdTreeShadowOptimized(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, float& distToLight);
    
    Vec3f raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes,  Triangle & t);
    Vec3f evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos,const unsigned int& profondeur,const unsigned int& profondeurMax);
    Vec3f evaluateResponsePath(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax);
    
    // Effet
    
    Vec3f glassEffect(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax, const bool& isPath, const bool& withSpecular);
    Vec3f mirrorEffect(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax, const bool& isPath);
    

    

    
};

bool isInShadow(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials ,const  Vec3f& p,const  Vec3f& lightPos);

#endif /* defined(__my_ray_tracer__Ray__) */
