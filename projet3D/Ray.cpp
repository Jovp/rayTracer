//
//  Ray.cpp
//  my_ray_tracer
//
//  Created by Alexandre on 14/04/2015.
//  Copyright (c) 2015 Alexandre. All rights reserved.
//

#include "Ray.h"



void Ray::rayTriangleIntersection (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d){
    
    Vec3f e0 = p1-p0;
    Vec3f e1 = p2-p0;
    Vec3f n = normalize(cross(e0,e1));
    Vec3f q = cross(direction,e1);
    float a = dot(e0,q);
    if( dot(n,direction) >= 0 || std::abs(a) < 0.0000001 ){
        //return nullptr;
        return;
    }
    Vec3f s = (origin - p0)/a;
    Vec3f r = cross(s,e0);
    float b0 = dot(s,q);
    float b1 = dot(r,direction);
    float b2 = 1 - b0 - b1;
    if( b0 < 0 || b1 < 0 || b2 < 0){
        //return nullptr;
        return;
    }
    float t = dot(e1,r);
    if( t >= 0.0 ){
        //return  Vec3f(b0,b1,b2);
        b = Vec3f(b0, b1, b2);
        d = t;
    }
    // return nullptr;
    return;
};

Vec3f Ray::raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes){
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
            rayTriangleIntersection(p0, p1, p2, b, d);
            if (dist(origin, b)<distMin) {
                distMin = dist(origin, b);
                intersection = b;
            }
        }
    }
    return intersection;
};

bool Ray::rayBBoxIntersection(const BBox& box,const float& t0,const float& t1){
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    std::vector<Vec3f> bounds;
    bounds.push_back(box.coin);
    bounds.push_back(box.coin+Vec3f(box.xL,box.yL,box.zL));
    if (direction[0] >= 0) {
        tmin = (bounds[0][0] - origin[0]) / direction[0];
        tmax = (bounds[1][0] - origin[0]) / direction[0];
    }
    else {
        tmin = (bounds[1][0] - origin[0]) / direction[0];
        tmax = (bounds[0][0] - origin[0]) / direction[0];
    }
    if (direction[1] >= 0) {
        tymin = (bounds[0][1] - origin[1]) / direction[1];
        tymax = (bounds[1][1] - origin[1]) / direction[1];
    }
    else {
        tymin = (bounds[1][1] - origin[1]) / direction[1];
        tymax = (bounds[0][1] - origin[1]) / direction[1];
    }
    if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    if (direction[2] >= 0) {
        tzmin = (bounds[0][2] - origin[2]) / direction[2];
        tzmax = (bounds[1][2] - origin[2]) / direction[2];
    }
    else {
        tzmin = (bounds[1][2] - origin[2]) / direction[2];
        tzmax = (bounds[0][2] - origin[2]) / direction[2];
    }
    if ( (tmin > tzmax) || (tzmin > tmax) )
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    return ( (tmin < t1) && (tmax > t0) );
    
}

void Ray::raySceneIntersectionKdTree(const kdTree& tree){
    /*
     if (!node.getFeuilleT().empty())
     {
     int l=0;
     while (l < node.getFeuilleT().size()){
     if (IntersectionRayonTriangle(origin,direction,mesh.V[node.getFeuilleT()[l].v[0]].p,mesh.V[node.getFeuilleT()[l].v[1]].p,mesh.V[node.getFeuilleT()[l].v[2]].p)!=-1) {
     //std::cout << "niveau feuille" << std::endl;
     return true;
     }
     l++;
     }
     
     return false;
     }
     
     else if (!IntersectionRayonSphere(origin, direction, node.getCenter(), node.getRadius())){
     //std::cout << "on coupe ce nombre de feuilles : " << node.getNbT() << std::endl;
     return false;
     }
     else if (IntersectionMeshBSH(mesh,*node.getLeftChild(),origin,direction))
     return true;
     
     else return IntersectionMeshBSH(mesh,*node.getRightChild(),origin,direction);
     */
    if (!tree.feuilleT.empty()) {
        int l=0;
        while (1==1) {
            
        }
    }
    
    else if (!this->rayBBoxIntersection(tree.boite,0,MAXFLOAT)) {
        
    }
    
    
}

float evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, Vec3f intersection){
    
    return 0;//Brdf_Lambert(1);
};



