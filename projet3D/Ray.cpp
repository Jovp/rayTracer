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
    if( dot(n,w) >= 0 || std::abs(a) < 0.01 ){
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


Vec3f Ray::raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, Triangle & t){
    float distMin = INFINITY;
    Vec3f intersection;
    for (size_t s = 0; s < shapes.size (); s++){
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
            unsigned int index[3];
            for (size_t v = 0; v  < 3; v++) {
                index[v] = 3*shapes[s].mesh.indices[3*f+v];
            }
            Vec3f p0, p1, p2;
            
            p0[0] = shapes[s].mesh.positions[index[0]];
            p0[1] = shapes[s].mesh.positions[index[0]+1];
            p0[2] = shapes[s].mesh.positions[index[0]+2];
            
            p1[0] = shapes[s].mesh.positions[index[1]+0];
            p1[1] = shapes[s].mesh.positions[index[1]+1];
            p1[2] = shapes[s].mesh.positions[index[1]+2];
            
            p2[0] = shapes[s].mesh.positions[index[2]];
            p2[1] = shapes[s].mesh.positions[index[2]+1];
            p2[2] = shapes[s].mesh.positions[index[2]+2];
            Vec3f b;
            float d;
            rayTriangleIntersection(p0, p1, p2, b, d);
            if (dist(origin, b)<distMin) {
                distMin = dist(origin, b);
                intersection = b;
                t.v[0] = index[0];
                t.v[1] = index[1];
                t.v[2] = index[2];
                t.v[3] = s;
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
     //std::cout << "box vs ray : " << box.xL << std::endl;
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
    //std::cout << "tmin et t max : "<< tmin << " " << tmax << std::endl;
    return ( (tmin < t1) && (tmax > t0) );
    
}

void Ray::raySceneIntersectionKdTree(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes,Triangle&
    triIntersect,Vec3f& b, float& t){
    //std::cout << "entrée dans le prog " << std::endl;
    
    if (!tree.feuilleT.empty()) {
        
        int l=0;
        while (l<tree.feuilleT.size()) {
            Triangle tri =tree.feuilleT[l];
            Vec3f bTemp;
            float tTemp=INFINITY;
            // origin, direction, coord triangle , coordonnée barycentrique, distance caméra !
            IntersectionRayonTriangle(origin, direction,
                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[0]],shapes[tri.v[3]].mesh.positions[tri.v[0]+1],shapes[tri.v[3]].mesh.positions[tri.v[0]+2]),
                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[1]],shapes[tri.v[3]].mesh.positions[tri.v[1]+1],shapes[tri.v[3]].mesh.positions[tri.v[1]+2]),
                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[2]],shapes[tri.v[3]].mesh.positions[tri.v[2]+1],shapes[tri.v[3]].mesh.positions[tri.v[2]+2]),
                                      bTemp, tTemp);
            if (tTemp<t && tTemp>1){
                t=tTemp;
                b=bTemp;
                triIntersect=tri;
                //std::cout << "intersection à : " << t << std::endl;
            }
            l++;
        }
    }
    
    else if (!this->rayBBoxIntersection(tree.boite,0,MAXFLOAT)) {
        
    }
    else {
        this->raySceneIntersectionKdTree(*tree.leftChild, shapes, triIntersect, b, t);
        this->raySceneIntersectionKdTree(*tree.rightChild, shapes, triIntersect, b, t);
    }
    
    //std::cout << t << std:: endl;
    
}

Vec3f Ray::evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t){
    
    int index = shapes[t.v[3]].mesh.material_ids[0];
    /*int toto=materials.size();
    int toot=shapes[t.v[3]].mesh.indices.size();
    int toto2=shapes[t.v[3]].mesh.material_ids.size();
    int toto21=shapes[t.v[3]].mesh.material_ids[1];
    int toto22=shapes[t.v[3]].mesh.material_ids[2];*/
    return Vec3f(255,255,255/*255*materials[index].diffuse[0],255*materials[index].diffuse[1],255*materials[index].diffuse[2]*/);
};



