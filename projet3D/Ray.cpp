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
        std::cout << "feuille !!!" << tree.boite.xL << std::endl;
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
            if (tTemp<t && tTemp>0.01){
                t=tTemp;
                b=bTemp;
                triIntersect=tri;
                
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
    
    std::cout << t << std:: endl;
    
}

Vec3f Ray::evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t){
    
    int index = shapes[t.v[3]].mesh.indices[t.v[0]/3];
    int toto=materials.size();
    int toto2=shapes[t.v[3]].mesh.indices.size();
    return Vec3f(255*materials[index].diffuse[0],255*materials[index].diffuse[1],255*materials[index].diffuse[2]);
};



