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


Vec3f Ray::raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, Triangle & t, float& d){

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
            float d=INFINITY;
            rayTriangleIntersection(p0, p1, p2, b, d);
            if (d<distMin) {
                distMin = d;
                intersection = b;
                t.v[0] = index[0];
                t.v[1] = index[1];
                t.v[2] = index[2];
                t.v[3] = s;
                t.v[4] = f;
            }
        }
    }
    d=distMin;
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
            this->rayTriangleIntersection(
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
float Brdf_Lambert(float Kd) {
    return (Kd/M_PI);
}
float Brdf_GGX(const Vec3f & p, const Vec3f & n, Vec3f Light_toV, Vec3f cam_pos) {
    // Paramètres :
    
    const float alpha=1;
    const float F0=0.3; //Plastique (diélectrique) : 0.3 à 0.5 Aluminium (conducteur) : [0.91, 0.92, 0.92], « reflet coloré », variance significative selon la longueur d’onde
    
    // A FAIRE  f_s = D.F.G
    Vec3f Wo=normalize(Light_toV);
    
    Vec3f Wi=normalize(cam_pos - p);
    
    Vec3f Wh=normalize(Wi+Wo);
    
    // Distribution
    float D=(alpha*alpha)/(M_PI*pow((1+(alpha*alpha-1)*pow(dot(n,Wh),2)),2));
    
    // Terme de fresnel
    float Wih=dot(Wi, Wh);
    float F=F0+(1-F0)*pow((1-(0.0>Wih ? 0.0 : Wih)),5); // Attention on a pas mis le max ! Sert-il a qqch ?
    
    // Terme Géométrique Cook-Torrance
    
    float Ombr=2*(dot(n,Wh)*dot(n,Wi))/(dot(Wo, Wh));
    float Masq=2*(dot(n,Wh)*dot(n,Wo))/(dot(Wo, Wh));
    float min1=(Masq<Ombr ? Masq : Ombr);
    float G = 1<min1 ? 1 :min1;
    
    return D*F*G/(4*dot(n,Wi)*dot(n,Wo));
}


Vec3f Ray::evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, Vec3f lightPos){
    Vec3f p, n;
    float L[3];
    for (int i=0; i<3; i++) {
        p[0] = shapes[t.v[3]].mesh.positions[t.v[i]];
        p[1] = shapes[t.v[3]].mesh.positions[t.v[i]+1];
        p[2] = shapes[t.v[3]].mesh.positions[t.v[i]+2];
        
        n[0] = shapes[t.v[3]].mesh.normals[t.v[i]];
        n[1] = shapes[t.v[3]].mesh.normals[t.v[i]+1];
        n[2] = shapes[t.v[3]].mesh.normals[t.v[i]+2];
        
        L[i] = Brdf_GGX(p, n, normalize(p-lightPos), origin);
    }
    
    int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
    return Vec3f(/*255,255,255*/255*materials[index].diffuse[0],255*materials[index].diffuse[1],255*materials[index].diffuse[2]);

};



