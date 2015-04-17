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
        b = Vec3f(b2, b0, b1);
        d = t;
    }
    // return nullptr;
    return;
};


void Ray::raySceneIntersection(const std::vector<tinyobj::shape_t> & shapes, Triangle & t, float& d, Vec3f& intersection){
    
    float distMin = INFINITY;
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
            
            Vec3f b=Vec3f(0,0,0);
            float d=INFINITY;
            
            rayTriangleIntersection(p0, p1, p2, b, d);
            if (d<distMin) {
                distMin = d;
                intersection = b;
                t.v[0] = index[0];
                t.v[1] = index[1];
                t.v[2] = index[2];
                t.v[3] = (unsigned int)s;
                t.v[4] = (unsigned int)f;
            }
        }
    }
    d=distMin;
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
    
    if (tree.feuilleT.size()>0) {
        //std::cout << tree.feuilleT.size() << std::endl;
        int l=0;
        while (l<tree.feuilleT.size()) {
            Triangle tri = Triangle(tree.feuilleT[l]);
            
            Vec3f bTemp;
            float tTemp=INFINITY;
            // origin, direction, coord triangle , coordonnée barycentrique, distance caméra !
            this->rayTriangleIntersection(
                                          Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[0]],shapes[tri.v[3]].mesh.positions[tri.v[0]+1],shapes[tri.v[3]].mesh.positions[tri.v[0]+2]),
                                          Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[1]],shapes[tri.v[3]].mesh.positions[tri.v[1]+1],shapes[tri.v[3]].mesh.positions[tri.v[1]+2]),
                                          Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[2]],shapes[tri.v[3]].mesh.positions[tri.v[2]+1],shapes[tri.v[3]].mesh.positions[tri.v[2]+2]),
                                          bTemp, tTemp);
            if (tTemp<t && tTemp>0.1){
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
float Brdf_GGX(const Vec3f & p, const Vec3f & n,const Vec3f& light,const Vec3f& cam_pos) {
    // Paramètres :
    
    const float alpha=1;
    const float F0=0.3; //Plastique (diélectrique) : 0.3 à 0.5 Aluminium (conducteur) : [0.91, 0.92, 0.92], « reflet coloré », variance significative selon la longueur d’onde
    
    // A FAIRE  f_s = D.F.G
    Vec3f Wo=normalize(cam_pos - p);
    
    Vec3f Wi=normalize(light - p);
    
    // On teste si le triangle peut être éclairé
    if (dot(Wi,n)<0) {
        return 0;
    }
    else
    {
        Vec3f Wh=normalize(Wi+Wo);
        
        // Distribution
        float D=(alpha*alpha)/(M_PI*pow((1+(alpha*alpha-1)*pow(dot(n,Wh),2)),2));
        
        // Terme de fresnel
        float Wih=dot(Wi, Wh);
        float F=F0+(1-F0)*pow((1-(0.0>Wih ? 0.0 : Wih)),5); // Attention on a pas mis le max ! Sert-il a qqch ?
        
        // Terme Géométrique GGX
        
        float GWi=2*dot(n,Wi)/(dot(n,Wi)+sqrt(alpha*alpha+(1-alpha*alpha)*pow(dot(n,Wi),2)));
        float GWo=2*dot(n,Wo)/(dot(n,Wo)+sqrt(alpha*alpha+(1-alpha*alpha)*pow(dot(n,Wo),2)));
        
        float G = GWi*GWo;
        
        return D*F*G/(4*dot(n,Wi)*dot(n,Wo));
    }
}

float attenuation(Vec3f v){
    float ac=1,al=1e-3,aq=1e-5;
    float d=v.length();
    return 1/(ac+al*d+aq*d*d);
}

bool isInShadow(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes,const  Vec3f& p,const  Vec3f& lightPos,Triangle& triIntersect,Vec3f& b, float& t){
    
    Vec3f w = lightPos-p;
    Ray rayon = Ray(p, normalize(w));
    float epsilon=0.0000001;
    
    
    rayon.raySceneIntersectionKdTree(tree, shapes, triIntersect, b, t);
    //boucle sur tous les triangles de la scene
    
    if (t>epsilon && t<w.length()) {
        return true;
    } else {
        return false;
    }
}


Vec3f Ray::evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, Vec3f lightPos){
    if (intersection==Vec3f(0,0,0)) {
        return Vec3f(0,0,0);
    }
    else {
        float light_power= 2000;
        Vec3f p=Vec3f(0,0,0);
        Vec3f n=Vec3f(0,0,0);
        
        for (int i=0; i<3; i++) {
            //Interpolation de la position de l'intersection
            p[0] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]];
            p[1] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+1];
            p[2] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+2];
            
        }
        
        //Ombre portées
        Triangle useless;
        Vec3f uselessBary;
        float uselessDist=INFINITY;
        if (isInShadow(tree, shapes, p, lightPos, useless, uselessBary, uselessDist)){
            return Vec3f(0,0,0);
        }
        
        else{
            //Calcul de la normale au triangle t
            Vec3f e0 = Vec3f(shapes[t.v[3]].mesh.positions[t.v[1]],shapes[t.v[3]].mesh.positions[t.v[1]+1],shapes[t.v[3]].mesh.positions[t.v[1]+2]) - Vec3f(shapes[t.v[3]].mesh.positions[t.v[0]],shapes[t.v[3]].mesh.positions[t.v[0]+1],shapes[t.v[3]].mesh.positions[t.v[0]+2]);
            Vec3f e1 = Vec3f(shapes[t.v[3]].mesh.positions[t.v[2]],shapes[t.v[3]].mesh.positions[t.v[2]+1],shapes[t.v[3]].mesh.positions[t.v[2]+2]) - Vec3f(shapes[t.v[3]].mesh.positions[t.v[0]],shapes[t.v[3]].mesh.positions[t.v[0]+1],shapes[t.v[3]].mesh.positions[t.v[0]+2]);
            n = normalize(cross(e0,e1));
            
            if (dot(n,lightPos-p)<=0) {
                return Vec3f(0,0,0);
            }
            
            //Calcul de l'index pour materials
            int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
            
            //tinyobj::shape_t shape = (shapes[t.v[3]]);
            float LWi = attenuation(lightPos-p);
            float projection = dot(normalize(lightPos-p),normalize(n));
            
            float GGX = Brdf_GGX(p, n, lightPos,  origin);
            //std::cout << "attenuation : "<< LWi << std::endl;
            
            Vec3f diffu = Vec3f(Brdf_Lambert(materials[index].diffuse[0]),Brdf_Lambert(materials[index].diffuse[1]),Brdf_Lambert(materials[index].diffuse[2]));
            Vec3f spéculaire = Vec3f(GGX*(float)materials[index].specular[0],GGX*(float)materials[index].specular[1],GGX*(float)materials[index].specular[2]);
            Vec3f f = diffu ;
            
            //std::cout << projection << std::endl;
            
            return LWi*f;
        }
    }
    
};



