//
//  Ray.cpp
//  my_ray_tracer
//
//  Created by Alexandre on 14/04/2015.
//  Copyright (c) 2015 Alexandre. All rights reserved.
//

#include "Ray.h"
const unsigned int NUMBER_RAY=8;
const unsigned int NUMBER_SAMPLE_LIGHT=8;
const float LIGHT_RADIUS=2.5;
const float LIGHT_POWER=1;
const Vec3f LIGHT_N=Vec3f(-1,-1,-1);
const Vec3f U_LIGHT= normalize(cross(LIGHT_N,Vec3f(0.5,0.2,0.3)));
const Vec3f V_LIGHT= cross(LIGHT_N,U_LIGHT);



float Ray::rayCircleIntersection(const Vec3f& circleCenter,const Vec3f& circleNormal,const float& circleRadius){
    if (dot(direction,-circleNormal)<0)
        return -1;
    float t=dot((circleCenter-origin),-circleNormal)/dot(direction,-circleNormal);
    if (t<=0)
        return -1;
    
    Vec3f planeRayIntersect=origin+direction*t;
    if ((planeRayIntersect-circleCenter).length()>circleRadius)
        return -1;
    return t;
    
}

void Ray::rayTriangleIntersection (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d){
    
    Vec3f e0 = p1-p0;
    Vec3f e1 = p2-p0;
    Vec3f n = cross(e0,e1);//inutile de normaliser
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

void Ray::rayTriangleIntersectionInverted (const Vec3f & p0,const Vec3f & p1,const Vec3f & p2, Vec3f & b, float & d){
    
    Vec3f e0 = p2-p0;
    Vec3f e1 = p1-p0;
    Vec3f n = cross(e0,e1);//inutile de normaliser
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
        b = Vec3f(b2, b1, b0);
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
    Vec3f boundsMin=box.coin;
    Vec3f boundsMax=box.coin+Vec3f(box.xL,box.yL,box.zL);
    //std::cout << "box vs ray : " << box.xL << std::endl;
    if (direction[0] >= 0) {
        tmin = (boundsMin[0] - origin[0]) / direction[0];
        tmax = (boundsMax[0] - origin[0]) / direction[0];
    }
    else {
        tmin = (boundsMax[0] - origin[0]) / direction[0];
        tmax = (boundsMin[0] - origin[0]) / direction[0];
    }
    if (direction[1] >= 0) {
        tymin = (boundsMin[1] - origin[1]) / direction[1];
        tymax = (boundsMax[1] - origin[1]) / direction[1];
    }
    else {
        tymin = (boundsMax[1] - origin[1]) / direction[1];
        tymax = (boundsMin[1] - origin[1]) / direction[1];
    }
    if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    if (direction[2] >= 0) {
        tzmin = (boundsMin[2] - origin[2]) / direction[2];
        tzmax = (boundsMax[2] - origin[2]) / direction[2];
    }
    else {
        tzmin = (boundsMax[2] - origin[2]) / direction[2];
        tzmax = (boundsMin[2] - origin[2]) / direction[2];
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
                                     triIntersect,Vec3f& b, float& t, const bool& inverted){
    //std::cout << "entrée dans le prog " << std::endl;
    
    if (tree.feuilleT.size()>0) {
        //std::cout << tree.feuilleT.size() << std::endl;
        int l=0;
        while (l<tree.feuilleT.size()) {
            Triangle tri = Triangle(tree.feuilleT[l]);
            
            Vec3f bTemp;
            float tTemp=INFINITY;
            // origin, direction, coord triangle , coordonnée barycentrique, distance caméra !
            if(inverted){
                this->rayTriangleIntersectionInverted(
                                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[0]],shapes[tri.v[3]].mesh.positions[tri.v[0]+1],shapes[tri.v[3]].mesh.positions[tri.v[0]+2]),
                                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[1]],shapes[tri.v[3]].mesh.positions[tri.v[1]+1],shapes[tri.v[3]].mesh.positions[tri.v[1]+2]),
                                                      Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[2]],shapes[tri.v[3]].mesh.positions[tri.v[2]+1],shapes[tri.v[3]].mesh.positions[tri.v[2]+2]),
                                                      bTemp, tTemp);
            }
            else {
                this->rayTriangleIntersection(
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[0]],shapes[tri.v[3]].mesh.positions[tri.v[0]+1],shapes[tri.v[3]].mesh.positions[tri.v[0]+2]),
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[1]],shapes[tri.v[3]].mesh.positions[tri.v[1]+1],shapes[tri.v[3]].mesh.positions[tri.v[1]+2]),
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[2]],shapes[tri.v[3]].mesh.positions[tri.v[2]+1],shapes[tri.v[3]].mesh.positions[tri.v[2]+2]),
                                              bTemp, tTemp);
            }
            if (tTemp<t && tTemp>0.001){
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
        this->raySceneIntersectionKdTree(*tree.leftChild, shapes, triIntersect, b, t, inverted);
        this->raySceneIntersectionKdTree(*tree.rightChild, shapes, triIntersect, b, t, inverted);
    }
    
    //std::cout << t << std:: endl;
    
}

void Ray::raySceneIntersectionKdTreeShadowOptimized(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials, float& distToLight){
    //std::cout << "entrée dans le prog " << std::endl;
    float originalDist=distToLight;
    if (tree.feuilleT.size()>0) {
        //std::cout << tree.feuilleT.size() << std::endl;
        int l=0;
        while (l<tree.feuilleT.size()) {
            Triangle tri = Triangle(tree.feuilleT[l]);
            int index = shapes[tri.v[3]].mesh.material_ids[tri.v[4]];
            if (9==9/*materials[index].illum!=9*/) {
                Vec3f bTemp;
                float tTemp=INFINITY;
                // origin, direction, coord triangle , coordonnée barycentrique, distance caméra !
                this->rayTriangleIntersection(
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[0]],shapes[tri.v[3]].mesh.positions[tri.v[0]+1],shapes[tri.v[3]].mesh.positions[tri.v[0]+2]),
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[1]],shapes[tri.v[3]].mesh.positions[tri.v[1]+1],shapes[tri.v[3]].mesh.positions[tri.v[1]+2]),
                                              Vec3f(shapes[tri.v[3]].mesh.positions[tri.v[2]],shapes[tri.v[3]].mesh.positions[tri.v[2]+1],shapes[tri.v[3]].mesh.positions[tri.v[2]+2]),
                                              bTemp, tTemp);
                
                if (tTemp<distToLight && tTemp>0.00001){
                    distToLight=tTemp;
                    break;
                }
            }
            
            l++;
        }
    }
    
    else if (!this->rayBBoxIntersection(tree.boite,0,MAXFLOAT)) {
        
    }
    else {
        this->raySceneIntersectionKdTreeShadowOptimized(*tree.leftChild, shapes,materials, distToLight);
        if (distToLight<originalDist) {
            return;
        }
        else
            this->raySceneIntersectionKdTreeShadowOptimized(*tree.rightChild, shapes,materials, distToLight);
    }
    
    //std::cout << t << std:: endl;
    
}



float Brdf_Lambert(const float& Kd) {
    return (Kd/M_PI);
}
float Brdf_GGX(const Vec3f & p, const Vec3f & n,const Vec3f& light,const Vec3f& cam_pos, const float& Ns) {
    // Paramètres :
    
    const float alpha=1.01-log10f(Ns+1)/3;
    const float F0=0.5; //Plastique (diélectrique) : 0.3 à 0.5 Aluminium (conducteur) : [0.91, 0.92, 0.92], « reflet coloré », variance significative selon la longueur d’onde
    
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
        
        float GWi=2*dot(n,Wi)/(dot(n,Wi)+sqrtf(alpha*alpha+(1-alpha*alpha)*pow(dot(n,Wi),2)));
        float GWo=2*dot(n,Wo)/(dot(n,Wo)+sqrtf(alpha*alpha+(1-alpha*alpha)*pow(dot(n,Wo),2)));
        
        float G = GWi*GWo;
        
        return D*F*G/(4*dot(n,Wi)*dot(n,Wo));
    }
}

float attenuation(const Vec3f& v,const float& sceneSize){
    float ac=1,al=0.0/(sceneSize),aq=0.0/(sceneSize*sceneSize);
    float d=v.length();
    return 1/(ac+al*d+aq*d*d);
}



Vec3f Ray::evaluateResponse(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax){
    
    Vec3f radiance=Vec3f(0,0,0);
    Vec3f radianceAmbiante=Vec3f(0,0,0);
    Vec3f p=Vec3f(0,0,0);
    Vec3f n=Vec3f(0,0,0);
    
    
    //Calcul de l'index pour materials
    int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
    
    if (intersection==Vec3f(0,0,0)) {
        radiance= Vec3f(0,0,0);
    }
    
    
    // effet miroir
    else if(materials[index].illum==3 && profondeur>0){ // Si le matériel est un miroir
        
        radiance=mirrorEffect(shapes, tree, materials, intersection, t, lightPos, profondeur, profondeurMax,false);
    }
    
    // effet Verre
    else if((materials[index].illum==9 || materials[index].illum==7) && profondeur>0){ // Si le matériel est en verre
        
        
        radiance=glassEffect(shapes, tree, materials, intersection, t, lightPos, profondeur, profondeurMax,false,materials[index].illum==7);
    }
    
    
    // Cas "normal"
    else {
        
        
        
        for (int i=0; i<3; i++) {
            //Interpolation de la position de l'intersectiona
            p[0] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]];
            p[1] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+1];
            p[2] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+2];
            
            n[0] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]];
            n[1] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+1];
            n[2] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+2];
            
        }
        
        float distCircle=rayCircleIntersection(lightPos, LIGHT_N, LIGHT_RADIUS);
        if ( distCircle!=-1 && (origin-p).length()>distCircle)
            return Vec3f(LIGHT_POWER,LIGHT_POWER,LIGHT_POWER);
        
        //Ombre portées
        
        //Ombre  douces
        //float ratio=LIGHT_RADIUS; On verra plus tard
        
        for (int lightSample=0; lightSample<NUMBER_SAMPLE_LIGHT; lightSample++) {
            //on teste si on est sur la lampe
            Vec3f randomPoint=generateRandCircle();
            Vec3f lightPosDxy=lightPos+LIGHT_RADIUS*randomPoint[0]*U_LIGHT+LIGHT_RADIUS*randomPoint[1]*V_LIGHT;
            
            if (!isInShadow(tree, shapes,materials, p, lightPosDxy) && dot(n,lightPosDxy-p)>0.0 && dot(LIGHT_N,normalize(lightPos-p))<0.05){
                
                //Calcul de l'index pour materials
                int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
                float Ns=materials[index].shininess;
                if (Ns==0) {
                    Ns=40;
                }
                //tinyobj::shape_t shape = (shapes[t.v[3]]);
                float LWi = attenuation(lightPosDxy-p,tree.boite.maxDist());
                
                float GGX = Brdf_GGX(p, n, lightPosDxy,  origin,Ns);
                //std::cout << "attenuation : "<< LWi << std::endl;
                
                float projection=dot(normalize(lightPosDxy-p),normalize(n));
                Vec3f diffu = Vec3f(Brdf_Lambert(materials[index].diffuse[0]),Brdf_Lambert(materials[index].diffuse[1]),Brdf_Lambert(materials[index].diffuse[2]));
                Vec3f spéculaire = Vec3f(GGX*(float)materials[index].specular[0],GGX*(float)materials[index].specular[1],GGX*(float)materials[index].specular[2]);
                Vec3f f = diffu+spéculaire ;
                
                //std::cout << projection << std::endl;
                
                radiance+= LWi*projection*f;
            
            }
        }
        // On remet a l'échelle 
        radiance/=NUMBER_SAMPLE_LIGHT;
        
        float angleSolide=2*M_PI-(M_PI*dot(normalize(lightPos-p),normalize(n))*pow(LIGHT_RADIUS,2)/pow((lightPos-p).length(),2))<0 ? 0 : 2*M_PI-(M_PI*dot(normalize(lightPos-p),normalize(n))*pow(LIGHT_RADIUS,2)/pow((lightPos-p).length(),2));
        
        // On tire les rayons a partir du point courant !
        
        if(profondeur>0){
            for (int numberRay2=0; numberRay2<NUMBER_RAY;numberRay2++){
                // Nouveau chemin
                Vec3f g(0,0,0);
                //Path Tracing first generate a randon ray from p and n
                Ray path=Ray(p,generateRandDir(n));
                Triangle triIntersect(0,0,0,0,0);
                Vec3f coordBar=Vec3f(0,0,0);
                float t2=INFINITY;
                path.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, false);
                
                if (t2!=INFINITY){
                    
                    
                    Vec3f posPointHitByRay=Vec3f(0,0,0);
                    
                    for (int i=0; i<3; i++) {
                        //Interpolation de la position de l'intersection
                        posPointHitByRay[0] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]];
                        posPointHitByRay[1] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+1];
                        posPointHitByRay[2] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+2];
                        
                    }
                    
                    
                    // ici sur
                    Vec3f colorFromRay=path.evaluateResponsePath(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur-1,profondeurMax);
                    int indexFromRay = shapes[t.v[3]].mesh.material_ids[t.v[4]];
                    Vec3f diffuFromRay=Vec3f(Brdf_Lambert(materials[indexFromRay].diffuse[0]),Brdf_Lambert(materials[indexFromRay].diffuse[1]),Brdf_Lambert(materials[indexFromRay].diffuse[2]));
                    float Ns=materials[indexFromRay].shininess;
                    if (Ns==0) {
                        Ns=40;
                    }
                    float GGXFromRay=Brdf_GGX(p, n, posPointHitByRay,  origin,Ns);
                    Vec3f specularFromRay=Vec3f(GGXFromRay*(float)materials[indexFromRay].specular[0],GGXFromRay*(float)materials[indexFromRay].specular[1],GGXFromRay*(float)materials[indexFromRay].specular[2]);
                    radianceAmbiante+=angleSolide*attenuation(posPointHitByRay-p,tree.boite.maxDist())*(colorFromRay)*(diffuFromRay+specularFromRay);
                    
                    
                }
            }
            radianceAmbiante/=NUMBER_RAY;
            
        }
        
    }
    
    return radiance+radianceAmbiante;
    
    
};



Vec3f Ray::evaluateResponsePath(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax){
    
    Vec3f radiance=Vec3f(0,0,0);
    Vec3f radianceAmbiante=Vec3f(0,0,0);
    Vec3f p=Vec3f(0,0,0);
    Vec3f n=Vec3f(0,0,0);
    int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
    
    if (intersection==Vec3f(0,0,0)) {
        radiance= Vec3f(0,0,0);
    }
    
    // effet miroir
    else if(materials[index].illum==3 && profondeur>0){ // Si le matériel est un miroir
        
        radiance=mirrorEffect(shapes, tree, materials, intersection, t, lightPos, profondeur, profondeurMax,true);
        
    }
    
    // effet Verre
    else if((materials[index].illum==9 || materials[index].illum==7) && profondeur>0){ // Si le matériel est en verre
        
        radiance=glassEffect(shapes, tree, materials, intersection, t, lightPos, profondeur, profondeurMax,true,materials[index].illum==7);
        
        
    }
    
    // Cas Normal
    else {
        
        
        
        for (int i=0; i<3; i++) {
            //Interpolation de la position de l'intersection
            p[0] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]];
            p[1] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+1];
            p[2] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+2];
            
            n[0] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]];
            n[1] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+1];
            n[2] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+2];
            
        }
        
        
        // On verifie que l'on n'est pas dans l'ombre
        if (isInShadow(tree, shapes,materials, p, lightPos)){
            
            radiance= Vec3f(0,0,0);
        }
        
        // Eclairage direct si on n'est pas dans l'ombre
        
        else{
            
            if (dot(n,lightPos-p)<=0) {
                radiance= Vec3f(0,0,0);
            }
            else {
                //Calcul de l'index pour materials
                
                
                //tinyobj::shape_t shape = (shapes[t.v[3]]);
                float LWi = attenuation(lightPos-p,tree.boite.maxDist());
                float Ns=materials[index].shininess;
                if (Ns==0) {
                    Ns=40;
                }
                
                float GGX = Brdf_GGX(p, n, lightPos,  origin, Ns);
                //std::cout << "attenuation : "<< LWi << std::endl;
                
                Vec3f diffu = Vec3f(Brdf_Lambert(materials[index].diffuse[0]),Brdf_Lambert(materials[index].diffuse[1]),Brdf_Lambert(materials[index].diffuse[2]));
                Vec3f spéculaire = Vec3f(GGX*(float)materials[index].specular[0],GGX*(float)materials[index].specular[1],GGX*(float)materials[index].specular[2]);
                Vec3f f = diffu+spéculaire ;
                
                //std::cout << projection << std::endl;
                
                radiance= LWi*f;
                
                
            }
            
        }
        
        
        
        // Continuation du chemin si il reste de la profondeur
        if(profondeur>0){
            // Nouveau chemin
            Vec3f g(0,0,0);
            //Path Tracing first generate a randon ray from p and n
            Ray path=Ray(p,generateRandDir(n));
            Triangle triIntersect(0,0,0,0,0);
            Vec3f coordBar=Vec3f(0,0,0);
            float t2=INFINITY;
            path.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, false);
            
            if (t2!=INFINITY){
                
                // ici sur
                Vec3f colorFromRay=path.evaluateResponsePath(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur-1,profondeurMax);
                
                int indexFromRay = shapes[t.v[3]].mesh.material_ids[t.v[4]];
                Vec3f diffuFromRay=Vec3f(Brdf_Lambert(materials[indexFromRay].diffuse[0]),Brdf_Lambert(materials[indexFromRay].diffuse[1]),Brdf_Lambert(materials[indexFromRay].diffuse[2]));
                Vec3f posPointHitByRay=Vec3f(0,0,0);
                
                for (int i=0; i<3; i++) {
                    //Interpolation de la position de l'intersection
                    posPointHitByRay[0] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]];
                    posPointHitByRay[1] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+1];
                    posPointHitByRay[2] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+2];
                    
                }
                float Ns=materials[indexFromRay].shininess;
                if (Ns==0) {
                    Ns=40;
                }
                float GGXFromRay=Brdf_GGX(p, n, posPointHitByRay,  origin, Ns);
                Vec3f specularFromRay=Vec3f(GGXFromRay*(float)materials[indexFromRay].specular[0],GGXFromRay*(float)materials[indexFromRay].specular[1],GGXFromRay*(float)materials[indexFromRay].specular[2]);
                radiance+=attenuation(posPointHitByRay-p,tree.boite.maxDist())*(colorFromRay)*(diffuFromRay+specularFromRay);
                
            }
            
        }
        
    }
    
    return radiance;
    
};



/////////////
//
//  EFFET
//
/////////////


//ombre

bool isInShadow(const kdTree& tree, const std::vector<tinyobj::shape_t> & shapes, const std::vector<tinyobj::material_t> & materials ,const  Vec3f& p,const  Vec3f& lightPos){
    
    Vec3f w = lightPos-p;
    float dist=w.length();
    Ray rayon = Ray(p, normalize(w));
    float epsilon=0.0000001;
    
    
    rayon.raySceneIntersectionKdTreeShadowOptimized(tree, shapes,materials, dist);
    //boucle sur tous les triangles de la scene
    
    if (dist>epsilon && dist<w.length()) {
        return true;
    } else {
        return false;
    }
}


// Miroir
Vec3f Ray::mirrorEffect(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax, const bool& isPath){
    
    Vec3f p=Vec3f(0,0,0);
    Vec3f n=Vec3f(0,0,0);
    
    for (int i=0; i<3; i++) {
        //Interpolation de la position de l'intersection
        p[0] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]];
        p[1] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+1];
        p[2] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+2];
        
        n[0] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]];
        n[1] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+1];
        n[2] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+2];
    }
    
    // Couleur du matériaux.
    int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
    Vec3f reflectivity=Vec3f(materials[index].transmittance[0],materials[index].transmittance[1],materials[index].transmittance[2]);
    
    // générer le rayon symétrique
    Ray reflection=Ray(p,direction-2*dot(direction,n)*n);
    
    // Nouveau chemin
    Vec3f g(0,0,0);
    Triangle triIntersect(0,0,0,0,0);
    Vec3f coordBar=Vec3f(0,0,0);
    float t2=INFINITY;
    reflection.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, false);
    
    if(!isPath){
        return reflectivity*reflection.evaluateResponse(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur,profondeurMax);
    }
    else return reflectivity*reflection.evaluateResponsePath(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur-1,profondeurMax);
    
}


// Verre
Vec3f Ray::glassEffect(const std::vector<tinyobj::shape_t> & shapes, const kdTree& tree, const std::vector<tinyobj::material_t> & materials, const Vec3f & intersection, const Triangle & t, const Vec3f& lightPos, const unsigned int& profondeur,const unsigned int& profondeurMax, const bool& isPath, const bool& withSpecular){
    
    Vec3f radiance=Vec3f(0,0,0);
    Vec3f p=Vec3f(0,0,0);
    Vec3f n=Vec3f(0,0,0);
    
    for (int i=0; i<3; i++) {
        //Interpolation de la position de l'intersection
        p[0] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]];
        p[1] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+1];
        p[2] += intersection[i]*shapes[t.v[3]].mesh.positions[t.v[i]+2];
        
        n[0] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]];
        n[1] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+1];
        n[2] += intersection[i]*shapes[t.v[3]].mesh.normals[t.v[i]+2];
    }
    
    
    
    
    // générer le rayon symétrique
    float n1=1.0;
    float n2=1.6;
    float sinTt2=pow((n1/n2),2)*(1-pow(dot(-direction,n),2));
    //std::cout << sinTt2 << std::endl;
    
    if (sinTt2>1) {
        Ray reflection=Ray(p,direction-2*dot(direction,n)*n);
        
        // Nouveau chemin
        Vec3f g(0,0,0);
        Triangle triIntersect(0,0,0,0,0);
        Vec3f coordBar=Vec3f(0,0,0);
        float t2=INFINITY;
        reflection.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, false);
        
        if (!isPath){
            radiance=reflection.evaluateResponse(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur,profondeurMax);
        }
        else radiance=reflection.evaluateResponse(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur-1,profondeurMax);
        
    }
    
    else{
        
        // Rayon qui part du point de contact au rayon avec l'objet et qui va a l'intérieur.
        Ray refraction=Ray(p,(n1/n2)*direction+(float)((n1/n2)*dot(-direction,n)-sqrtf(1-sinTt2))*n);
        // Nouveau chemin
        Vec3f g(0,0,0);
        Triangle triIntersect(0,0,0,0,0);
        Vec3f coordBar=Vec3f(0,0,0);
        float t2=INFINITY;
        
        // étape1 Ok!
        
        
        
        //On est dans le verre on inverse les coefficients
        float n1=1.6;
        float n2=1.0;
        
        int nbRebond=0;
        
        Vec3f p2(0,0,0);
        Vec3f norm2(0,0,0);
        
        // On va calculer les rebonds dans l'objet tant qu'on y reste
        do {
            
            triIntersect=Triangle(0,0,0,0,0);
            coordBar=Vec3f(0,0,0);
            t2=INFINITY;
            // On calcule l'intersection avec l'objet en verre lui même (inverted = true)
            refraction.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, true);
            
            // Ce sont les coordonnées du point d'intersection sur la surface de l'objet en verre depuis l'intérieur
            p2=Vec3f(0,0,0);
            norm2=Vec3f(0,0,0);
            
            
            for (int i=0; i<3; i++) {
                //Interpolation de la position et normale de l'intersection
                p2[0] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]];
                p2[1] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+1];
                p2[2] += coordBar[i]*shapes[triIntersect.v[3]].mesh.positions[triIntersect.v[i]+2];
                
                norm2[0] -= coordBar[i]*shapes[triIntersect.v[3]].mesh.normals[triIntersect.v[i]];
                norm2[1] -= coordBar[i]*shapes[triIntersect.v[3]].mesh.normals[triIntersect.v[i]+1];
                norm2[2] -= coordBar[i]*shapes[triIntersect.v[3]].mesh.normals[triIntersect.v[i]+2];
            }
            
            float sinTt2=pow((n1/n2),2)*(1-pow(dot(-refraction.direction,norm2),2));
            if (sinTt2>1) {
                //On met a jour le rayon
                refraction=Ray(p2,refraction.direction-2*dot(refraction.direction,norm2)*norm2);
                nbRebond++;
            }
            
            
        } while(sinTt2>1 && nbRebond<4);
        
        
        if (nbRebond!=4) {
            // On calcule alors la réponse comme si on se trouvais sur ce pixel !
            // calcul de la deuxième réfraction.
            triIntersect=Triangle(0,0,0,0,0);
            coordBar=Vec3f(0,0,0);
            t2=INFINITY;
            Ray refraction2=Ray(p2,(n1/n2)*refraction.direction+(float)((n1/n2)*dot(-refraction.direction,norm2)-sqrtf(1-sinTt2))*(norm2));
            refraction2.raySceneIntersectionKdTree(tree, shapes, triIntersect, coordBar, t2, false);
            if (!isPath) {
                radiance=refraction2.evaluateResponse(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur,profondeurMax);
            }
            else radiance=refraction2.evaluateResponse(shapes, tree, materials, coordBar, triIntersect, lightPos, profondeur-1,profondeurMax);
            
            
        }
        
        
        
    }
    
    // Couleur du matériaux.
    int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
    Vec3f reflectivity=Vec3f(materials[index].transmittance[0],materials[index].transmittance[1],materials[index].transmittance[2]);
    
    if(withSpecular){
        int index = shapes[t.v[3]].mesh.material_ids[t.v[4]];
        float Ns=materials[index].shininess;
        if (Ns==0) {
            Ns=40;
        }
        float GGX=Brdf_GGX(p, n, lightPos,  origin,Ns);
        Vec3f specular=Vec3f(GGX*(float)materials[index].specular[0],GGX*(float)materials[index].specular[1],GGX*(float)materials[index].specular[2]);
        return reflectivity*radiance+specular;
    }
    else
        return reflectivity*radiance;
}








