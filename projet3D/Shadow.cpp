//
//  Shadow.cpp
//  projet3D
//
//  Created by Paola Vaulet on 15/04/2015.
//  Copyright (c) 2015 Julien Philip. All rights reserved.
//

#include "Shadow.h"

//Shadow test. Returns true if origin is IN the shadow, false otherwise.
bool Shadow::isInShadow(){
    
    Vec3f w = lightPos-origin;
    Ray rayon = Ray(origin, w);
    Vec3f b;
    Vec3f p0, p1, p2;
    float d;

    //boucle sur tous les triangles de la scene
    for (int i=0; i<T.size(); i++){
        p0=Vec3f(shapes[T[i].v[3]].mesh.positions[T[i].v[0]],shapes[T[i].v[3]].mesh.positions[T[i].v[0]+1], shapes[T[i].v[3]].mesh.positions[T[i].v[0]+2]);
        p1=Vec3f(shapes[T[i].v[3]].mesh.positions[T[i].v[1]],shapes[T[i].v[3]].mesh.positions[T[i].v[1]+1], shapes[T[i].v[3]].mesh.positions[T[i].v[1]+2]);
        p2=Vec3f(shapes[T[i].v[3]].mesh.positions[T[i].v[2]],shapes[T[i].v[3]].mesh.positions[T[i].v[2]+1], shapes[T[i].v[3]].mesh.positions[T[i].v[2]+2]);
        
        rayon.rayTriangleIntersection(p0,p1,p2,b,d);
        
        if (d>0) {
            return true;
        } else {
            d=-1.0;
        }
        
    }
    
    return false;
}