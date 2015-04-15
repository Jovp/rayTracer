//
//  Shadow.h
//  projet3D
//
//  Created by Paola Vaulet on 15/04/2015.
//  Copyright (c) 2015 Julien Philip. All rights reserved.
//

#ifndef __projet3D__Shadow__
#define __projet3D__Shadow__

#include <stdio.h>
#include <stdlib.h>
#include "tiny_obj_loader.h"
#include "Vec3.h"
#include "Ray.h"
#include "kdTree.h"

using namespace std;
using namespace tinyobj;

class Shadow {
    
public:
    Vec3f origin;
    Vec3f lightPos;
    vector<Triangle> T;
    vector<shape_t> shapes;
    
    Shadow(Vec3f _origin, Vec3f _lightPos, const vector<tinyobj::shape_t>& _shapes){
        T = TriangleListFromShapes(_shapes);
        shapes = _shapes;
        origin=_origin;
        lightPos=_lightPos;
    }
    
    Shadow(Vec3f _origin, Vec3f _lightPos, vector<Triangle> _T){
        T=_T;
        origin=_origin;
        lightPos=_lightPos;
    }
    
    //Shadow test. Returns true if origin is IN the shadow, false otherwise
    bool isInShadow();
    
    vector<Triangle> TriangleListFromShapes(const vector<tinyobj::shape_t>& shapes){
        vector<Triangle> TriangleF;
        for (size_t s = 0; s < shapes.size (); s++)
            for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
                unsigned int index0 = 3*shapes[s].mesh.indices[3*f];
                unsigned int index1 = 3*shapes[s].mesh.indices[3*f+1];
                unsigned int index2 = 3*shapes[s].mesh.indices[3*f+2];
                TriangleF.push_back(Triangle(index0,index1,index2,s));
            }
            return TriangleF;
    }
    
};

#endif /* defined(__projet3D__Shadow__) */
