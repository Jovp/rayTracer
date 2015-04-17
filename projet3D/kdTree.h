//
//  kdTree.h
//  projet3D
//
//  Created by Julien Philip on 14/04/2015.
//  Copyright (c) 2015 Julien Philip. All rights reserved.
//

#ifndef __projet3D__kdTree__
#define __projet3D__kdTree__

#include <stdio.h>
#include <stdlib.h>
#include "tiny_obj_loader.h"
#include "BBox.h"

class Triangle {
public:
    inline Triangle () {
        v[0] = v[1] = v[2] = v[3] = v[4] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        v[3] = t.v[3];
        v[4] = t.v[4];
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2 ,unsigned int v3, unsigned int v4) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        v[3] = v3;
        v[4] = v4;
    }
    inline virtual ~Triangle () {}
    inline Triangle & operator= (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        v[3] = t.v[3];
        v[4] = t.v[4];
        return (*this);
    }
     unsigned int v[5];
};

class kdTree{
    public :
    BBox boite;
    kdTree * leftChild;
    kdTree * rightChild ;
    
    kdTree(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList);
    kdTree(const std::vector<tinyobj::shape_t>& shapes);
    kdTree(){};
    std::vector<Triangle> feuilleT;
    
    
    private :
    Vec3f mediane;
    
    int axis;
    
    
};

std::vector<Triangle> TriangleListFromShapes(const std::vector<tinyobj::shape_t>& shapes);

BBox computeBoundingBox(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList);

float findMedianFloat(const std::vector<float>& liste);

Vec3f findMedianSample(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList);

void Partition(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList,const Vec3f& median,int Axis,std::vector<Triangle>& TriangleListU,std::vector<Triangle>& TriangleListI);
#endif /* defined(__projet3D__kdTree__) */
