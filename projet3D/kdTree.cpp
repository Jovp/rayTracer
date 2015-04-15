//
//  kdTree.cpp
//  projet3D
//
//  Created by Julien Philip on 14/04/2015.
//  Copyright (c) 2015 Julien Philip. All rights reserved.
//

#include "kdTree.h"
#include <float.h>


BBox computeBoundingBox(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList){
    float xmin=FLT_MAX,xmax=0,ymin=FLT_MAX,ymax=0,zmin=FLT_MAX,zmax=0;
    for (unsigned int i = 0; i < TriangleList.size (); i++){
        Triangle t = TriangleList[i];
        for (unsigned int j = 0; j < 3; j++) {
            const Vec3f & v = Vec3f(shapes[t.v[3]].mesh.positions[t.v[j]],shapes[t.v[3]].mesh.positions[t.v[j]+1],shapes[t.v[3]].mesh.positions[t.v[j]+2]);
            if(v[0]<xmin){
                xmin=v[0];
            }
            if(v[1]<ymin){
                ymin=v[1];
            }
            if(v[2]<zmin){
                zmin=v[2];
            }
            
            if(v[0]>xmax){
                xmax=v[0];
            }
            if(v[1]>ymax){
                ymax=v[1];
            }
            if(v[2]>zmax){
                zmax=v[2];
            }
            
        }
    }
    float xdiff=xmax-xmin;
    float ydiff=ymax-ymin;
    float zdiff=zmax-zmin;
    std::cout << "x min : " << xmin << std::endl;
    std::cout << "x max : " << xmax << std::endl;
    std::cout << "y min : " << ymin << std::endl;
    std::cout << "y max : " << ymax << std::endl;
    std::cout << "z min : " << zmin << std::endl;
    std::cout << "z max : " << zmax << std::endl;
    return BBox(xdiff, ydiff, zdiff, Vec3f(xmin,ymin,zmin));
}


float findMedianFloat(const std::vector<float>& liste){
    
    std::cout << "taille de la liste : " << liste.size() << std::endl;
    
    if (liste.size()==1){
        return liste[0];
        
    }
    else {
        std::vector<float> medians;
        for (int i=0; i<liste.size()/5; i++) {
            std::vector<float> liste5;
            for (int j=0; j<5; j++) {
                liste5.push_back(liste[5*i+j]);
            }
            std::sort(liste5.begin(),liste5.end());
            medians.push_back(liste5[2]);
        }
        
        
        
        std::vector<float> liste5;
        for (int j=0; j<liste.size()%5; j++) {
            liste5.push_back(liste[liste.size()-liste.size()%5+j]);
        }
        //std::cout << "c'est la que ça bug ?" << std::endl;
        std::sort(liste5.begin(),liste5.end());
        //std::cout << "non" << std::endl;) {
        if (!liste5.empty()) {
            switch (liste5.size()) {
                case 1:
                    medians.push_back(liste5[0]);
                    break;
                case 2:
                    medians.push_back(liste5[1]);
                    break;
                case 3:
                    medians.push_back(liste5[1]);
                    break;
                case 4:
                    medians.push_back(liste5[2]);
                    break;
                default:
                    break;
            }

        }
        
        
        
        return findMedianFloat(medians);
    }
}

// test hub

Vec3f findMedianSample(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList){
    
    std::vector<float> xCentre;
    std::vector<float> yCentre;
    std::vector<float> zCentre;
    
    //std::cout << "cherche la médiane" << std::endl;
    
    for (unsigned int i = 0; i < TriangleList.size (); i++){
        Triangle t = TriangleList[i];
        
        xCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]]+shapes[t.v[3]].mesh.positions[t.v[1]]+shapes[t.v[3]].mesh.positions[t.v[2]])/3);
        yCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]+1]+shapes[t.v[3]].mesh.positions[t.v[1]+1]+shapes[t.v[3]].mesh.positions[t.v[2]+1])/3);
        zCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]+2]+shapes[t.v[3]].mesh.positions[t.v[1]+2]+shapes[t.v[3]].mesh.positions[t.v[2]+2])/3);
        
    }
    
    //std::cout << "on va lancer le calcul des médianes" << std::endl;
    //std::cout << "triangle x" << std::endl;
    return Vec3f(findMedianFloat(xCentre),findMedianFloat(yCentre),findMedianFloat(zCentre));
    
}

Vec3f findMedianSampleLong(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList){
    
    std::vector<float> xCentre;
    std::vector<float> yCentre;
    std::vector<float> zCentre;
    
    //std::cout << "cherche la médiane" << std::endl;
    
    for (unsigned int i = 0; i < TriangleList.size (); i++){
        Triangle t = TriangleList[i];
        
        
         xCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]]+shapes[t.v[3]].mesh.positions[t.v[1]]+shapes[t.v[3]].mesh.positions[t.v[2]])/3);
         yCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]+1]+shapes[t.v[3]].mesh.positions[t.v[1]+1]+shapes[t.v[3]].mesh.positions[t.v[2]+1])/3);
         zCentre.push_back((shapes[t.v[3]].mesh.positions[t.v[0]+2]+shapes[t.v[3]].mesh.positions[t.v[1]+2]+shapes[t.v[3]].mesh.positions[t.v[2]+2])/3);
        
        /*
        xCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[0]]);
        xCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[1]]);
        xCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[2]]);
        yCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[0]+1]);
        yCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[1]+1]);
        yCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[2]+1]);
        zCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[0]+2]);
        zCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[1]+2]);
        zCentre.push_back(shapes[t.v[3]].mesh.positions[t.v[2]+2]);*/
        
    }
    
    std::sort(xCentre.begin(),xCentre.end());
    std::sort(yCentre.begin(),yCentre.end());
    std::sort(zCentre.begin(),zCentre.end());
    return Vec3f(xCentre[xCentre.size()/2],yCentre[yCentre.size()/2],zCentre[zCentre.size()/2]);
    
}

void Partition(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList,const Vec3f& median,int Axis,std::vector<Triangle>& TriangleListU,std::vector<Triangle>& TriangleListI){
    
    for (unsigned int i = 0; i < TriangleList.size (); i++){
        Triangle t = TriangleList[i];
        
        /*std::cout << " position centre discriminant x : " << (shapes[t.v[3]].mesh.positions[t.v[0]+Axis]+shapes[t.v[3]].mesh.positions[t.v[1]+Axis]+shapes[t.v[3]].mesh.positions[t.v[2]+Axis])/3 << std::endl;
        std::cout << " position centre discriminant y : " << (shapes[t.v[3]].mesh.positions[t.v[0]+1]+shapes[t.v[3]].mesh.positions[t.v[1]+1]+shapes[t.v[3]].mesh.positions[t.v[2]+1])/3 << std::endl;
        std::cout << " position centre discriminant z : " << (shapes[t.v[3]].mesh.positions[t.v[0]+0]+shapes[t.v[3]].mesh.positions[t.v[1]+0]+shapes[t.v[3]].mesh.positions[t.v[2]+0])/3 << std::endl;
        std::cout << "      z en question : " << shapes[t.v[3]].mesh.positions[t.v[0]+Axis]
        << " " <<shapes[t.v[3]].mesh.positions[t.v[1]+Axis]<<" "<< shapes[t.v[3]].mesh.positions[t.v[2]+Axis] << std::endl;
        */
        if (/*shapes[t.v[3]].mesh.positions[t.v[0]+Axis]>=median[Axis] &&
             shapes[t.v[3]].mesh.positions[t.v[1]+Axis]>=median[Axis] &&
             shapes[t.v[3]].mesh.positions[t.v[2]+Axis]>=median[Axis] */
            ((shapes[t.v[3]].mesh.positions[t.v[0]+Axis]+shapes[t.v[3]].mesh.positions[t.v[1]+Axis]+shapes[t.v[3]].mesh.positions[t.v[2]+Axis])/3)>=median[Axis]) {
            TriangleListU.push_back(t);
        }
        else if (/*shapes[t.v[3]].mesh.positions[t.v[0]+Axis]<=median[Axis] &&
                  shapes[t.v[3]].mesh.positions[t.v[1]+Axis]<=median[Axis] &&
                  shapes[t.v[3]].mesh.positions[t.v[2]+Axis]<=median[Axis] */
                 ((shapes[t.v[3]].mesh.positions[t.v[0]+Axis]+shapes[t.v[3]].mesh.positions[t.v[1]+Axis]+shapes[t.v[3]].mesh.positions[t.v[2]+Axis])/3)<median[Axis]){
            TriangleListI.push_back(t);
        }
        else {
            std::cout << "yop yop" << std::endl;
            TriangleListU.push_back(t);
            TriangleListI.push_back(t);
        }
        
    }
}

std::vector<Triangle> TriangleListFromShapes(const std::vector<tinyobj::shape_t>& shapes){
    std::vector<Triangle> TriangleF;
    for (size_t s = 0; s < shapes.size (); s++)
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
                unsigned int index0 = 3*shapes[s].mesh.indices[3*f];
                unsigned int index1 = 3*shapes[s].mesh.indices[3*f+1];
                unsigned int index2 = 3*shapes[s].mesh.indices[3*f+2];
                TriangleF.push_back(Triangle(index0,index1,index2,s));
        }
    return TriangleF;
}

kdTree::kdTree(const std::vector<tinyobj::shape_t>& shapes,const std::vector<Triangle>& TriangleList){
    
    if (TriangleList.size()<=3) {
        feuilleT=TriangleList;
    }
    else {
        //std::cout << "entrée dans kd tree" << std::endl;
        //std::cout << "Taille de la liste de triangle : "<< TriangleList.size() << std::endl;
        boite= computeBoundingBox(shapes, TriangleList);
        //std::cout << "boite ok" << std::endl;
        mediane= findMedianSample(shapes, TriangleList);
        //std::cout << "médiane ok : " << mediane << std::endl;
        axis= boite.maxAxis();
        //std::cout << "max axis : " << axis << std::endl;
        std::vector<Triangle> TriangleListI;
        std::vector<Triangle> TriangleListU;
        Partition(shapes, TriangleList, mediane, axis, TriangleListU, TriangleListI);
        if (TriangleListU.size()==TriangleList.size() || TriangleListI.size()==TriangleList.size()) {
            feuilleT=TriangleList;
        }
        else {
            //std::cout << "taille upper : " << TriangleListU.size() << std::endl;
            //std::cout << "taille inner : " << TriangleListI.size() << std::endl;
            leftChild=new kdTree(shapes,TriangleListU);
            rightChild=new kdTree(shapes,TriangleListI);
        }
        
        
    }
    
}

kdTree::kdTree(const std::vector<tinyobj::shape_t>& shapes){
    kdTree::kdTree(shapes,TriangleListFromShapes(shapes));
}





