
// ----------------------------------------------
// Informatique Graphique 3D & R�alit� Virtuelle.
// Projet
// Lancer de Rayon de Monte Carlo
// Copyright (C) 2015 Tamy Boubekeur
// All rights reserved.
// ----------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <GLUT/glut.h>
#include "Vec3.h"
#include "tiny_obj_loader.h"
#include "kdTree.h"
#include "Ray.h"
#include <thread>
#include <unistd.h>

using namespace std;

// App parameters
static const unsigned int DEFAULT_SCREENWIDTH = 768;
static const unsigned int DEFAULT_SCREENHEIGHT = 768;
static const char * DEFAULT_SCENE_FILENAME = "scenes/All_cornell_box/CornellBox-Sphere.obj";
static string appTitle ("MCRT - Monte Carlo Ray Tracer");
static GLint window;
static unsigned int screenWidth;
static unsigned int screenHeight;
static bool rayDisplayMode = false;
static bool interactifMode =false;
void control();
// Camera parameters
static float fovAngle;
static float aspectRatio;
static float nearPlane;
static float farPlane;
static Vec3f camEyePolar; // Expressing the camera position in polar coordinate, in the frame of the target
static Vec3f camTarget;

// Scene elements
static Vec3f lightPos = Vec3f (1.f, 1.f, 1.f);
static Vec3f lightColor = Vec3f (1.f, 1.f, 1.f);
static Vec3f sceneCenter = Vec3f (0.f, 0.f, 0.f);
static float sceneRadius = 1.f;
static vector<tinyobj::shape_t> shapes;
static vector<tinyobj::material_t> materials;

// Mouse parameters
static bool mouseLeftButtonClicked = false;
static int clickedX, clickedY;
static float baseCamPhi;
static float baseCamTheta;

// Tree
static kdTree tree;

//Quality
const unsigned int NUMBER_RAY=1;
const unsigned int nombreRayPixAA=8;
const unsigned int nbRebond=3;

// optimisation
const unsigned int nbBloc=32;
bool isOutOfDateMouse=true;


// Raytraced image
static unsigned char * rayImage = NULL;
static float * rayImageNotNormal = NULL;
static int * rayImageNumber = NULL;

//Defocus
static bool defocus = false;
static float focale = 1;
static float ouverture = 0.0f; // Valeur typiques 0.01 à 0.1

void printUsage () {
    std::cerr << std::endl // send a line break to the standard error output
    << appTitle << std::endl
    << "Author : Tamy Boubekeur" << std::endl << std::endl
    << "Usage : ./myRayTracer [<file.obj>]" << std::endl
    << "Commandes clavier :" << std::endl
    << "------------------" << std::endl
    << " ?: Print help" << std::endl
    << " <space>: Toggle raytracing/rasterization (GL)  display mode" << std::endl
    << " r: Ray trace an image from the current point of view" << std::endl
    << " s: Save the current ray traced image under raytraced_image.ppm" << std::endl
    << " <drag>+<left button>: rotate model" << std::endl
    << " <drag>+<right button>: move model" << std::endl
    << " <drag>+<middle button>: zoom" << std::endl
    << " q, <esc>: Quit" << std::endl << std::endl;
}

void initOpenGL () {
    glCullFace (GL_BACK);     // Specifies the faces to cull (here the ones pointing away from the camera)
    glEnable (GL_CULL_FACE); // Enables face culling (based on the orientation defined by the CW/CCW enumeration).
    glDepthFunc (GL_LESS); // Specify the depth test for the z-buffer
    glEnable (GL_DEPTH_TEST); // Enable the z-buffer in the rasterization
    glClearColor (0.0f, 0.0f, 0.0f, 1.0f); // Background color
    glEnable (GL_COLOR_MATERIAL);
}

void computeSceneNormals () {
    for (unsigned int s = 0; s < shapes.size (); s++)
        if (shapes[s].mesh.normals.empty ()) {
            shapes[s].mesh.normals.resize (shapes[s].mesh.positions.size (), 0.f);
            for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
                Vec3f q[3];
                for (size_t v = 0; v < 3; v++) {
                    unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                    for (unsigned int i = 0; i < 3; i++)
                        q[v][i] = shapes[s].mesh.positions[index+i];
                }
                Vec3f e01 = q[1] - q[0];
                Vec3f e02 = q[2] - q[0];
                Vec3f nf = normalize (cross (e01, e02));
                for (size_t v = 0; v < 3; v++) {
                    unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                    for (unsigned int i = 0; i < 3; i++)
                        shapes[s].mesh.normals[index+i] += nf[i];
                }
            }
            for (unsigned int i = 0; i < shapes[s].mesh.normals.size () / 3; i++) {
                Vec3f n;
                for (unsigned int j = 0; j < 3; j++)
                    n[j] = shapes[s].mesh.normals[3*i+j];
                n.normalize ();
                for (unsigned int j = 0; j < 3; j++)
                    shapes[s].mesh.normals[3*i+j] = n[j];
            }
        }
}

void computeSceneBoundingSphere () {
    sceneCenter = Vec3f (0.f, 0.f, 0.f);
    unsigned int count = 0;
    for (unsigned int s = 0; s < shapes.size (); s++)
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
            sceneCenter += Vec3f (shapes[s].mesh.positions[3*p],
                                  shapes[s].mesh.positions[3*p+1],
                                  shapes[s].mesh.positions[3*p+2]);
            count++;
        }
    sceneCenter /= count;
    sceneRadius = 0.f;
    for (unsigned int s = 0; s < shapes.size (); s++)
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
            float d = dist (sceneCenter, Vec3f (shapes[s].mesh.positions[3*p],
                                                shapes[s].mesh.positions[3*p+1],
                                                shapes[s].mesh.positions[3*p+2]));
            if (d > sceneRadius)
                sceneRadius = d;
        }
}

// Loads an OBJ file using tinyOBJ (http://syoyo.github.io/tinyobjloader/)
bool loadScene(const string & filename, const string & basepath = "") {
    shapes.clear ();
    materials.clear ();
    std::cout << "Loading " << filename << std::endl;
    std::string err = tinyobj::LoadObj(shapes, materials, filename.c_str (), basepath.c_str ());
    if (!err.empty()) {
        std::cerr << err << std::endl;
        return false;
    }
    computeSceneNormals ();
    computeSceneBoundingSphere ();
    return true;
}

void initCamera () {
    fovAngle = 45.f;
    nearPlane = sceneRadius/10000.0f;
    farPlane = 10*sceneRadius;
    camTarget = sceneCenter;
    camEyePolar = Vec3f (2.f * sceneRadius, M_PI/2.f, M_PI/2.f);
}

void initLighting () {
    lightPos = 2.f * Vec3f (sceneRadius, sceneRadius, sceneRadius);
    glEnable (GL_LIGHTING);
    GLfloat position[4] = {lightPos[0], lightPos[1], lightPos[2], 1.0f};
    GLfloat color[4] = {lightColor[0], lightColor[1], lightColor[2], 1.0f};
    glLightfv (GL_LIGHT0, GL_POSITION, position);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, color);
    glLightfv (GL_LIGHT0, GL_SPECULAR, color);
    glEnable (GL_LIGHT0);
    lightPos=Vec3f(278,546,279.5); // Cornell cube
    //lightPos=Vec3f(0,1.57,0); // Cornel sphère
    //lightPos=Vec3f(0,8,0); // mitsuba
    //lightPos=Vec3f(0,2,0); // dragon
}

void init (const string & filename) {
    initOpenGL ();
    unsigned int i = (unsigned int) filename.find_last_of ("/");
    loadScene (filename, filename.substr (0, i+1));
    Triangle tri;
    Vec3f b;
    float t=INFINITY-1;
    tree=kdTree(shapes,TriangleListFromShapes(shapes));
    //std::cout << "largeur boite : " << tree.boite.xL << std::endl;
    //Ray(polarToCartesian(camEyePolar),normalize(normalize(camTarget-polarToCartesian(camEyePolar)+normalize(Vec3f(rand(),rand(),rand()))))).raySceneIntersectionKdTree(tree, shapes, tri, b, t);
    //std::cout << "distance pixel central" << t << std::endl;
    initCamera ();
    initLighting ();
}

void setupCamera () {
    glMatrixMode (GL_PROJECTION); // Set the projection matrix as current. All upcoming matrix manipulations will affect it.
    glLoadIdentity ();
    gluPerspective (fovAngle, aspectRatio, nearPlane, farPlane); // Set the current projection matrix with the camera intrinsics
    glMatrixMode (GL_MODELVIEW); // Set the modelview matrix as current. All upcoming matrix manipulations will affect it.
    glLoadIdentity ();
    Vec3f eye = polarToCartesian (camEyePolar);
    swap (eye[1], eye[2]); // swap Y and Z to keep the Y vertical
    eye += camTarget;
    gluLookAt (eye[0], eye[1], eye[2],
               camTarget[0], camTarget[1], camTarget[2],
               0.0, 1.0, 0.0); // Set up the current modelview matrix with camera transform
}

void reshape (int w, int h) {
    screenWidth = w;
    screenHeight = h;
    aspectRatio = static_cast<float>(w)/static_cast<float>(h);
    glViewport (0, 0, (GLint)w, (GLint)h); // Dimension of the drawing region in the window
    setupCamera ();
    if (rayImage != NULL)
        delete [] rayImage;
    if (rayImageNotNormal != NULL)
        delete [] rayImageNotNormal;
    if (rayImageNumber != NULL)
        delete [] rayImageNumber;
    unsigned int l = 3*screenWidth*screenHeight;
    rayImage = new unsigned char [l];
    rayImageNotNormal= new float [l];
    rayImageNumber= new int [l];
    memset (rayImage, 0, l);
    memset(rayImageNumber,0,l);
    memset(rayImageNotNormal,0,l);
}

void rasterize () {
    setupCamera ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Erase the color and z buffers.
    glBegin (GL_TRIANGLES);
    glColor3f (1.f, 1.f, 1.f);
    for (size_t s = 0; s < shapes.size (); s++)
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
            if (!materials.empty ()) {
                unsigned int i = shapes[s].mesh.material_ids[f];
                glColor3f (materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
            }
            for (size_t v = 0; v  < 3; v++) {
                unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                glNormal3f (shapes[s].mesh.normals[index],
                            shapes[s].mesh.normals[index+1],
                            shapes[s].mesh.normals[index+2]);
                glVertex3f (shapes[s].mesh.positions[index],
                            shapes[s].mesh.positions[index+1],
                            shapes[s].mesh.positions[index+2]);
            }
        }
    glEnd ();
    glFlush (); // Ensures any previous OpenGL call has been executed
    glutSwapBuffers ();  // swap the render buffer and the displayed (screen) one
}

void displayRayImage () {
    glDisable (GL_DEPTH_TEST);
    glDrawPixels (screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, static_cast<void*>(rayImage));
    glutSwapBuffers ();
    glEnable (GL_DEPTH_TEST);
}


float* rayTraceThreaded(const int& numThreadx,const int& numThready, float rayImage2[], float max[]) {
    Vec3f eye = polarToCartesian (camEyePolar);
    swap (eye[1], eye[2]); // swap Y and Z to keep the Y vertical
    eye += camTarget;
    
    Vec3f pCentre = eye + normalize(camTarget - eye);
    float tmin=INFINITY;
    float tmax=0;
    float fovx=(fovAngle/1.855)*(2*M_PI/360);
    float fovy=fovx*float(screenHeight)/float(screenWidth);
    // on prend un plan image a une distance 1
    Vec3f up = normalize( Vec3f(0,1,0)-dot(normalize(camTarget - eye),Vec3f(0,1,0))*normalize(camTarget - eye) );
    //std::cout << "direction regard : " << normalize(camTarget - eye) << std::endl;
    //std::cout << "taille up : " << up.length() << std::endl;
    //std::cout << "produit scalaire : " << dot(normalize(camTarget - eye),up) << std::endl;
    
    // Raytraced image intermediaire en float
    
    
    float min[3] = {INFINITY,INFINITY,INFINITY};
    
    
    float rapport=(sqrt(nombreRayPixAA)-1)/sqrt(nombreRayPixAA);
    
    for (int bI=0; bI<nbBloc/2; bI++)
        for (int bJ=0; bJ<nbBloc/2; bJ++) {
            std::cout << "thread " << numThreadx << numThready << " : "<< floor((100.f/(nbBloc*nbBloc/4))*(bI*(nbBloc/2)+bJ)) << "%" << std::endl;
            for (int i = (2*bI+numThreadx)*(screenHeight/nbBloc); i < (2*bI+1+numThreadx)*(screenHeight/nbBloc); i++){
                float y = (float((2*i-int(screenHeight)))*tan(fovy))/int(screenHeight);
                
                for ( int  j = (2*bJ+numThready)*(screenWidth/nbBloc); j < (2*bJ+1+numThready)*screenWidth/nbBloc; j++) {
                    unsigned int index = 3*(j+i*screenWidth);
                    rayImage[index] = rayImage[index+1] = rayImage[index+2] = 0;
                    float x = -(float((2*j-int(screenWidth)))*tan(fovx))/int(screenWidth);
                    
                    Vec3f radiance=Vec3f(0,0,0);
                    
                    
                    
                    for (int k=0; k<nombreRayPixAA; k++) {
                        
                        // taille pixel : 2*tan(fovy))/int(screenHeight)
                        float dx=(float(rand())/RAND_MAX)*2*(tan(fovx)/int(screenWidth))-(tan(fovx)/int(screenWidth));
                        float dy=(float(rand())/RAND_MAX)*2*(tan(fovy)/int(screenHeight))-(tan(fovy)/int(screenHeight));
                        
                        //std::cout << "x : " << x << "   dx : " << dx << std::endl;
                        //std::cout << "y : " << y << "   dx : " << dy << std::endl;
                        
                        
                        
                        
                        Vec3f posPix = pCentre + (x+dx*rapport)*normalize(cross(up, normalize(camTarget - eye))) + (y+dy*rapport)*up;
                        Vec3f direction = normalize(posPix - eye);
                        //std::cout << x << " " << y << std::endl;
                        Ray myRay = Ray(eye, direction);
                        //Vec3f intersection = myRay.raySceneIntersection(shapes, t);
                        Triangle tri(0,0,0,0,0);
                        Vec3f coordBar(0,0,0);
                        float t=INFINITY;
                        myRay.raySceneIntersectionKdTree(tree, shapes, tri, coordBar, t, false);
                        
                        if (t< tmin){
                            tmin=t;
                        }
                        // Test if there is any intersection
                        if (t<INFINITY) {
                            //std::cout << tri[0] << std::endl;
                            radiance += myRay.evaluateResponse(shapes, tree, materials, coordBar, tri, lightPos, nbRebond, nbRebond);
                            //std::cout << t << std::endl;
                        }
                        
                        
                        
                    }
                    radiance/=nombreRayPixAA;
                    
                    for (int n=0; n<3; n++) {
                        if (radiance[n]>max[n])
                            max[n] = radiance[n];
                    }
                    
                    rayImage2[index] = radiance[0];
                    rayImage2[index+1] = radiance[1];
                    rayImage2[index+2] = radiance[2];
                    
                    
                }
            }
        }
    return max;
    
    
}


// MAIN FUNCTION TO CHANGE !
void rayTrace () {
    
    tree=kdTree(shapes,TriangleListFromShapes(shapes));
    /*glMatrixMode (GL_PROJECTION); // Set the projection matrix as current. All upcoming matrix manipulations will affect it.
     glLoadIdentity ();
     gluPerspective (fovAngle, aspectRatio, nearPlane, farPlane); // Set the current projection matrix with the camera intrinsics
     glMatrixMode (GL_MODELVIEW); // Set the modelview matrix as current. All upcoming matrix manipulations will affect it.
     glLoadIdentity ();*/
    
    // Raytraced image intermediaire en float
    float * rayImage2 = NULL;
    if (rayImage2 != NULL)
        delete [] rayImage2;
    unsigned int l = 3*screenWidth*screenHeight;
    rayImage2 = new float [l];
    memset (rayImage2, 0, l);
    float max00[3] = {-INFINITY,-INFINITY,-INFINITY};
    float max01[3] = {-INFINITY,-INFINITY,-INFINITY};
    float max10[3] = {-INFINITY,-INFINITY,-INFINITY};
    float max11[3] = {-INFINITY,-INFINITY,-INFINITY};
    
    std::thread th00(rayTraceThreaded,0,0,rayImage2,max00);
    std::thread th01(rayTraceThreaded,0,1,rayImage2,max01);
    std::thread th10(rayTraceThreaded,1,0,rayImage2,max10);
    std::thread th11(rayTraceThreaded,1,1,rayImage2,max11);
    
    th00.join();
    th01.join();
    th10.join();
    th11.join();
    
    float max=0;
    for (int m=0; m<3; m++) {
        if (max00[m]>max) {
            max=max00[m];
        }
        if (max10[m]>max) {
            max=max10[m];
        }
        if (max01[m]>max) {
            max=max01[m];
        }
        if (max11[m]>max) {
            max=max11[m];
        }
    }
    
    unsigned int numberIndex = 3*screenHeight*screenWidth;
    if (max>1) {
        
        for (int i = 0; i < numberIndex; i++){
            if (rayImage2[i]>= 1) {
                rayImage[i]=255;
            }
            else
                rayImage[i]= floor(255*rayImage2[i]);
            
        }
    }
    else{
        for (int i = 0; i < numberIndex; i++){
            rayImage[i]= floor(255*rayImage2[i])/max;
            
        }
    }
    
    std::cout << "FINI" << std::endl;
    std::cout << "MAx : " << max << std::endl;
}

void rayThrow512(const Vec3f& eye, const Vec3f& pCentre, const Vec3f& up , const float& rapport, const float& fovx, const float& fovy, float& focale){
    
    
    for (int throwNumb=0; throwNumb<128; throwNumb++) {
        //chaque ray :
        float xR=screenWidth*float(rand())/RAND_MAX;
        float yR=screenHeight*float(rand())/RAND_MAX;
        
        int xPix= floor(xR);
        int yPix= floor(yR);
        unsigned int index = 3*(xPix+yPix*screenWidth);
        rayImageNumber[index]++;
        rayImageNumber[index+1]++;
        rayImageNumber[index+2]++;
        
        float y = (float((2*yR-float(screenHeight)))*tan(fovy))/screenHeight;
        
        float x = -(float((2*xR-float(screenWidth)))*tan(fovx))/screenWidth;
        
        Vec3f posPix = pCentre + x*focale*normalize(cross(up, normalize(camTarget - eye))) + y*focale*up;
        Vec3f direction = normalize(posPix - eye);
        
        float tmin=INFINITY;
        
        float min[3] = {INFINITY,INFINITY,INFINITY};
        
        
        
        Vec3f radiance=Vec3f(0,0,0);
        
        //std::cout << x << " " << y << std::endl;
        Ray myRay = Ray(eye, direction);
        //Vec3f intersection = myRay.raySceneIntersection(shapes, t);
        Triangle tri(0,0,0,0,0);
        Vec3f coordBar(0,0,0);
        float t=INFINITY;
        myRay.raySceneIntersectionKdTree(tree, shapes, tri, coordBar, t, false);
        
        if (t< tmin){
            tmin=t;
        }
        // Test if there is any intersection
        if (t<INFINITY) {
            //std::cout << tri[0] << std::endl;
            radiance= myRay.evaluateResponse(shapes, tree, materials, coordBar, tri, lightPos, nbRebond, nbRebond);
            //std::cout << t << std::endl;
            rayImageNotNormal[index]+=radiance[0];
            rayImageNotNormal[index+1]+=radiance[1];
            rayImageNotNormal[index+2]+=radiance[2];
            
        }
    }
    
}


void rayTraceInIteract(const Vec3f& currentCamPos,const float& currentFov,const Vec3f& eye, const Vec3f& pCentre, const Vec3f& up , const float& rapport, const float& fovx, const float& fovy, float& focale){
    
    
    
    
    std::thread th00(rayThrow512,eye,pCentre,up,rapport,fovx,fovy,focale);
    std::thread th01(rayThrow512,eye,pCentre,up,rapport,fovx,fovy,focale);
    std::thread th10(rayThrow512,eye,pCentre,up,rapport,fovx,fovy,focale);
    std::thread th11(rayThrow512,eye,pCentre,up,rapport,fovx,fovy,focale);
    
    th00.join();
    th01.join();
    th10.join();
    th11.join();
    
    
    // Calculer l'image normalisée par rapport au nombre de rayon
    unsigned int index = 3*screenHeight*screenWidth;
    float maxX=-INFINITY;
    for (int i = 0; i < index; i++){
        if (rayImageNumber[i]>0){
            if ((rayImageNotNormal[i]/(rayImageNumber[i]))>maxX) {
                maxX=(rayImageNotNormal[i]/(rayImageNumber[i]));
            }
        }
        
    }
    // Calculer l'image normalisée
    
    unsigned int numberIndex = 3*screenHeight*screenWidth;
    if (maxX>1) {
        
        for (int i = 0; i < index; i++){
            
            
            if (rayImageNotNormal[i]/(rayImageNumber[i])>1) {
                rayImage[i]=255;
            }
            
            else
                rayImage[i]   = 255*(rayImageNotNormal[i]/(rayImageNumber[i]));
            
        }
    }
    else{
        for (int i = 0; i < numberIndex; i++){
            rayImage[i]= 255*(rayImageNotNormal[i]/(rayImageNumber[i]))/maxX;
            
        }
    }
    
    
    //std::cout<< maxX << std::endl;
    // Afficher l'image
    
    
}

void rayTraceInteractif(){
    
    
    unsigned int l = 3*screenWidth*screenHeight;
    if(isOutOfDateMouse){
        memset(rayImage,0,l);
        memset(rayImageNotNormal,0,l*sizeof(float));
        memset(rayImageNumber,0,l*sizeof(int));
        isOutOfDateMouse=false;
    }
    
    Vec3f eye = polarToCartesian (camEyePolar);
    swap (eye[1], eye[2]); // swap Y and Z to keep the Y vertical
    eye += camTarget;
    
    focale = dist(camTarget, eye);
    
    Vec3f pCentre = eye + focale*normalize(camTarget - eye);
    
    float fovx=(fovAngle/1.855)*(2*M_PI/360);
    float fovy=fovx*float(screenHeight)/float(screenWidth);
    float rapport=(sqrt(nombreRayPixAA)-1)/sqrt(nombreRayPixAA);
    ouverture=sceneRadius/10;
    
    Vec3f up = normalize( Vec3f(0,1,0)-dot(normalize(camTarget - eye),Vec3f(0,1,0))*normalize(camTarget - eye) );
    Vec3f currentCamPos=camEyePolar;
    float currentFov = fovAngle;
    
    if (defocus) {
        Vec3f right = cross(up, normalize(camTarget-eye));
        Vec3f randC = generateRandCircle();
        eye += ouverture*randC[0]*right + ouverture*randC[1]*up;
        currentCamPos[0] = sqrt(pow(eye[0], 2)+pow(eye[1], 2)+pow(eye[2], 2)) ;
        currentCamPos[1] =acos(eye[2]/currentCamPos[0]);
        currentCamPos[2] =atan(eye[1]/eye[0]);
    }
    
    std::cout << eye[0] << std::endl;
    
    rayTraceInIteract(currentCamPos,currentFov,eye,pCentre,up,rapport,fovx,fovy,focale);
    if(rayDisplayMode){
        //std::cout << "test" << std::endl;
    displayRayImage();
    }
    
    
    
};



void saveRayImage (const string & filename) {
    if (rayImage != NULL) {
        std::ofstream out (filename.c_str ());
        out << "P3" << endl
        << screenWidth << " " << screenHeight << endl
        << "255" << endl;
        for (unsigned int i = 0; i < 3*screenWidth*screenHeight; i++)
            out << static_cast<int>(rayImage[i]) << " ";
        out.close ();
    }
}

void keyboard (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
        case ' ':
            rayDisplayMode = !rayDisplayMode;
            interactifMode = false;
            glutPostRedisplay ();
            break;
        case 'i':
            rayDisplayMode = true;
            interactifMode = ! interactifMode;
            glutPostRedisplay ();
            break;
        case 'r':
            rayTrace ();
            glutPostRedisplay ();
            break;
        case '+':
            fovAngle-=5;
            rasterize();
            break;
        case '-':
            fovAngle+=5;
            rasterize();
            break;
        case 'd':
            defocus = !defocus;
            break;
        case 'p':
            focale += 50;
            break;
        case 'm':
            focale -= 50;
            break;
        case 's':
            saveRayImage ("raytraced_image.ppm");
            break;
        case 'q':
        case 27:
            exit (0);
            break;
        default:
            printUsage ();
            break;
    }
}

void mouse (int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            rayDisplayMode=false;
            mouseLeftButtonClicked = true;
            clickedX = x;
            clickedY = y;
            baseCamPhi = camEyePolar[1];
            baseCamTheta = camEyePolar[2];
        } else {
            rayDisplayMode=true;
            isOutOfDateMouse=true;
            mouseLeftButtonClicked = false;
        }
    }
}

void motion (int x, int y) {
    if (mouseLeftButtonClicked == true) {
        camEyePolar[1] =  baseCamPhi  + (float (clickedY-y)/screenHeight) * M_PI;
        camEyePolar[2] = baseCamTheta + (float (x-clickedX)/screenWidth) * M_PI;
        int l = 3*screenHeight*screenWidth;
        memset (rayImageNumber, 0, l);
        memset (rayImageNotNormal, 0, l);
        glutPostRedisplay (); // calls the display function
    }
}


void display () {
    
    if (rayDisplayMode){
        if (!interactifMode){
            //rayTraceInteractif();
            displayRayImage ();
        }
        
    }
    else
        rasterize ();
}

// This function is executed in an infinite loop.
void idle () {
    if (interactifMode){
        rayTraceInteractif();
    }
    
}

int main (int argc, char ** argv) {
    
    glutInit (&argc, argv); // Initialize a glut app
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE); // Setup a RGBA framebuffer to display, with a depth buffer (z-buffer), in double buffer mode (fill a buffer then update the screen)
    glutInitWindowSize (DEFAULT_SCREENWIDTH, DEFAULT_SCREENHEIGHT); // Set the window app size on screen
    window = glutCreateWindow (appTitle.c_str ()); // create the window
    if (argc > 1)
        init (argv[1]); // Your initialization code (OpenGL states, geometry, material, lights, etc)
    else
        init (DEFAULT_SCENE_FILENAME);
    glutReshapeFunc (reshape); // Callback function executed whenever glut need to setup the projection matrix
    glutDisplayFunc (display); // Callback function executed when the window app need to be redrawn
    glutKeyboardFunc (keyboard); // Callback function executed when the keyboard is used
    glutMouseFunc (mouse); // Callback function executed when a mouse button is clicked
    glutMotionFunc (motion); // Callback function executed when the mouse move
    glutIdleFunc (idle); // Callback function executed continuously when no other event happens (good for background procesing or animation for instance).
    printUsage (); // By default, display the usage help of the program
    glutMainLoop ();
    return 0;
}

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:

