
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

using namespace std;

// App parameters
static const unsigned int DEFAULT_SCREENWIDTH = 512;
static const unsigned int DEFAULT_SCREENHEIGHT = 512;
static const char * DEFAULT_SCENE_FILENAME = "scenes/cornell_box/cornell_box.obj";
static string appTitle ("MCRT - Monte Carlo Ray Tracer");
static GLint window;
static unsigned int screenWidth;
static unsigned int screenHeight;
static bool rayDisplayMode = false;
static bool blurDisplayMode = false;

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


// Raytraced image
static unsigned char * rayImage = NULL;
static float * t_buffer = NULL;
static unsigned char * blurImage;
static unsigned char *DixImage;

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
    lightPos=Vec3f(278,540,279.5);
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
    unsigned int l = 3*screenWidth*screenHeight;
    rayImage = new unsigned char [l];
    memset (rayImage, 0, l);
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

// MAIN FUNCTION TO CHANGE !
void rayTrace () {
    
    tree=kdTree(shapes,TriangleListFromShapes(shapes));
    /*glMatrixMode (GL_PROJECTION); // Set the projection matrix as current. All upcoming matrix manipulations will affect it.
     glLoadIdentity ();
     gluPerspective (fovAngle, aspectRatio, nearPlane, farPlane); // Set the current projection matrix with the camera intrinsics
     glMatrixMode (GL_MODELVIEW); // Set the modelview matrix as current. All upcoming matrix manipulations will affect it.
     glLoadIdentity ();*/
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
    std::cout << "direction regard : " << normalize(camTarget - eye) << std::endl;
    std::cout << "taille up : " << up.length() << std::endl;
    std::cout << "produit scalaire : " << dot(normalize(camTarget - eye),up) << std::endl;
    
    // Raytraced image intermediaire en float
    float * rayImage2 = NULL;
    if (rayImage2 != NULL)
        delete [] rayImage2;
    unsigned int l = 3*screenWidth*screenHeight;
    rayImage2 = new float [l];
    t_buffer = new float[screenWidth*screenHeight];
    
    memset (rayImage2, 0, l);
    float max[3] = {-INFINITY,-INFINITY,-INFINITY};
    float min[3] = {INFINITY,INFINITY,INFINITY};
    
    unsigned int nombreRayPixAA=4;
    float rapport=(sqrt(nombreRayPixAA)-1)/sqrt(nombreRayPixAA);
    unsigned int nbRebond=2;
    
    for (int i = 0; i < screenHeight; i++){
        if (i%(screenHeight/100)==0)
            std::cout << i/(screenHeight/100) << "%" << std::endl;
        float y = (float((2*i-int(screenHeight)))*tan(fovy))/int(screenHeight);
        
        for ( int  j = 0; j < screenWidth; j++) {
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
                myRay.raySceneIntersectionKdTree(tree, shapes, tri, coordBar, t);
                t_buffer[i*screenHeight + j] = t;
                if (t< tmin){
                    tmin=t;
                }
                // Test if there is any intersection
                
                //std::cout << tri[0] << std::endl;
                radiance += myRay.evaluateResponse(shapes, tree, materials, coordBar, tri, lightPos, nbRebond, nbRebond);
                //std::cout << t << std::endl;
                
                
            }
            //radiance/=nombreRayPix;
            
            if (tmin!=INFINITY ) {
                for (int n=0; n<3; n++) {
                    if (radiance[n]>max[n])
                        max[n] = radiance[n];
                    if (radiance[n]<min[n])
                        min[n] = radiance[n];
                }
                
                rayImage2[index] = radiance[0];
                rayImage2[index+1] = radiance[1];
                rayImage2[index+2] = radiance[2];
                
            }
            
        }
    }
    
    
    
    for (int i = 0; i < screenHeight; i++){
        for ( int  j = 0; j < screenWidth; j++) {
            unsigned int index = 3*(j+i*screenWidth);
            rayImage[index] = rayImage[index+1] = rayImage[index+2] = 0;
            
            rayImage[index]   = 255*(rayImage2[index]   )/(max[0]);
            rayImage[index+1] = 255*(rayImage2[index+1] )/(max[1]);
            rayImage[index+2] = 255*(rayImage2[index+2] )/(max[2]);
        }
    }
    std::cout << " tmin : " << max << std::endl;
    std::cout << " tmax : " << min << std::endl;
    
    //Liberer la memoire delete
    delete rayImage2;
}

void displayBlurImage () {
    glDisable (GL_DEPTH_TEST);
    glDrawPixels (screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, static_cast<void*>(blurImage));
    glutSwapBuffers ();
    glEnable (GL_DEPTH_TEST);
}

void blur(){
    int Agrand = 3;
    int l = Agrand*Agrand*3*screenWidth*screenHeight;
    unsigned char *blurImageDix = new unsigned char[l];
    if (blurImage!=nullptr) {
        delete blurImage;
    }
    blurImage = new unsigned char[3*screenWidth*screenHeight];
    float ouverture = 20;
    float focale = 500;
    float planMiseAuPoint = 917;
    if (DixImage!=nullptr) {
        delete DixImage;
    }
    DixImage = new unsigned char[3*screenWidth*Agrand*screenHeight*Agrand];
    float *DixDist = new float[screenWidth*Agrand*screenHeight*Agrand];
    float poids0=0, poids1=0, poids2=0, poids3=0;
    
    for (float i=0; i<screenWidth*Agrand; i++) {
        for (float j=0; j<screenHeight*Agrand; j++) {
            unsigned int index = 3*(j+(i*screenWidth*Agrand));
            unsigned int index2 = j+(i*screenWidth*Agrand);

            float is = (float)floor(i/Agrand);
            float js = (float)floor(j/Agrand);
            //cout<<"is : "<<is<<"  js : "<<js<<endl;
            if ((int)(i)%Agrand!=0 && (int)(j)%Agrand!=0) {
                poids0 = 1.f/sqrt( pow((i/Agrand)-is,2) + pow(((j/Agrand)-js),2) );
                poids1 = 1.f/sqrt( pow((i/Agrand)-is-1.f,2) + pow((j/Agrand)-js,2) );
                poids2 = 1.f/sqrt( pow((i/Agrand)-is,2) + pow((j/Agrand)-js-1.f,2) );
                poids3 = 1.f/sqrt( pow((i/Agrand)-is-1.f,2) + pow((j/Agrand)-js-1.f,2) );
                for (int n=0; n<3; n++) {
                    DixImage[index+n] = rayImage[(int)(3*(js+is*screenWidth)+n)]*poids0 + rayImage[(int)(3*(js+(is+1)*screenWidth)+n)]*poids1 + rayImage[(int)(3*(js+1+is*screenWidth)+n)]*poids2 + rayImage[(int)(3*(js+1+(is+1)*screenWidth)+n)]*poids3 ;
                    DixImage[index+n] /= ( poids0 + poids1 + poids2 + poids3 );
                }
                DixDist[index2] = t_buffer[(int)(js+is*screenWidth)]*poids0 + t_buffer[(int)(js+(is+1)*screenWidth)]*poids1 + t_buffer[(int)(js+1+is*screenWidth)]*poids2 + t_buffer[(int)(js+1+(is+1)*screenWidth)]*poids3 ;
                DixDist[index2] /= ( poids0 + poids1 + poids2 + poids3 );
            }else{
                //cout<<"index : "<< index <<endl;
                
                DixImage[index]   = rayImage[(int)(3*(js+is*screenWidth))];
                DixImage[index+1] = rayImage[(int)(3*(js+is*screenWidth)+1)];
                DixImage[index+2] = rayImage[(int)(3*(js+is*screenWidth)+2)];
                DixDist[index2]   = t_buffer[(int)(js+is*screenWidth)];
            }
            
            
        }
    }
    
    for (int i = 22; i < Agrand*screenWidth-22; i++){
        for ( int  j = 22; j < Agrand*screenHeight-22; j++) {
            unsigned int index = 3*(j+i*screenWidth*Agrand);
            float radius = 0.f;
            if (DixDist[Agrand*i*screenHeight+j]!=INFINITY) {
                radius = abs( ouverture*( focale*(planMiseAuPoint - DixDist[Agrand*i*screenWidth+j]) )/(DixDist[Agrand*i*screenWidth+j]*( planMiseAuPoint-focale)) );
            } else radius = 10;
            
            int r = radius<1 ? 1 : floor(radius);
            
            int W = r<10 ? 2*r-1 : 21;
            
            
            float kernel[W][W];
            float sigma = 1;
            float mean = 0;
            float sum = 0.0; // For accumulating the kernel values
            for (int x = 0; x < W; ++x){
                for (int y = 0; y < W; ++y) {
                    kernel[x][y] = exp( -0.5 * (pow((x-mean)/sigma, 2.0) + pow((y-mean)/sigma,2.0)) )
                    / (2 * M_PI * sigma * sigma);
                    // Accumulate the kernel values
                    sum += kernel[x][y];
                }
            }
            // Normalize the kernel
            for (int x = 0; x < W; ++x){
                for (int y = 0; y < W; ++y){
                    kernel[x][y] /= sum;
                }
            }
            
            
            for (int x = 0; x < r+1; ++x){
                for (int y = 0; y < r+1; ++y){
                    blurImageDix[index]   += kernel[x][y]*DixImage[3*((j-x)-y+i*screenWidth*Agrand)];
                    blurImageDix[index+1] += kernel[x][y]*DixImage[3*((j-x)-y+i*screenWidth*Agrand)+1];
                    blurImageDix[index+2] += kernel[x][y]*DixImage[3*((j-x)-y+i*screenWidth*Agrand)+2];
                    blurImageDix[index]   += kernel[x][y]*DixImage[3*((j+x)+y+i*screenWidth*Agrand)];
                    blurImageDix[index+1] += kernel[x][y]*DixImage[3*((j+x)+y+i*screenWidth*Agrand)+1];
                    blurImageDix[index+2] += kernel[x][y]*DixImage[3*((j+x)+y+i*screenWidth*Agrand)+2];

                    if (blurImageDix[index]>1) {
                        //cout<<"coucou1"<<endl;
                    }
                }
            }
        }
    }
    
    for (int i=0; i<screenWidth; i++) {
        for (int j=0; j<screenHeight; j++) {
            unsigned int index = 3*(j+i*screenWidth);
            blurImage[index]   = blurImageDix[ 3*((j*Agrand)+((i*Agrand)*screenWidth)) ];
            blurImage[index+1] = blurImageDix[3*((j*Agrand)+((i*Agrand)*screenWidth))+1];
            blurImage[index+2] = blurImageDix[3*((j*Agrand)+((i*Agrand)*screenWidth))+2];
            if (blurImage[index]>1) {
                //cout<<"coucou2"<<endl;
            }
        }
    }
   
    
    
    //free memory
    delete blurImageDix;
    delete DixDist;
    
}


void display () {
    if (rayDisplayMode){
        if (blurDisplayMode) {
            displayBlurImage ();
        }else
            displayRayImage ();
    }
    else
        rasterize ();
}

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
            glutPostRedisplay ();
            break;
        case 'b':
            blur();
            glutPostRedisplay ();
            cout << "blur ok "<<endl;
            break;
        case 'n':
            blurDisplayMode = !blurDisplayMode;
            glutPostRedisplay ();
            break;
        case 'r':
            rayTrace ();
            glutPostRedisplay ();
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
            mouseLeftButtonClicked = true;
            clickedX = x;
            clickedY = y;
            baseCamPhi = camEyePolar[1];
            baseCamTheta = camEyePolar[2];
        } else {
            mouseLeftButtonClicked = false;
        }
    }
}

void motion (int x, int y) {
    if (mouseLeftButtonClicked == true) {
        camEyePolar[1] =  baseCamPhi  + (float (clickedY-y)/screenHeight) * M_PI;
        camEyePolar[2] = baseCamTheta + (float (x-clickedX)/screenWidth) * M_PI;
        glutPostRedisplay (); // calls the display function
    }
}

// This function is executed in an infinite loop.
void idle () {
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

