#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectV2.h"
#include "ofxKinectMeshRecorder.h"
#include "triangulateMesh.h"
#include "metaData.h"
#include "ofxXmlSettings.h"
#include "ofxImGui.h"
#include "ofxCv.h"
#include "dataStructures.h"



// VOLCA: experimental volumetric camera/apparatus v0.1a
// © 2017 Daniel Buzzo. Dan@buzzo.com http://www.buzzo.com
// https://github.com/danbz/volume-camera
// all rights reserved

// Windows users: You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:     ofxKinect/libs/libfreenect/platform/windows/inf
// This should install the Kinect camera, motor, & audio drivers.
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager. No way around the Windows driver dance, sorry.
// inspired in part by kinect recorder hack by Pelayo MŽndez   https://github.com/pelayomendez
// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS


class ofApp :

public ofBaseApp {
    
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    void drawAnyPointCloud();
    void savePointCloud();
    void writeMetaData();
    void loadRecording();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void drawGui();
    void drawScreenOverlay();
    int getRecordStep();
    
    ofxKinect kinect;
    ofxKinectV2 kinect2;
    
    bool kinectConnected;
    
#ifdef USE_TWO_KINECTS
    ofxKinect kinect2;
#endif
    
    ofImage colorImage, filteredColorImage;
    ofShortImage depthImage, filteredDepthImage;
    ofShortPixels depthPixels, filteredDepthPixels;
    
    bool bThreshWithOpenCV, bDrawPointCloud;
    float nearThreshold, farThreshold;
    int angle;
    
    ofEasyCam easyCam; 	// used for viewing the point cloud
    int camDist;
    volca volca;
    vRenderer volcaRenderer;
    ///// moved into datastructures.h
    //    struct volca { // central volca object
    //        bool recording;
    //        bool playing;
    //        bool paused;
    //        bool singleShot;
    //        int recordFPS;
    //        int recordWidth, recordHeight, recordStep;
    //        string recordingDate;
    //    } volca;
    //
    //    struct vRenderer { // rendering data for volca object
    //        bool showGui;
    //        bool paintMesh;
    //        int frameToPlay;
    //        int renderStyle;
    //        bool showNormals;
    //        bool illuminateScene;
    //        bool renderFlatQuads;
    //        bool showAxes;
    //        float depthFactor;
    //        float perspectiveFactor;
    //    } volcaRenderer;
    ///// end // moved into datastructures.h

    ofDirectory dirHelper;
    string generateFileName();
    int frame;
    string saveTo;
    
    // GUI Configuration
    ofxImGui::Gui imGui;
    ImVec4 imBackgroundColor;
    bool show_test_window, blur, erodeImage, dilateImage, bfilterColorImage ;
    int playbackFPS, blobSize, gridSize, backPlane, frontPlane, blurRadius, erodeAmount, dilateAmount;
    
    // Rendering Reproduction
    ofMesh mesh;
    ofxKinectMeshRecorder volcaRecorder;
    triangulateMesh volcaMeshMaker;
    ofLight light;
    
    // XML exif data save and load
    ofxXmlSettings exifSettings;
    
    void saveExifData(); // move these into meshRecorder
    bool loadExifData( string filePath);
    string exifModel;
    
    //GUI sounds
    ofSoundPlayer startupSound;
    ofSoundPlayer shutterSound;
    ofSoundPlayer errorSound;
    
    /// use kinect 2
    ofTexture texDepth;
    ofTexture texRGB;
    
   
    
};



