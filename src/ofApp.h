#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectMeshRecorder.h"
#include "metaData.h"
#include "ofxXmlSettings.h"
#include "ofxImGui.h"
#include "ofxCv.h"

// Windows users: You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:     ofxKinect/libs/libfreenect/platform/windows/inf
// This should install the Kinect camera, motor, & audio drivers.
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager. No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
    void drawAnyPointCloud();
    void triangulateMesh(ofMesh &mesh);
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
    
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	//ofxCvColorImage colorCvImage; // RGB image from Kinect
//	ofxCvGrayscaleImage grayImage; // grayscale depth image from Kinect
//	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
//	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
//	ofxCvContourFinder contourFinder;
    
    
    ofImage colorImage;
    ofImage depthImage;
    ofPixels depthPixels;
    
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	int nearThreshold;
	int farThreshold;
	int angle;
	
    ofEasyCam easyCam; 	// used for viewing the point cloud
    
    // with elements of kinect recorder hack from code by Pelayo MŽndez   https://github.com/pelayomendez
    
    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } pointData;
    
    typedef struct {
        int x;
        int y;
        int z;
    } colorData;
    
    ofDirectory dirHelper;
    string generateFileName();
    int frame;
    string saveTo;
    int distanceMinima;
    int distanceMaxima;
    int recordWidth;
    int recordHeight;
    
    //////////////////////////////////////////////////////
    // GUI  Configuration
    //////////////////////////////////////////////////////
    bool showGui;
    bool paintMesh;
    bool drawTriangles;
    bool recording;
    bool playing;
    bool colorMode;
    
    ofxImGui::Gui imGui;
    ImVec4 imBackgroundColor;
    bool show_test_window;
    int playbackFPS, blobSize, gridSize, backPlane, frontPlane, recordingStep;
    
    // shot timing GUI elements
    bool singleShot;
    float exposureTime;
    int exposureStart;
    int recordFPS;
    int lastRecordedFrame;
    //unsigned char exposureBuffer; // array for racking frames from Kinext camera into for exposure timing
    float numOfFramesInExposureBuffer; // number of frames read from kinect webcam into exposure buffer (actual exposure = buffer numbers/num of frames read in)
    
    //////////////////////////////////////////////////////
    // Rendering Reproduction
    //////////////////////////////////////////////////////
    ofMesh mesh;
    ofLight light;
    ofxKinectMeshRecorder meshRecorder;
    
    //Universal function which sets normals for the triangle mesh
    void setNormals( ofMesh &mesh ); // move this into meshrecorder // render section?

    int frameToPlay;
    int renderStyle;
    bool showNormals;
    bool illuminateScene;
    bool renderFlatQuads;
    
    //////////////////////////////////////////////////////
    // XML exif data save and load
    //////////////////////////////////////////////////////
    ofxXmlSettings exifSettings;
     
    void saveExifData();
    void loadExifData( string filePath);
    
};
