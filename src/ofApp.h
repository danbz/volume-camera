#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofxKinectMeshRecorder.h"
#include "metaData.h"
#include "ofxXmlSettings.h"
#include "ofxImGui.h"

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
    void savePointCloud();
    void writeMetaData();
    void loadRecording();
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
    
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	ofxCvContourFinder contourFinder;
	
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
