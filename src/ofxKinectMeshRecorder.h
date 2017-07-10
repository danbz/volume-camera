//
//  ofxKinectMeshRecorder.hpp
//  myKinectHack
//
//  modified by Daniel Buzzo on 14/05/2017.
//
//
//  ofxKinectMeshRecorder.h
//  ofxKinectMeshRecorder
//
//  Created by welovecode on 15/07/12 .

#ifndef ofxKinectMeshRecorder_ofxKinectMeshRecorder_h
#define ofxKinectMeshRecorder_ofxKinectMeshRecorder_h

#include "ofMain.h"

class ofxKinectMeshRecorder :
public ofThread {
    
private:
    
    typedef struct {
        int framenum;
        float x;
        float y;
        float z;
        int hexcolor;
    } frameData;
    
    vector < vector<frameData> > recordedMeshData;
    
    string fileToload;
    
    
    void loadMeshData(const string _file);
    int countFrames(const string _file);
    void threadedFunction();
    
public:
    
    
    bool readyToPlay;
    
    int TotalFrames;
    int FramesLoaded;
    
    ofxKinectMeshRecorder();
    void startLoading(const string _file);
    
    ofVec3f getVectorAt(int framenum, int coord);
    ofColor getColorAt(int framenum, int coord);
    
};


#endif /* ofxKinectMeshRecorder_h */
