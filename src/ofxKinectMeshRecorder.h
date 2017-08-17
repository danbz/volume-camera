//
//  ofxKinectMeshRecorder.h
//  volca: volume camera
//
//  modified by Daniel Buzzo on 14/05/2017.
// http://www.buzzo/com
// https://github.com/danbz/volume-camera
//
//  ofxKinectMeshRecorder.h
//  ofxKinectMeshRecorder
//
//  Created by welovecode on 15/07/12 .

#ifndef ofxKinectMeshRecorder_ofxKinectMeshRecorder_h
#define ofxKinectMeshRecorder_ofxKinectMeshRecorder_h

#include "ofMain.h"
#include "ofxXmlSettings.h"


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
    vector <ofImage>  recordedColorImageData;
    vector <ofShortImage> recordedDepthImageData;
    string fileToload;
    
    
    //void loadMeshData(const string _file); // old routine to load mesh from text files
   // int countFrames(const string _file);
    int countImageFrames(const string _file);
    //void threadedFunction();
    
public:
        
    bool readyToPlay;
    
    int totalFrames;
    int framesLoaded;
    
    ofxKinectMeshRecorder();
    //void startLoading(const string _file, int width, int height);
    bool loadImageData(const string _file, int width, int height); //new routine to load rgb and depth data from png images

    void clearImageData();
    
    ofImage getColorImageAt ( int framenum);
    ofShortImage getDepthImageAt ( int framenum);
    
    int imageWidth =640;
    int imageHeight =480;
    ofImage colImage;
    ofShortImage depImage;
    /// moving in metadata functions

    ofxXmlSettings exifSettings;
    
};


#endif /* ofxKinectMeshRecorder_h */
