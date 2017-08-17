//
//  ofxKinectMeshRecorder.cpp
//
//  modified by Daniel Buzzo on 14/05/2017.
// https://github.com/danbz/volume-camera
//
//  ofxKinectMeshRecorder.cpp
//  ofxKinectMeshRecorder
//
//  Created by welovecode on 15/07/12.
//

#include "ofxKinectMeshRecorder.h"

ofxKinectMeshRecorder::ofxKinectMeshRecorder() {
    
    readyToPlay = true;
    
}

//------------------------------------

//void ofxKinectMeshRecorder::startLoading(const string _file, int width, int height) {
//    
//    imageWidth = width;
//    imageHeight = height;
//    fileToload = _file;
//   // startThread(true, true);
//    
//    totalFrames=0;
//     loadImageData(fileToload);
//    
//}

//------------------------------------

//void ofxKinectMeshRecorder::threadedFunction() {
//    
//    cout << "ofxKinectMeshRecorder::loadMesh() | Start Thread..." << endl;
//    readyToPlay = false;
//    lock();
//    
//    //while(isThreadRunning()) {
//        // loadMeshData(fileToload);
//        loadImageData(fileToload);
//    //}
//    
//    //readyToPlay = true;
//    cout << "ofxKinectMeshRecorder::loadMesh() | Stop Thread..." << endl;
//}

//_____________________________________

bool ofxKinectMeshRecorder::loadImageData(const string _file, int width, int height ) { // new routine to load rgb & depth data from png files
    
    imageWidth = width;
    imageHeight = height;
    
    totalFrames = (countImageFrames(_file)/2);
    cout << "totalFrames: " << totalFrames << endl;
    if (totalFrames == 0){
        ofSystemAlertDialog("A loading error occurred, your PNG files appear to be messed up...");
        cout << "stopped loading due to PNG file loading error" << endl ;
        return false;
        //unlock();
        //stopThread();
        
    } else {
        readyToPlay = false;

        ofFile file(ofToDataPath(_file + "/"));
        string path = file.getAbsolutePath();
        file.close();
        recordedColorImageData.clear();
        recordedDepthImageData.clear();
        //recordedMeshData.resize(totalFrames);
        framesLoaded = 0;
        colImage.setUseTexture(false); // prevent images loading to GPU while in thread (causes crashing https://stackoverflow.com/questions/35634525/in-openframeworksc-cannot-initialize-ofimage-object-in-thread-class-due-to-t#35653210)
        depImage.setUseTexture(false);
        
        colImage.allocate(imageWidth, imageHeight, OF_IMAGE_COLOR);
        depImage.allocate(imageWidth, imageHeight, OF_IMAGE_GRAYSCALE);
        
        for(int i = 0; i < totalFrames; i += 1) {
            cout << "ofxKinectMeshRecorder::loadImageMesh() | Loading frame " << i+1 << " of " << totalFrames << endl;
            string colorFileToload = path + "colorData" + ofToString(i) + ".png";
            string depthFileToload = path + "depthData" + ofToString(i) + ".png";
            
            colImage.load(colorFileToload);
            depImage.load(depthFileToload);
            recordedColorImageData.push_back(colImage);
            recordedDepthImageData.push_back(depImage);
            
            framesLoaded = i;
//            //if(i == totalFrames-1) {
//                readyToPlay = true;
//                return true;
//                //                unlock();
//                //                stopThread();
//                
//                  cout << "stopped successful loading" << endl ;
//            //}
        }
        readyToPlay = true;
        return true;
        cout << "stopped successful loading" << endl ;
    }
}

//------------------------------------
int ofxKinectMeshRecorder::countImageFrames(const string _file) {
    
    string path = ofToDataPath(_file + "/");
    ofDirectory dir(path);
    dir.allowExt("png");
    dir.listDir();
    return dir.size();
}

//------------------------------------
ofImage ofxKinectMeshRecorder::getColorImageAt(int framenum) {
    
    ofImage colorImage;
    colorImage= recordedColorImageData[framenum];
    return colorImage;
}

//------------------------------------
ofShortImage ofxKinectMeshRecorder::getDepthImageAt(int framenum) {
    
    ofShortImage depthImage;
    depthImage= recordedDepthImageData[framenum];
    return depthImage;
}

//-------------------------------------------------------------------

void ofxKinectMeshRecorder::clearImageData() {
    recordedColorImageData.clear();
    recordedDepthImageData.clear();
}















