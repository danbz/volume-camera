 //
//  ofxKinectMeshRecorder.cpp
//  myKinectHack
//
//  modified by Daniel Buzzo on 14/05/2017.
//
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

void ofxKinectMeshRecorder::startLoading(const string _file) {
    
    fileToload = _file;
    startThread(true, true);
    //loadMeshData(fileToload); //bypassing thread
    //readyToPlay = true;
    
}

void ofxKinectMeshRecorder::loadMeshData(const string _file) {
    
    totalFrames = countFrames(_file); 
    ofFile file(ofToDataPath(_file + "/"));
    string path = file.getAbsolutePath();
    file.close();
    recordedMeshData.clear();
    recordedMeshData.resize(totalFrames);
    framesLoaded = 0;
    
    //cout << "ofxKinectMeshRecorder::loadMesh() | Loading frame x "  " of " << totalFrames << endl;
    
    for(int i = 0; i < totalFrames; i += 1) {
        
        cout << "ofxKinectMeshRecorder::loadMesh() | Loading frame " << i << " of " << totalFrames << endl;
        
        string fileToload = path + "frame" + ofToString(i) + ".txt";
        ifstream fin;
        fin.open( ofToDataPath(fileToload).c_str() );
        
        vector<frameData> data;
<<<<<<< HEAD
=======
<<<<<<< HEAD
        
       // while(fin!=NULL)
=======
        int lineCounter = 0;
>>>>>>> c681df83a984c9ef5a712d0898e9ee9ced4b5be6
        //while(fin!=NULL)
        while(fin) {
            string str;
            getline(fin, str);
            vector<string> pointcoords = ofSplitString(str, ",");
            
            if(str != "") {
                frameData pc;
                pc.framenum = ofToInt(pointcoords[0]);
                pc.x = ofToFloat(pointcoords[1]);
                pc.y = ofToFloat(pointcoords[2]);
                pc.z = ofToFloat(pointcoords[3]);
                pc.hexcolor = ofToInt(pointcoords[4]);
                data.push_back(pc); //push the string onto a vector of strings
            }
        }
        fin.close();
        
        recordedMeshData[i].resize(data.size()); //appears to crash here occasionally....
        recordedMeshData[i] = data;
        
        framesLoaded = i;
        if(i == totalFrames-1) {
            unlock();
            stopThread();
            cout << "stopped thread" << endl ;
            //stopThread(false); deprecated call
        }
    }
}

void ofxKinectMeshRecorder::threadedFunction() {
    
    cout << "ofxKinectMeshRecorder::loadMesh() | Start Thread..." << endl;
    readyToPlay = false;
    lock();
    
    while(isThreadRunning()) {
        loadMeshData(fileToload);
    }
    
    readyToPlay = true;
    cout << "ofxKinectMeshRecorder::loadMesh() | Stop Thread..." << endl;
}

//------------------------------------
int ofxKinectMeshRecorder::countFrames(const string _file) {
    
    string path = ofToDataPath(_file + "/");
    ofDirectory dir(path);
    dir.allowExt("txt");
    dir.listDir();
    return dir.size();
}

//------------------------------------
ofVec3f ofxKinectMeshRecorder::getVectorAt(int framenum, int coord) {
    
    ofVec3f v;
    v.set(recordedMeshData[framenum][coord].x,recordedMeshData[framenum][coord].y,recordedMeshData[framenum][coord].z);
    return v;
}

//------------------------------------
ofColor ofxKinectMeshRecorder::getColorAt(int framenum, int coord) {
    
    ofColor c;
    c.setHex(recordedMeshData[framenum][coord].hexcolor);
    return c;
}


//-------------------------------------------------------------------

//void ofxKinectMeshRecorder::loadExifData(string filePath) { // load exifXML file from the sele ted folder and get the values out
//    
//    //cout << filePath << "/exifSettings.xml" << endl;
//    
//    exifSettings.loadFile(filePath + "/exifSettings.xml");
//    recordWidth = exifSettings.getValue("exif:ImageWidth", 0);
//    recordHeight = exifSettings.getValue("exif:ImageLength", 0);
//    recordingStep = 1; // always default to 1:1 step when loading recorded meshes
//    string recordingDate = exifSettings.getValue("exif:DateTimeDigitized", "");
//    string myXml;
//    exifSettings.copyXmlToString(myXml);
//    
//    cout << "loaded exif data: " << myXml <<endl ;
//}
//
//void ofxKinectMeshRecorder::saveExifData() { //put some some settings into a file
//    
//    string path = saveTo;
//    string today =  _timestamp = ofToString(ofGetDay()) + //generate date
//    ofToString(ofGetMonth()) +
//    ofToString(ofGetYear()) +
//    ofToString(ofGetHours()) +
//    ofToString(ofGetMinutes()) +
//    ofToString(ofGetSeconds());
//    
//    //exifSettings.addTag("exifData");
//    exifSettings.setValue("exif:make", "Buzzo");
//    exifSettings.setValue("exif:model", "Volca: Experimental volumetric camera/apparatus v0.1");
//    exifSettings.setValue("exif:orientation", "top left");
//    exifSettings.setValue("exif:ImageWidth", recordWidth/recordingStep);
//    exifSettings.setValue("exif:ImageLength", recordHeight/recordingStep);
//    exifSettings.setValue("exif:DateTimeDigitized", today);
//    exifSettings.setValue("exif:ExposureTime", exposureTime);
//    exifSettings.setValue("exifSensingMethod", "Kinect depth sensor");
//    
//    exifSettings.saveFile(path + "exifSettings.xml"); //puts exifSettings.xml file in the current recordedframe folder
//    string myXml;
//    exifSettings.copyXmlToString(myXml);
//    cout << myXml <<endl ;
//}
















