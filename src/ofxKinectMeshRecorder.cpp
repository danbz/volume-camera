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
    
    cout << "ofxKinectMeshRecorder::loadMesh() | Loading..." << endl;
    
    TotalFrames = countFrames(_file)-1;
    ofFile file(ofToDataPath(_file + "/"));
    string path = file.getAbsolutePath();
    file.close();
    
    recordedMeshData.clear();
    recordedMeshData.resize(TotalFrames);
    
    FramesLoaded = 0;
    
    for(int i = 0; i <= TotalFrames; i += 1) {
        
        //string fileToload = path + "/frame" + ofToString(i) + ".txt";
        string fileToload = path + "frame" + ofToString(i) + ".txt";

        
        // TODO: Este metodo de parseo es muy lento.
        // Habria que volcar con fscanf o algoa si el txt en la estrutura framedata directamente.
        
        /*
         vector<frameData> data;
         char line[200];
         FILE* fp = fopen(ofToDataPath(fileToload).c_str(),"rt");
         while(fgets(line,1,fp) != NULL)
         {
         frameData pc;
         sscanf(line,"%i,%f,%f,%f,%i\n",pc.framenum, pc.x, pc.y, pc.z, pc.hexcolor);
         data.push_back(pc);
         // do something
         }
         fclose(fp);
        
         */
        
        ifstream fin;
        fin.open( ofToDataPath(fileToload).c_str() );
        
        vector<frameData> data;
        int lineCounter = 0;
        //while(fin!=NULL)
        while(fin)
            
        {
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
                //cout << "loading line: " << lineCounter << endl;
                lineCounter ++;
            }
                    }
        fin.close();
        
        recordedMeshData[i].resize(data.size());
        recordedMeshData[i] = data;
        
        FramesLoaded = i;
        cout << "total line: " << lineCounter << endl;

       cout << fileToload << endl;
        
        if(i == TotalFrames) {
            unlock();
            stopThread();
             //stopThread(false);
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
















