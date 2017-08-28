//
//  triangulateMesh.cpp
//  volume-camera
//
//  Created by Dan Buzzo on 24/08/2017.
//
//

#include "triangulateMesh.h"

//-------------------------------------

void triangulateMesh::setup(){
    
};


//-------------------------------------
void triangulateMesh::makeMesh( ofShortImage &filteredDepthImage, ofImage &filteredColorImage, ofMesh &mesh ){
    
    ofColor c;
    ofShortColor zGrey = 0;
    // int pCount =0;
    //ofMesh mesh;
    
   //int step = ofApp::recordStep;
    int step =1;
   //int width = ofApp::recordWidth;
  // int height = ofApp::recordHeight;
    
    int width = filteredDepthImage.getWidth();
    int height = filteredDepthImage.getHeight();
    
    bool paintMesh = true;
    
    int index =0;
    //int i=0;
    int z = 0;
    int ind = 0;
    ofVec3f v3;
    for(int y = 0; y < height; y += step) {
        
        // vector tempindexs;
        // indexs.push_back(tempindexs);
        
        for(int x = 0; x < width; x +=  step) {
            zGrey = filteredDepthImage.getPixels()[x+y*width];
            z = zGrey.r;
            v3.set(0,0,0);
           // if(z > frontPlane & z < backPlane) {
                //  indexs[y/recordStep].push_back(ind);
           //     ind++;
            if (paintMesh) {
                    c = (filteredColorImage.getColor(x,y)); // getting RGB from ofShortImage
                } else {
                    float h = ofMap(z, 0, 65535, 0, 255, true);
                    c.setHsb(h, 255, 255);
                }
            //} else {
           //     z= backPlane;
            //    c.setHsb(0, 0, 0);
                //    indexs[y/recordStep].push_back(-1);
            //} // clip out pixels
            //v3.set((x - (width/2)) * (perspectiveFactor * z) ,(y -(height/2)) * (perspectiveFactor *z) , z * depthFactor);
            v3.set((x - (width/2))  ,(y -(height/2)) , z );

            mesh.addVertex(v3);
            mesh.addColor(c);
        }
    }
    //
    int meshW =  width/step ;
    int meshH = height/step;
    for (int y = 0; y<height-step; y+= step){
        for (int x=0; x<width-step; x+= step){
            v3.set(0,0,0);
            //  if ((mesh.getVertex(x+y*meshW))==v3 or (mesh.getVertex((x+1)+y*(meshW)))==v3 or (mesh.getVertex(x+(y+1)*meshW)==v3)){
            //   } else {
            mesh.addIndex(x+y*meshW);               // 0
            mesh.addIndex((x+1)+y*meshW);           // 1
            mesh.addIndex(x+(y+1)*meshW);           // 10
            //}
            mesh.addIndex((x+1)+y*meshW);           // 1
            mesh.addIndex((x+1)+(y+1)*meshW);       // 11
            mesh.addIndex(x+(y+1)*meshW);           // 10
        }
    }
    
};
