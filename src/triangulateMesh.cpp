//
// triangulateMesh.cpp
// volca project
//
// Created by Dan Buzzo on 24/08/2017.
// take input RGB & greycale depth images and create triangulated point cloud mesh
//

#include "triangulateMesh.h"
//--------------------------------------------------------------

void triangulateMesh::setup(){
    
};


//--------------------------------------------------------------

void triangulateMesh::makeMesh( ofShortImage &filteredDepthImage, ofImage &filteredColorImage, ofMesh &mesh ){
    
    ofColor c;
    ofShortColor zGrey = 0;
    // int pCount =0;
    //ofMesh mesh;
    
   //int step = volca.getRecordStep();
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
            v3.set((x - (width/2))  ,(y -(height/2)) , z/10.0 ); 

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
    
    // example japanese code to cull non valid points and remove from triangualted mesh...
    // mesh„Å´TriangleËøΩÂä† from http://blog.rettuce.com/mediaart/kinect_of_delaunay/
    //    int W = int(recordWidth/recordStep);
    //    for (int b = 0; b < recordHeight-recordStep; b+=recordStep){
    //        for (int a = 0; a < recordWidth-1; a+=recordStep)
    //        {
    //            if( (indexs[int(b/recordStep)][int(a/recordStep)]!=-1 && indexs[int(b/recordStep)][int(a/recordStep+1)]!=-1) && (indexs[int(b/recordStep+1)][int(a/recordStep+1)]!=-1 && indexs[int(b/recordStep+1)][int(a/recordStep)]!=-1) ){
    //
    //                mesh.addTriangle(indexs[int(b/recordStep)][int(a/recordStep)],indexs[int(b/recordStep)][int(a/recordStep+1)],indexs[int(b/recordStep+1)][int(a/recordStep+1)]);
    //                mesh.addTriangle(indexs[int(b/recordStep)][int(a/recordStep)],indexs[int(b/recordStep+1)][int(a/recordStep+1)],indexs[int(b/recordStep+1)][int(a/recordStep)]);
    //            }
    //        }
    //    }
    
};


//--------------------------------------------------------------

void triangulateMesh::setNormals( ofMesh &mesh ){ //Universal function which sets normals for the triangle mesh
    int nV = mesh.getNumVertices();     //The number of the vertices
    int nT = mesh.getNumIndices() / 3;     //The number of the triangles
    
    vector<ofPoint> norm( nV ); //Array for the normals
    //Scan all the triangles. For each triangle add its normal to norm's vectors of triangle's vertices
    for (int t=0; t<nT; t++) {
        //Get indices of the triangle t
        int i1 = mesh.getIndex( 3 * t );
        int i2 = mesh.getIndex( 3 * t + 1 );
        int i3 = mesh.getIndex( 3 * t + 2 );
        //Get vertices of the triangle
        const ofPoint &v1 = mesh.getVertex( i1 );
        const ofPoint &v2 = mesh.getVertex( i2 );
        const ofPoint &v3 = mesh.getVertex( i3 );
        //Compute the triangle's normal
        ofPoint dir = ( (v2 - v1).crossed( v3 - v1 ) ).normalized();
        norm[ i1 ] += dir;  //Accumulate it to norm array for i1, i2, i3
        norm[ i2 ] += dir;
        norm[ i3 ] += dir;
    }
    for (int i=0; i<nV; i++) {     //Normalize the normal's length
        norm[i].normalize();
    }
    mesh.clearNormals();     //Set the normals to mesh
    mesh.addNormals( norm );
}

//--------------------------------------------------------------

