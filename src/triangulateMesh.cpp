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

void triangulateMesh::makeMesh( ofShortImage &filteredDepthImage, ofImage &filteredColorImage, ofMesh &mesh, volca volca,
                               vRenderer &volcaRenderer ){
    ofColor c;
    ofShortColor zGrey = 0;
    
    int step =volca.recordStep;
    int width = filteredDepthImage.getWidth();
    int height = filteredDepthImage.getHeight();
    
    int index =0;
    int z = 0, minBrightness =0, minBrightnessX=0, minBrightnessY = 0;
    ofVec3f v3, v3b;
    
    for (int y=0; y<height; y+= step) { // find farthest pixel in depth map
        for(int x=0; x<width; x+= step) {
            zGrey = filteredDepthImage.getPixels()[x+y*width];
            z = zGrey.r;
            
            if (z)
                if (z > minBrightness){
                    minBrightness = z;
                    minBrightnessX = x;
                    minBrightnessY = y;
                }
        }
    }
    
    for(int y = 0; y < height; y += step) { // create point cloud
        
        // vector tempindexs;
        // indexs.push_back(tempindexs);
        
        for(int x = 0; x < width; x +=  step) {
            zGrey = filteredDepthImage.getPixels()[x+y*width];
            z = zGrey.r;
            
            v3.set(0,0,0);
            // if (volcaRenderer.setBackWall) { // this crashes as it makes holes in array that mesh creator falls over on.. need to implement japanese iterating vertices routine to render around holes...
                if (z==0 && minBrightness>volcaRenderer.backWallDepth){
                    
                z = minBrightness; //find and set to furthest (darkest pixel) data
                } else {
                 if (z==0)  z= volcaRenderer.backWallDepth; //or use override back wall depth from GUI;
                }
            //} // end set backwall 
            if(z > volca.frontPlane & z < volca.backPlane) {
                //  indexs[y/recordStep].push_back(ind);
                //     ind++;
                if (volcaRenderer.paintMesh) {
                    c = (filteredColorImage.getColor(x,y)); // getting RGB from ofShortImage
                } else {
                    if (volcaRenderer.paintMeshWhite) {
                        c.set(255, 255, 255);
                    } else {
                        //float h = ofMap(z, 0, 65535, 0, 255, true);
                        float h = ofMap(z, 0, minBrightness, 0, 255, true);
                        c.setHsb(h, 255, 255);
                    }
                }
                //} else {
                //     z= backPlane;
                //    c.setHsb(0, 0, 0);
                //    indexs[y/recordStep].push_back(-1);
                //} // clip out pixels
                
                v3.set((x - (width/2)) * (volcaRenderer.perspectiveFactor * z) ,(y -(height/2)) * (volcaRenderer.perspectiveFactor * z) , z * volcaRenderer.depthFactor );
                mesh.addVertex(v3);
                mesh.addColor(c);
            }
            
        }
    }
    
    int meshW = width/step ;
    int meshH = height/step;
    index =0;
    
    for (int y = 0; y<height-step-1; y+= step){ // triangulate mesh
        for (int x=0; x<width-step-1; x+= step){
            v3.set(0,0,0);
//              if ((mesh.getVertex(x+y*meshW))==v3 or (mesh.getVertex((x+1)+y*(meshW)))==v3 or (mesh.getVertex(x+(y+1)*meshW)==v3)){
//              } else {
            v3 = mesh.getVertex(index);
            v3b = mesh.getVertex(index+1);
            index ++;
           // if (abs (v3.z-v3b.z)>0 && abs(v3.z-v3b.z) < volcaRenderer.triLength){
                //cout <<v3.z - v3b.z << endl;
            mesh.addIndex(x+y*meshW);               // 0
            mesh.addIndex((x+1)+y*meshW);           // 1
            mesh.addIndex(x+(y+1)*meshW);           // 10
           // }
            mesh.addIndex((x+1)+y*meshW);           // 1
            mesh.addIndex((x+1)+(y+1)*meshW);       // 11
            mesh.addIndex(x+(y+1)*meshW);           // 10
           // }
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
    
    
//    //////////   new version
//    
//    vector < ofVec3f > points;
//    vector < ofColor > colors;
//    vector < int > indexs;
//    vector < int > tempindexs;
//    
//    //// update ////
//    
//    //clear
//    mesh.clear();
//    points.clear();
//    colors.clear();
//    indexs.clear();
//    
//    // 3点情報と色情報取得
//   // int step = int(5 + int(scaledVol*15));
//    int total = 0;
//    for (int j = 0; j < height; j+=step)
//    {
//        ofVec3f  temppoints;
//        ofColor tempcolors;
//        points.push_back(temppoints);
//        colors.push_back(tempcolors);
//        
//        for (int i = 0; i < width; i+=step)
//        {
//            float distance = kinect.getDistanceAt(i, j);
//            
//            if(distance>50 && distance<1000)
//            {
//                ofVec3f tempPoint;
//                ofColor tempColor;
//                
//                tempPoint = ofVec3f(i, j, distance*-2.0 );
//               // tempColor = ofColor(kinect.getColorAt(i,j));
//                tempColor = filteredColorImage.getColor(i,j);
//                
//               ind points[j/step].push_back(tempPoint);
//                colors[j/step].push_back(tempColor);
//                
//                total++;
//            }else{
//                ofVec3f tempPoint2;
//                ofColor tempColor2;
//                tempPoint2 = ofVec3f(i,j,0);	//範囲外には深度0
//                tempColor2 = ofColor(0);
//                points[j/step].push_back(tempPoint2);
//                colors[j/step].push_back(tempColor2);
//            }
//        }
//    }
//    
//    // 深度情報をindexを付与
//    int ind = 0;
//    for (int m = 0; m < kinect.height; m+=step)
//    {
//        vector tempindexs;
//        indexs.push_back(tempindexs);
//        
//        for (int n = 0; n < kinect.width; n+=step)
//        {
//            if(points[m/step][n/step].z != 0){
//                //          cout << points[m][n] << endl;
//                mesh.addColor(colors[m/step][n/step]);
//                mesh.addVertex(points[m/step][n/step]);
//                
//                indexs[m/step].push_back(ind);
//                ind++;
//            }else{
//                indexs[m/step].push_back(-1);
//            }
//        }
//    }
//    
//    
//    
//    // meshにTriangle追加
//    int W = int(kinect.width/step);
//    for (int b = 0; b < kinect.height-step; b+=step){
//        for (int a = 0; a < kinect.width-1; a+=step)
//        {
//            if( (indexs[int(b/step)][int(a/step)]!=-1 && indexs[int(b/step)][int(a/step+1)]!=-1) && (indexs[int(b/step+1)][int(a/step+1)]!=-1 && indexs[int(b/step+1)][int(a/step)]!=-1) ){
//                
//                mesh.addTriangle(indexs[int(b/step)][int(a/step)],indexs[int(b/step)][int(a/step+1)],indexs[int(b/step+1)][int(a/step+1)]);
//                mesh.addTriangle(indexs[int(b/step)][int(a/step)],indexs[int(b/step+1)][int(a/step+1)],indexs[int(b/step+1)][int(a/step)]);
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

