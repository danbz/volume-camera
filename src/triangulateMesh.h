//
//  triangulateMesh.h
//  volume-camera
//
//  Created by Dan Buzzo on 24/08/2017.
//
//

#ifndef triangulateMesh_h
#define triangulateMesh_h

#include <stdio.h>
#include "ofMain.h"

#endif /* triangulateMesh_h */

class triangulateMesh
{
    public :
    
    triangulateMesh()
    {
        
    };
    
    void setup();
    void makeMesh( ofShortImage &filteredDepthImage, ofImage &filteredColorImage, ofMesh &mesh, float &depthFactor, float &perspectiveFactor);
    void setNormals( ofMesh &mesh);
    
    ~triangulateMesh()
    {
        
    };
    
};
