//
//  triangulateMesh.hpp
//  volume-camera
//
//  Created by Dan Buzzo on 24/08/2017.
//
//

#ifndef triangulateMesh_hpp
#define triangulateMesh_hpp

#include <stdio.h>
#include "ofMain.h"


#endif /* triangulateMesh_hpp */

class triangulateMesh
{
    public :
    
    triangulateMesh()
    {
        
    };
    
    void setup();
    void makeMesh( ofShortImage &filteredDepthImage, ofImage &filteredColorImage, ofMesh &mesh );
    void setNormals( ofMesh &mesh);
    
    ~triangulateMesh()
    {
        
    };
    
};
