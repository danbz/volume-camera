//
//  dataStructures.h
//  volume-camera
//
//  Created by Dan Buzzo on 01/09/2017.
//
//

#ifndef dataStructures_h
#define dataStructures_h


#endif /* dataStructures_h */

struct vRenderer { // rendering data for volca object
    bool showGui;
    bool paintMesh;
    int frameToPlay;
    int renderStyle;
    bool showNormals;
    bool illuminateScene;
    bool renderFlatQuads;
    bool showAxes;
    float depthFactor;
    float perspectiveFactor;
} ;
