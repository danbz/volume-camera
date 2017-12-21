//
//  dataStructures.h
//  volume-camera
//
//  Created by Dan Buzzo on 01/09/2017.
//
//

#pragma once

#ifndef dataStructures_h
#define dataStructures_h


#endif /* dataStructures_h */


struct volca { // central volca object
    bool recording, playing, paused, singleShot;
    int recordFPS, recordWidth, recordHeight, recordStep, frontPlane, backPlane ;
    string recordingDate;
};


struct vRenderer { // rendering data for volca object passed to render object and triangulate object.
    bool paintMesh, paintMeshWhite, showGui, showNormals, illuminateScene, renderFlatQuads, showAxes, setBackWall;
    int frameToPlay, renderStyle, backWallDepth, triLength;
    float depthFactor, perspectiveFactor;
} ;

