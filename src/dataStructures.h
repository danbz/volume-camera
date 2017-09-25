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
    bool recording;
    bool playing;
    bool paused;
    bool singleShot;
    int recordFPS;
    int recordWidth, recordHeight, recordStep;
    string recordingDate;
};


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

