#include "ofApp.h"

// If you are struggling to get the device to connect ( especially Windows Users )
// please look at the ReadMe: in addons/ofxKinect/README.md
// kinect.setDepthClipping(float nearClip=500, float farClip=10000); //set depth clipping range

string _timestamp = "default"; //default value for filepath opening on playback
int step = 4; // default point cloud step size for recorded mesh playback
int recordingStep =1; // default point cloud step size for recorded mesh quality
bool paused;


//----------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	kinect.setRegistration(true); // enable depth->video image calibration
	kinect.init(); //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
		
	if(kinect.isConnected()) { // print the intrinsic IR sensor values
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
    //////////////////////////////////////////////////////
    // application / depth sensing configuration
    //////////////////////////////////////////////////////
    colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	ofSetFrameRate(60);
	angle = 0; // zero the tilt on startup
	kinect.setCameraTiltAngle(angle);
    
    //////////////////////////////////////////////////////
    // Recording / Playing configuration
    //////////////////////////////////////////////////////
    recording = false;
    playing = false;
    paintMesh = true;
    bDrawPointCloud = true;   // start from the camera view
    kinect.setDepthClipping( 500,  10000); //set depth clipping range
    frame = 0; //play back frame initialisation
    distanceMinima = 200;
    distanceMaxima = 5000;
    paused = false;
    drawTriangles = false;
    renderStyle = 1;
    
    //////////////////////////////////////////////////////
    // Gui Configuration
    //////////////////////////////////////////////////////
    myFont.load("fonts/profaisal-elitetahreerv1-0/ProfaisalEliteTahreer.ttf",9);
    gui.setup( "Parameters", "settings.xml" );
    gui.add( blobSize.setup( "blobSize", 3, 1, 100 ) );
    gui.add( gridSize.setup( "gridSize", 2, 1, 50 ) );
    gui.add( frontPlane.setup( "frontPlane", 0, 0, 2550 ) );
    gui.add( backPlane.setup( "backPlane", 3000, 0, 15000 ) );
    gui.add(backgroundColor.setup("background color",
                              ofColor::black,
                              ofColor(0,0,0,0),
                              ofColor::white));
    showGui = true;
    
    if( !kinect.hasAccelControl()) {
        ofSystemAlertDialog("Note: this is a newer Xbox Kinect or Kinect For Windows device, motor / led / accel controls are not currently supported" );
    }
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(backgroundColor); // background color
	kinect.update();
    
    //////////////////////////////////////////////////////
    // mesh capture
    //////////////////////////////////////////////////////
    if(recording) {
        savePointCloud();
    }
    
    //////////////////////////////////////////////////////
    // Playback
    //////////////////////////////////////////////////////
    if(playing) { // if we are in playback mode
        if(!paused){ // and have not paused the playback
        frameToPlay += 1; // increment the frame we are playing
        if(frameToPlay >= meshRecorder.TotalFrames) frameToPlay = 0; //or start at the beginning of the recorded loop
        }
    }
    
    //////////////////////////////////////////////////////
    // Kinect Live Render updating
    //////////////////////////////////////////////////////
    if(kinect.isFrameNew()) { 	// if there is a new frame and we are connected to a kinect device
		grayImage.setFromPixels(kinect.getDepthPixels()); // load grayscale depth image from the kinect source
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage; 	// we do two thresholds - one for the far plane and one for the near plane
            grayThreshFar = grayImage;   		// we then do a cvAnd to get the pixels which are a union of the two thresholds
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			ofPixels & pix = grayImage.getPixels(); // or we do it ourselves - show people how they can work with the pixels
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		grayImage.flagImageChanged(); // update the cv images
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	 
    //////////////////////////////////////////////////////
    // Draw Live rendering
    //////////////////////////////////////////////////////
	if(bDrawPointCloud) { //show pointcloud view or 3 camera view
		easyCam.begin();
        drawAnyPointCloud(); //call new generic point render function
        
//        if(!playing) { // if we are not playing then draw live pointcloud
//            drawPointCloud();
//        } else {
//            if(meshRecorder.readyToPlay) {
//                drawRecordedPointCloud(); //draw recorded point cloud
//            }
//        }
        easyCam.end();
	} else { 		// draw from the live kinect as 3 windows
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
    
    //////////////////////////////////////////////////////
    // Do Recording
    //////////////////////////////////////////////////////
    if(!meshRecorder.readyToPlay) {    //-- recorder  // Loadinf info:
        string l = ofToString(meshRecorder.FramesLoaded);
        string t = ofToString(meshRecorder.TotalFrames);
        ofDrawBitmapString("loading... " + l + "/" + t,20, 20);
    }
		
    //////////////////////////////////////////////////////
    // Reporting and help text
    //////////////////////////////////////////////////////
	ofSetColor(255, 255, 255);
	stringstream reportStream; // draw instructions
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "pause with space bar, press: < > to scrub frames " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
    << "a: paint mesh   t: toggle triangles/pointcloud   g: show/hide gui"<< endl
     << "1, 2 or 3: to change rendering styles"<< endl
    << "r: START RECORDING   s: STOP RECORDING" << endl
    << "l: LAST RECORDING / LIVE MODE" << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
        //<< "press 1-5 & 0 to change the led mode" << endl;
    }
    
    if (showGui) { // show or hide the gui and instruction texts
        ofDrawBitmapString(reportStream.str(), 20, 600);
        gui.draw();
    }
}

//--------------------------------------------------------------
void ofApp::drawAnyPointCloud() {
    int w = 640;
    int h = 480;
    int pCount =0;
    ofMesh mesh;
    
    switch (renderStyle) { //set render style
        case 1:
             mesh.setMode(OF_PRIMITIVE_POINTS);
            break;
            
        case 2:
            mesh.setMode(OF_PRIMITIVE_TRIANGLES);
            break;
            
        case 3:
            mesh.setMode(OF_PRIMITIVE_LINE_STRIP);
            break;
    }
    
    if(playing) { // if we are playing then render data from file ---------------
        if(meshRecorder.readyToPlay) {
            pCount = 0;
            //int step = gridSize; //DB this crashes the mesh playback  .....
            for(int y = 0; y < h; y += step) { //load data from recording into mesh as pointcloud
                for(int x = 0; x < w; x += step) {
                    ofVec3f v2;
                    v2.set(0,0,0);
                    v2 = meshRecorder.getVectorAt(frameToPlay, pCount);
                    mesh.addVertex(v2);
                    ofColor c;
                    c = meshRecorder.getColorAt(frameToPlay, pCount);
                    if(paintMesh) mesh.addColor(c); // add colour from map into mesh at each point
                    pCount ++;
                }
            }
        }
    } else {
        //draw  pointcloud mesh from live source --------
        int step = gridSize;
        for(int y = 0; y < h; y += step) {
            for(int x = 0; x < w; x += step) {
                if(kinect.getDistanceAt(x, y) > frontPlane & kinect.getDistanceAt(x, y) < backPlane) {
                    if (paintMesh)mesh.addColor(kinect.getColorAt(x,y));
                    mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
                    // mesh.addTriangle(kinect.getWorldCoordinateAt(x, y) , (kinect.getDistanceAt(x, y)));
                }
            }
        }
    }
 
    //----- then generate triangles for mesh --
    int numofVertices = mesh.getNumVertices();
    pCount = 0;
    ofVec3f v2;
    v2.set(0,0,0); //add triangles to mesh from vertices - move this to the 'read mesh' ?
    for(int n = 0; n < numofVertices-1-w/step; n ++) {
        // cout << "points:" << n  <<"," << n+1+w/step << "," << n+w/step <<endl;
        // add in culling for zero location points from triangle mesh
        // optimise to check less of the dupliacte points
        //  if(kinect.getDistanceAt(x, y) > frontPlane & kinect.getDistanceAt(x, y) < backPlane)  // use backplane value to cull deeper points from cloud // to be added
        if ((mesh.getVertex(pCount))==v2 or (mesh.getVertex(pCount+1))==v2 or (mesh.getVertex(pCount+1+w/step))==v2){
            //cout << "culled point" << pCount << endl ;
        }else{
            mesh.addTriangle(n, n+1, n+1+w/step); //even triangles for each mesh square
        }
        if ((mesh.getVertex(pCount))==v2 or (mesh.getVertex(pCount+1+w/step))==v2 or (mesh.getVertex(pCount+w/step))==v2){
            //cout << "culled point" << pCount << endl ;
        }else{
            mesh.addTriangle(n, n+1+w/step, n+w/step); //odd triangles for each mesh square
        }
         pCount ++;
   } //------ end add triangles
    
    glPointSize(blobSize);
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    ofPushMatrix();
    ofScale(1, -1, -1);  // the projected points are 'upside down' and 'backwards'
    ofTranslate(0, 0, -750); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    //mesh.drawVertices();
    //mesh.drawFaces();
    ofSetColor( 0, 128, 0 );  //set render colour for unpainted points, faces and lines
    mesh.draw();
    glDisable(GL_DEPTH_TEST);
    //mesh.clear();
    ofPopMatrix();
}

//-----------------------------------------
void ofApp::drawPointCloud() { //deprecated function - now run by drawAnyPointCloud
	int w = 640;
	int h = 480;
	ofMesh mesh;
    //OF_PRIMITIVE_TRIANGLES, OF_PRIMITIVE_TRIANGLE_STRIP, OF_PRIMITIVE_TRIANGLE_FAN, OF_PRIMITIVE_LINES, OF_PRIMITIVE_LINE_STRIP, OF_PRIMITIVE_LINE_LOOP, OF_PRIMITIVE_POINTS
    
    switch (renderStyle) { //set render style
        case 1:
            mesh.setMode(OF_PRIMITIVE_POINTS);
            break;
            
        case 2:
            mesh.setMode(OF_PRIMITIVE_TRIANGLES);
            break;
            
        case 3:
            mesh.setMode(OF_PRIMITIVE_LINES);
            break;
    }

	int step = gridSize;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > frontPlane & kinect.getDistanceAt(x, y) < backPlane) {
				if (paintMesh)mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
               // mesh.addTriangle(kinect.getWorldCoordinateAt(x, y) , (kinect.getDistanceAt(x, y)));
            }
		}
	}
	glPointSize(blobSize);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
    mesh.draw();
	//mesh.drawFaces(); //alternative draw modes //mesh.drawVertices(); //mesh.drawWireframe();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
string ofApp::generateFileName() {
    string _root = "";
    _timestamp = ofToString(ofGetDay()) +
    ofToString(ofGetMonth()) +
    ofToString(ofGetYear()) +
    ofToString(ofGetHours()) +
    ofToString(ofGetMinutes()) +
    ofToString(ofGetSeconds());
    
    string _filename = (_root + _timestamp + "/");
    
    dirHelper.createDirectory(_filename);
    ofFile file(ofToDataPath(_filename));
    //cout << file.getAbsolutePath();
    return file.getAbsolutePath() + "/";
}

//--------------------------------------------------------------
void ofApp::savePointCloud() {
    int w = 640;
    int h = 480;
    //char buff[200];
    //stringstream stringToSave;
    FILE* fout = fopen((saveTo + "frame" + ofToString(frame) + ".txt").c_str(), "w");
    int pIndex = 0;
    for(int y = 0; y < h; y += recordingStep) {
        for(int x = 0; x < w; x += recordingStep) {
            ofVec3f v2;
            v2.set(0,0,0);
            float distance;
            distance = kinect.getDistanceAt(x, y);
            
            if(distance> distanceMinima && distance < distanceMaxima) {
                v2 = kinect.getWorldCoordinateAt(x, y);
            }
            ofColor pColor;
            pColor = kinect.getColorAt(x, y);
            /*
             stringToSave << pIndex << ","
             << v2.x << ","
             << v2.y << ","
             << v2.z << ","
             << pColor.getHex()
             << "\n";
             */
            //sprintf(buff, "%i%s%f%s%f%s%f%s%i%s", pIndex, ",", v2.x, ",",  v2.y, ",",  v2.z, ",",  pColor.getHex(), "\n");
            fprintf(fout, "%i%s%f%s%f%s%f%s%i%s", pIndex, ",", v2.x, ",",  v2.y, ",",  v2.z, ",",  pColor.getHex(), "\n");
            //stringToSave << buff;
            pIndex++;
        }
    }
    fclose(fout);
    /*
     ofstream framesFile ((saveTo + "frame" + ofToString(frame) + ".txt").c_str());
     framesFile << stringToSave.str();
     //framesFile << buff;
     framesFile.close();
     */
    frame++;
}

//--------------------------------------------------------------
void ofApp::exit() {
    
    meshRecorder.unlock();
    //  meshRecorder.stopThread(false); //DB - depracated call
    meshRecorder.stopThread();
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			//bThreshWithOpenCV = !bThreshWithOpenCV;
            paused=!paused;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
//		case '>':
//		case '.':
//			farThreshold ++;
//			if (farThreshold > 255) farThreshold = 255;
//			break;
			
//		case '<':
//		case ',':
//			farThreshold --;
//			if (farThreshold < 0) farThreshold = 0;
//			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			renderStyle=1;
			break;
			
		case '2':
			renderStyle=2;
			break;
			
		case '3':
			renderStyle=3;
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
            
            case 'g':
            showGui=!showGui;
            break;
            
        case 'a':
            paintMesh=!paintMesh;
            break;
            
        case 'l':
            if(!meshRecorder.readyToPlay) return;
            // Todo: Cuando sepamos detener el Thread que pueda hacerse en cualquier
            // momento durante la carga.
            if(recording) return;
            if(!playing) {
                
                ofFileDialogResult result = ofSystemLoadDialog("Choose a folder of recorded data", true, ofToDataPath(""));
                if (result.getPath() != "") {
                    // filePath =(result.getPath());
                }
                playing = true;
                frameToPlay = 0;
                meshRecorder.startLoading(result.getPath());
            } else {
                playing = false;
            }
            break;
            
        case 'r':
            if(!meshRecorder.readyToPlay) return;
            if(recording) return;
            if(playing) return;
            saveTo = generateFileName();
            frame = 0;
            recording = true;
            break;
            
        case 's':
            if(!meshRecorder.readyToPlay) return;
            if(!recording) return;
            if(playing) return;
            saveTo = "";
            recording = false;
            break;
            
            case '<':
            case ',':
            if(playing) {
                if(paused){
                    if (frameToPlay>1){
                        frameToPlay --;
                    }
                }
            }
            break;
            
        case '>':
        case '.':
            if(playing) {
                if(paused){
                    if(frameToPlay < meshRecorder.TotalFrames){
                     frameToPlay ++;
                    }
                }
            }
            break;
            
        case 't':
            drawTriangles = !drawTriangles;//swap between point cloud rendering and primitive triangle rendering 
            break;
    }
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}
