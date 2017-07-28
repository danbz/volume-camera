#include "ofApp.h"

// VOLCA: experimental volumetric camera/apparatus v0.1
// © 2017 Daniel Buzzo. Dan@buzzo.com http://www.buzzo.com
// all rights reserved

// kinect.setDepthClipping(float nearClip=500, float farClip=10000); //set depth clipping range

string _timestamp = "default"; //default value for filepath opening on playback
string filePath ="";
int step = 1; // default point cloud step size for mesh file playback
bool paused;
uint64 timeNow =ofGetSystemTime(); // for timing elapsed time since past frame for playbackFPS control

bool doThemeColorsWindow = false;


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
   int  kWidth=kinect.width;
    int kHeight=kinect.height;
    colorCvImage.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    colorImage.allocate(kWidth, kHeight, OF_IMAGE_COLOR);
    depthImage.allocate(kWidth, kHeight, OF_IMAGE_GRAYSCALE);
    
    nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	ofSetFrameRate(120); //make this into a separate variable for playback speed framerate alteration
	angle = 0; // zero the tilt on startup
	kinect.setCameraTiltAngle(angle);
    
    //////////////////////////////////////////////////////
    // Recording / Playing configuration
    //////////////////////////////////////////////////////
    recording = false;
    playing = false;
    paintMesh = true;
    bDrawPointCloud = true;   // start from the camera view
    kinect.setDepthClipping( 100,  20000); //set depth clipping range
    frame = 0; //play back frame initialisation
    paused = false;
    drawTriangles = false;
    renderStyle = 1;
    recordWidth =640; //default width for recording and playback of meshes, overridden by Exifmedta data when recorded files are loaded.
    recordHeight=480;
    // shot length, exposure variables and recording FPS timing
    singleShot = true;
    exposureTime = 0.5; // length of exposure capture in seconds
    recordFPS = 25;
    lastRecordedFrame = 0;
    
    //added in new thread class etc 8/july/17
    ofxKinectMeshRecorder thread;
    
    //////////////////////////////////////////////////////
    // Rendering Configuration
    //////////////////////////////////////////////////////
    //light.enable(); //enable world light
    illuminateScene = false;
    showNormals = false;
    renderFlatQuads = false;
    
    // easyCam setup
    
    //////////////////////////////////////////////////////
    // Gui Configuration
    //////////////////////////////////////////////////////
    showGui = true;
    imGui.setup(); //ofxImGui set up
    ImGui::CaptureMouseFromApp();
    ImGui::GetIO().MouseDrawCursor = false;
    //backgroundColor is stored as an ImVec4 type but can handle ofColor
    imBackgroundColor = ofColor(44, 44, 54);
    show_test_window = false;
    playbackFPS=15;
    blobSize =4;
    //gridSize =1;
    backPlane =25000;
    frontPlane=0;
    recordingStep =4;
    
    if( !kinect.hasAccelControl()) {
        ofSystemAlertDialog("Note: this is a newer Xbox Kinect or Kinect For Windows device, motor / led / accel controls are not currently supported" );
    }
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(imBackgroundColor); // background color
	kinect.update();
    
    if(recording) { // mesh capture
        savePointCloud();
    }
    
    if(playing) { // if we are in playback mode
        if(!paused){ // and have not paused the playback
            if (timeNow < (ofGetSystemTime() - (1000/playbackFPS))) { // check playback FPS
                frameToPlay += 1; // increment the frame we are playing
                timeNow = ofGetSystemTime();
                //cout << timeNow/1000 << " : " << ofGetSystemTime() << endl;
            }
        if(frameToPlay >= meshRecorder.totalFrames) frameToPlay = 0; //or start at the beginning of the recorded loop
        }
    }
    
    if(kinect.isFrameNew()) { 	// if there is a new frame and we are connected to a kinect device Kinect Live Render CV updating
		grayImage.setFromPixels(kinect.getDepthPixels()); // load grayscale depth image from the kinect source
        colorCvImage.setFromPixels(kinect.getPixels()); // load RGB image from the kinect source
        colorImage.setFromPixels(kinect.getPixels());
        depthImage.setFromPixels(kinect.getDepthPixels());
        depthPixels = grayImage.getPixels();
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
	if(bDrawPointCloud) { //show pointcloud view
		easyCam.begin();
        if (illuminateScene) light.enable(); //enable world light
            //if (exposureStart < (ofGetSystemTime() - (1000*exposureTime))) {
            drawAnyPointCloud(); //call new generic point render function
         //   exposureStart = ofGetSystemTime();
        //}
        ofDisableLighting(); //disable world light
        easyCam.end();
	} else { 		// draw from the live kinect as 3 windows
        grayImage=(kinect.getDepthPixels());
		kinect.drawDepth(10, 10, 480, 360);
		kinect.draw(490, 10, 480, 360);
       // grayImage.blurGaussian(11);
        grayImage.dilate();
        grayImage.dilate();
        //grayImage.erode();

		depthImage.draw(10, 370, 480, 360);
		contourFinder.draw(10, 370, 480, 360);
        grayImage.draw(490, 370, 480, 360);

#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
    
    //////////////////////////////////////////////////////
    // Load Recording
    //////////////////////////////////////////////////////
    if(!meshRecorder.readyToPlay) {    //-- recorder  // Loadinf info:
        string l = ofToString(meshRecorder.framesLoaded);
        string t = ofToString(meshRecorder.totalFrames);
        ofDrawBitmapString("loading... " + l + "/" + t,700, 20); // if loading then draw progress to screen
    }
    
    if (showGui) {
        drawGui();
    }
    
    if (recording) {
        string f = to_string(frame);
        ofDrawBitmapString("recording frame: " + f, 700,20);
    }
}

//--------------------------------------------------------------
void ofApp::drawAnyPointCloud() { // modified to read from  loaded ofcvimages rather than direct from kinect  - 28-7-17
    int w = recordWidth;
    int h = recordHeight;
   // unsigned char *exposureBuffer = new unsigned char [recordWidth * recordHeight * 4];
   // unsigned char *exposureBuffer = new unsigned char ;
   // numOfFramesInExposureBuffer = 0;
    ofColor c;
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
            for(int y = 0; y < h; y += step) { //load data from recording into mesh as pointcloud
                for(int x = 0; x < w; x += step) {
                    ofVec3f v2;
                    v2.set(0,0,0);
                    v2 = meshRecorder.getVectorAt(frameToPlay, pCount);
                    mesh.addVertex(v2);
                    c = meshRecorder.getColorAt(frameToPlay, pCount);
                    if(paintMesh) mesh.addColor(c); // add colour from map into mesh at each point
                    pCount ++;
                }
            }
        }
    } else { // draw  pointcloud mesh from live source --------
        //int step = recordingStep;
        int index =0;
        int i=0;
        for(int y = 0; y < h; y += recordingStep) {
            for(int x = 0; x < w; x += recordingStep) {
                if(kinect.getDistanceAt(x, y) > frontPlane & kinect.getDistanceAt(x, y) < backPlane) { // exclude out of range data
                   // int zGrey = depthPixels[i];
                    //ofColor  = depthImage.getColor(x, y);
                     //float z = zGrey;
                    cout << "dp: " << (grayImage.getPixels())[i] << endl;
                    //ofVec3f v3;
                    //v3.set(x,y,z);
                    // mesh.addVertex(v3);
                    mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
                    if (paintMesh) {
                        //c = (kinect.getColorAt(x,y));
                        c = (colorImage.getColor(x,y)); // getting RGB from ofImage rather than direct from kinect.
                        mesh.addColor(c);
                    }
                    
                }
                i++;
            }
        }
    }
 
    int numofVertices = mesh.getNumVertices(); //----- then generate triangles for mesh --
    pCount = 0;
    ofVec3f v2;
    v2.set(0,0,0); //add triangles to mesh from vertices - move this to the 'read mesh' ?
    for(int n = 0; n < numofVertices-1-w/step; n ++) {
        // cout << "points:" << n  <<"," << n+1+w/step << "," << n+w/step <<endl;
        // add in culling for zero location points from triangle mesh
        // optimise to check less of the duplicate points
        //  if(kinect.getDistanceAt(x, y) > frontPlane & kinect.getDistanceAt(x, y) < backPlane)  // use backplane value to cull deeper points from cloud // to be added
        if ((mesh.getVertex(pCount))==v2 or (mesh.getVertex(pCount+1))==v2 or (mesh.getVertex(pCount+1+w/step))==v2){
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
    
    if (showNormals) {//set normals for faces
        setNormals( mesh );
    }
    glPointSize(blobSize);
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    ofPushMatrix();
    ofScale(1, -1, -1);  // the projected points are 'upside down' and 'backwards'
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    
    glDepthRange(0, 2000);//expwriment with gldepth range
    
    if (renderFlatQuads){ // render as flat quads
        glShadeModel(GL_FLAT);
    }else {
        glShadeModel(GL_TRIANGLES);
    }
    
    //mesh.drawVertices();
    //mesh.drawFaces();
    ofSetColor( 255, 255, 255);  //set render colour for unpainted points, faces and lines
    mesh.draw();
    glDisable(GL_DEPTH_TEST);
    //mesh.clear();
    ofPopMatrix();
}

//--------------------------------------------------------------

//Universal function which sets normals for the triangle mesh
void ofApp::setNormals( ofMesh &mesh ){
    int nV = mesh.getNumVertices();     //The number of the vertices
    int nT = mesh.getNumIndices() / 3;     //The number of the triangles

    vector<ofPoint> norm( nV ); //Array for the normals
    //Scan all the triangles. For each triangle add its
    //normal to norm's vectors of triangle's vertices
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

//------------------------------------------------------------
void ofApp::loadRecording() {
    
    if(!meshRecorder.readyToPlay) return;
    if(recording) return;
    if(!playing) {
        ofFileDialogResult result = ofSystemLoadDialog("Choose a folder of recorded data", true, ofToDataPath(""));
        if (result.getPath() != "") {
            filePath =result.getPath();
            playing = true;
            frameToPlay = 0;
            loadExifData(filePath);
            meshRecorder.startLoading(filePath);
        }
    } else {
        playing = false;
    }
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
    int w = recordWidth;
    int h = recordHeight;
    
    if (timeNow < (ofGetSystemTime() - (1000/recordFPS))) {     // add in timing element for recordFPS setting
        FILE* fout = fopen((saveTo + "frame" + ofToString(frame) + ".txt").c_str(), "w");
        int pIndex = 0;
        for(int y = 0; y < h; y += recordingStep) {
            for(int x = 0; x < w; x += recordingStep) {
                ofVec3f v2;
                v2.set(0,0,0);
                float distance;
                distance = kinect.getDistanceAt(x, y);
                
                if(distance> frontPlane && distance < backPlane) {//only record points into v2 if within min & max distance
                    v2 = kinect.getWorldCoordinateAt(x, y);
                }
                ofColor pColor;
                pColor = kinect.getColorAt(x, y);
                
                fprintf(fout, "%i%s%f%s%f%s%f%s%i%s", pIndex, ",", v2.x, ",",  v2.y, ",",  v2.z, ",",  pColor.getHex(), "\n");
                pIndex++;
            }
        }
        fclose(fout);
        frame++; // increment the frame we are recording
        cout << "Recording frame: " << frame << endl;
        timeNow = ofGetSystemTime();
        //cout << timeNow/1000 << " : " << ofGetSystemTime() << endl;
        
    }
    if (singleShot) {
        recording=false; //if in singleShot mode then stop after this frame is recorded
        cout << "Single Shot mode, frame" << frame << endl;
    }
}


//--------------------------------------------------------------

//////////////////////////////////////////////////////
// XML exif data save and load
// to include; ImageDescription ,  , DateTimeOriginal , DateTimeDigitized, ShutterSpeedValue , ApertureValue, FocalLength, MakerNote, RelatedSoundFile, SensingMethod, WhiteBalance, DeviceSettingDescription, etc
//////////////////////////////////////////////////////

void ofApp::saveExifData() { //put some some settings into a file
    
    string path = saveTo;
    string today =  _timestamp = ofToString(ofGetDay()) + //generate date
    ofToString(ofGetMonth()) +
    ofToString(ofGetYear()) +
    ofToString(ofGetHours()) +
    ofToString(ofGetMinutes()) +
    ofToString(ofGetSeconds());
    
    //exifSettings.addTag("exifData");
    exifSettings.setValue("exif:make", "Buzzo");
    exifSettings.setValue("exif:model", "Volca: Experimental volumetric camera/apparatus v0.1");
    exifSettings.setValue("exif:orientation", "top left");
    exifSettings.setValue("exif:ImageWidth", recordWidth/recordingStep);
    exifSettings.setValue("exif:ImageLength", recordHeight/recordingStep);
    exifSettings.setValue("exif:DateTimeDigitized", today);
    exifSettings.setValue("exif:ExposureTime", exposureTime);
    exifSettings.setValue("exifSensingMethod", "Kinect depth sensor");
   
    exifSettings.saveFile(path + "exifSettings.xml"); //puts exifSettings.xml file in the current recordedframe folder
    string myXml;
    exifSettings.copyXmlToString(myXml);
    cout << myXml <<endl ;
}

//-------------------------------------------------------------------

void ofApp::loadExifData(string filePath) { // load exifXML file from the sele ted folder and get the values out
    
    exifSettings.loadFile(filePath + "/exifSettings.xml");
    //cout << filePath << "/exifSettings.xml" << endl;
    recordWidth = exifSettings.getValue("exif:ImageWidth", 0);
    recordHeight = exifSettings.getValue("exif:ImageLength", 0);
    string recordingDate = exifSettings.getValue("exif:DateTimeDigitized", "");
    string myXml;
    exifSettings.copyXmlToString(myXml);
    cout << "loaded exif data: " << myXml <<endl ;
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

void ofApp::drawGui() {
    imGui.begin(); //begin GUI
    
    ImGuiIO& io = ImGui::GetIO(); // hide mouse input from rest of app
    if (io.WantCaptureMouse){ //prevent mousemessages going to app while using imGui
        easyCam.disableMouseInput();
    }else {
        easyCam.enableMouseInput();
    };
    
    { // 1. Show window
        ImGui::Text("Welcome to Volca v0.0");
        //ImGui::SliderFloat("Float", &floatValue, 0.0f, 1.0f);
        //if (ImGui::CollapsingHeader("Capture options")) {
        ImGui::Text("Capture parameters");
        ImGui::Checkbox("Single shot capture", &singleShot);
        ImGui::SliderFloat("Exposure time (s)", &exposureTime, 0.01, 5.0);
        ImGui::SliderInt("Recording FPS", &recordFPS, 1, 60);
        ImGui::SliderInt("RecordingMesh Step",&recordingStep, 1, 10);
        // }
        
        if (ImGui::CollapsingHeader("Depth options")){
            ImGui::SliderInt("Frontplane", &frontPlane, 0, 10000);
            ImGui::SliderInt("Backplane", &backPlane, 100, 20000);
        }
        if (ImGui::CollapsingHeader("RGB options")){
            // ImGui::SliderInt("Frontplane", &frontPlane, 0, 10000);
            // ImGui::SliderInt("Backplane", &backPlane, 100, 15000);
        }
        
        if (ImGui::CollapsingHeader("Render options")) {
            ImGui::Text("Render style");
            ImGui::RadioButton("cloud", &renderStyle, 1); ImGui::SameLine();
            ImGui::RadioButton("faces", &renderStyle, 2); ImGui::SameLine();
            ImGui::RadioButton("mesh", &renderStyle, 3);
            
            ImGui::Text("Surface style");
            ImGui::Checkbox("paint mesh", &paintMesh); ImGui::SameLine();
            ImGui::Checkbox("world light", &illuminateScene); ImGui::SameLine();
            ImGui::Checkbox("normals", &showNormals); ImGui::SameLine();
            ImGui::Checkbox("flatQuads", &renderFlatQuads);
            ImGui::SliderInt("Cloud pointsize", &blobSize, 1, 15);
            //ImGui::SliderInt("Mesh spacing", &gridSize, 1, 20);
            ImGui::ColorEdit3("Background Color", (float*)&imBackgroundColor);
        }
        if (ImGui::CollapsingHeader("Playback options")) {
            ImGui::Text("Playback style");
            ImGui::SliderInt("Playback FPS", &playbackFPS, 1, 120);
        }
        
        if(ImGui::Button("Test Window")) {
            show_test_window = !show_test_window;
        }
        
        ImGui::SameLine();
        
        if (ImGui::Button("reset camera")) {
            easyCam.reset();//reset easycam settings to re-centre 3d view
        }
        ImGui::SameLine();
        
        if (ImGui::Button("load recording")) {
            loadRecording();   ImGui::SameLine();
        }
        ImGui::Checkbox("show live mesh", &bDrawPointCloud);
        
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("Recording mesh size %.1d , %1d", recordWidth/step , recordHeight/step);
    }
    
    if (show_test_window) {     // 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
        ImGui::SetNextWindowPos(ofVec2f(650, 20), ImGuiSetCond_FirstUseEver);
        ImGui::ShowTestWindow(&show_test_window);
    }
    
    if(doThemeColorsWindow) {
        imGui.openThemeColorWindow();
    }
    imGui.end(); //end GUI
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			//bThreshWithOpenCV = !bThreshWithOpenCV;
            //paused=!paused;
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
            loadRecording();
            break;
            
        case 'r':
        case 'R':
            if(!meshRecorder.readyToPlay) return;
            if(recording) return;
            if(playing) return;
            saveTo = generateFileName();
            frame = 0;
            exposureStart = ofGetSystemTime();
            cout << "exposte start" << exposureStart << endl;
            recording = true;
            saveExifData();
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
                    if(frameToPlay < meshRecorder.totalFrames){
                     frameToPlay ++;
                    }
                }
            }
            break;
            
        case 't':
            drawTriangles = !drawTriangles;//swap between point cloud rendering and primitive triangle rendering 
            break;
            
        case 'n':
            showNormals = !showNormals;//swap between normals on mesh on and off
            break;
        
        case 'i':
            if (!illuminateScene) { //swap on and off world light
                light.enable();
                illuminateScene=!illuminateScene;
            } else {
                ofDisableLighting();
                illuminateScene=!illuminateScene;
            }
            break;
            
        case 'h':
            easyCam.reset();//reset easycam settings to re-centre 3d view
            break;
            
        case 'f':
            ofToggleFullscreen();
            break;
            
    }    
}

//-------------------------

void ofApp::keyReleased(int key){
    

    switch (key) {
            
//        case ' ':
//            paused=false;
//            break;
//            
//        case 'r':
//        case 'R':
//            if(!meshRecorder.readyToPlay) return;
//            if(!recording) return;
//            if(playing) return;
//            saveExifData();
//            saveTo = "";
//            recording = false;
//            break;

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
