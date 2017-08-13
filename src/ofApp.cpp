#include "ofApp.h"

// VOLCA: experimental volumetric camera/apparatus v0.1
// © 2017 Daniel Buzzo. Dan@buzzo.com http://www.buzzo.com
// all rights reserved

using namespace ofxCv;
using namespace cv;

string _timestamp = "default"; //default value for filepath opening on playback
string filePath ="";
bool paused;
bool doThemeColorsWindow = false;
uint64 timeNow =ofGetSystemTime(); // for timing elapsed time since past frame for playbackFPS control


//----------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	kinect.setRegistration(true); // enable depth->video image calibration
	kinect.init(); //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect (model 1 - 1414)
    kinect.setDepthClipping( 100,  20000); //set depth clipping range
		
	if(kinect.isConnected()) { // print the intrinsic IR sensor values
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
        kinect.setLed(ofxKinect::LED_BLINK_GREEN);
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
    //kinect.setDepthClipping( 100,  20000); //set depth clipping range
    frame = 0; //play back frame initialisation
    paused = false;
    drawTriangles = false;
    renderStyle = 1;
    recordWidth =640; //default width for recording and playback of meshes, overridden by Exifmedta data when recorded files are loaded.
    recordHeight=480;
    singleShot = true;     // shot length, exposure variables and recording FPS timing
    exposureTime = 0.5; // length of exposure capture in seconds
    recordFPS = 25;
    lastRecordedFrame = 0;
    
    //added in new thread class etc 8/july/17
    //ofxKinectMeshRecorder thread;
    
    //////////////////////////////////////////////////////
    // Rendering Configuration
    //////////////////////////////////////////////////////
    illuminateScene = false;
    showNormals = false;
    renderFlatQuads = false;
    depthFactor=1.0; //multiplier for rendering zdepth
    perspectiveFactor = 0.002;
    // easyCam setup
    
    //////////////////////////////////////////////////////
    // Gui Configuration
    //////////////////////////////////////////////////////
    showGui = true;
    imGui.setup(); //ofxImGui set up
    ImGui::CaptureMouseFromApp();
    ImGui::GetIO().MouseDrawCursor = false;
    imBackgroundColor = ofColor(44, 44, 54);
    show_test_window = false;
    playbackFPS=15;
    blobSize =4;
    backPlane =25000;
    frontPlane=0;
    recordingStep =1;
    blur =false;
    blurRadius=10;
    erodeImage=false;
    erodeAmount=2;
    dilateImage=false;
    dilateAmount=2;
    bfilterColorImage = true;
    
//    if( !kinect.hasAccelControl()) {
//        ofSystemAlertDialog("Note: this is a newer Xbox Kinect or Kinect For Windows device, motor / led / accel controls are not currently supported" );
//    }
    
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofColor c;
    ofShortColor zGrey = 0;
    //int pCount =0;
    ofVec3f v3;
    
    filteredColorImage=colorImage;
    filteredDepthImage=depthImage;
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
            }
            if(frameToPlay >= meshRecorder.totalFrames) frameToPlay = 0; //or start at the beginning of the recorded loop
        }
         // if we are playing then load data from meshrecorder object into images
            if(meshRecorder.readyToPlay) {
                //colorImage = meshRecorder.getColorImageAt(frameToPlay);
                colorImage.setFromPixels(meshRecorder.getColorImageAt(frameToPlay)); //use set from pixels to update after loading in threaded function
                depthImage.setFromPixels(meshRecorder.getDepthImageAt(frameToPlay));
        }
    } else {
        if(kinect.isFrameNew()) {// if new frame and connected to kinect Live Render CV updating
            colorImage.setFromPixels(kinect.getPixels());
            depthImage.setFromPixels(kinect.getRawDepthPixels());
        }
    }
   
    if (bfilterColorImage) { //process depth or RGB image holders //re write as pipeline rather than discrete operations
        if (blur){
            ofxCv::GaussianBlur(filteredColorImage, blurRadius);
        }
        if (erodeImage) {
            ofxCv::erode(colorImage, filteredColorImage, erodeAmount);
        }
        
        if (dilateImage) {
            ofxCv::dilate(colorImage, filteredColorImage, dilateAmount);
        }
    } else {
        if (blur){
            ofxCv::GaussianBlur(filteredDepthImage, blurRadius);
        }
        if (erodeImage) {
            ofxCv::erode(depthImage, filteredDepthImage, erodeAmount);
        }
        
        if (dilateImage) {
            ofxCv::dilate(depthImage, filteredDepthImage, dilateAmount);
        }
    }

    depthImage.update();
    colorImage.update();
    filteredDepthImage.update();
    filteredColorImage.update();
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
    // Draw Live rendering
	if(bDrawPointCloud) { //show pointcloud view
		easyCam.begin();
        if (illuminateScene) light.enable(); //enable world light
        drawAnyPointCloud(); //call new generic point render function
        ofDisableLighting(); //disable world light
        easyCam.end();
	} else { // draw from the live kinect and image arrays
		depthImage.draw(10, 10, 480, 360);
        colorImage.draw(490, 10, 480, 360);
        filteredColorImage.draw(10, 370, 480, 360);
        ofxCv::invert(filteredDepthImage,filteredDepthImage);
        filteredDepthImage.update();
        filteredDepthImage.draw(490, 370, 480, 360);
	}
#ifdef USE_TWO_KINECTS
    kinect2.draw(420, 320, 400, 300);
#endif
    if(!meshRecorder.readyToPlay) {    //-- recorder  // Loadinf info:
        string l = ofToString(meshRecorder.framesLoaded);
        string t = ofToString(meshRecorder.totalFrames);
        ofDrawBitmapString("loading... " + l + "/" + t,700, 20); // if loading then draw progress to screen
    }
    
    if (recording) {
        string f = to_string(frame);
        ofDrawBitmapString("recording frame: " + f, 700,20);
    }
    
    if (showGui) {
        drawGui();
    }
}

//--------------------------------------------------------------
void ofApp::drawAnyPointCloud() { // modified to read from loaded ofcvimages rather than direct from kinect  - 28-7-17
    
    ofColor c;
    ofShortColor zGrey = 0;
   // int pCount =0;
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

    int index =0;
    //int i=0;
    int z = 0;
    ofVec3f v3;
    for(int y = 0; y < recordHeight; y += recordingStep) {
        for(int x = 0; x < recordWidth; x += recordingStep) {
            zGrey = filteredDepthImage.getPixels()[x+y*recordWidth];
            z = zGrey.r;
            if(z > frontPlane & z < backPlane) { // clip out pixels
                v3.set(0,0,0);
                v3.set((x - (recordWidth/2)) * (perspectiveFactor * z) ,(y -(recordHeight/2)) * (perspectiveFactor *z) , z * depthFactor);
                mesh.addVertex(v3);
                if (paintMesh) {
                    c = (filteredColorImage.getColor(x,y)); // getting RGB from ofShortImage
                    mesh.addColor(c);
                }
            }
        }
    }
    triangulateMesh(mesh);
    
    if (showNormals) {//set normals for faces
        setNormals( mesh );
    }
    glPointSize(blobSize);
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    ofPushMatrix();
    ofScale(1, -1, -1);  // the projected points are 'upside down' and 'backwards'
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    //glDepthRange(0, 20000);//experiment with gldepth range
    //gluPerspective(57.0, 1.5, 0.1, 20000.0); // fov,
    if (renderFlatQuads){ // render as flat quads
        glShadeModel(GL_FLAT);
    } else {
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

void ofApp::triangulateMesh(ofMesh &mesh){
    
    int pCount =0;
    int numofVertices = mesh.getNumVertices(); //----- then generate triangles for mesh -- improve to clip edge triangles that wrap round....
    pCount = 0;
    ofVec3f v2;
    
    for(int n = 0; n < numofVertices-1-recordWidth/recordingStep; n ++) { // add in culling for zero location points from  mesh & optimise for less of duplicate points
         v2.set(0,0,0);
        if ((mesh.getVertex(pCount))!=v2 and (mesh.getVertex(pCount+1))!=v2 and (mesh.getVertex(pCount+1+recordWidth/recordingStep))!=v2){
            mesh.addTriangle(n, n+1, n+1+recordWidth/recordingStep); //even triangles for each mesh square
        }
        
        if ((mesh.getVertex(pCount))!=v2 and (mesh.getVertex(pCount+1+recordWidth/recordingStep))!=v2 and (mesh.getVertex(pCount+recordWidth/recordingStep))!=v2){
            mesh.addTriangle(n, n+1+recordWidth/recordingStep, n+recordWidth/recordingStep); //odd triangles for each mesh square
        }
        pCount ++;
    }
}

//--------------------------------------------------------------

void ofApp::setNormals( ofMesh &mesh ){ //Universal function which sets normals for the triangle mesh
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

//------------------------------------------------------------

void ofApp::loadRecording() {
    
    if(!meshRecorder.readyToPlay) return;
    if(recording) return;
    if(!playing) {
        ofFileDialogResult result = ofSystemLoadDialog("Choose a folder of recorded PNG data", true, ofToDataPath(""));
        if (result.getPath() != "") {
            filePath =result.getPath();
            playing = true;
            frameToPlay = 0;
            loadExifData(filePath);
            meshRecorder.startLoading(filePath, recordWidth, recordHeight);
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
    
    if (timeNow < (ofGetSystemTime() - (1000/recordFPS))) {     // timing element for recordFPS setting
        timeNow = ofGetSystemTime();
        string frameNum = to_string(frame);
        string path = saveTo;
        cout << " saving frame number " << frameNum << endl;
        colorImage.save(path + "colorData" + frameNum + ".png", OF_IMAGE_QUALITY_BEST); //  save depth and color data as png images
        depthImage.save(path + "depthData" + frameNum + ".png", OF_IMAGE_QUALITY_BEST);
        frame++; // increment the frame we are recording
    }
    
    if (singleShot) {
        recording=false; //if in singleShot mode then stop after this frame is recorded
        cout << "Single Shot mode, frame " << frame << endl;
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
   // exifSettings.setValue("exif:ExposureTime", exposureTime);
    exifSettings.setValue("exifSensingMethod", "Kinect depth sensor");
    exifSettings.setValue("exifDataProcess", "RGB and Depth Image"); //use to tag whether using old render or new render method.
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
    //dataProcess =exifSettings.getValue("exifDataProcess", 0); //use to tag whether using old render or new render method.
    
    recordingStep = 1; // always default to 1:1 step when loading recorded meshes
    string recordingDate = exifSettings.getValue("exif:DateTimeDigitized", "");
    string myXml;
    exifSettings.copyXmlToString(myXml);
    cout << "loaded exif data: " << myXml <<endl ;
}

//--------------------------------------------------------------

void ofApp::exit() {
    
    meshRecorder.unlock();
    //  meshRecorder.stopThread(false); //DB - deprecated call
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
    } else {
        easyCam.enableMouseInput();
    };
    
    { // 1. Show window
        ImGui::Text("Welcome to Volca v0.0");
        //ImGui::SliderFloat("Float", &floatValue, 0.0f, 1.0f);
        if (ImGui::CollapsingHeader("Capture options")) {
        ImGui::Text("Capture parameters");
        ImGui::Checkbox("Single shot capture", &singleShot);
       // ImGui::SliderFloat("Exposure time (s)", &exposureTime, 0.01, 5.0);
        ImGui::SliderInt("Recording FPS", &recordFPS, 1, 60);
        ImGui::SliderInt("RecordingMesh Step",&recordingStep, 1, 10);
         }
        
        if (ImGui::CollapsingHeader("Depth options")){
            ImGui::SliderInt("Frontplane", &frontPlane, 0, 10000);
            ImGui::SliderInt("Backplane", &backPlane, 100, 20000);
        }
        if (ImGui::CollapsingHeader("RGB options")){
            // ImGui::SliderInt("Frontplane", &frontPlane, 0, 10000);
            // ImGui::SliderInt("Backplane", &backPlane, 100, 15000);
        }
        
        if (ImGui::CollapsingHeader("Render options")) {
            ImGui::SliderFloat("Depth factor", &depthFactor, 0.05, 5.0);
            ImGui::SliderFloat("Perspective factor", &perspectiveFactor, 0.0001, 0.1);
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
        
        if (ImGui::CollapsingHeader("Image filters")) {
            
            ImGui::Text("Playback style");
            ImGui::Checkbox("Filter Color Image / depth image", &bfilterColorImage);
 
            ImGui::Checkbox("Blur", &blur);
            ImGui::SameLine();
            ImGui::SliderInt("Radius ", &blurRadius, 1, 200);
            ImGui::Checkbox("Erode", &erodeImage);
            ImGui::SameLine();
            ImGui::SliderInt("Amount ", &erodeAmount, 1, 50);
            
            ImGui::Checkbox("Dilate", &dilateImage);
            ImGui::SameLine();
            ImGui::SliderInt("Amount ", &dilateAmount, 1, 50);
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
        ImGui::Text("Recording mesh size %.1d , %1d", recordWidth/recordingStep , recordHeight/recordingStep);
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
            paused=!paused;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '+':
		case '=':
			backPlane +=10;
			if (backPlane > 20000) backPlane = 20000;
			break;
			
		case '-':
			backPlane -=10;
			if (backPlane < frontPlane) backPlane = frontPlane;
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
            recording = true;
            saveExifData();
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case 's':
            if(!meshRecorder.readyToPlay) return;
            if(!recording) return;
            if(playing) return;
            saveTo = "";
            recording = false;
            kinect.setLed(ofxKinect::LED_GREEN);
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

void ofApp::keyReleased(int key)
{

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
