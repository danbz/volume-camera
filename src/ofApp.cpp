#include "ofApp.h"



// VOLCA: experimental volumetric camera/apparatus v0.1
// © 2017 Daniel Buzzo. Dan@buzzo.com http://www.buzzo.com
// https://github.com/danbz/volume-camera
// all rights reserved

using namespace ofxCv;
using namespace cv;

string _timestamp = "default"; //default value for filepath opening on playback
string filePath ="";
//bool paused;
bool doThemeColorsWindow = false;
uint64 timeNow =ofGetSystemTime(); // for timing elapsed time since past frame for playbackFPS control


//----------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);
	
    kinectConnected = false;
	kinect.setRegistration(true); // enable depth->video image calibration
	kinect.init(); //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect (model 1 - 1414)
    kinect.setDepthClipping( 100,  20000); //set depth clipping range
		
	if(kinect.isConnected()) { // print the intrinsic IR sensor values
        kinectConnected = true;
        
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
    
	ofSetFrameRate(120); //make this into a separate variable for playback speed framerate alteration
	angle = 0; // zero the tilt on startup
	kinect.setCameraTiltAngle(angle);
    //kinect.setDepthClipping( 100,  20000); //set depth clipping range
    
    //////////////////////////////////////////////////////
    // Recording / Playing configuration
    //////////////////////////////////////////////////////
    
    colorImage.allocate(volca.recordWidth, volca.recordHeight, OF_IMAGE_COLOR);
    depthImage.allocate(volca.recordWidth, volca.recordHeight, OF_IMAGE_GRAYSCALE);
    
    bDrawPointCloud = true;   // start from the camera view
    frame = 0; //play back frame initialisation
   
    volca.paused = false;
    volca.singleShot = true;
    volca.recordFPS = 25;
    volca.recording = false;
    volca.playing = false;
    volca.recordWidth=kinect.width; //from kinect1
    volca.recordHeight=kinect.height;
    volca.recordStep =1;
    // if kinect 2 then
    volca.recordWidth=512;
    volca.recordHeight=424; //default width and height for meshes, overridden by Exifmedta data when recorded files are loaded
    
    //////////////////////////////////////////////////////
    // Rendering Configuration
    //////////////////////////////////////////////////////
    volcaRenderer.paintMesh = true;
    volcaRenderer.illuminateScene = false;
    volcaRenderer.showNormals = false;
    volcaRenderer.renderFlatQuads = false;
    volcaRenderer.showGui = true;
    volcaRenderer.depthFactor=1.0; //multiplier for rendering zdepth
    volcaRenderer.perspectiveFactor = 0.002;
    volcaRenderer.renderStyle = 1;
    volcaRenderer.showAxes = true;

    // easyCam setup
    nearThreshold = 10;
    farThreshold = 50000;
    easyCam.setNearClip(nearThreshold);
    easyCam.setFarClip(farThreshold);

    //////////////////////////////////////////////////////
    // Gui Configuration
    //////////////////////////////////////////////////////
    imGui.setup(); //ofxImGui set up
    ImGui::CaptureMouseFromApp();
    ImGui::GetIO().MouseDrawCursor = false;
    imBackgroundColor = ofColor(44, 44, 54);
    show_test_window = false;
    playbackFPS=15;
    blobSize =4;
    backPlane =25000;
    frontPlane=0;
    
    // CV processing settings
    blur =false;
    blurRadius=10;
    erodeImage=false;
    erodeAmount=2;
    dilateImage=false;
    dilateAmount=2;
    bfilterColorImage = true;
    
    if (!startupSound.load("sounds/pad_confirm.wav", false)){
        ofSystemAlertDialog("Unable to load system sounds");
    }else {
         startupSound.setVolume(0.5f);
        startupSound.play();
        ofSoundUpdate();
    
        shutterSound.load("sounds/beep_short_on.wav", false);
        errorSound.load("sounds/beep_short_off.wav", false);
    };
    
    // add in kinect 2 support
//    ofxKinectV2 tmp;
//    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
//    
//    //allocate for this many devices
//    kinects.resize(deviceList.size());
//    texDepth.resize(kinects.size());
//    texRGB.resize(kinects.size());
//    
//    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
//    for(int d = 0; d < kinects.size(); d++){
//        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
//        kinects[d]->open(deviceList[d].serial);
//        //panel.add(kinects[d]->params);
//        //kinect2 = new ofxKinectV2();
//    }

    kinect2.open(0);
    kinectConnected = true;
    cout << kinect2.params << endl;
    kinect2.minDistance = 1.0;
    kinect2.maxDistance = 100000.0;
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofColor c;
    ofShortColor zGrey = 0;
    ofVec3f v3;
    
    filteredColorImage=colorImage;
    filteredDepthImage=depthImage;
    ofBackground(imBackgroundColor); // background color
    kinect.update();
    
    if(volca.recording) { // mesh capture
        savePointCloud();
    }
    
    if(volca.playing) { // if we are in playback mode
        if(!volca.paused){ // and have not paused the playback
            if (timeNow < (ofGetSystemTime() - (1000/playbackFPS))) { // check playback FPS
                volcaRenderer.frameToPlay += 1; // increment the frame we are playing
                timeNow = ofGetSystemTime();
            }
            if(volcaRenderer.frameToPlay >= volcaRecorder.totalFrames) volcaRenderer.frameToPlay = 0; //or start at the beginning of the recorded loop
        }
        if(volcaRecorder.readyToPlay) {    // we are playing so load data from volcaRecorder object into images
            colorImage.setFromPixels(volcaRecorder.getColorImageAt(volcaRenderer.frameToPlay));
            depthImage.setFromPixels(volcaRecorder.getDepthImageAt(volcaRenderer.frameToPlay));
        }
    } else { // we are running from live depth source
        if(kinect.isFrameNew()) {// if new frame and connected to kinect Live Render CV updating
            colorImage.setFromPixels(kinect.getPixels());
            depthImage.setFromPixels(kinect.getRawDepthPixels());
        }
    }
    
    if (bfilterColorImage) { //process depth or RGB image holders //re-write as pipeline chain rather than discrete operations
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
    
    easyCam.setNearClip(nearThreshold);
    easyCam.setFarClip(farThreshold);
    
    // update kinect2
    kinect2.update();
    if( kinect2.isFrameNew() ){
        depthImage.setFromPixels(kinect2.getDepthPixels()) ;
        colorImage.setFromPixels(kinect2.getRgbPixels());
        cout << "depth w x h:" << depthImage.getWidth() << " " << depthImage.getHeight()<< endl;
        cout << "color w x h:" << colorImage.getWidth() << " " << colorImage.getHeight()<< endl;
        
    }
    
    ofSoundUpdate();
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
    
	if(bDrawPointCloud) {  // Draw Live rendering - show pointcloud view
		easyCam.begin();
        if (volcaRenderer.showAxes)ofDrawAxis(100);
        if (volcaRenderer.illuminateScene) light.enable(); //enable world light
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
        
        // draw kinect 2 to screen
//        for(int d = 0; d < kinects.size(); d++){
//            float dwHD = 1920/2;
//            float dhHD = 1080/2;
//            float shiftY = 100 + ((10 + texDepth[d].getHeight()) * d);
//            texDepth[d].draw(200, shiftY);
//            texRGB[d].draw(210 + texDepth[d].getWidth(), shiftY, dwHD, dhHD);
//        }
        
	}
#ifdef USE_TWO_KINECTS
    kinect2.draw(420, 320, 400, 300);
#endif
    if(!volcaRecorder.readyToPlay) {    //-- recorder  // Loadinf info:
        string l = ofToString(volcaRecorder.framesLoaded);
        string t = ofToString(volcaRecorder.totalFrames);
        ofDrawBitmapString("loading... " + l + "/" + t,700, 20); // if loading then draw progress to screen
    }
    
    if (volca.recording) {
        string f = to_string(frame);
        ofDrawBitmapString("recording frame: " + f, 700,20);
    }
    
    if (volcaRenderer.showGui) {
        drawGui();
    }
}

//--------------------------------------------------------------
void ofApp::drawAnyPointCloud() { // modified to read from loaded ofcvimages rather than direct from kinect  - 28-7-17
    
//    ofColor c;
//    ofShortColor zGrey = 0;
    ofMesh mesh;
   
    indexs.clear();
    
    switch (volcaRenderer.renderStyle) { //set render style
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
    
    volcaMeshMaker.makeMesh(filteredDepthImage, filteredColorImage, mesh, volcaRenderer.depthFactor, volcaRenderer.perspectiveFactor);
    
    if (volcaRenderer.showNormals) {//set normals for faces
        volcaMeshMaker.setNormals( mesh );
    }
    glPointSize(blobSize);
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    ofPushMatrix();
    ofScale(1, -1, -1);  // the projected points are 'upside down' and 'backwards'
    ofTranslate(0, 0, -500); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    //glDepthRange(0, 20000);//experiment with gldepth range
    //gluPerspective(57.0, 1.5, 0.1, 20000.0); // fov,
    if (volcaRenderer.renderFlatQuads){ // render as flat quads
        glShadeModel(GL_FLAT);
    } else {
        glShadeModel(GL_TRIANGLES);
    }
    
    //mesh.drawVertices();
    //mesh.drawFaces();
    ofSetColor( 255, 255, 255);  //set render colour for unpainted points, faces and lines
    mesh.draw();
    glDisable(GL_DEPTH_TEST);
    mesh.clear();
    ofPopMatrix();
}

//------------------------------------------------------------

void ofApp::loadRecording() {
    
    bool loadSuccess = false;
    if(!volcaRecorder.readyToPlay) return;
    if(volca.recording) return;
    if(!volca.playing) {
        ofFileDialogResult result = ofSystemLoadDialog("Choose a folder of recorded Volca PNG data", true, ofToDataPath(""));
        if (result.getPath() != "") {
            filePath =result.getPath();
            volcaRenderer.frameToPlay = 0;
            if (loadExifData(filePath)) {
                loadSuccess = volcaRecorder.loadImageData(filePath, volca.recordWidth, volca.recordHeight);
                
                if (loadSuccess){
                    volca.playing = true;
                    cout << loadSuccess << " playing is true" << endl;
                } else {
                    volca.playing = false;
                    cout << loadSuccess << " playing is false" << endl;
                }
            }
        }
    } else {
        volca.playing = false;
        // generate system dialog to ask if user wants to stop playing and load new files        
        volcaRecorder.clearImageData(); // clear mesh buffer
        errorSound.play();
        ofSystemAlertDialog("You have stopped playing the currently loaded mesh ");
        
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
    
    if (timeNow < (ofGetSystemTime() - (1000/volca.recordFPS))) {     // timing element for recordFPS setting
        timeNow = ofGetSystemTime();
        string frameNum = to_string(frame);
        string path = saveTo;
        cout << " saving frame number " << frameNum << endl;
        colorImage.save(path + "colorData" + frameNum + ".png", OF_IMAGE_QUALITY_BEST); //  save depth and color data as png images
        depthImage.save(path + "depthData" + frameNum + ".png", OF_IMAGE_QUALITY_BEST);
        frame++; // increment the frame we are recording
    }
    
    if (volca.singleShot) {
        volca.recording=false; //if in singleShot mode then stop after this frame is recorded
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
    exifSettings.setValue("exif:model", "Volca: Experimental volumetric camera/apparatus");
    exifSettings.setValue("exif:orientation", "top left");
    exifSettings.setValue("exif:ImageWidth", volca.recordWidth/volca.recordStep);
    exifSettings.setValue("exif:ImageLength", volca.recordHeight/volca.recordStep);
    exifSettings.setValue("exif:DateTimeDigitized", today);
    exifSettings.setValue("exifSensingMethod", "Kinect depth sensor");
    exifSettings.setValue("exifDataProcess", "RGB and Depth Image"); //use to tag whether using old render or new render method.
    exifSettings.saveFile(path + "exifSettings.xml"); //puts exifSettings.xml file in the current recordedframe folder
    string myXml;
    exifSettings.copyXmlToString(myXml);
    cout << myXml <<endl ;
}

//-------------------------------------------------------------------

bool ofApp::loadExifData(string filePath) { // load exifXML file from the sele ted folder and get the values out
    
    if (exifSettings.loadFile(filePath + "/exifSettings.xml")){
        exifModel = exifSettings.getValue("exif:model", "");
        string thisModel ="Volca";
        if (exifModel.find(thisModel) != string::npos){
            //cout << filePath << "/exifSettings.xml" << endl;
            volca.recordWidth = exifSettings.getValue("exif:ImageWidth", 0);
            volca.recordHeight = exifSettings.getValue("exif:ImageLength", 0);
            //dataProcess =exifSettings.getValue("exifDataProcess", 0); //use to tag whether using old render or new render method.
            
            volca.recordStep = 1; // always default to 1:1 step when loading recorded meshes
            string recordingDate = exifSettings.getValue("exif:DateTimeDigitized", "");
            string myXml;
            exifSettings.copyXmlToString(myXml);
            cout << "loaded exif data: " << myXml <<endl ;
            return true;
        } else {
            errorSound.play();
            ofSystemAlertDialog("Correct Volca EXIF metadata not found. Is the exifSettings.xml file corrupt?");
            
        }
    } else {
        errorSound.play();
        ofSystemAlertDialog("No Volca EXIF metadata file found. Is this a Volca recording folder?");
        return false;
    }
    
}

//--------------------------------------------------------------

int ofApp::getRecordStep() {
    return volca.recordStep;
}

//--------------------------------------------------------------


void ofApp::exit() {
    
    volcaRecorder.unlock();
    volcaRecorder.stopThread();
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
        if (ImGui::CollapsingHeader("Capture options")) {
            ImGui::Text("Capture parameters");
            ImGui::Checkbox("Single shot capture", &volca.singleShot);
            ImGui::SliderInt("Mesh play/record spacing",&volca.recordStep, 1, 20);
            ImGui::SliderInt("Recording FPS", &volca.recordFPS, 1, 60);
            //ImGui::Text("Playback style");
            ImGui::SliderInt("Playback FPS", &playbackFPS, 1, 120);
            
            ImGui::SliderInt("Frontplane", &frontPlane, 0, 10000);
            ImGui::SliderInt("Backplane", &backPlane, 100, 20000);
        }
        
        if (ImGui::CollapsingHeader("Render options")) {
            ImGui::SliderFloat("Depth factor", &volcaRenderer.depthFactor, 0.05, 5.0);
            ImGui::SliderFloat("Perspective factor", &volcaRenderer.perspectiveFactor, 0.0001, 0.1);
            
            ImGui::Text("Render style");
            ImGui::RadioButton("cloud", &volcaRenderer.renderStyle, 1); ImGui::SameLine();
            ImGui::RadioButton("faces", &volcaRenderer.renderStyle, 2); ImGui::SameLine();
            ImGui::RadioButton("mesh", &volcaRenderer.renderStyle, 3);
            
            ImGui::Text("Surface style");
            ImGui::Checkbox("paint mesh", &volcaRenderer.paintMesh); ImGui::SameLine();
            ImGui::Checkbox("world light", &volcaRenderer.illuminateScene); ImGui::SameLine();
            ImGui::Checkbox("normals", &volcaRenderer.showNormals); ImGui::SameLine();
            ImGui::Checkbox("flatQuads", &volcaRenderer.renderFlatQuads);
            ImGui::SliderInt("Cloud pointsize", &blobSize, 1, 15);
            ImGui::ColorEdit3("Background Color", (float*)&imBackgroundColor);
            
            ImGui::SliderFloat("Far threshhold", &farThreshold, 0, 100000);
            ImGui::SliderFloat("Near threshhold", &nearThreshold, 0, 10000);

        }
        
        if (ImGui::CollapsingHeader("Image filters")) {
            ImGui::Checkbox("Filter colorImage/depthImage", &bfilterColorImage);
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
        
        if(ImGui::Button("Test Window")) {
            show_test_window = !show_test_window;
        }
        
        if (ImGui::Button("reset camera")) {
            easyCam.reset();//reset easycam settings to re-centre 3d view
        }
        ImGui::SameLine();
        ImGui::Checkbox("show live mesh", &bDrawPointCloud);
        ImGui::SameLine();
        if (ImGui::Button("load recording")) {
            loadRecording();
        }
        
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("Recording mesh size %.1d , %1d", volca.recordWidth/volca.recordStep , volca.recordHeight/volca.recordStep);
        if (volca.playing){
           int totalFrames = volcaRecorder.totalFrames;
            ImGui::Text("Playing frame %.1d of %.2d frames in sequence", volcaRenderer.frameToPlay +1, totalFrames);
        }

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
            volca.paused=!volca.paused;
			break;
			
		case'p':
        case'P':
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
        case 'W':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
        case 'O':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
            if(kinect.isConnected()) {
                kinectConnected=true;
            }
			break;
			
		case 'c':
        case 'C':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			volcaRenderer.renderStyle=1;
			break;
			
		case '2':
			volcaRenderer.renderStyle=2;
			break;
			
		case '3':
			volcaRenderer.renderStyle=3;
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
        case 'G':
            volcaRenderer.showGui=!volcaRenderer.showGui;
            break;
            
        case 'a':
        case 'A':
            volcaRenderer.paintMesh=!volcaRenderer.paintMesh;
            break;
            
        case 'l':
        case 'L':
            loadRecording();
            break;
            
        case 'r':
        case 'R':
            if(!volcaRecorder.readyToPlay) return;
            if(volca.recording) return;
            if(volca.playing) return;
            if (kinectConnected){
                shutterSound.play();
                saveTo = generateFileName();
                frame = 0;
                volca.recording = true;
                saveExifData();
                kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            } else {
                errorSound.play();
                ofSystemAlertDialog("No connected kinect device detected");
            }
            
            break;
            
        case 's':
        case 'S':
            if(!volcaRecorder.readyToPlay) return;
            if(!volca.recording) return;
            if(volca.playing) return;
            saveTo = "";
            volca.recording = false;
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '<':
        case ',':
            if(volca.playing) {
                if(volca.paused){
                    if (volcaRenderer.frameToPlay>1){
                        volcaRenderer.frameToPlay --;
                    }
                }
            }
            break;
            
        case '>':
        case '.':
            if(volca.playing) {
                if(volca.paused){
                    if(volcaRenderer.frameToPlay < volcaRecorder.totalFrames){
                     volcaRenderer.frameToPlay ++;
                    }
                }
            }
            break;
            
        case 'n':
        case 'N':
            volcaRenderer.showNormals = !volcaRenderer.showNormals;//swap between normals on mesh on and off
            break;
        
        case 'i':
        case 'I':
            if (!volcaRenderer.illuminateScene) { //swap on and off world light
                light.enable();
                volcaRenderer.illuminateScene=!volcaRenderer.illuminateScene;
            } else {
                ofDisableLighting();
                volcaRenderer.illuminateScene=!volcaRenderer.illuminateScene;
            }
            break;
            
        case 'h':
        case 'H':
            easyCam.reset();//reset easycam settings to re-centre 3d view
            break;
            
        case 'f':
        case 'F':
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


