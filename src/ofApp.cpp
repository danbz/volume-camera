#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/


  //kinect.setDepthClipping(float nearClip=500, float farClip=10000); //set depth clipping range


//--------------------------------------------------------------

//-- recorder hack

string _timestamp = "default";
int step = 4;

//--


void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the camera
	bDrawPointCloud = true;
    
    //-- DB
    gui.setup( "Parameters", "settings.xml" );
    
    gui.add( blobSize.setup( "blobSize", 3, 1, 100 ) );
    gui.add( gridSize.setup( "gridSize", 2, 1, 50 ) );
    gui.add( frontPlane.setup( "frontPlane", 0, 0, 2550 ) );
     gui.add( backPlane.setup( "backPlane", 3000, 0, 15000 ) );
    
    showGui = true;
    paintMesh = true;
    
    //-- recorder hack
    
    //////////////////////////////////////////////////////
    // Recording / Playing:
    //////////////////////////////////////////////////////
    
    recording = false;
    playing = false;
    colorMode = true;
    
    frame = 0;
    distanciaMinima = 200;
    distanciaMaxima = 5000;
    
    //////////////////////////////////////////////////////
    // Configuracion de la APP/GUI
    //////////////////////////////////////////////////////
    
    myFont.load("fonts/profaisal-elitetahreerv1-0/ProfaisalEliteTahreer.ttf",9);
 
    
    
    
    kinect.setDepthClipping( 500,  10000); //set depth clipping range
    
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	kinect.update();
    
    //////////////////////////////////////////////////////
    // Grabando
    //////////////////////////////////////////////////////
    // Guardamos la info del Mesh:
    
    if(recording) {
        savePointCloud();
    }
    
    //////////////////////////////////////////////////////
    // Reproduciendo
    //////////////////////////////////////////////////////
    
    if(playing) {
        frameToPlay += 1;
        if(frameToPlay >= meshRecorder.TotalFrames) frameToPlay = 0;
    }
    
    //--
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
    
    //-- recorder hack
    
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		//drawPointCloud();
        
        if(!playing) {
            drawPointCloud();
        } else {
            if(meshRecorder.readyToPlay) {
                drawRecordedPointCloud();
            }
        }
        easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
    
    
    
    //-- recorder  // Loadinf info:
    
    if(!meshRecorder.readyToPlay) {
        string l = ofToString(meshRecorder.FramesLoaded);
        string t = ofToString(meshRecorder.TotalFrames);
        myFont.drawString("loading... " + l + "/" + t,10, 612);
    }
    //--
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
    << "r: START RECORDING" << endl
    << "s: STOP RECORDING" << endl
    << "l: LAST RECORDING / LIVE MODE" << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    if (showGui) gui.draw();
}

//--------------------------------------------------------------
void ofApp::savePointCloud() {
    int w = 640;
    int h = 480;
    //char buff[200];
    //stringstream stringToSave;
    FILE* fout = fopen((saveTo + "frame" + ofToString(frame) + ".txt").c_str(), "w");
    int pIndex = 0;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            ofVec3f v2;
            v2.set(0,0,0);
            float distancia;
            distancia = kinect.getDistanceAt(x, y);
            
            if(distancia > distanciaMinima && distancia < distanciaMaxima) {
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
void ofApp::drawRecordedPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int pCount = 0;
    //int step = gridSize; //DB
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            
            ofVec3f v2;
            v2.set(0,0,0);
            //float distancia;
            //distancia = kinect.getDistanceAt(x, y);
            //if(distancia > distanciaMinima && distancia < distanciaMaxima) {
            v2 = meshRecorder.getVectorAt(frameToPlay, pCount);
            //if(v2.z > 0)
            mesh.addVertex(v2);
            ofColor c;
            c = meshRecorder.getColorAt(frameToPlay, pCount);
            if(colorMode) mesh.addColor(c);
            //}
            pCount ++;
        }
    }
    
    glPointSize(1);
    //glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -750); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    mesh.clear();
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

//-----------------------------------------
void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
    //OF_PRIMITIVE_TRIANGLES, OF_PRIMITIVE_TRIANGLE_STRIP, OF_PRIMITIVE_TRIANGLE_FAN, OF_PRIMITIVE_LINES, OF_PRIMITIVE_LINE_STRIP, OF_PRIMITIVE_LINE_LOOP, OF_PRIMITIVE_POINTS
    
	mesh.setMode(OF_PRIMITIVE_POINTS);
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
	//mesh.drawFaces();
    //mesh.drawVertices();
    //mesh.drawWireframe();
	ofDisableDepthTest();
	ofPopMatrix();

    
}

//--------------------------------------------------------------
void ofApp::exit() {
    
    // recorder hack
    
    // Todo: Esto dentro de la clase!!
    // No sabemos si funciona muy bien lo de detener el Thread :(
    meshRecorder.unlock();
    //  meshRecorder.stopThread(false); //DB
    meshRecorder.stopThread();

    //-------
    
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
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
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
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
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
            
        case 'k':
            colorMode = !colorMode;
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
