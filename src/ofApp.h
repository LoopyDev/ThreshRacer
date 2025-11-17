#pragma once

#include "ofMain.h"
#include "ofxDatGui.h" // ofxDatGui-LoopyDev

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void onButtonEvent(ofxDatGuiButtonEvent e);

private:
	// Camera ---------------------------------
	ofVideoGrabber cam;
	int camWidth = 640;
	int camHeight = 480;

	// Lanes as draggable quadrilaterals ------
	static const int NUM_LANES = 2;
	static const int NUM_CORNERS = 4;

	ofPoint lanePts[NUM_LANES][NUM_CORNERS];
	ofPolyline lanePolys[NUM_LANES];
	float laneAreas[NUM_LANES];

	bool draggingCorner = false;
	int draggingLane = -1; // 0 = left, 1 = right
	int draggingCornerIndex = -1;
	float cornerPickRadius = 15.0f;

	// Motion analysis ------------------------
	ofPixels prevFrame; // previous frame
	ofPixels diffPixels; // thresholded diff
	ofImage diffImage; // for debug drawing
	bool hasPrevFrame = false;

	float leftScore = 0.0f; // smoothed *instant* motion [0..1-ish]
	float rightScore = 0.0f;

	float motionThreshold = 30.0f; // 0..255
	float smoothing = 0.2f; // 0..1 (lerp factor)
	bool showDiff = false; // toggle 'd'

	// Progress-based race --------------------
	bool roundActive = false;
	float progressLeft = 0.0f; // 0..1 => 0..100%
	float progressRight = 0.0f; // 0..1 => 0..100%
	float progressGainLeft = 0.5f; // per-lane multipliers
	float progressGainRight = 0.5f;

	// GUI ------------------------------------
	int currentPage = 1;

	ofxDatGui * guiPage1 = nullptr; // lane editor
	ofxDatGui * guiPage2 = nullptr; // motion params
	ofxDatGui * guiPage3 = nullptr; // round controls

	// Page 2 widgets
	ofxDatGuiSlider * thresholdSlider = nullptr;
	ofxDatGuiSlider * smoothingSlider = nullptr;
	ofxDatGuiLabel * leftScoreLabel = nullptr;
	ofxDatGuiLabel * rightScoreLabel = nullptr;

	// Page 3 widgets
	ofxDatGuiSlider * leftGainSlider = nullptr;
	ofxDatGuiSlider * rightGainSlider = nullptr;
	ofxDatGuiButton * startRoundButton = nullptr;
	ofxDatGuiLabel * roundStatusLabel = nullptr;

	void setupGuiPages();
	void setActivePage(int page);

	void updateLaneGeometry();
	void computeLaneAreas();
	float computePolygonArea(const ofPoint * pts, int count);

	void computeMotion();
	void updateGuiLabels();
	void updateRoundStatus();
};
