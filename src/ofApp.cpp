#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	ofSetBackgroundColor(0);
	ofSetWindowTitle("ThreshRacer");

	// --- Camera setup ---
	cam.setDeviceID(0); // change if you have multiple cameras
	cam.setDesiredFrameRate(30);
	cam.initGrabber(camWidth, camHeight);

	// Window big enough for cam + GUI
	ofSetWindowShape(camWidth + 260, camHeight + 40);

	// Initialise lane quads as simple left/right halves
	float halfW = camWidth * 0.5f;
	lanePts[0][0].set(0, 0);
	lanePts[0][1].set(halfW, 0);
	lanePts[0][2].set(halfW, camHeight);
	lanePts[0][3].set(0, camHeight);

	lanePts[1][0].set(halfW, 0);
	lanePts[1][1].set(camWidth, 0);
	lanePts[1][2].set(camWidth, camHeight);
	lanePts[1][3].set(halfW, camHeight);

	updateLaneGeometry();

	// GUI
	setupGuiPages();
	setActivePage(1);

	// Diff image for debug
	diffImage.allocate(camWidth, camHeight, OF_IMAGE_COLOR);
}

//--------------------------------------------------------------
void ofApp::setupGuiPages() {
	// -------- Page 1: Lane editor --------
	guiPage1 = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	guiPage1->setWidth(260);
	guiPage1->addHeader("  ThreshRacer  -  Page 1");
	guiPage1->addBreak();
	guiPage1->addLabel("Lane Editor");
	guiPage1->addLabel("Drag corners of each lane");
	guiPage1->addLabel("Keys: [1] Lanes  [2] Motion  [3] Rounds");

	// -------- Page 2: Motion parameters --------
	guiPage2 = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	guiPage2->setWidth(260);
	guiPage2->addHeader("  ThreshRacer  -  Page 2");
	guiPage2->addBreak();
	guiPage2->addLabel("Motion Parameters");

	thresholdSlider = guiPage2->addSlider("Threshold", 0.0f, 128.0f, motionThreshold);
	thresholdSlider->setPrecision(0);

	smoothingSlider = guiPage2->addSlider("Smoothing", 0.0f, 1.0f, smoothing);
	smoothingSlider->setPrecision(2);

	guiPage2->addBreak();
	leftScoreLabel = guiPage2->addLabel("Left motion:  0.000");
	rightScoreLabel = guiPage2->addLabel("Right motion: 0.000");

	// -------- Page 3: Progress race controls --------
	guiPage3 = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	guiPage3->setWidth(260);
	guiPage3->addHeader("  ThreshRacer  -  Page 3");
	guiPage3->addBreak();
	guiPage3->addLabel("Progress Race");

	// Per-lane gains
	leftGainSlider = guiPage3->addSlider("Left gain", 0.05f, 2.0f, progressGainLeft);
	rightGainSlider = guiPage3->addSlider("Right gain", 0.05f, 2.0f, progressGainRight);
	leftGainSlider->setPrecision(2);
	rightGainSlider->setPrecision(2);

	guiPage3->addBreak();
	startRoundButton = guiPage3->addButton("Start race");

	guiPage3->addBreak();
	roundStatusLabel = guiPage3->addLabel("Race idle");

	// Button events
	startRoundButton->onButtonEvent(this, &ofApp::onButtonEvent);
}

//--------------------------------------------------------------
void ofApp::setActivePage(int page) {
	currentPage = ofClamp(page, 1, 3);

	if (guiPage1) guiPage1->setVisible(currentPage == 1);
	if (guiPage2) guiPage2->setVisible(currentPage == 2);
	if (guiPage3) guiPage3->setVisible(currentPage == 3);
}

//--------------------------------------------------------------
float ofApp::computePolygonArea(const ofPoint * pts, int count) {
	if (count < 3) return 0.0f;
	double area = 0.0;
	for (int i = 0; i < count; ++i) {
		const ofPoint & p0 = pts[i];
		const ofPoint & p1 = pts[(i + 1) % count];
		area += p0.x * p1.y - p1.x * p0.y;
	}
	return fabs(area) * 0.5f;
}

//--------------------------------------------------------------
void ofApp::computeLaneAreas() {
	for (int lane = 0; lane < NUM_LANES; ++lane) {
		laneAreas[lane] = computePolygonArea(lanePts[lane], NUM_CORNERS);
	}
}

//--------------------------------------------------------------
void ofApp::updateLaneGeometry() {
	for (int lane = 0; lane < NUM_LANES; ++lane) {
		lanePolys[lane].clear();
		for (int i = 0; i < NUM_CORNERS; ++i) {
			lanePolys[lane].addVertex(lanePts[lane][i]);
		}
		lanePolys[lane].close();
	}
	computeLaneAreas();
}

//--------------------------------------------------------------
void ofApp::update() {
	// Update GUI
	if (guiPage1) guiPage1->update();
	if (guiPage2) guiPage2->update();
	if (guiPage3) guiPage3->update();

	cam.update();

	if (cam.isFrameNew()) {
		if (thresholdSlider) {
			motionThreshold = thresholdSlider->getValue();
		}
		if (smoothingSlider) {
			smoothing = ofClamp(smoothingSlider->getValue(), 0.0f, 1.0f);
		}
		if (leftGainSlider) {
			progressGainLeft = leftGainSlider->getValue();
		}
		if (rightGainSlider) {
			progressGainRight = rightGainSlider->getValue();
		}

		computeMotion();
		updateGuiLabels();
	}

	updateRoundStatus();
}

//--------------------------------------------------------------
void ofApp::computeMotion() {
	ofPixels & cur = cam.getPixels();
	if (!cur.isAllocated()) return;

	int w = cur.getWidth();
	int h = cur.getHeight();
	int channels = cur.getNumChannels();

	if (!prevFrame.isAllocated()) {
		prevFrame = cur;
		hasPrevFrame = true;
		return;
	}

	if (!diffPixels.isAllocated() || diffPixels.getWidth() != w || diffPixels.getHeight() != h || diffPixels.getNumChannels() != channels) {
		diffPixels.allocate(w, h, channels);
	}

	float frameLeftSum = 0.0f;
	float frameRightSum = 0.0f;

	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			int idx = (y * w + x) * channels;

			unsigned char rCur = cur[idx + 0];
			unsigned char gCur = cur[idx + 1];
			unsigned char bCur = cur[idx + 2];

			unsigned char rPrev = prevFrame[idx + 0];
			unsigned char gPrev = prevFrame[idx + 1];
			unsigned char bPrev = prevFrame[idx + 2];

			int dR = abs((int)rCur - (int)rPrev);
			int dG = abs((int)gCur - (int)gPrev);
			int dB = abs((int)bCur - (int)bPrev);

			float d = (dR + dG + dB) / 3.0f;

			bool isMotion = d > motionThreshold;

			unsigned char out = isMotion ? 255 : 0;
			diffPixels[idx + 0] = out;
			diffPixels[idx + 1] = out;
			diffPixels[idx + 2] = out;

			if (isMotion) {
				bool inLeft = lanePolys[0].size() > 0 && lanePolys[0].inside(x, y);
				bool inRight = lanePolys[1].size() > 0 && lanePolys[1].inside(x, y);

				if (inLeft) frameLeftSum += d;
				if (inRight) frameRightSum += d;
			}
		}
	}

	diffImage.setFromPixels(diffPixels);

	// Normalise per lane using polygon area
	float leftNorm = 0.0f;
	float rightNorm = 0.0f;

	float maxLeft = laneAreas[0] * 255.0f;
	float maxRight = laneAreas[1] * 255.0f;

	if (maxLeft > 0.0f) {
		leftNorm = frameLeftSum / maxLeft;
	}
	if (maxRight > 0.0f) {
		rightNorm = frameRightSum / maxRight;
	}

	// Smooth *instant* scores (for live “who’s stronger now”)
	leftScore = ofLerp(leftScore, leftNorm, smoothing);
	rightScore = ofLerp(rightScore, rightNorm, smoothing);

	// Progress accumulation when race is active (per-lane gains)
	if (roundActive) {
		progressLeft = ofClamp(progressLeft + leftNorm * progressGainLeft, 0.0f, 1.0f);
		progressRight = ofClamp(progressRight + rightNorm * progressGainRight, 0.0f, 1.0f);

		if (progressLeft >= 1.0f || progressRight >= 1.0f) {
			roundActive = false;
		}
	}

	// Store current as previous
	prevFrame = cur;
}

//--------------------------------------------------------------
void ofApp::updateGuiLabels() {
	if (!guiPage2) return;

	if (leftScoreLabel) {
		leftScoreLabel->setLabel("Left motion:  " + ofToString(leftScore, 3));
	}
	if (rightScoreLabel) {
		rightScoreLabel->setLabel("Right motion: " + ofToString(rightScore, 3));
	}
}

//--------------------------------------------------------------
void ofApp::updateRoundStatus() {
	if (!guiPage3 || !roundStatusLabel) return;

	int pL = (int)ofClamp(progressLeft * 100.0f, 0.0f, 100.0f);
	int pR = (int)ofClamp(progressRight * 100.0f, 0.0f, 100.0f);

	std::string label;

	if (roundActive) {
		label = "Running: L " + ofToString(pL) + "%  R " + ofToString(pR) + "%";
	} else {
		if (progressLeft == 0.0f && progressRight == 0.0f) {
			label = "Race idle";
		} else {
			if (fabs(progressLeft - progressRight) < 0.001f && progressLeft >= 1.0f && progressRight >= 1.0f) {
				label = "Finished: Tie (100%)";
			} else if (progressLeft >= 1.0f && progressLeft > progressRight) {
				label = "Finished: Left wins (" + ofToString(pL) + "% vs " + ofToString(pR) + "%)";
			} else if (progressRight >= 1.0f && progressRight > progressLeft) {
				label = "Finished: Right wins (" + ofToString(pR) + "% vs " + ofToString(pL) + "%)";
			} else {
				// Race ended but nobody hit exact 100% (should be rare)
				if (progressLeft > progressRight) {
					label = "Finished: Left leads (" + ofToString(pL) + "% vs " + ofToString(pR) + "%)";
				} else if (progressRight > progressLeft) {
					label = "Finished: Right leads (" + ofToString(pR) + "% vs " + ofToString(pL) + "%)";
				} else {
					label = "Finished: Tie (" + ofToString(pL) + "%)";
				}
			}
		}
	}

	roundStatusLabel->setLabel(label);
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0);

	// Draw camera
	cam.draw(0, 0, camWidth, camHeight);

	ofPushStyle();

	// Colours
	ofColor leftColor(0, 200, 255);
	ofColor rightColor(255, 0, 200);

	// Filled lane quads with low alpha
	ofSetColor(leftColor.r, leftColor.g, leftColor.b, 40);
	ofBeginShape();
	for (int i = 0; i < NUM_CORNERS; ++i) {
		ofVertex(lanePts[0][i]);
	}
	ofEndShape(true);

	ofSetColor(rightColor.r, rightColor.g, rightColor.b, 40);
	ofBeginShape();
	for (int i = 0; i < NUM_CORNERS; ++i) {
		ofVertex(lanePts[1][i]);
	}
	ofEndShape(true);

	// Outlines
	ofSetLineWidth(2.0f);
	ofSetColor(leftColor);
	for (int i = 0; i < NUM_CORNERS; ++i) {
		const ofPoint & a = lanePts[0][i];
		const ofPoint & b = lanePts[0][(i + 1) % NUM_CORNERS];
		ofDrawLine(a, b);
	}
	ofSetColor(rightColor);
	for (int i = 0; i < NUM_CORNERS; ++i) {
		const ofPoint & a = lanePts[1][i];
		const ofPoint & b = lanePts[1][(i + 1) % NUM_CORNERS];
		ofDrawLine(a, b);
	}

	// Corner handles (Page 1 only)
	if (currentPage == 1) {
		ofSetColor(leftColor);
		for (int i = 0; i < NUM_CORNERS; ++i) {
			ofDrawCircle(lanePts[0][i], 6);
		}
		ofSetColor(rightColor);
		for (int i = 0; i < NUM_CORNERS; ++i) {
			ofDrawCircle(lanePts[1][i], 6);
		}
	}

	// Live motion status (based on instantaneous scores)
	ofSetColor(255);
	std::string liveStatus;
	if (fabs(leftScore - rightScore) < 0.01f) {
		liveStatus = "Live motion: Tie";
	} else if (leftScore > rightScore) {
		liveStatus = "Live motion: LEFT stronger";
	} else {
		liveStatus = "Live motion: RIGHT stronger";
	}
	ofDrawBitmapStringHighlight(liveStatus, 20, camHeight - 15);

	// Progress bars at bottom of camera
	float barMaxHeight = 100.0f;
	float barWidth = 60.0f;

	float leftBarHeight = barMaxHeight * ofClamp(progressLeft, 0.0f, 1.0f);
	float rightBarHeight = barMaxHeight * ofClamp(progressRight, 0.0f, 1.0f);

	float leftBarX = camWidth * 0.25f - barWidth * 0.5f;
	float rightBarX = camWidth * 0.75f - barWidth * 0.5f;
	float barBaseY = camHeight - 30.0f;

	// Draw bars
	ofSetColor(leftColor);
	ofDrawRectangle(leftBarX, barBaseY - leftBarHeight, barWidth, leftBarHeight);

	ofSetColor(rightColor);
	ofDrawRectangle(rightBarX, barBaseY - rightBarHeight, barWidth, rightBarHeight);

	// White target line at 100% height
	ofSetColor(255);
	ofSetLineWidth(2.0f);
	float targetY = barBaseY - barMaxHeight;
	ofDrawLine(leftBarX - 10, targetY, rightBarX + barWidth + 10, targetY);
	ofDrawBitmapString("100%", leftBarX - 40, targetY + 4);

	// Optional diff view
	if (showDiff) {
		ofSetColor(255);
		float diffW = camWidth * 0.35f;
		float diffH = camHeight * 0.35f;
		diffImage.draw(10, camHeight - diffH - 10, diffW, diffH);
		ofDrawBitmapStringHighlight("DIFF", 15, camHeight - diffH - 20);
	}

	ofPopStyle();

	// Draw the active GUI page
	if (currentPage == 1 && guiPage1) guiPage1->draw();
	if (currentPage == 2 && guiPage2) guiPage2->draw();
	if (currentPage == 3 && guiPage3) guiPage3->draw();

	// Page indicator
	ofSetColor(255);
	ofDrawBitmapStringHighlight("Page " + ofToString(currentPage) + "  (1=Lanes  2=Motion  3=Rounds)",
		20, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'f' || key == 'F') {
		ofToggleFullscreen();
	}
	if (key == 'd' || key == 'D') {
		showDiff = !showDiff;
	}

	if (key == '1') {
		setActivePage(1);
	} else if (key == '2') {
		setActivePage(2);
	} else if (key == '3') {
		setActivePage(3);
	}
}

//--------------------------------------------------------------
void ofApp::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target == startRoundButton) {
		// Start a new progress race
		progressGainLeft = leftGainSlider ? leftGainSlider->getValue() : progressGainLeft;
		progressGainRight = rightGainSlider ? rightGainSlider->getValue() : progressGainRight;

		roundActive = true;
		progressLeft = 0.0f;
		progressRight = 0.0f;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) { }

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) { }

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	if (currentPage != 1) return;
	if (!draggingCorner) return;

	// Clamp to camera area
	x = ofClamp(x, 0, camWidth - 1);
	y = ofClamp(y, 0, camHeight - 1);

	if (draggingLane >= 0 && draggingLane < NUM_LANES && draggingCornerIndex >= 0 && draggingCornerIndex < NUM_CORNERS) {
		lanePts[draggingLane][draggingCornerIndex].set(x, y);
		updateLaneGeometry();
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	if (currentPage != 1) return;

	// Only interact within the camera area
	if (x < 0 || x >= camWidth || y < 0 || y >= camHeight) {
		return;
	}

	draggingCorner = false;
	draggingLane = -1;
	draggingCornerIndex = -1;

	// Find nearest corner within pick radius
	float bestDistSq = cornerPickRadius * cornerPickRadius;

	for (int lane = 0; lane < NUM_LANES; ++lane) {
		for (int i = 0; i < NUM_CORNERS; ++i) {
			float dx = x - lanePts[lane][i].x;
			float dy = y - lanePts[lane][i].y;
			float d2 = dx * dx + dy * dy;
			if (d2 <= bestDistSq) {
				bestDistSq = d2;
				draggingCorner = true;
				draggingLane = lane;
				draggingCornerIndex = i;
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	draggingCorner = false;
	draggingLane = -1;
	draggingCornerIndex = -1;
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) { }
void ofApp::mouseExited(int x, int y) { }
void ofApp::windowResized(int w, int h) { }
void ofApp::gotMessage(ofMessage msg) { }
void ofApp::dragEvent(ofDragInfo dragInfo) { }
