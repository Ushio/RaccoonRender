#include "ofApp.h"
#include "ofxRaccoonImGui.hpp"
#include "linear_transform.hpp"
#include "cubic_bezier.hpp"

//--------------------------------------------------------------
void ofApp::setup() {
	ofxRaccoonImGui::initialize();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

}
void ofApp::exit() {
	ofxRaccoonImGui::shutdown();
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw(){
	static int NPoints = 100;
	static int Iteration = 5;

	ofEnableDepthTest();

	ofClear(0);

	_camera.begin();
	ofPushMatrix();
	ofRotateYDeg(90.0f);
	ofSetColor(64);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofDrawAxis(50);

	using namespace rt;

	float cx[4] = { 0.0f, 0.256f, 0.394f, 0.730918f };
	float cy[4] = { 1.0f, 1.0f,   0.056f, 0.0f };

	{
		ofPolyline line;
		LinearTransform<float> toT(0, NPoints - 1, 0, 1);
		for (int i = 0; i < NPoints; ++i) {
			auto t = toT.evaluate(i);
			auto x = evaluate_cubic_bezier(t, cx[0], cx[1], cx[2], cx[3]);
			auto y = evaluate_cubic_bezier(t, cy[0], cy[1], cy[2], cy[3]);
			line.addVertex(x, y);
		}
		ofSetColor(255);
		line.draw();
	}
	{
		ofPolyline line;
		LinearTransform<float> toX(0, NPoints - 1, 0, 1);
		for (int i = 0; i < NPoints; ++i) {
			auto x = toX.evaluate(i);
			auto y = evaluate_bezier_funtion(x, cx, cy, Iteration);
			line.addVertex(x, y, 0.1f);
		}
		ofSetColor(255, 0, 0);
		line.draw();
	}

	_camera.end();

	ofDisableDepthTest();
	ofSetColor(255);

	ofxRaccoonImGui::ScopedImGui imgui;

	// camera control                                          for control clicked problem
	if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || (ImGui::IsWindowFocused(ImGuiFocusedFlags_AnyWindow) && ImGui::IsAnyMouseDown())) {
		_camera.disableMouseInput();
	}
	else {
		_camera.enableMouseInput();
	}

	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
	ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
	ImGui::SetNextWindowBgAlpha(0.5f);

	ImGui::Begin("settings", nullptr);
	ImGui::InputInt("NPoints", &NPoints);
	ImGui::InputInt("Iteration", &Iteration);
	ImGui::End();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
