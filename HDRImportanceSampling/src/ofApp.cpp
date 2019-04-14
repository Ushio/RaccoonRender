#include "ofApp.h"

#include <array>
#include "peseudo_random.hpp"
#include "image2d.hpp"
#include "alias_method.hpp"
#include "online.hpp"
#include "assertion.hpp"
#include "envmap.hpp"
#include "spherical_sampler.hpp"

rt::Image2D image;
std::shared_ptr<rt::EnvironmentMap> envmap;

//--------------------------------------------------------------
void ofApp::setup() {
	ofxRaccoonImGui::initialize();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	envmap = std::shared_ptr<rt::EnvironmentMap>(new rt::ImageEnvmap(ofToDataPath("zhengyang_gate_1k.hdr")));

	image.load(ofToDataPath("zhengyang_gate_tiny.hdr").c_str());
}
void ofApp::exit() {
	ofxRaccoonImGui::shutdown();
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofEnableDepthTest();

	ofClear(0);

	_camera.begin();
	ofPushMatrix();
	ofRotateZDeg(90.0f);
	ofSetColor(64);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofDrawAxis(50);

	ofSetColor(255);

	static bool showImage = false;
	static float L = 5.0f;
	static bool sample_on_sphere = false;


	auto reinhard = [](float x, float L) {
		return x / (x + 1.0f) * (1.0f + x / (L * L));
	};
	auto to_8bit = [](float x) {
		int v = x * 256.0f;
		return glm::clamp(v, 0, 255);
	};

	float kScale = 0.1f;

	ofPushMatrix();
	ofScale(1, -1, 1);
	ofScale(kScale);

	if (showImage) {
		for (int y = 0; y < image.height(); ++y) {
			for (int x = 0; x < image.width(); ++x) {
				auto radiance = image(x, y);
				int r = to_8bit(reinhard(radiance.x, L));
				int g = to_8bit(reinhard(radiance.y, L));
				int b = to_8bit(reinhard(radiance.z, L));

				ofRectangle rect(x, y, 1, 1);
				ofSetColor(r, g, b);
				ofDrawRectangle(rect);
			}
		}
	}

	ofPopMatrix();

	static ofMesh mesh;
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);

	rt::PCG32 random;

	if (sample_on_sphere) {
		for (int i = 0; i < 10000; ++i) {
			auto wi = envmap->sample(&random, glm::vec3(0, 1, 0));
			auto L = envmap->radiance(wi);
			float Y = 0.2126f * L.x + 0.7152f * L.y + 0.0722f * L.z;

			mesh.addVertex(reinhard(Y, 20) * wi);
			mesh.addColor(ofFloatColor(
				reinhard(L.x, 20),
				reinhard(L.y, 20),
				reinhard(L.z, 20),
				1.0f
			));
		}
		ofSetColor(255);
		mesh.draw();
	}
	else {
		static std::vector<glm::vec3> ps;

		ps.clear();
		int N = 10000;
		for (int i = 0; i < N; ++i) {
			glm::vec3 rd;
			float maxYZ;
			do {
				rd = rt::sample_on_unit_sphere(random.uniform(), random.uniform());
				glm::vec3 abs_rd = glm::abs(rd);
				maxYZ = std::max(abs_rd.y, abs_rd.z);
			} while (rd.x < maxYZ);

			ps.push_back(rd);

			//glm::vec3 abs_rd = glm::abs(rd);
			//float maxYZ = std::max(abs_rd.y, abs_rd.z);
			//float maxXZ = std::max(abs_rd.x, abs_rd.z);

			//Axis direction;
			//if (maxYZ < abs_rd.x) {
			//	direction = 0.0f < rd.x ? Axis_XPlus : Axis_XMinus;
			//}
			//else if (maxXZ < abs_rd.y) {
			//	direction = 0.0f < rd.y ? Axis_YPlus : Axis_YMinus;
			//}
			//else {
			//	direction = 0.0f < rd.z ? Axis_ZPlus : Axis_ZMinus;
			//}
		}
		for (int i = 0; i < N; ++i) {
			// glm::vec3 rd = ps[i];
			glm::vec3 rd = rt::sample_on_unit_sphere(random.uniform(), random.uniform());

			float sum = 0;
			for (glm::vec3 o : ps) {
				sum += glm::max(glm::dot(o, rd), 0.0f);
			}
			float avg = sum / N;

			//mesh.addVertex(rd);
			//mesh.addColor(ofColor(255));

			mesh.addVertex(rd * avg);
			mesh.addColor(ofColor(255, 0, 0));
		}


		ofSetColor(255);
		mesh.draw();
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
	ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_Appearing);
	ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
	ImGui::SetNextWindowBgAlpha(0.5f);

	ImGui::Begin("settings", nullptr);
	ImGui::Checkbox("show image", &showImage);
	ImGui::SliderFloat("L (Reinhard)", &L, 1.0f, 10.0f);

	ImGui::Checkbox("sample on sphere", &sample_on_sphere);

	if (ImGui::Button("Save Mesh")) {
		mesh.save("mesh.ply");
	}
	
	ImGui::Separator();
	if (_image.isAllocated()) {
		ofxRaccoonImGui::image(_image);
	}
	ImGui::End();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
