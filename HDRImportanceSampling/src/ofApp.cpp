#include "ofApp.h"

#include "peseudo_random.hpp"
#include "image2d.hpp"
#include "alias_method.hpp"
#include <array>
#include "online.hpp"
#include "assertion.hpp"

rt::Image2D image;
rt::AliasMethod aliasMethod;

//--------------------------------------------------------------
void ofApp::setup() {
	ofxRaccoonImGui::initialize();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	image.load(ofToDataPath("zhengyang_gate_tiny.hdr").c_str());
	// image.load(ofToDataPath("zhengyang_gate_1k.hdr").c_str());

	image.clamp_rgb(0.0f, 20.0f);

	std::vector<float> weights(image.width() * image.height());
	for (int y = 0; y < image.height(); ++y) {
		for (int x = 0; x < image.width(); ++x) {
			auto radiance = image(x, y);
			float Y = 0.2126f * radiance.x + 0.7152f * radiance.y + 0.0722f * radiance.z;
			weights[y * image.width() + x] = Y;
		}
	}
	aliasMethod.prepare(weights);

	//rt::PCG32 random;
	//for (int y = 0; y < image.height(); ++y) {
	//	for (int x = 0; x < image.width(); ++x) {
	//		// image(x, y) = glm::vec4(random.uniform32f(), random.uniform32f(), random.uniform32f(), 1.0f);
	//		float v = random.uniform32f();
	//		image(x, y) = glm::vec4(v, v, v, 1.0f);
	//	}
	//}
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

	static bool showImage = true;
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

	float must_be_one = 0.0f;

	if (sample_on_sphere) {
		rt::OnlineMean<float> mean;

		for (int i = 0; i < 10000; ++i) {
			int index = aliasMethod.sample(random.uniform32f(), random.uniform32f());
			int ix = index % image.width();
			int iy = index / image.width();
			float sample_x = ix + random.uniform32f();
			// float sample_y = iy + random.uniform32f();

			float phi = -glm::two_pi<float>() * (sample_x / image.width());

			float theta_step = glm::pi<float>() / image.height();
			float beg_theta = theta_step * iy;
			float end_theta = beg_theta + theta_step;
			float beg_y = cos(beg_theta);
			float end_y = cos(end_theta);

			float y = glm::mix(beg_y, end_y, random.uniform32f());
			float r_xz = std::sqrt(std::max(1.0f - y * y, 0.0f));

			float x = r_xz * sin(phi);
			float z = r_xz * cos(phi);

			glm::vec3 wi(x, y, z);
			mesh.addVertex(wi);

			float cosTheta = glm::max(glm::dot(wi, glm::vec3(0, 1, 0)), 0.0f);
			float value = cosTheta / glm::pi<float>();

			float sr = (beg_y - end_y) * (glm::two_pi<float>() / image.width());
			float pdf = (1.0f / sr) * aliasMethod.probability(index);
			mean.addSample(value / pdf);

			//{
			//	glm::vec3 rd = wi;
			//	float z = rd.y;
			//	float x = rd.z;
			//	float y = rd.x;
			//	float theta = std::acos(z);
			//	float phi = atan2(y, x);

			//	//if (isfinite(theta) == false || isfinite(phi) == false) {
			//	//	continue;
			//	//}

			//	if (phi < 0.0) {
			//		phi += glm::two_pi<float>();
			//	}

			//	// 1.0f - is clockwise order envmap
			//	float u = 1.0f - phi / (2.0f * glm::pi<float>());
			//	float v = theta / glm::pi<float>();

			//	int ix = u * image.width();
			//	int iy = v * image.height();
			//	ix = glm::clamp(ix, 0, image.width() - 1);
			//	iy = glm::clamp(iy, 0, image.height() - 1);

			//	int inversed_index = iy * image.width() + ix;
			//	RT_ASSERT(inversed_index == index);

			//	//float sr = 4.0f * glm::pi<float>() / (_texture->width() * _texture->height());
			//	//return (1.0f / sr) * _aliasMethod.probability(iy * _texture->width() + ix);
			//}
		}
		must_be_one = mean.mean();

		ofSetColor(255);
		mesh.draw();
	}
	else {
		ofPushMatrix();

		ofScale(1, -1, 1);
		ofScale(kScale);
		for (int i = 0; i < 10000; ++i) {
			int index = aliasMethod.sample(random.uniform32f(), random.uniform32f());
			int x = index % image.width();
			int y = index / image.width();
			float sample_x = x + random.uniform32f();
			float sample_y = y + random.uniform32f();
			mesh.addVertex(glm::vec3(sample_x, sample_y, 0.001f));
		}

		ofSetColor(255);
		mesh.draw();

		ofPopMatrix();
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

	ImGui::Text("must_be_one : %f", must_be_one);
	
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
