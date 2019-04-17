#include "ofApp.h"

inline ofPixels toOf(const rt::Image &image) {
	ofPixels pixels;
	pixels.allocate(image.width(), image.height(), OF_IMAGE_COLOR);
	uint8_t *dst = pixels.getPixels();

	double scale = 1.0;
	for (int y = 0; y < image.height(); ++y) {
		for (int x = 0; x < image.width(); ++x) {
			int index = y * image.width() + x;
			const auto &px = *image.pixel(x, y);
			auto L = px.color / (double)px.sample;
			dst[index * 3 + 0] = (uint8_t)glm::clamp(glm::pow(L.x * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 1] = (uint8_t)glm::clamp(glm::pow(L.y * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 2] = (uint8_t)glm::clamp(glm::pow(L.z * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
		}
	}
	return pixels;
}

inline ofFloatPixels toOfLinear(const rt::Image &image) {
	ofFloatPixels pixels;
	pixels.allocate(image.width(), image.height(), OF_IMAGE_COLOR);
	float *dst = pixels.getPixels();

	for (int y = 0; y < image.height(); ++y) {
		for (int x = 0; x < image.width(); ++x) {
			int index = y * image.width() + x;
			const auto &px = *image.pixel(x, y);
			auto L = px.color / (double)px.sample;
			dst[index * 3 + 0] = L[0];
			dst[index * 3 + 1] = L[1];
			dst[index * 3 + 2] = L[2];
		}
	}
	return pixels;
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofxRaccoonImGui::initialize();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	loadScene();
}

void ofApp::loadScene() {
	std::string abcPath = ofToDataPath("../../../scenes/CornelBox.abc", true);
	houdini_alembic::AlembicStorage storage;
	std::string error_message;
	storage.open(abcPath, error_message);

	if (storage.isOpened()) {
		std::string error_message;
		_alembicscene = storage.read(0, error_message);
	}
	if (error_message.empty() == false) {
		printf("sample error_message: %s\n", error_message.c_str());
	}

	_camera_model.load("../../../scenes/camera_model.ply");

	std::filesystem::path absDirectory(abcPath);
	absDirectory.remove_filename();
	_scene = std::shared_ptr<rt::Scene>(new rt::Scene(_alembicscene, absDirectory));
	_renderer = std::shared_ptr<rt::PTRenderer>(new rt::PTRenderer(_scene));
}
void ofApp::exit() {
	ofxRaccoonImGui::shutdown();
}

//--------------------------------------------------------------
void ofApp::update() {

}

inline bool isPowerOfTwo(uint32_t n) {
	return (n & (n - 1)) == 0;
}

//--------------------------------------------------------------
void ofApp::draw() {
	static bool show_scene_preview = false;
	static int frame = 0;

	if (_renderer) {
		_renderer->step();

		ofDisableArbTex();

		if (ofGetFrameNum() % 5 == 0) {
			_image.setFromPixels(toOf(_renderer->_image));
			_image.getTexture().setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		}
		uint32_t n = _renderer->stepCount();
		if (32 <= n && isPowerOfTwo(n)) {
			_image.setFromPixels(toOf(_renderer->_image));
			char name[64];
			sprintf(name, "%dspp.png", n);
			_image.save(name);
			printf("elapsed %fs\n", ofGetElapsedTimef());
		}

		//if (_renderer->stepCount() == 512) {
		//	_image.setFromPixels(toOf(_renderer->_image));
		//	char name[64];
		//	sprintf(name, "images/frame_%03d.png", frame);
		//	_image.save(name);

		//	frame++;

		//	houdini_alembic::AlembicStorage storage;
		//	std::string error_message;
		//	storage.open(ofToDataPath("../../../scenes/CornelBox.abc"), error_message);

		//	if (storage.isOpened()) {
		//		std::string error_message;
		//		_alembicscene = storage.read(frame, error_message);
		//	}
		//	if (error_message.empty() == false) {
		//		printf("sample error_message: %s\n", error_message.c_str());
		//	}

		//	_scene = std::shared_ptr<rt::Scene>(new rt::Scene(_alembicscene));
		//	_renderer = std::shared_ptr<rt::PTRenderer>(new rt::PTRenderer(_scene));
		//}


		ofEnableArbTex();
	}

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

	if (_alembicscene && show_scene_preview) {
		drawAlembicScene(_alembicscene.get(), _camera_model, true /*draw camera*/);
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

	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	float xscale, yscale;
	glfwGetMonitorContentScale(monitor, &xscale, &yscale);

	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(1100 * xscale, 900 * yscale), ImGuiCond_Appearing);
	ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
	ImGui::SetNextWindowBgAlpha(0.5f);

	ImGui::Begin("settings", nullptr);
	ImGui::Checkbox("scene preview", &show_scene_preview);
	
	ImGui::Text("frame : %d", frame);
	ImGui::Separator();
	ImGui::Text("%d sample, fps = %.3f", _renderer->stepCount(), ofGetFrameRate());
	ImGui::Text("%d bad sample nan", _renderer->badSampleNanCount());
	ImGui::Text("%d bad sample inf", _renderer->badSampleInfCount());
	ImGui::Text("%d bad sample neg", _renderer->badSampleNegativeCount());
	ImGui::Text("%d bad sample firefly", _renderer->badSampleFireflyCount());
	ImGui::Text("%f pdf_mismatch_ratio", rt::radiance_stat::instance().pdf_mismatch_ratio());
	
	if (ofGetFrameNum() % 10 == 0) {
		_renderer->measureRaysPerSecond();
	}
	ImGui::Text("%.3f MRays/s", (double)_renderer->getRaysPerSecond() * 0.001 * 0.001);

	if (_image.isAllocated()) {

		ofxRaccoonImGui::image(_image, _image.getWidth() * xscale, _image.getHeight() * yscale);
	}
	ImGui::End();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'r') {
		loadScene();
	}
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
