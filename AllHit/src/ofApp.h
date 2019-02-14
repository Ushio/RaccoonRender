#pragma once

#include "ofMain.h"
#include <embree3/rtcore.h>
#include "houdini_alembic.hpp"
#include "alembic_preview.hpp"
#include "assertion.hpp"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofEasyCam _camera;
	houdini_alembic::AlembicStorage _storage;
	std::shared_ptr<houdini_alembic::AlembicScene> _scene;

	std::shared_ptr<RTCDeviceTy> _embreeDevice;
	std::shared_ptr<RTCSceneTy> _embreeScene;
};
