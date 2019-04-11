#include "ofMain.h"
#include "ofApp.h"


#define USE_MODERN_OPENGL 0

//========================================================================
int main() {
	glfwInit();
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	float xscale, yscale;
	glfwGetMonitorContentScale(monitor, &xscale, &yscale);

	float w = 1280;
	float h = 960;
	ofSetupOpenGL(w * xscale, h * yscale, OF_WINDOW);

	//#if USE_MODERN_OPENGL
	//	ofGLWindowSettings settings;
	//	settings.setGLVersion(4, 2);
	//	ofCreateWindow(settings);
	//#else
	//	ofSetupOpenGL(1280, 960,OF_WINDOW);
	//#endif
		// this kicks off the running of my app
		// can be OF_WINDOW or OF_FULLSCREEN
		// pass in width and height too:
	ofRunApp(new ofApp());

}
