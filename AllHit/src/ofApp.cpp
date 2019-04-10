#include "ofApp.h"
#include "triangle_util.hpp"

inline void EmbreeErorrHandler(void* userPtr, RTCError code, const char* str) {
	printf("Embree Error [%d] %s\n", code, str);

}
struct CustomContext : RTCIntersectContext {
	int i = 0;
};

static void filter_function(const struct RTCFilterFunctionNArguments *argument) {
	glm::vec3 ro = { RTCRayN_org_x(argument->ray, 1, 0), RTCRayN_org_y(argument->ray, 1, 0), RTCRayN_org_z(argument->ray, 1, 0) };
	glm::vec3 rd = { RTCRayN_dir_x(argument->ray, 1, 0), RTCRayN_dir_y(argument->ray, 1, 0), RTCRayN_dir_z(argument->ray, 1, 0) };
	*argument->valid = 0;

	float tfar = RTCRayN_tfar(argument->ray, 1, 0);

	ofSetColor(255, 0, 0);
	ofDrawSphere(ro + rd * tfar, 0.02f);

	CustomContext *custom =(CustomContext *)argument->context;

	ofSetColor(255);
	char buffer[64];
	snprintf(buffer, sizeof(buffer), "i(%d), g(%d)", custom->i, RTCHitN_geomID(argument->hit, 1, 0));
	ofDrawBitmapString(buffer, ro + rd * tfar);

	custom->i++;
}

//--------------------------------------------------------------
void ofApp::setup(){
	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	std::string error_message;
	if (_storage.open(ofToDataPath("allhit.abc"), error_message) == false) {
		printf("AlembicStorage::open error: %s\n", error_message.c_str());
	}
	_scene = _storage.read(0, error_message);
	if (!_scene) {
		printf("_storage.read error: %s\n", error_message.c_str());
	}

	_embreeDevice = std::shared_ptr<RTCDeviceTy>(rtcNewDevice("set_affinity=1"), rtcReleaseDevice);
	rtcSetDeviceErrorFunction(_embreeDevice.get(), EmbreeErorrHandler, nullptr);

	_embreeScene = std::shared_ptr<RTCSceneTy>(rtcNewScene(_embreeDevice.get()), rtcReleaseScene);
	rtcSetSceneBuildQuality(_embreeScene.get(), RTC_BUILD_QUALITY_HIGH);
	//rtcSetSceneFlags(_embreeScene.get(), RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);

	int gIndex = 0;
	for (auto o : _scene->objects) {
		if (o->visible == false) {
			continue;
		}

		if (auto polymesh = o.as_polygonMesh()) {
			bool isTriangleMesh = std::all_of(polymesh->faceCounts.begin(), polymesh->faceCounts.end(), [](int32_t f) { return f == 3; });
			RT_ASSERT(isTriangleMesh);

			// add to embree
			RTCGeometry g = rtcNewGeometry(_embreeDevice.get(), RTC_GEOMETRY_TYPE_TRIANGLE);

			size_t vertexStride = sizeof(glm::vec3);
			rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, 0 /*slot*/, RTC_FORMAT_FLOAT3, polymesh->P.data(), 0 /*byteoffset*/, vertexStride, polymesh->P.size());

			size_t indexStride = sizeof(uint32_t) * 3;
			size_t primitiveCount = polymesh->indices.size() / 3;
			rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0 /*slot*/, RTC_FORMAT_UINT3, polymesh->indices.data(), 0 /*byteoffset*/, indexStride, primitiveCount);

			rtcCommitGeometry(g);
			rtcAttachGeometryByID(_embreeScene.get(), g, gIndex++);
			rtcReleaseGeometry(g);
		}
	}

	rtcCommitScene(_embreeScene.get());
}

//--------------------------------------------------------------
void ofApp::update(){

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

	drawAlembicScene(_scene.get(), ofMesh(), false /*draw camera*/);

	std::vector<glm::dvec3> ros;
	std::vector<glm::dvec3> rds;

	for (auto o : _scene->objects) {
		if (o->visible == false) {
			continue;
		}
		auto point = o.as_point();
		if (point == nullptr) {
			continue;
		}
		auto N_attrib = point->points.column_as_vector3("N");
		for (int i = 0; i < point->P.size(); ++i) {
			glm::vec3 P = { point->P[i].x, point->P[i].y, point->P[i].z};
			glm::vec3 N;
			N_attrib->get(i, glm::value_ptr(N));

			ros.push_back(P);
			rds.push_back(N);
		}
	}

	for (int i = 0; i < ros.size(); ++i) {
		glm::vec3 ro = ros[i];
		glm::vec3 rd = rds[i];

		for (auto o : _scene->objects) {
			if (o->visible == false) {
				continue;
			}
			auto polymesh = o.as_polygonMesh();
			if (polymesh == nullptr) {
				continue;
			}
			for (int i = 0; i < polymesh->indices.size(); i += 3) {
				auto to = [](houdini_alembic::Vector3f p) {
					return glm::dvec3(p.x, p.y, p.z);
				};
				glm::vec3 p0 = to(polymesh->P[polymesh->indices[i]]);
				glm::vec3 p1 = to(polymesh->P[polymesh->indices[i + 1]]);
				glm::vec3 p2 = to(polymesh->P[polymesh->indices[i + 2]]);

				float tmin = std::numeric_limits<float>::max();
				if (rt::intersect_ray_triangle(ro, rd, p0, p1, p2, &tmin)) {
					ofSetColor(255, 0, 0);
					ofDrawLine(ro, ro + rd * tmin);
					ofDrawSphere(ro + rd * tmin, 0.05f);
				}
			}
		}
	}

	//for (auto o : _scene->objects) {
	//	if (o->visible == false) {
	//		continue;
	//	}
	//	auto point = o.as_point();
	//	if (point == nullptr) {
	//		continue;
	//	}
	//	auto N_attrib = point->points.column_as_vector3("N");
	//	for (int i = 0; i < point->P.size(); ++i) {
	//		glm::vec3 P = { point->P[i].x, point->P[i].y, point->P[i].z};
	//		glm::vec3 N;
	//		N_attrib->get(i, glm::value_ptr(N));

	//		RTCRayHit rayhit;
	//		rayhit.ray.org_x = P.x;
	//		rayhit.ray.org_y = P.y;
	//		rayhit.ray.org_z = P.z;
	//		rayhit.ray.dir_x = N.x;
	//		rayhit.ray.dir_y = N.y;
	//		rayhit.ray.dir_z = N.z;
	//		rayhit.ray.time = 0.0f;

	//		rayhit.ray.tfar = FLT_MAX;
	//		rayhit.ray.tnear = 0.0f;

	//		rayhit.ray.mask = 0;
	//		rayhit.ray.id = 0;
	//		rayhit.ray.flags = 0;
	//		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	//		rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
	//		CustomContext context;
	//		rtcInitIntersectContext(&context);
	//		context.filter = &filter_function;

	//		// Intersectは順序はあくまで近傍から。アルファテストの場合はこちらのほうがいいだろう
	//		// rtcIntersect1(_embreeScene.get(), &context, &rayhit);

	//		// Occludedの場合は順序は気にされない。ALL-Hitならこちらのほうが有力か
	//		rtcOccluded1(_embreeScene.get(), &context, &rayhit.ray);

	//		RT_ASSERT(rayhit.ray.tfar == FLT_MAX);

	//		if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
	//			ofSetColor(128);
	//			ofDrawLine(P, P + N * 10.0f);
	//		}
	//		else {
	//			ofSetColor(255, 0, 0);
	//			float tfar = rayhit.ray.tfar;
	//			ofDrawLine(P, P + N * tfar);
	//		}
	//	}
	//}

	_camera.end();

	ofDisableDepthTest();
	ofSetColor(255);
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
