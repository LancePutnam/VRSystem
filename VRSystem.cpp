#include <algorithm> // sort
#include <cmath> // atan2
#include <stdio.h>

#if defined(__APPLE__) && defined(__MACH__)
	#include <OpenGL/OpenGL.h>
	#include <OpenGL/gl.h>
#else
	#define GLEW_NO_GLU
	#include <GL/glew.h>
	#include <GL/gl.h>
#endif

#include "VRSystem.h"

// Enable this to use multisampling method from HelloOpenVR demo.
// Not recommended as it is rather expensive and seems to improve nothing.
//#define MULTISAMPLING

//#define DPRINTF(...)
#define DPRINTF(...) printf("[VRSystem::%s] ", __FUNCTION__); printf(__VA_ARGS__)

typedef VRSystem::Matrix4 Matrix4;
typedef VRSystem::Vec4 Vec4;

void Matrix4::print() const {
	for(int r=0; r<4; ++r){
		for(int c=0; c<4; ++c){
			printf("% 4.3f ", m[c*4+r]);
		}
		printf("\n");
	}
}

void printGLError(const char * note=""){
	GLenum err = glGetError();
	if(GL_NO_ERROR != err){
		const char * errStr;
		switch(err){
		#define C(val) case val: errStr=#val; break;
		C(GL_INVALID_ENUM) C(GL_INVALID_VALUE) C(GL_INVALID_OPERATION)
		C(GL_INVALID_FRAMEBUFFER_OPERATION) C(GL_OUT_OF_MEMORY)
		C(GL_STACK_UNDERFLOW) C(GL_STACK_OVERFLOW)
		#undef C
		default: errStr="unknown";
		}
		printf("GL error: %s (%s)\n", errStr, note);
	}
}


//unsigned fromOVREye(vr::Hmd_Eye eye){ return unsigned(vr::Eye_Left!=eye); }

vr::EVREye toOVREye(int eye){
	return VRSystem::LEFT==eye ? vr::Eye_Left : vr::Eye_Right;
}

vr::ETrackedControllerRole toOVRControllerRole(int hand){
	return VRSystem::LEFT==hand ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand;
}

VRSystem::DeviceType fromOVRDeviceClass(vr::ETrackedDeviceClass v){
	switch(v){
	case vr::TrackedDeviceClass_HMD: return VRSystem::HMD;
	case vr::TrackedDeviceClass_Controller: return VRSystem::CONTROLLER;
	case vr::TrackedDeviceClass_GenericTracker: return VRSystem::TRACKER;
	case vr::TrackedDeviceClass_TrackingReference: return VRSystem::TRACKING_REFERENCE;
	default: return VRSystem::INVALID;
	}
}
/*
		EventType type;
		DeviceType deviceType;
		int deviceIndex;
		float age;
		union{
			unsigned button;	///< Button number
			struct{unsigned axis; float x,y;};	///< Axis number and coordinates
		};
*/
void VRSystem::Event::print() const {
	printf("%s (dev %d) %s", toString(deviceType), deviceIndex, toString(type));
	switch(type){
	case BUTTON_DOWN:
	case BUTTON_UP:
	case TOUCH:
	case UNTOUCH:
		printf(" on %d", button);
		break;
	default:;
	}
	printf("\n");
}

int VRSystem::Controller::axisSector(Button b, float divs, float rotate) const {
	const auto * xy = axis(b);
	if(xy[0]==0 || xy[1]==0) return -1;
	auto arg1 = std::atan2(xy[1], xy[0])*0.15915494309189533576888376337251;
	arg1 += rotate;
	if(arg1 < 0.) arg1 += 1.;
	else if(arg1 > 1.) arg1 -= 1.;
	return int(arg1*divs);
}

void VRSystem::Controller::print() const {
	auto printBits = [](uint64_t v){
		for(int i=63; i>=0; --i){
			printf("%c", (v>>i)&1 ? '1' : '0');
		} printf("\n");
	};
	printf("buttons:       "); printBits(mButtons);
	printf("button changes:"); printBits(mButtonChanges);
	printf("touches:       "); printBits(mTouches);
	printf("touch changes: "); printBits(mTouchChanges);
	printf("axes:          ");
	for(int j=0; j<5; ++j){
		printf("(");
		for(int i=0; i<2; ++i){
			printf("% 3.2f ", mAxes[j][i]);
		}
		printf(") ");
	} printf("\n");
}

int VRSystem::controllerIndex(int hand) const {
	// hand is 0 or 1
	const auto& indices = mDeviceIndices[CONTROLLER];
	if(indices.size() >= 2){
		return indices[hand];
	} else if(indices.size() == 1){ // only 1 controller
		return indices[0]; // ignore hand
	}
	return 0; // always return valid index (for accessing mControllers)

	//return mImpl->GetTrackedDeviceIndexForControllerRole(toOVRControllerRole(hand));
}

VRSystem::VRSystem(int flags)
:	 mFlags(flags)
{
	mParentPose.identity();
	for(auto& v : mTrackedDevices){
		v.pose.identity();
		v.posePrev.identity();
	}
	mViewHMD.identity();
	for(auto& v : mView) v.identity();
	for(auto& v : mEyeToScreen) v.identity();
	for(auto& v : mHeadToEye) v.identity();
	for(auto& v : mEyeToHead) v.identity();
	init(); // it's probably okay to call this here
}

VRSystem::~VRSystem(){
	shutdown();
}

bool VRSystem::init(){

	// Already inited?
	if(mImpl != NULL) return true;

	// Load VR Runtime
	{
		auto err = vr::VRInitError_None;
		mImpl = vr::VR_Init(&err, vr::VRApplication_Scene);

		if(err != vr::VRInitError_None){
			mImpl = NULL;
			DPRINTF("Unable to init VR runtime: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err));
			return false;
		}

		// This doesn't work!
		/*vr::TrackedDeviceIndex_t devHMD;
		if(mImpl->GetSortedTrackedDeviceIndicesOfClass(vr::TrackedDeviceClass_HMD, &devHMD, 1)){
			mDevIdxHMD = devHMD;
		}*/
		for(unsigned i=0; i<vr::k_unMaxTrackedDeviceCount; ++i){
			if(vr::TrackedDeviceClass_HMD == mImpl->GetTrackedDeviceClass(i)){
				mDevIdxHMD = i;
				break;
			}
		}

		/*
		auto hiddenAreaMesh = mImpl->GetHiddenAreaMesh(vr::Eye_Left);
	
		if(NULL != hiddenAreaMesh.pVertexData){
		}
		else{
			DPRINTF("Unable to obtain hidden area mesh\n");
		}
		//*/

		/*struct HiddenAreaMesh_t{
			const HmdVector2_t *pVertexData;
			uint32_t unTriangleCount;
		};

		struct HmdVector2_t{ float v[2]; };
		*/

		if(!vr::VRCompositor()){ // Needed to get poses
			DPRINTF("Failed to initialize VR Compositor!\n");
			return false;
		}
		
		if(!vr::VRSettings()){
			DPRINTF("Failed to initialize VR Settings!\n");
			return false;
		}

		//DPRINTF("VR runtime path: %s\n", vr::VR_RuntimePath());
	}
	
	{
		//auto err = vr::VRSettingsError_None;

		// Turn off chaperone display
		//vr::VRSettings()->SetString(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_PlayAreaColor_String, "#00000000");
		//vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_DisplayDebug_Bool, false);

		//float ss = vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_RenderTargetMultiplier_Float, -1);
		//DPRINTF("Supersampling %f\n", ss);

		// Supersampling
		//vr::VRSettings()->SetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_RenderTargetMultiplier_Float, 1.);

		//vr::VRSettings()->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool, false);
		//vr::VRSettings()->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool, false);
		//vr::VRSettings()->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, false);

		/* Hide collision bounds
		int boundsAlpha = vr::VRSettings()->GetInt32(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_ColorGammaA_Int32);
		//printf("Collision bounds alpha is %d\n", boundsAlpha);
		//vr::VRSettings()->SetInt32(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_ColorGammaA_Int32, 0);
		//*/

		/* Disables chaperone just for this app; does not modify global settings!
		vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseSeated);
		*/

		// Allow asynchronous reprojection
		//vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_AllowAsyncReprojection_Bool, true);

		// Apply settings and save to steamvr.vrsettings (for Windows usually in c:\Program Files (x86)\Steam\config)
		//vr::VRSettings()->Sync();
	}

	/*
	if(vr::VRChaperoneSetup()){
		vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(0.01,0.01);
	}//*/

	renderSize(mRenderWidth, mRenderHeight);

	//DPRINTF("OpenVR successfully initialized\n");
	return true;
}

bool VRSystem::gpuCreate(){

	if(!valid()) return false;

	renderSize(mRenderWidth, mRenderHeight); // in case called before init

	//DPRINTF("Using render target size %d x %d\n", mRenderWidth, mRenderHeight);
	if(!mFBOLeft.create(mRenderWidth, mRenderHeight)){
		printf("%s - Unable to create left FBO @ %d x %d\n", __FUNCTION__, mRenderWidth, mRenderHeight);
		return false;
	}
	if(!mFBORight.create(mRenderWidth, mRenderHeight)){
		printf("%s - Unable to create right FBO @ %d x %d\n", __FUNCTION__, mRenderWidth, mRenderHeight);
		mFBOLeft.destroy();
		return false;
	}

	// Setup matrices
	/*for(int i=0; i<2; ++i){
		mEyeToScreen[i] = eyeToScreen(i);
		mHeadToEye[i] = headToEye(i);
	}*/
	//DPRINTF("Set eye projection matrices\n");

	return true;
}

VRSystem& VRSystem::renderSize(unsigned w, unsigned h, double mult){
	if(w==0 || h==0){
		if(valid()){
			// Note that the recommended resolution is 1.4x native in each dimension
			mImpl->GetRecommendedRenderTargetSize(&mRenderWidth, &mRenderHeight);
			//DPRINTF("Recommended render target size is %d x %d\n", mRenderWidth, mRenderHeight);
			mRenderWidth  = mRenderWidth *mult + 0.5;
			mRenderHeight = mRenderHeight*mult + 0.5;
			//mRenderWidth = 1080; mRenderHeight = 1200; // Vive 1:1 render ratio to improve performance
			//mRenderWidth = 1440; mRenderHeight = 1600; // VivePro 1:1 render ratio to improve performance
		}
	} else {
		mRenderWidth  = w*mult + 0.5;
		mRenderHeight = h*mult + 0.5;
	}
	//DPRINTF("%s - set render size to %d x %d\n", __FUNCTION__, mRenderWidth, mRenderHeight);
	return *this;
}

void VRSystem::gpuDestroy(){
	//if(!valid()) return;
	mFBOLeft.destroy();
	mFBORight.destroy();
}

void VRSystem::shutdown(){
	if(valid()){
		stopCamera();
		vr::VR_Shutdown();
		mImpl = NULL;
	}
}

float VRSystem::frameRate() const {
	if(valid() && mDevIdxHMD>=0){
		vr::ETrackedPropertyError err;
		auto v = mImpl->GetFloatTrackedDeviceProperty(mDevIdxHMD, vr::Prop_DisplayFrequency_Float, &err);
		if(vr::TrackedProp_Success == err) return v;
		//printf("%d\n", err);
	}
	return 90.f; // TODO: let user specify this
}

void VRSystem::updateVigMesh(){
	static const int N = 24; // must be less than 85
	static float circle[N+N];
	static bool genCircle = true;
	if(genCircle){ genCircle = false;
		for(int i=0; i<N; ++i){
			float t = float(i)/N * 6.283185307;
			circle[2*i  ] = std::cos(t);
			circle[2*i+1] = std::sin(t);
		}
	}

	mVigPos.clear();
	mVigCol.clear();
	mVigInd.clear();

	float r1=mVigRad, r2=mVigRad+mVigFade, r3=2.;

	for(int i=0; i<N; ++i){
		auto cs = circle[2*i  ];
		auto sn = circle[2*i+1];
		mVigPos.push_back(r1*cs); mVigPos.push_back(r1*sn); // inner
		mVigPos.push_back(r2*cs); mVigPos.push_back(r2*sn); // middle
		mVigPos.push_back(r3*cs); mVigPos.push_back(r3*sn); // outer
		//for(int k=0;k<3;++k) mVigCol.push_back(255); // for multiplicative (not used)
		//for(int k=0;k<3;++k) mVigCol.push_back(  0);
		//for(int k=0;k<3;++k) mVigCol.push_back(  0);
		for(int k=0;k<3;++k) mVigCol.push_back(mBackground[k]); mVigCol.push_back(000);
		for(int k=0;k<3;++k) mVigCol.push_back(mBackground[k]); mVigCol.push_back(255);
		for(int k=0;k<3;++k) mVigCol.push_back(mBackground[k]); mVigCol.push_back(255);
	}

	auto addInd = [this](int i, int j){ mVigInd.push_back(i); mVigInd.push_back(j); };
	for(int i=0; i<N; ++i){ addInd(3*i, 3*i+1); }
	addInd(0, 1);
	addInd(1, 1); // degenerate
	for(int i=0; i<N; ++i){ addInd(3*i+1, 3*i+2); }
	addInd(1, 2);
}

void VRSystem::render(std::function<void (void)> userDraw){
	if(!active()){ // no VR, just call draw function with current state
		userDraw();
		return;
	}

	if(!mFBOLeft.valid()) gpuCreate(); // Ensure FBOs are created

	bool updatePosesBeforeRender = false;

	// In the comment for WaitGetPoses, it says to call at the last minute before rendering. This does appear to work best in practice, however, any poses used before this call are one frame behind the ones used for render. The OpenVR example updates the poses after present to fix the delay, but calling WaitGetPoses after render introduces jitter.
	if(updatePosesBeforeRender || mFirstRender) updatePoses();

	pushViewport(); // Push current viewport since it's global!
	glDisable(GL_SCISSOR_TEST);

	/*
	vr::Compositor_FrameTiming timing;
	timing.m_nSize = sizeof(timing);
	if(vr::VRCompositor()->GetFrameTiming(&timing)){
		printf("frame presents: %u\n", timing.m_nNumFramePresents);
	}//*/

	auto renderEye = [this, &userDraw](int eye, const FBO& fbo){
		mEyePass = eye;
		#ifdef MULTISAMPLING
			glEnable(GL_MULTISAMPLE);
			glBindFramebuffer(GL_FRAMEBUFFER, fbo.mRenderBuf);
		#else
			glBindFramebuffer(GL_FRAMEBUFFER, fbo.mResolveBuf);
		#endif
		//printGLError("glBindFramebuffer in render");
		glViewport(0, 0, mRenderWidth, mRenderHeight);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		// 
		if(mHiddenAreaMask && !(mLeftPresent && (LEFT==mEyePass))){
			//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); // for no color write
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			/* Note: the OpenVR mask seems to be rather conservative
			static const float mv[] = {2,0,0,0, 0,2,0,0, 0,0,2,0, -1,-1,-1,1};
			glLoadMatrixf(mv);
			auto hiddenAreaMesh = mImpl->GetHiddenAreaMesh(mEyePass);
			if(NULL != hiddenAreaMesh.pVertexData){
				//DPRINTF("%d\n", hiddenAreaMesh.unTriangleCount);
				if(0){
					for(int i=0; i<hiddenAreaMesh.unTriangleCount; ++i){
						const auto& a = hiddenAreaMesh.pVertexData[i*3];
						const auto& b = hiddenAreaMesh.pVertexData[i*3+1];
						const auto& c = hiddenAreaMesh.pVertexData[i*3+2];
						printf("[%2d]: (%f %f) (%f %f) (%f %f)\n", i, a.v[0], a.v[1], b.v[0], b.v[1], c.v[0], c.v[1]);
					}
				}
				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer(2, GL_FLOAT, 0, (const GLvoid *)hiddenAreaMesh.pVertexData);
				glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); glColor4ub(255,0,0,255); // red mask debug
				glDrawArrays(GL_TRIANGLES, 0, 3*hiddenAreaMesh.unTriangleCount);
				glDisableClientState(GL_VERTEX_ARRAY);
			}//*/
			/*struct HiddenAreaMesh_t{
				const HmdVector2_t *pVertexData;
				uint32_t unTriangleCount;
			};
			struct HmdVector2_t{ float v[2]; };*/

			//* Hidden area mask
			//static const float R = 1.14; // mask radius, will depend on lens
			static const float R = 1.; // mask radius, will depend on lens
			static const char ellipse[] = {93,0, 127,0, 97,27, 127,34, 89,51, 127,73, 71,73, 127,127, 47,89, 73,127, 19,99, 34,127, -10,103, 0,127, -40,99, -34,127, -67,89, -73,127, -91,73, -127,127, -109,51, -127,73, -121,27, -127,34, -124,0, -127,0, -121,-27, -127,-34, -109,-51, -127,-73, -91,-73, -127,-127, -67,-89, -73,-127, -40,-99, -34,-127, -10,-103, 0,-127, 19,-99, 34,-127, 47,-89, 73,-127, 71,-73, 127,-127, 89,-51, 127,-73, 97,-27, 127,-34, 93,0, 127,0};
			
			static const char rl=-117, rr=97, rb=-94, rt=114; // Vive Pro w/ min lens-to-eye
			static const char rect[] = {rl,rb, -127,-127, rr,rb, 127,-127, rr,rt, 127,127, rl,rt, -127,127, rl,rb, -127,-127};
			
			static const float s = R/127.;
			static const float mv[] = {
				 s,0,0,0, 0,s,0,0, 0,0,s,0, 0,0,-1,1,
				-s,0,0,0, 0,s,0,0, 0,0,s,0, 0,0,-1,1,
			};

			const auto * verts = ellipse;
			int vertBytes = sizeof(ellipse);
			switch(mMaskShape){
			case RECT: verts=rect; vertBytes=sizeof(rect); break;
			}

			glLoadMatrixf(mv + (LEFT==mEyePass ? 0 : 16));
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(2, GL_BYTE, 0, (const GLvoid *)verts);
			glColor4ub(mBackground[0],mBackground[1],mBackground[2],255);
			//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); glColor4ub(255,0,0,255); // red mask debug
			glDrawArrays(GL_TRIANGLE_STRIP, 0, vertBytes/sizeof(verts[0])/2);
			glDisableClientState(GL_VERTEX_ARRAY);
			//*/

			glPopMatrix();
			//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); // for no color write
		}

		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
			// Apply view here so we don't have to pre-multiply the modelview which req's a fetch.
			// This will only mess up the deprecated gl_* matrix built-ins in GLSL.
			glLoadMatrixf((projection() * view()).data());
			//glLoadMatrixf(projection().get());
		glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			/* Pre-multiply modelview by HMD view
			Matrix4 mv;
			glGetFloatv(GL_MODELVIEW_MATRIX, mv.get());
			glLoadMatrixf((viewHMD() * mv).get());
			//*/
			if(mOverrideFixedModelView) glLoadIdentity();
			userDraw();

			if(mVigRad < 1.8){ // exact threshold will depend on lens
				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				float dx = eyeToHead()[12]*2.;
				float mv[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, dx,0,0,1};
				glLoadMatrixf(mv);
				glDepthMask(GL_FALSE);
				glEnable(GL_BLEND);
				glBlendEquation(GL_FUNC_ADD);
				//glBlendFunc(GL_DST_COLOR, GL_ZERO); // multiplicative
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // transparent (so we can blend custom color)

				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer(2, GL_FLOAT, 0, (const GLvoid *)(&mVigPos[0]));
				glEnableClientState(GL_COLOR_ARRAY);
				//glColorPointer(3, GL_UNSIGNED_BYTE, 0, (const GLvoid *)(&mVigCol[0]));
				glColorPointer(4, GL_UNSIGNED_BYTE, 0, (const GLvoid *)(&mVigCol[0]));
				glDrawElements(GL_TRIANGLE_STRIP, mVigInd.size(), GL_UNSIGNED_BYTE, (const GLvoid *)(&mVigInd[0]));
				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_COLOR_ARRAY);
				glDepthMask(GL_TRUE);
				glDisable(GL_BLEND);
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(GL_MODELVIEW);
				glPopMatrix();
			}

			glPopMatrix();
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		#ifdef MULTISAMPLING
			glDisable(GL_MULTISAMPLE);
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo.mRenderBuf);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo.mResolveBuf);
			glBlitFramebuffer(0, 0, mRenderWidth, mRenderHeight, 0, 0, mRenderWidth, mRenderHeight, 
				GL_COLOR_BUFFER_BIT,
				GL_LINEAR);
			glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		#endif
	};

	renderEye(LEFT , mFBOLeft );
	renderEye(RIGHT, mFBORight);

	//glEnable(GL_SCISSOR_TEST);
	popViewport();

	// Send render textures over to HMD
	auto sendTexToHMD = [this](int eye, const FBO& fbo){
		//auto colorSpace = vr::ColorSpace_Auto;
		auto colorSpace = vr::ColorSpace_Gamma;
		//auto colorSpace = vr::ColorSpace_Linear;
		vr::Texture_t eyeTex = {(void*)fbo.mResolveTex, vr::TextureType_OpenGL, colorSpace};
		vr::VRTextureBounds_t texBounds = {0,0,1,1}; // umin, vmin, umax, vmax
		if(vr::VRCompositorError_None != vr::VRCompositor()->Submit(toOVREye(eye), &eyeTex, &texBounds)){
			DPRINTF("error submitting eye texture to HMD\n");
		}
	};

	sendTexToHMD(LEFT , mFBOLeft );
	sendTexToHMD(RIGHT, mFBORight);
	//printGLError("sendTexToHMD"); // FIXME: throwing GL error "GL_INVALID_OPERATION" here

	// vr::IVRCompositor::Submit recommends to call glFlush after submitting both eyes

	// Tell the compositor it is free to start its rendering work
	// Necessary???
	//vr::VRCompositor()->PostPresentHandoff();

	// OpenVR HelloVR example did this, but it doesn't seem necessary
	//glClearColor(0, 0, 0, 1);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glFlush();
	//glFinish();

	// OpenVR example updates poses _after_ render commands, however this introduces tons of jitter.
	// https://github.com/ValveSoftware/openvr/blob/5aa6c5f0f6520c59c4dce124541ecc62604fd7a5/samples/hellovr_opengl/hellovr_opengl_main.cpp#L847
	if(!updatePosesBeforeRender) updatePoses();
	
	mFirstRender = false;
}

void VRSystem::drawTexture(unsigned tex, float sx, float sy, float ax, float ay) const {
	//printf("VRSystem::drawTexture(%d)\n", tex);
	//GLint vp[4];
	//glGetIntegerv(GL_VIEWPORT, vp);
	//drawDistortion(vp[2], vp[3], vp[0], vp[1]);
	//glViewport(x,y, w,h);
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glDepthMask(GL_FALSE);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	//glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	//glPixelStorei(GL_PACK_ALIGNMENT, 4);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	float l = -1.f + 2.f*ax;
	float b = -1.f + 2.f*ay;
	float r = l + 2.f*sx;
	float t = b + 2.f*sy;
	float quadVerts[] = { l,b, r,b, l,t, r,t };
	float quadTexCoords[] = { 0,0, 1,0, 0,1, 1,1 };
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 0, quadVerts);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, quadTexCoords);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glColor4f(mBright,mBright,mBright,1.f);
	glDrawArrays(GL_TRIANGLE_STRIP,	0, 4);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	//glBindTexture(GL_TEXTURE_2D, 0);
	glDepthMask(GL_TRUE);
	glDisable(GL_TEXTURE_2D);
}

void VRSystem::drawFrameBuffer(int eye, float sx, float sy, float ax, float ay) const {
	if(!active()) return;
	drawTexture(eye==LEFT ? mFBOLeft.mResolveTex : mFBORight.mResolveTex, sx,sy, ax,ay);
}

VRSystem& VRSystem::eyeDistScale(float v){
	mEyeDistScale = v;
	return *this;
}
/*VRSystem& VRSystem::eyeDist(float v){
	float shift = v*0.5;
	if(shift != mEyeShift){
		//printf("Set eye dist to %f\n", v);
		mEyeShift=shift;
		for(int i=0; i<2; ++i) mHeadToEye[i] = headToEye(i);
	}
	return *this;
}*/

VRSystem& VRSystem::near(float v){
	if(v != mNear){
		mNear = v;
		//for(int i=0; i<2; ++i) mEyeToScreen[i] = eyeToScreen(i);
	}
	return *this;
}

VRSystem& VRSystem::far(float v){
	if(v != mFar){
		mFar = v;
		//for(int i=0; i<2; ++i) mEyeToScreen[i] = eyeToScreen(i);
	}
	return *this;
}

const Matrix4& VRSystem::projection(int eye) const {
	return eyeToScreen(eye);// * eyeToHead(eye);
}

const Vec4& VRSystem::eye(int which) const {
	return mEye[which];
}

const Matrix4& VRSystem::eyeToScreen(int eye) const {
	return mEyeToScreen[eye];
}

Matrix4 VRSystem::headToScreen() const {
	auto ans = eyeToScreen(LEFT);
	ans.col(3)[0] = 0;
	return ans;
}

const Matrix4& VRSystem::view(int eye) const {
	return mView[eye];
}

const Matrix4& VRSystem::headToEye(int eye) const {
	return mHeadToEye[eye];
}

const Matrix4& VRSystem::eyeToHead(int eye) const {
	return mEyeToHead[eye];
}


void VRSystem::pushViewport(){
	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);
	for(unsigned i=0; i<4; ++i) mViewport[i] = vp[i];
}

void VRSystem::popViewport(){
	glViewport(mViewport[0], mViewport[1], mViewport[2], mViewport[3]);
}

void VRSystem::updatePoses(){
	if(!valid()) return;

	// Sort generic indices into class specific ones
	vr::ETrackedDeviceClass ovrDevClasses[] = {vr::TrackedDeviceClass_HMD, vr::TrackedDeviceClass_Controller, vr::TrackedDeviceClass_GenericTracker, vr::TrackedDeviceClass_TrackingReference};

	vr::TrackedDeviceIndex_t ovrIndices[MAX_TRACKED_DEVICES];

	for(auto ovrDevClass : ovrDevClasses){
		int numIndices = mImpl->GetSortedTrackedDeviceIndicesOfClass(ovrDevClass, ovrIndices, MAX_TRACKED_DEVICES);
		auto devType = fromOVRDeviceClass(ovrDevClass);
		//printf("Dev type %d has a count of %d\n", devType, numIndices);
		auto& indices = mDeviceIndices[devType];
		indices.clear();
		for(int i=0; i<numIndices; ++i){
			indices.push_back(ovrIndices[i]);
		}
	}

	// Spatially "unsort" device indices
	// GetSortedTrackedDeviceIndicesOfClass sorts devices right-to-left wrt HMD.
	// (Passing in -1 for last arg does not produce unsorted list.)
	for(auto& indices : mDeviceIndices){
		std::sort(indices.begin(), indices.end());
	}

	/*
	printf("====\n");
	for(unsigned i=0; i<sizeof(ovrDevClasses)/sizeof(ovrDevClasses[0]); ++i){
		auto devType = fromOVRDeviceClass(ovrDevClasses[i]);
		printf("%s: ", toString(devType));
		for(auto& i : mDeviceIndices[devType]){
			printf("%d ", i);
		} printf("\n");
	}//*/

	// Get poses of all attached devices
	vr::VRCompositor()->WaitGetPoses(mTrackedDevicePoses, MAX_TRACKED_DEVICES, NULL, 0);

	for(unsigned i=0; i < MAX_TRACKED_DEVICES; ++i){

		auto& dev = mTrackedDevices[i];

		if(mTrackedDevicePoses[i].bPoseIsValid){	
			dev.updatePose(mParentPose * toMatrix4(mTrackedDevicePoses[i].mDeviceToAbsoluteTracking));

			/* Warn about bad pose values (just a sanity check, should never happen)
			for(auto v : dev.pose.m){
				if(std::isnan(v) || std::isinf(v)){
					printf("OpenVR gave bad pose value for device %d!\n", i); break;
				}
			}//*/

			dev.type = fromOVRDeviceClass(mImpl->GetTrackedDeviceClass(i));
			//printf("%s\n", toString(dev.type));

			//if(vr::k_unTrackedDeviceIndex_Hmd == i){
			if(HMD == dev.type){
				//auto mat=mTrackedDevicePoses[i].mDeviceToAbsoluteTracking; printf("%g %g %g\n", mat.m[2][0],mat.m[2][1],mat.m[2][2]);
				//dev.pose.print();
				mViewHMD = dev.pose;
				mViewHMD.invertRigid(); // view matrix is inverse of camera pose
			}
			else if(CONTROLLER == dev.type){
				/*auto& indices = mDeviceIndices[CONTROLLER];
				for(unsigned j=0; j<indices.size(); ++j){
					//printf("TrackedDevice %d against %d\n", i, indices[j]);
					if(indices[j]==i){ // get controller index mapped to this tracked device index
						mControllers[j] = dev;
						//printf("%d %d\n", mControllers[j].valid(), dev.valid());
						break;
					}
				}*/

				mControllers[i] = dev;
			}

		} else {
			dev.type = INVALID;
		}
	}

	// From https://github.com/ValveSoftware/openvr/blob/master/samples/hellovr_opengl/hellovr_opengl_main.cpp :
	// matMVP = m_mat4Projection * m_mat4eyePos * m_mat4HMDPose;
	// where m_mat4eyePos is computed from
	// 		GetEyeToHeadTransform( nEye )^-1
	// and m_mat4HMDPose is actually hmdPose^-1 !!!

	// Update matrices
	for(int i=0; i<2; ++i){
		mEyeToScreen[i] = toMatrix4(mImpl->GetProjectionMatrix(toOVREye(i), mNear, mFar));
		// Note: GetEyeToHeadTransform is really a head to eye translation matrix. From openvr.h:
		/** Returns the transform from eye space to the head space. Eye space is the per-eye flavor of head
		* space that provides stereo disparity. Instead of Model * View * Projection the sequence is Model * View * Eye^-1 * Projection. 
		* Normally View and Eye^-1 will be multiplied together and treated as View in your application. 
		*/
		// Presumably, 'Eye' in above is result of GetEyeToHeadTransform.
		mHeadToEye[i] = toMatrix4(mImpl->GetEyeToHeadTransform(toOVREye(i)));
			//printf("mHeadToEye (eye %d) =\n", eye); mHeadToEye[i].print();
		mHeadToEye[i].pos()[0] *= mEyeDistScale;
		mEye[i].set((poseHMD() * mHeadToEye[i]).pos());
		mEyeToHead[i] = mHeadToEye[i].inverseRigid(); // could be faster, but do this for safety
		mView[i] = mEyeToHead[i] * mViewHMD;
	}

	// Experiments with proj:
		/* Orthographic (flat and close)
		ans.col(0)[0] /= mNear;
		ans.col(1)[1] /= mNear;
		ans.col(3)[0] = -ans.col(2)[0];
		ans.col(3)[1] = -ans.col(2)[1];
		ans.col(2)[0] = 0;
		ans.col(2)[1] = 0;
		std::swap(ans.col(2)[2], ans.col(3)[2]);
		ans.col(2)[2] /= mNear*mFar;
		ans.col(2)[3] = 0;
		ans.col(3)[3] = 1;
		//ans.print();
		//*/

		/* FOV; adds a lot of distortion on edges
		float fov = 1.5;
		ans.col(0)[0] /= fov; // FOV x
		ans.col(1)[1] /= fov; // FOV y
		//*/

		/*
		float depth = 0.75;
		ans.col(2)[0] /= depth;
		ans.col(2)[1] /= depth;
		//ans.col(0)[0] /= depth;
		//ans.col(1)[1] /= depth;
		//*/

		//ans.col(1)[1] *= -1; // swap b and t for upside-down world; nauseating
		//ans.col(3)[3] = 1; // does weird things with depth
		//ans.col(0)[1] = 0.5; // skew/rotation

	// Experiments with eyes:
		//ans.col(1)[1] *= -1; // upside-down; nauseating
		//ans.col(3)[1] = -0.5; // eyes lower on head, weird
		//ans.col(3)[2] = -0.5; // eyes behind head
		//ans.col(0)[0] = ans.col(1)[1] = ans.col(2)[2] = 4; // scale up; clips vertices inside radius
		//ans.col(0)[0] = ans.col(1)[1] = ans.col(2)[2] = 0.1; // scale down; clips vertices outside radius
		/*{ // very bad; crossed-eye after taking off HMD!!!
			float ang = (3 * 3.14159265358979/180) * (LEFT == eye ? 1:-1);
			ans.col(0)[0] = cos(ang);
			ans.col(0)[2] =-sin(ang);
			ans.col(2)[0] = sin(ang);
			ans.col(2)[2] = cos(ang);
		}*/

}

unsigned VRSystem::numTrackedDevice(DeviceType t, unsigned maxNum) const {
	int N = mDeviceIndices[t].size();
	return N < maxNum ? N : maxNum;
}

const VRSystem::Controller& VRSystem::controller(int hand) const {
	return mControllers[controllerIndex(hand)];
}
VRSystem::Controller& VRSystem::controller(int hand){
	return mControllers[controllerIndex(hand)];
}

unsigned VRSystem::numControllers(unsigned maxNum) const {
	return numTrackedDevice(CONTROLLER, maxNum);
}

const VRSystem::TrackedDevice& VRSystem::tracker(int i) const {
	auto& indices = mDeviceIndices[TRACKER];
	if(i < int(indices.size())){
		return mTrackedDevices[indices[i]];
	}
	static TrackedDevice invalidDevice;
	return invalidDevice;
}

unsigned VRSystem::numTrackers(unsigned maxNum) const {
	return numTrackedDevice(TRACKER, maxNum);
}

const Matrix4& VRSystem::poseDevice(int i) const {
	return mTrackedDevices[i].pose;
}

const Matrix4& VRSystem::poseHMD() const {
	return poseDevice(0);
}

const Matrix4& VRSystem::poseController(int hand) const {
	return poseDevice(controllerIndex(hand));
}

bool VRSystem::pollEvent(){ //DPRINTF("\n");
	if(!valid()) return false;

	auto updateControllerState = [this](int devIndex){
		vr::VRControllerState_t state;
		if(mImpl->GetControllerState(devIndex, &state, sizeof(state))){
			if(state.unPacketNum != mControllerStates[devIndex].unPacketNum){
				mControllerStates[devIndex] = state;
				mControllers[devIndex].setButtons(state.ulButtonPressed);
				mControllers[devIndex].setTouches(state.ulButtonTouched);
				for(int i=0; i<Controller::MAX_AXES; ++i){
					mControllers[devIndex].setAxis(i, state.rAxis[i].x, state.rAxis[i].y);
				}

				/* Calc number of clicks
				If time betwee current click and last click is below threshold, then add to number of clicks.
				clickTimer -= dt;
				if(button clicked){
					if(clickTimer < 0){
						clicks = 0;
					} else {
						clicks++;
						clickTimer = threshold;
					}
				}
				*/

				return true;
			}
		}
		return false;
	};

	// Poll discrete events (buttons, touch, status changes, etc.)
	if(mImpl->PollNextEvent(&mVREvent, sizeof(mVREvent))){
		auto devIndex = mVREvent.trackedDeviceIndex;
		auto devClass = mImpl->GetTrackedDeviceClass(devIndex);
		auto type = vr::EVREventType(mVREvent.eventType);

		mEvent.deviceType = fromOVRDeviceClass(devClass);
		mEvent.deviceIndex = devIndex;
		mEvent.age = mVREvent.eventAgeSeconds; // relative age of event
		//VREvent_Data_t data = mVREvent.data;
		//printf("class %d [%d]: %s\n", devClass, devIndex, toString(type));

		switch(type){
		case vr::VREvent_TrackedDeviceActivated:
			mEvent.type = ACTIVATED;
			break;
		case vr::VREvent_TrackedDeviceDeactivated:
			mEvent.type = DEACTIVATED;
			break;
		case vr::VREvent_TrackedDeviceRoleChanged:
			mEvent.type = ROLE_CHANGED;
			break;
		case vr::VREvent_TrackedDeviceUserInteractionStarted:
			mEvent.type = INTERACTION_STARTED;
			break;
		case vr::VREvent_TrackedDeviceUserInteractionEnded:
			mEvent.type = INTERACTION_ENDED;
			break;
		case vr::VREvent_ButtonPress:
			mEvent.type = BUTTON_DOWN;
			mEvent.button = mVREvent.data.controller.button;
			break;
		case vr::VREvent_ButtonUnpress:
			mEvent.type = BUTTON_UP;
			mEvent.button = mVREvent.data.controller.button;
			break;
		case vr::VREvent_ButtonTouch:
			mEvent.type = TOUCH;
			mEvent.button = mVREvent.data.controller.button;
			break;
		case vr::VREvent_ButtonUntouch:
			mEvent.type = UNTOUCH;
			mEvent.button = mVREvent.data.controller.button;
			break;
		default:
			mEvent.type = (decltype(mEvent.type))(type);		
		}

		// Automatic actions for specific devices
		switch(mEvent.deviceType){
		case CONTROLLER:{
			// Ensure local controller state is synced
			/*if(updateControllerState(devIndex)){
				// Ensure the continuous control triggers its own event
				mControllerStates[devIndex].unPacketNum--;
			}*/

			// Set role?
			// TrackedControllerRole_LeftHand, TrackedControllerRole_RightHand, TrackedControllerRole_Invalid
			//auto ctrlRole = mImpl->GetControllerRoleForTrackedDeviceIndex(devIndex);
		} break;
		case HMD:{
			switch(mEvent.type){
			case BUTTON_DOWN: mWearingHMD=true; break;
			case BUTTON_UP: mWearingHMD=false; break;
			default:;
			}
		} break;
		default:;
		}

		return true;
	}

	// If we made it here, we are done processing events...

	// Update hand to controller number table
	// Note this will be behind by one empty of the event queue, but it seems like the only reasonable place to call it.
	for(int i=0; i<2; ++i){
		// Note that GetTrackedDeviceIndexForControllerRole should not be called in performance sensitive sections
		auto deviceIdx = mImpl->GetTrackedDeviceIndexForControllerRole(toOVRControllerRole(i));
		if(vr::k_unTrackedDeviceIndexInvalid != deviceIdx){
			mHandToDevice[i] = deviceIdx;
		}
		//printf("hand %d assigned to device %d\n", i, deviceIdx);
	}

	//DPRINTF("updatePoses\n");
	// FIXME: seems like it needs to be called just before render to avoid stalling
	//updatePoses();

	// Update the local controller states
	for(unsigned i=0; i<vr::k_unMaxTrackedDeviceCount; ++i){
		auto devClass = mImpl->GetTrackedDeviceClass(i);
		if(vr::TrackedDeviceClass_Controller == devClass){

			// Clear changes since they should be one-offs
			mControllers[i].mButtonChanges = 0;
			mControllers[i].mTouchChanges = 0;

			if(updateControllerState(i)){
				// Trigger an axis event---how to get axis number????
				/*mEvent.type = AXIS;
				mEvent.deviceType = convertDeviceClass(devClass);
				mEvent.age = 0;
				//mEvent.axis = 
				mEvent.x = mControllerStates[i].rAxis[0].x;
				mEvent.y = mControllerStates[i].rAxis[0].y;
				//printf("%s [%d]: (% 1.2f, % 1.2f)\n", toString(mEvent.deviceType), i, mEvent.x, mEvent.y);
				return true;*/
			}
		}
	}
	return false;
}

void VRSystem::hapticPulse(int hand, int axisID, unsigned short microSec){
	if(!valid()) return;
	mImpl->TriggerHapticPulse(controllerIndex(hand), axisID - AXIS0, microSec);
}

bool VRSystem::FBO::create(int w, int h){

	glGetError(); // clear any existing errors

	// Only RGBA8 supported! See https://github.com/ValveSoftware/openvr/issues/290
	GLint texelFormat=GL_RGBA8; // original values used by OpenVR
	//GLint texelFormat=GL_RGB32F; // nothing in HMD
	//GLint texelFormat=GL_R11F_G11F_B10F; // wrong in HMD
	//GLint texelFormat=GL_RGB16F; // nothing in HMD
	//GLint texelFormat=GL_RGBA16F; // nothing in HMD
	//GLint texelFormat=GL_RGBA32F; // nothing in HMD
	//GLint texelFormat=GL_RGB10_A2; // wrong in HMD
	//GLint texelFormat=GL_RGBA12; // OK in HMD
	//GLint texelFormat=GL_RGBA16; // nothing in HMD
	//GLint texelFormat=GL_RGB10; // wrong in HMD
	//GLint texelFormat=GL_RGB10; // wrong in HMD
	//GLint texelFormat=GL_RGB12; // wrong in HMD
	//GLint texelFormat=GL_RGB16_SNORM; // nothing in HMD

	#ifdef MULTISAMPLING
	glGenFramebuffers(1, &mRenderBuf);
	glBindFramebuffer(GL_FRAMEBUFFER, mRenderBuf);

		int numSamples = 4;
		glGenRenderbuffers(1, &mDepthBuf);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBuf);
		glRenderbufferStorageMultisample(GL_RENDERBUFFER, numSamples, GL_DEPTH_COMPONENT, w, h);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,	mDepthBuf);

		// Render texture is multisampled to smooth polygon edges
		glGenTextures(1, &mRenderTex);
		glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, mRenderTex);
		glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, numSamples, GL_RGBA8, w, h, true);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, mRenderTex, 0);

		printGLError("GL_TEXTURE_2D_MULTISAMPLE");
	#endif //MULTISAMPLING

	glGenFramebuffers(1, &mResolveBuf);
	glBindFramebuffer(GL_FRAMEBUFFER, mResolveBuf);

		#ifndef MULTISAMPLING
		glGenRenderbuffers(1, &mDepthBuf);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBuf);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,	mDepthBuf);
		#endif

		// Resolve texture is the antialiased texture that we send to the HMD
		glGenTextures(1, &mResolveTex);
		glBindTexture(GL_TEXTURE_2D, mResolveTex);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, texelFormat, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		printGLError("glTexImage2D on resolve tex");
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mResolveTex, 0);
		printGLError("glFramebufferTexture2D on resolve frame buf");

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	printGLError("glCheckFramebufferStatus");
	if(status != GL_FRAMEBUFFER_COMPLETE){
		return false;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return true;
}

void VRSystem::FBO::destroy(){
	if(!mDepthBuf) return;
	glDeleteRenderbuffers(1, &mDepthBuf);
	mDepthBuf = 0; // flags that FBO is destroyed
	glDeleteFramebuffers(1, &mRenderBuf);
	glDeleteTextures(1, &mRenderTex);
	glDeleteFramebuffers(1, &mResolveBuf);
	glDeleteTextures(1, &mResolveTex);
}


bool VRSystem::startCamera(){

	auto cam = vr::VRTrackedCamera();

	// First, perform some checks to see if we have a valid camera
	if(!cam){
		DPRINTF("Unable to get Tracked Camera interface.\n");
		return false;
	}

	auto dev = vr::k_unTrackedDeviceIndex_Hmd;

	bool hasCamera = false;
	auto err = cam->HasCamera(dev, &hasCamera);
	if(vr::VRTrackedCameraError_None != err || !hasCamera){
		DPRINTF("No Tracked Camera available: %s\n", cam->GetCameraErrorNameFromEnum(err));
		return false;
	}

	/*
	vr::ETrackedPropertyError propertyError;
	char buffer[128];
	mImpl->GetStringTrackedDeviceProperty(dev, vr::Prop_CameraFirmwareDescription_String, buffer, sizeof(buffer), &propertyError );
	if(propertyError != vr::TrackedProp_Success){
		DPRINTF("Failed to get tracked camera firmware description.\n");
		return false;
	}
	DPRINTF("Camera firmware: %s\n", buffer);//*/

	auto frameLayout = mImpl->GetInt32TrackedDeviceProperty(dev, vr::Prop_CameraFrameLayout_Int32);
	if(vr::EVRTrackedCameraFrameLayout_Mono == frameLayout & 0xF){
		mFrameType = MONO;
	} else {
		switch(frameLayout & 0xF0){
		case vr::EVRTrackedCameraFrameLayout_VerticalLayout:   mFrameType = STEREO_V; break;
		case vr::EVRTrackedCameraFrameLayout_HorizontalLayout: mFrameType = STEREO_H; break;
		}
	}
	
	mNumCameras = mImpl->GetInt32TrackedDeviceProperty(dev, vr::Prop_NumCameras_Int32);
	//printf("%d\n", frameLayout);
	//printf("%d\n", numCameras);

	// Get camera resolution and set up local frame buffer
	uint32_t camFrameSize;
	err = cam->GetCameraFrameSize(dev, mCameraFrameType, &mCameraWidth, &mCameraHeight, &camFrameSize);
    if(vr::VRTrackedCameraError_None != err){
		DPRINTF("GetCameraFrameBounds() failed: %s\n", cam->GetCameraErrorNameFromEnum(err));
		return false;
	}

	DPRINTF("Camera dimensions: %d x %d x %d\n", mCameraWidth, mCameraHeight, camFrameSize/(mCameraWidth*mCameraHeight));

	for(int i=0; i<mNumCameras; ++i){
		// device, cam num, vr::EVRTrackedCameraFrameType, flZNear, flZFar, vr::HmdMatrix44_t *
		vr::HmdMatrix44_t ovrProj;
		err = cam->GetCameraProjection(dev, i, mCameraFrameType, mNear, mFar, &ovrProj);
		auto proj = toMatrix4(ovrProj);
		mCameraProjs.push_back(proj);
		//proj.print(); printf("\n");
	}

	mCameraFrame.resize(camFrameSize);
	
	err = cam->AcquireVideoStreamingService(dev, &mCamera);
    if(INVALID_TRACKED_CAMERA_HANDLE == mCamera){
		DPRINTF("Failed to start video streaming: %s\n", cam->GetCameraErrorNameFromEnum(err));
		return false;
    }

	return true;
}

void VRSystem::stopCamera(){
	if(INVALID_TRACKED_CAMERA_HANDLE != mCamera){
		vr::VRTrackedCamera()->ReleaseVideoStreamingService(mCamera);
		mCamera = INVALID_TRACKED_CAMERA_HANDLE;
	}
}

bool VRSystem::grabCameraFrame(){
    if(INVALID_TRACKED_CAMERA_HANDLE == mCamera)
		return false;

	auto cam = vr::VRTrackedCamera();

	// Get the frame header only
	auto err = cam->GetVideoStreamFrameBuffer(
		mCamera, mCameraFrameType,
		nullptr, 0,
		&mCameraFrameHeader, sizeof(mCameraFrameHeader)
	);
	if(vr::VRTrackedCameraError_None != err)
		return false;

	if(mCameraFrameHeader.nFrameSequence == mCameraLastSeq){
		// frame hasn't changed yet, nothing to do
		return false;
	}

	// Frame has changed, do the more expensive frame buffer copy
	err = cam->GetVideoStreamFrameBuffer(
		mCamera, mCameraFrameType,
		mCameraFrame.data(), mCameraFrame.size(),
		//&mCameraFrameHeader, sizeof(mCameraFrameHeader)
		nullptr, 0
	);
	if(vr::VRTrackedCameraError_None != err)
		return false;

	mCameraLastSeq = mCameraFrameHeader.nFrameSequence;

	return true;
}

void VRSystem::print() const {
	printf("[VR] %d x %d @ %g Hz, near: %g, far: %g\n", renderWidth(), renderHeight(), frameRate(), near(), far());
}

Matrix4 toMatrix4(const vr::HmdMatrix34_t& mat){
	return Matrix4{{ // appears transposed since Matrix4 data is column-major
		mat.m[0][0], mat.m[1][0], mat.m[2][0], 0.f,
		mat.m[0][1], mat.m[1][1], mat.m[2][1], 0.f,
		mat.m[0][2], mat.m[1][2], mat.m[2][2], 0.f,
		mat.m[0][3], mat.m[1][3], mat.m[2][3], 1.f
	}};
}

Matrix4 toMatrix4(const vr::HmdMatrix44_t& mat){
	return Matrix4{{ // appears transposed since Matrix4 data is column-major
		mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
		mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
		mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
		mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
	}};
}

const char * toString(vr::EVREventType v){
	return vr::VRSystem()->GetEventTypeNameFromEnum(v);
}

#define CS(x) case VRSystem::x: return #x;
const char * toString(VRSystem::EventType v){
	switch(v){
		CS(ACTIVATED) CS(DEACTIVATED) CS(ROLE_CHANGED)
		CS(INTERACTION_STARTED) CS(INTERACTION_ENDED)
		CS(STANDBY_STARTED) CS(STANDBY_ENDED)
		CS(BUTTON_DOWN) CS(BUTTON_UP) CS(TOUCH) CS(UNTOUCH)
		default: return toString(vr::EVREventType(v));
	}
}

const char * toString(VRSystem::DeviceType v){
	switch(v){
		CS(INVALID) CS(HMD) CS(CONTROLLER) CS(TRACKER) CS(TRACKING_REFERENCE)
		default: return "";
	}
}
#undef CS
