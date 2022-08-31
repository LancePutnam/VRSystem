#ifndef VRSYSTEM_HPP_INC
#define VRSYSTEM_HPP_INC

#include <functional>
#include <type_traits> // is_same
#include <vector>
	#if defined(__MSYS__) || defined(__MINGW32__)
	// MinGW/MinGW-w64 compatible header courtesy of:
	// https://gist.github.com/tunabrain/84a3f234d4baa6d438b642f14522999c
	// https://github.com/ValveSoftware/openvr/issues/133
	#include <openvr_mingw.hpp>
#else
	#include <openvr.h>
#endif


/// VR headset and controller I/O
class VRSystem{
public:

	enum{
		MAX_TRACKED_DEVICES = vr::k_unMaxTrackedDeviceCount
	};

	enum{
		USE_DISPLAY	= 1<<0,
		USE_CAMERA	= 1<<1
	};

	enum{
		LEFT = 0,
		RIGHT = 1
	};

	enum DeviceType{
		INVALID				= 0,
		HMD					= 1,
		CONTROLLER			= 2,
		TRACKER				= 3,
		TRACKING_REFERENCE	= 4,
		NUM_DEVICE_TYPES
	};

	/// Event types

	/// This is a simplified subset of OpenVR events.
	/// Event types not listed here can be checked against those in EVREventType.
	enum EventType{
		ACTIVATED				= vr::VREvent_TrackedDeviceActivated,
		DEACTIVATED				= vr::VREvent_TrackedDeviceDeactivated,
		ROLE_CHANGED			= vr::VREvent_TrackedDeviceRoleChanged,
		INTERACTION_STARTED		= vr::VREvent_TrackedDeviceUserInteractionStarted,
		INTERACTION_ENDED		= vr::VREvent_TrackedDeviceUserInteractionEnded,
		STANDBY_STARTED			= vr::VREvent_EnterStandbyMode,
		STANDBY_ENDED			= vr::VREvent_LeaveStandbyMode,

		BUTTON_DOWN				= vr::VREvent_ButtonPress,
		BUTTON_UP				= vr::VREvent_ButtonUnpress,
		TOUCH					= vr::VREvent_ButtonTouch,
		UNTOUCH					= vr::VREvent_ButtonUntouch
	};

	enum Button{
		SYSTEM = 0,
		MENU = 1,
		GRIP = 2,
		AXIS0 = 32,
		AXIS1 = 33,
		NO_BUTTON,

		TOUCHPAD = AXIS0,
		TRIGGER = AXIS1
	};

	enum Shape{
		ELLIPSE,
		RECT
	};

	class Matrix4;

	struct Vec4{
		typedef float value_type;
		float x, y, z, w;
		Vec4(const Vec4& v){ *this = v; }
		Vec4(float xi=0, float yi=0, float zi=0, float wi=0): x(xi), y(yi), z(zi), w(wi){}
		Vec4(const float * src){ set(src); }
		float * data(){ return &x; }
		const float * data() const { return &x; }
		template <class T> const T& as() const {
			static_assert(sizeof(T)<=sizeof(*this),"Size mismatch");
			static_assert(std::is_same<typename T::value_type,value_type>::value,"Type mismatch");
			return *((T*)this);
		}
		Vec4& operator= (const Vec4& v){ x=v.x; y=v.y; z=v.z; w=v.w; return *this; }
		float& operator[] (unsigned i){ return (&x)[i]; }
		const float& operator[] (unsigned i) const { return (&x)[i]; }
		void set(const float * src){ for(int i=0;i<4;++i) (*this)[i]=src[i]; }
		float dot(const Vec4& v) const { return x*v.x + y*v.y + z*v.z + w*v.w; }
		Vec4& operator+=(const Vec4& v){ x+=v.x; y+=v.y; z+=v.z; w+=v.w; return *this; }
		Vec4  operator+ (const Vec4& v) const { return Vec4(*this)+=v; }
		Vec4& operator*=(float s){ x*=s; y*=s; z*=s; w*=s; return *this; }
		Vec4  operator* (float s) const { return Vec4(*this)*=s; }
		Vec4  operator* (const Matrix4& m) const;
	};


	/// A 4-by-4 matrix (used to represent a pose)

	/// This assumes a right-handed coordinate system:
	///		+x is right
	///		+y is up
	///		+z is backwards
	struct Matrix4{
		typedef float value_type;

		float m[16]; // Elements, in column-major order

		template <class T>
		Matrix4& set(const T * src){
			for(unsigned i=0; i<16; ++i) m[i] = src[i];
			return *this;
		}

		template <class T> const T& pun() const { return *(T*)m; }
		template <class T> T& pun(){ return *(T*)m; }
		float * data(){ return m; }
		const float * data() const { return m; }
		template <class T> const T& as() const {
			static_assert(sizeof(T)<=sizeof(*this),"Size mismatch");
			static_assert(std::is_same<typename T::value_type,value_type>::value,"Type mismatch");
			return *((T*)this);
		}
		float& operator[] (unsigned i){ return m[i]; }
		const float& operator[] (unsigned i) const { return m[i]; }
		/// Get column vector
		Vec4& col(int i){ return *(Vec4*)(m + 4*i); }
		const Vec4& col(int i) const { return const_cast<Matrix4*>(this)->col(i); }
		template <unsigned i> Vec4& col(){
			static_assert(i<4, "Invalid column");
			constexpr auto j = 4*i;
			return *(Vec4*)(m + j);
		}
		template <unsigned i> const Vec4& col() const {
			return const_cast<Matrix4*>(this)->col<i>(); 
		}
		/// Get row vector
		Vec4 row(int i) const { return Vec4(m[i],m[i+4],m[i+8],m[i+12]); }
		template <unsigned i> Vec4 row() const {
			static_assert(i<4, "Invalid row");
			constexpr auto i2 = i+ 4;
			constexpr auto i3 = i+ 8;
			constexpr auto i4 = i+12;
			return Vec4(m[i],m[i2],m[i3],m[i4]);
		}
		/// Get local direction vector along x axis
		Vec4& ux(){ return col<0>(); }
		const Vec4& ux() const { return col<0>(); }
		/// Get local direction vector along y axis
		Vec4& uy(){ return col<1>(); }
		const Vec4& uy() const { return col<1>(); }
		/// Get local direction vector along z axis
		Vec4& uz(){ return col<2>(); }
		const Vec4& uz() const { return col<2>(); }
		/// Get position/translation amount
		Vec4& pos(){ return col<3>(); }
		const Vec4& pos() const { return col<3>(); }

		Matrix4& identity(){
			m[0]=1; m[4]=0; m[ 8]=0; m[12]=0;
			m[1]=0; m[5]=1; m[ 9]=0; m[13]=0;
			m[2]=0; m[6]=0; m[10]=1; m[14]=0;
			m[3]=0; m[7]=0; m[11]=0; m[15]=1;
			return *this;
		}

		Matrix4 operator* (const Matrix4& n) const {
			return Matrix4{{
				m[0]*n[ 0] + m[4]*n[ 1] + m[8]*n[ 2] + m[12]*n[ 3], m[1]*n[ 0] + m[5]*n[ 1] + m[9]*n[ 2] + m[13]*n[ 3], m[2]*n[ 0] + m[6]*n[ 1] + m[10]*n[ 2] + m[14]*n[ 3], m[3]*n[ 0] + m[7]*n[ 1] + m[11]*n[ 2] + m[15]*n[ 3],
				m[0]*n[ 4] + m[4]*n[ 5] + m[8]*n[ 6] + m[12]*n[ 7], m[1]*n[ 4] + m[5]*n[ 5] + m[9]*n[ 6] + m[13]*n[ 7], m[2]*n[ 4] + m[6]*n[ 5] + m[10]*n[ 6] + m[14]*n[ 7], m[3]*n[ 4] + m[7]*n[ 5] + m[11]*n[ 6] + m[15]*n[ 7],
				m[0]*n[ 8] + m[4]*n[ 9] + m[8]*n[10] + m[12]*n[11], m[1]*n[ 8] + m[5]*n[ 9] + m[9]*n[10] + m[13]*n[11], m[2]*n[ 8] + m[6]*n[ 9] + m[10]*n[10] + m[14]*n[11], m[3]*n[ 8] + m[7]*n[ 9] + m[11]*n[10] + m[15]*n[11],
				m[0]*n[12] + m[4]*n[13] + m[8]*n[14] + m[12]*n[15], m[1]*n[12] + m[5]*n[13] + m[9]*n[14] + m[13]*n[15], m[2]*n[12] + m[6]*n[13] + m[10]*n[14] + m[14]*n[15], m[3]*n[12] + m[7]*n[13] + m[11]*n[14] + m[15]*n[15]
			}};
		}

		Vec4 operator* (const Vec4& v) const {
			return Vec4(row<0>().dot(v), row<1>().dot(v), row<2>().dot(v), row<3>().dot(v));
		}

		Matrix4& transpose(){
			for(auto i:{1,6,11}) std::swap(m[i], m[i+3]);
			for(auto i:{2,7   }) std::swap(m[i], m[i+6]);
			for(auto i:{3     }) std::swap(m[i], m[i+9]);
			return *this;
		}

		Matrix4& invertRigid(){
			// Given A = T * R, A^-1 = R^-1 * T^-1
			// R^-1 = R^T; transpose rotation part to invert it
			std::swap(m[1],m[4]);
			std::swap(m[2],m[8]);
			std::swap(m[6],m[9]);

			// compute R^-1 * T^-1
			auto itx = m[ 0]*-m[12] + m[ 4]*-m[13] + m[ 8]*-m[14];
			auto ity = m[ 1]*-m[12] + m[ 5]*-m[13] + m[ 9]*-m[14];
			auto itz = m[ 2]*-m[12] + m[ 6]*-m[13] + m[10]*-m[14];

			m[12]=itx;
			m[13]=ity;
			m[14]=itz;
			return *this;
		}

		Matrix4 inverseRigid() const {
			return Matrix4(*this).invertRigid();
		}

		Matrix4& invertOrthogonal(){
			// Given A = T * R * S, A^-1 = S^-1 * R^-1 * T^-1
			auto is = 1./(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
			for(int i:{0,1,2, 4,5,6, 8,9,10}) m[i] *= is;
			return invertRigid();
		}

		/// Translate in world space by tx*ux + ty*uy + tz*uz
		Matrix4& translate(float tx, float ty, float tz){
			m[12] += tx;
			m[13] += ty;
			m[14] += tz;
			return *this;
		}

		template <class Vec3>
		Matrix4& translate(const Vec3& t){ return translate(t[0],t[1],t[2]); }

		/// Translate in local space tx*ux + ty*uy + tz*uz
		Matrix4& shift(float tx, float ty, float tz){
			return translate(
				tx*col(0)[0] + ty*col(1)[0] + tz*col(2)[0],
				tx*col(0)[1] + ty*col(1)[1] + tz*col(2)[1],
				tx*col(0)[2] + ty*col(1)[2] + tz*col(2)[2]);
		}
		Matrix4 shifted(float tx, float ty, float tz) const {
			return Matrix4(*this).shift(tx,ty,tz);
		}

		template <class Vec3>
		Matrix4& shift(const Vec3& t){ return shift(t[0],t[1],t[2]); }

		void print() const;
	};


	struct Event{
		EventType type;
		DeviceType deviceType;
		int deviceIndex;
		float age;
		/*union{
			unsigned button;	///< Button number
			struct{unsigned axis; float x,y;};	///< Axis number and coordinates
		};*/
		unsigned button;	///< Button number
		float x,y;			///< Axis coordinates

		void print() const;
	};


	struct TrackedDevice{
		DeviceType type = INVALID;
		Matrix4 pose;		///< Virtual world pose (can have transform parent)
		Matrix4 posePrev;
		Matrix4 poseAbs;	///< Absolute (tracking space) pose
		bool valid() const { return INVALID!=type; }
		void updatePose(const Matrix4& v){ posePrev=pose; pose=v; poseAbs=v; }
		void updatePose(const Matrix4& v, const Matrix4& parent){ posePrev=pose; pose=parent*v; poseAbs=v; }
		/// Get pose differential
		Matrix4 poseDiff() const { return pose * posePrev.inverseRigid(); }
	};


	class Controller : public TrackedDevice {
	public:

		enum{ MAX_AXES = 5 };


		/// Get button state
		bool button(unsigned i) const { return state(mButtons,i); }

		/// Get button change status
		bool buttonChanged(unsigned i) const { return state(mButtonChanges,i); }

		/// Get whether button transitioned to down
		bool buttonWentDown(unsigned i) const { return wentDown(mButtons,mButtonChanges,i); }

		/// Get whether button transitioned to up
		bool buttonWentUp(unsigned i) const { return wentUp(mButtons,mButtonChanges,i); }


		/// Get touch state
		bool touch(unsigned i) const { return state(mTouches,i); }

		/// Get touch change status
		bool touchChanged(unsigned i) const { return state(mTouchChanges,i); }

		/// Get whether touch transitioned to down
		bool touchWentDown(unsigned i) const { return wentDown(mTouches,mTouchChanges,i); }

		/// Get whether touch transitioned to up
		bool touchWentUp(unsigned i) const { return wentUp(mTouches,mTouchChanges,i); }


		/// Get an axis state (position)
		const float * axis(Button b) const { return mAxes[b - AXIS0]; }

		template <class Vec2>
		Vec2 axis(Button b) const { return Vec2(axis(b)[0], axis(b)[1]); }

		/// Get an axis change state (velocity)
		const float * axisChange(Button b) const { return mAxisChanges[b - AXIS0]; }

		template <class Vec3>
		Vec3 axisInWorld(Button b, float w=0.) const {
			Vec4 v(axis(b)[0], 0, -axis(b)[1], w);
			v = pose * v;
			return Vec3(v.x, v.y, v.z);
		}

		/// Get circular sector pressed on touchpad

		/// Sector numbers start at (x,y)=(1,0) and increase counter-clockwise
		/// around the unit circle.
		/// \returns the sector number or -1 if the origin was pressed
		int axisSector(Button b, float divs=4., float rotate=0.125) const;

		Controller& operator= (const TrackedDevice& d){
			if(&d != this) static_cast<TrackedDevice&>(*this) = d;
			return *this;
		}

		/// Print controller state
		void print() const;

		void updateClicks(float dt){
			if(mButtonChanges) mClickTimer = 0.;
			if(mClickTimer < mClickTimeMax){
				if(mButtonChanges){
					for(int i=0; i<NO_BUTTON; ++i){
						if(button(i)) mClickSeq.push_back(i);
					}
				}
			} else { // timer up
				mClickSeqFinished = mClickSeq;
				mClickSeq.clear();
			}
			mClickTimer += dt;
		}

		const std::vector<unsigned char>& clickSeq() const { return mClickSeq; }
		const std::vector<unsigned char>& clickSeqFin() const { return mClickSeqFinished; }

		unsigned clicks(int button){
			unsigned r=0;
			for(auto b : mClickSeqFinished) r += (b==button);
			//mClickSeqFinished.clear(); // handled in updateClicks
			return r;
		}

		int hand() const { return mHand; }

	private:
		friend class VRSystem;

		typedef uint64_t Bits;		static_assert(sizeof(Bits)*8 >= NO_BUTTON,"");
		Bits mButtons = 0;			// Button bit states
		Bits mTouches = 0;			// Touch bit states
		Bits mButtonChanges = 0;	// Button change statuses
		Bits mTouchChanges = 0;		// Touch change statuses
		//unsigned char mClicks[sizeof(Bits)*8] = {0};
		float mAxes[MAX_AXES][2] ={{0}};// Axis coordinates
		float mAxisChanges[MAX_AXES][2] ={{0}};// Axis changes (velocities)
		//Matrix4 mDownPoses[];

		std::vector<unsigned char> mClickSeq, mClickSeqFinished;
		float mClickTimer=0., mClickTimeMax=0.2;
		unsigned char mHand = LEFT;
		

		static bool state(Bits states, unsigned i){
			return bool((Bits(1) << i) & states);
		}
		static bool wentDown(Bits states, Bits changes, unsigned i){
			return state(states & changes, i);
		}
		static bool wentUp(Bits states, Bits changes, unsigned i){
			return state(~states & changes, i);
		}

		void setAxis(int axisNum, float x, float y){
			float newVals[2] = {x,y};
			for(unsigned i=0; i<2; ++i){
				mAxisChanges[axisNum][i] = newVals[i] - mAxes[axisNum][i];
				mAxes[axisNum][i] = newVals[i];
			}
		}

		void setButtons(Bits v){
			mButtonChanges = mButtons ^ v;
			mButtons = v;
		}

		void setTouches(Bits v){
			mTouchChanges = mTouches ^ v;
			mTouches = v;
		}
	};



	VRSystem(int flags=0);

	~VRSystem();

	/// Whether VR initialized
	bool valid() const { return nullptr != mImpl; }

	/// Destroy resources on GPU
	void gpuDestroy();
	
	/// Renders user provided draw call to HMD

	/// The projection matrix will be determined by the HMD while the modelview
	/// is left unchanged.
	void render(std::function<void (void)> userDraw);
	
	/// Whether render is doing the first eye pass
	bool firstEyePass() const { return eyePass() == LEFT; }

	/// Draw rendered scene to current viewport
	
	/// @param[in] eye			eye frame buffer to draw
	/// @param[in] stretchx		stretch factor along x axis
	/// @param[in] stretchy		stretch factor along y axis
	/// @param[in] anchorx		origin of x stretch; [0,1] -> [left, right]
	/// @param[in] anchory		origin of y stretch; [0,1] -> [bottom, top]
	void drawFrameBuffer(int eye=0, float stretchx=1, float stretchy=1, float anchorx=0, float anchory=0) const;

	/// Set brightness of drawFrameBuffer
	VRSystem& drawBrightness(float v){ mBright = v; return *this; }

	/// Get generic tracked device
	TrackedDevice& trackedDevice(int i){ return mTrackedDevices[i]; }
	const TrackedDevice& trackedDevice(int i) const { return mTrackedDevices[i]; }

	/// Get number of active devices of a specific type
	unsigned numTrackedDevice(DeviceType t, unsigned maxNum=0xffffffff) const;

	/// Get controller object for specified hand
	const Controller& controller(int hand) const;
	Controller& controller(int hand);

	/// Get number of active controllers
	unsigned numControllers(unsigned maxNum=0xffffffff) const;

	/// Get tracker

	/// Orientation is relative to back of tracker:
	/// +x text direction
	/// +y towards pins, USB port
	/// +z perp. away from tracker
	const TrackedDevice& tracker(int i) const;

	/// Get number of active trackers
	unsigned numTrackers(unsigned maxNum=0xffffffff) const;

	/// Get next event on event queue
	
	/// \see #event to get the most recently polled event
	/// \returns true while there are more events
	bool pollEvent();

	/// Get last polled event
	const Event& event() const { return mEvent; }


	/// Update all cached poses and associated matrices
	void updatePoses();

	/// Set parent pose (rigid transformation) of all device poses.
	/// This also affects the view matrices.
	/// By default, any fixed pipeline modelview matrix will be overridden by
	/// poseParent * poseHMD. This is necessary if the fixed pipeline view is 
	/// already set to poseParent^-1.
	template <class Mat4>
	VRSystem& poseParent(const Mat4& m, bool overrideFixedModelView = true){
		static_assert(sizeof(m)/sizeof(typename Mat4::value_type) >= 16,
			"Matrix dimensions too small");
		const auto oldParentPose = mParentPose;
		mParentPose.set((const typename Mat4::value_type *)&m);
		mOverrideFixedModelView = true;
		if(!valid()){
			for(auto& dev : mTrackedDevices) dev.updatePose(mParentPose);
			mViewHMD = mParentPose;
			mViewHMD.invertRigid();
			for(auto& v : mView) v = mViewHMD;
		}/* else { // update current poses
			const auto parentPoseDiff = mParentPose * oldParentPose.inverseRigid();
			
			for(auto& dev : mTrackedDevices) dev.updatePose(parentPoseDiff * dev.pose);
			if(mDeviceIndices[HMD].size()){
				mViewHMD = mTrackedDevices[mDeviceIndices[HMD][0]].pose;
				mViewHMD.invertRigid();
			}
			for(auto& v : mView) v = mViewHMD;
		}*/
		return *this;
	}

	/// Get head pose, in world space
	const Matrix4& poseHMD() const;

	/// Get position of HMD
	Vec4 posHMD() const { return Vec4(poseHMD().col(3)); }

	/// Get HMD view (inverse of pose)
	const Matrix4& viewHMD() const { return mViewHMD; }

	/// Get whether the HMD is being worn
	bool wearingHMD() const { return mWearingHMD; }

	/// Get view matrix for specified eye
	const Matrix4& view(int eye) const;
	const Matrix4& view() const { return view(eyePass()); }

	/// Get projection (eye to screen) transform
	const Matrix4& projection(int eye) const;
	const Matrix4& projection() const { return projection(eyePass()); }

	/// Get current eye being rendered (LEFT or RIGHT)
	int eyePass() const { return mEyePass; }

	/// Get eye position
	const Vec4& eye(int which) const;
	const Vec4& eye() const { return eye(eyePass());  } 

	/// Get eye to screen transform
	const Matrix4& eyeToScreen(int eye) const;
	const Matrix4& eyeToScreen() const { return eyeToScreen(eyePass()); }

	/// Get head to screen transform
	Matrix4 headToScreen() const;

	/// Get head to eye transform
	const Matrix4& headToEye(int eye) const;
	const Matrix4& headToEye() const { return headToEye(eyePass()); }

	/// Get eye to head transform
	const Matrix4& eyeToHead(int eye) const;
	const Matrix4& eyeToHead() const { return eyeToHead(eyePass()); }


	/// Set scale amount on eye distance
	VRSystem& eyeDistScale(float v);

	VRSystem& near(float v);
	float near() const { return mNear; }

	VRSystem& far(float v);
	float far() const { return mFar; }

	/// Current frame rate of HMD
	float frameRate() const;

	/// Whether displaying graphics on headset
	bool display() const { return mDisplay; }
	VRSystem& display(bool v){ mDisplay=v; return *this; }
	VRSystem& displayToggle(){ mDisplay^=true; return *this; }

	/// Whether VR is active (initialized and presenting)
	bool active() const { return valid() && display(); }

	/// Whether to apply a hidden area mask pre-render to reduce raster load
	VRSystem& hiddenAreaMask(bool v){ mHiddenAreaMask=v; return *this; }
	bool hiddenAreaMask() const { return mHiddenAreaMask; }
	
	VRSystem& hiddenAreaShape(Shape v){ mMaskShape=v; return *this; }

	/// Background color for hidden area mask and vignette
	template <class RGB>
	VRSystem& background(const RGB& rgb){
		static_assert(sizeof(rgb)/sizeof(rgb[0])>=3,"Requires 3-component RGB");
		mBackground[0] = rgb[0]*255.99;
		mBackground[1] = rgb[1]*255.99;
		mBackground[2] = rgb[2]*255.99;
		return *this;
	}

	/// Set size of vignette used to reduce optical flow and vection
	VRSystem& vignette(float rad, float fade=0.1){
		if(mVigRad != rad || mVigFade != fade){
			mVigRad=rad; mVigFade=fade;
			updateVigMesh();
		}
		return *this;
	}

	VRSystem& leftPresent(bool v){ mLeftPresent=v; return *this; }
	bool leftPresent() const { return mLeftPresent; }

	/// Set frame buffer width and height (must call before rendering)

	/// Good values are the native resolution of the HMD and a scalar of 1.4.
	/// If either value is zero, then a "recommended" value is chosen which may
	/// depend on the native headset resolution and/or GPU performance.
	VRSystem& renderSize(unsigned width, unsigned height, double mult=1.);

	unsigned renderWidth() const { return mRenderWidth; }
	unsigned renderHeight() const { return mRenderHeight; }

	/** Trigger a single haptic pulse on a controller. After this call the application may not trigger another haptic pulse on this controller and axis combination for 5ms. */
	/// Note: At the moment, the HTC Vive only supports TOUCHPAD for the axisID.
	void hapticPulse(int hand, int axisID, unsigned short microSec);

	enum FrameType{
		MONO,			///< Mono
		STEREO_V,		///< Stereo top/bottom (left/right eye)
		STEREO_H		///< Stereo left/right
	};

	bool startCamera();
	void stopCamera();

	/// Grab a frame from the camera

	/// Note that this can take a significant amount of time (~8 ms), so should
	/// be done outside the graphics thread.
	bool grabCameraFrame();

	unsigned cameraWidth() const { return mCameraWidth; }
	unsigned cameraHeight() const { return mCameraHeight; }
	/// Pixel format is RGBA
	const unsigned char * cameraFrame() const { return &mCameraFrame[0]; }
	unsigned numCameras() const { return mNumCameras; }
	FrameType frameType() const { return mFrameType; }
	const Matrix4& cameraProj(int i) const { return mCameraProjs[i]; }

	void print() const;

	vr::IVRSystem& impl(){ return *mImpl; }
	const vr::IVRSystem& impl() const { return *mImpl; }

private:

	vr::IVRSystem * mImpl = nullptr;
	int mFlags = 0;
	TrackedDevice mTrackedDevices[MAX_TRACKED_DEVICES];
	vr::TrackedDevicePose_t mTrackedDevicePoses[MAX_TRACKED_DEVICES];
	std::vector<unsigned> mDeviceIndices[NUM_DEVICE_TYPES];
	int mDevIdxHMD = -1;

	vr::VRControllerState_t mControllerStates[MAX_TRACKED_DEVICES];
	Controller mControllers[MAX_TRACKED_DEVICES];

	int mEyePass = LEFT;
	float mNear = 0.1;
	float mFar = 100;
	float mEyeDistScale = 1.;
	int mViewport[4];
	bool mDisplay = true;
	bool mHiddenAreaMask = true;
	unsigned char mBackground[3] = {0,0,0};
	bool mLeftPresent = false;
	bool mWearingHMD = false;
	bool mOverrideFixedModelView = false;
	bool mFirstRender = true;
	Shape mMaskShape = ELLIPSE;

	Matrix4 mParentPose;
	Matrix4 mViewHMD;
	Matrix4 mView[2];
	Vec4 mEye[2];
	Matrix4 mHeadToEye[2];
	Matrix4 mEyeToHead[2];
	Matrix4 mEyeToScreen[2];
	vr::VREvent_t mVREvent;

	Event mEvent;

	int mHandToDevice[2] = {1,2};

	struct FBO{
		unsigned mDepthBuf = 0;
		unsigned mRenderTex = 0;
		unsigned mRenderBuf = 0;
		unsigned mResolveTex = 0;
		unsigned mResolveBuf = 0;
		bool create(int w, int h);
		void destroy();
		bool valid() const { return mDepthBuf; }
	};

	FBO mFBOLeft;
	FBO mFBORight;
	unsigned mRenderWidth=0, mRenderHeight=0; // 0 == get recommended value

	float mBright = 1.f;

	float mVigRad = 2., mVigFade = 0.1;
	std::vector<float> mVigPos;
	std::vector<unsigned char> mVigCol, mVigInd;
	void updateVigMesh();

	bool init();
	void shutdown();
	bool flag(int v){ return mFlags&v; }

	void pushViewport();
	void popViewport();

	const Matrix4& poseDevice(int i) const;
	int controllerIndex(int hand) const;

	vr::TrackedCameraHandle_t mCamera = INVALID_TRACKED_CAMERA_HANDLE;
	uint32_t mCameraWidth=0, mCameraHeight=0;
	uint32_t mCameraLastSeq=0;
	unsigned mNumCameras = 0;
	FrameType mFrameType = MONO;
	std::vector<Matrix4> mCameraProjs;
	vr::CameraVideoStreamFrameHeader_t mCameraFrameHeader;
	vr::EVRTrackedCameraFrameType mCameraFrameType = vr::VRTrackedCameraFrameType_Undistorted;
	std::vector<unsigned char> mCameraFrame;

	// Create resources on GPU (this is called automatically by render)
	bool gpuCreate();

	// Draws lens-distorted HMD textures to current viewport
	void drawDistortion(int w, int h, int x=0, int y=0);
	void drawDistortion();

public:
	void drawTexture(unsigned tex, float stretchx=1, float stretchy=1, float anchorx=0, float anchory=0) const;

	// Get a controller pose
	[[deprecated]] const Matrix4& poseController(int hand) const;
};

inline VRSystem::Vec4 VRSystem::Vec4::operator* (const VRSystem::Matrix4& m) const {
	return { dot(m.col<0>()), dot(m.col<1>()), dot(m.col<2>()), dot(m.col<3>()) };
}

VRSystem::Matrix4 toMatrix4(const vr::HmdMatrix34_t& m);
VRSystem::Matrix4 toMatrix4(const vr::HmdMatrix44_t& m);
const char * toString(vr::EVREventType v);
const char * toString(VRSystem::EventType v);
const char * toString(VRSystem::DeviceType v);

#endif // include guard
