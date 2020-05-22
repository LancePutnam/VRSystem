#include <stdio.h>
#include <functional>
#include <string>
#include <thread>
#include "ViveSR/SRanipal.h"
#include "ViveSR/SRanipal_Eye.h"
#include "ViveSR/SRanipal_Lip.h"
#include "ViveSR/SRanipal_Enums.h"
#pragma comment (lib, "SRanipal.lib")

#ifndef FACE_TRACKER_SLEEP
#warning FACE_TRACKER_SLEEP not defined. The data query thread will run as fast as possible which is probably not what you want.
#define FACE_TRACKER_SLEEP(sec)
#endif

/*
Coordinate system is right-handed:
+x is right
+y is up
-z is forward
*/

class FaceTracker{
public:

	enum{
		LEFT=0,
		RIGHT
	};

	struct EyeData{
		float gazePos[3] = {0,0, 0}; // normalized origin of gaze
		float gazeDir[3] = {0,0,-1}; // direction vector of gaze
		float openness[2] = {1,1}; // how open the left/right eyes are
		float convergence = 0; // convergence depth of eyes
		float pupilPos[2] = {0,0}; // normalized combined pupil position in sensor area
		float pupilDiam = 0; // combined pupil diameter
		
		bool gazePosValid = false;
		bool gazeDirValid = false;
		bool opennessValid = false;
		bool convergenceValid = false;
		bool pupilPosValid = false;
		bool pupilDiamValid = false;
		
		bool anyDataValid() const { return gazePosValid || gazeDirValid || opennessValid || convergenceValid || pupilPosValid || pupilDiamValid; }
	};


	~FaceTracker(){
		stop();
		release();
	}

	
	const EyeData& eyeData() const { return mEyeData; }

	bool eyeTracking() const { return STATUS_INIT==mEyeTracking; }
	bool lipTracking() const { return STATUS_INIT==mLipTracking; }

	FaceTracker& eyeTracking(bool v){
		if(STATUS_INIT != mEyeTracking){
			mEyeTracking = v ? STATUS_ENABLE : STATUS_DISABLE;
		}
		return *this;
	}

	FaceTracker& lipTracking(bool v){
		if(STATUS_INIT != mLipTracking){
			mLipTracking = v ? STATUS_ENABLE : STATUS_DISABLE;
		}
		return *this;
	}

	FaceTracker& onEyeData(const std::function<void(void)>& f){
		mOnEyeData = f; return *this; }

	FaceTracker& onLipData(const std::function<void(void)>& f){
		mOnLipData = f; return *this; }

	/// Set data query period, in seconds
	FaceTracker& period(float sec){ mPeriod=sec; return *this; }

	float period() const { return mPeriod; }

	bool init(){
		// Note that this will automatically start up the SR_Runtime if it is
		// not already running.
		using namespace ViveSR;

		if(!ViveSR::anipal::Eye::IsViveProEye()){
			printf("Eye tracking not supported on this HMD\n");
			return false;
		}

		if(STATUS_ENABLE == mEyeTracking){
			auto err = anipal::Initial(anipal::Eye::ANIPAL_TYPE_EYE, NULL);
			if (ViveSR::Error::WORK == err){
				mEyeTracking = 2;
				//printf("Successfully initialized eye engine.\n");
			} else if (Error::RUNTIME_NOT_FOUND == err){
				printf("Error: SR_Runtime not found.\n");
				return false;
			} else {
				printf("Error: Failed to initialize Eye engine (ViveSR::Error %d)\n", err);
				return false;
			}
		}
        if(STATUS_ENABLE == mLipTracking){
			auto err = anipal::Initial(anipal::Lip::ANIPAL_TYPE_LIP, NULL);
			if (Error::WORK == err){
				mLipTracking = 2;
				mLipData.image = mLipImage;
				//printf("Successfully initialized lip engine.\n");
			} else if (Error::RUNTIME_NOT_FOUND == err){
				printf("Error: SR_Runtime not found.\n");
				return false;
			} else {
				printf("Error: Failed to initialize Eye engine (ViveSR::Error %d)\n", err);
				return false;
			}
		}
		return true;
	}

	bool start(){
		using namespace ViveSR;

		if(needsInit()){
			auto good = init();
			if(!good) return false;
		}

		if(initGood() && mThread == nullptr){
			mThread = new std::thread([this](){
				while(mRunning){
					if(STATUS_INIT == mEyeTracking){
						anipal::Eye::EyeData srEyeData;
						auto err = anipal::Eye::GetEyeData(&srEyeData);
						if(Error::WORK == err){
							// Defs in SRanipal_EyeDataType.h
							const auto& l = srEyeData.verbose_data.left;
							const auto& r = srEyeData.verbose_data.right;
							const auto& c = srEyeData.verbose_data.combined.eye_data;
							const auto& lvalid = l.eye_data_validata_bit_mask;
							//const auto& rvalid = r.eye_data_validata_bit_mask;
							const auto& cvalid = c.eye_data_validata_bit_mask;

							mEyeData.gazePosValid = anipal::Eye::DecodeBitMask(cvalid, anipal::Eye::SINGLE_EYE_DATA_GAZE_ORIGIN_VALIDITY);
							mEyeData.gazeDirValid = anipal::Eye::DecodeBitMask(cvalid, anipal::Eye::SINGLE_EYE_DATA_GAZE_DIRECTION_VALIDITY);
							mEyeData.opennessValid = anipal::Eye::DecodeBitMask(lvalid, anipal::Eye::SINGLE_EYE_DATA_EYE_OPENNESS_VALIDITY);
							mEyeData.pupilDiamValid = anipal::Eye::DecodeBitMask(lvalid, anipal::Eye::SINGLE_EYE_DATA_PUPIL_DIAMETER_VALIDITY);
							mEyeData.pupilPosValid = anipal::Eye::DecodeBitMask(lvalid, anipal::Eye::SINGLE_EYE_DATA_PUPIL_POSITION_IN_SENSOR_AREA_VALIDITY);
							mEyeData.convergenceValid = srEyeData.verbose_data.combined.convergence_distance_validity;
							
							const bool eyesValid = mEyeData.gazePosValid || mEyeData.gazeDirValid;

							// Bugs in AniPal?
							// Openness is always valid for single eye and never valid for combined
							// Pupil pos is sometimes valid when HMD not worn
							// Gaze responds reliably to HMD worn

							if(mEyeData.gazePosValid){
								for(int i=0; i<3; ++i)
									mEyeData.gazePos[i] = c.gaze_origin_mm.elem_[i];
								fixCoord(mEyeData.gazePos);
							}
							if(mEyeData.gazeDirValid){
								for(int i=0; i<3; ++i)
									mEyeData.gazeDir[i] = c.gaze_direction_normalized.elem_[i];
								fixCoord(mEyeData.gazeDir);
							}
							if(mEyeData.opennessValid){
								mEyeData.openness[LEFT] = l.eye_openness;
								mEyeData.openness[RIGHT] = r.eye_openness;
							}
							if(mEyeData.pupilPosValid){
								for(int i=0; i<2; ++i)
									mEyeData.pupilPos[i] = c.pupil_position_in_sensor_area.elem_[i];
							}
							if(mEyeData.pupilDiamValid){
								mEyeData.pupilDiam = c.pupil_diameter_mm;
							}
							if(mEyeData.convergenceValid){
								mEyeData.convergence = srEyeData.verbose_data.combined.convergence_distance_mm;
							}

							if(mOnEyeData) mOnEyeData();
							//printf("[Eye] Gaze: %.2f %.2f %.2f\n", gaze()[0], gaze()[1], gaze()[2]);
						}
					}
					if(STATUS_INIT == mLipTracking){
						auto err = anipal::Lip::GetLipData(&mLipData);
						if(Error::WORK == err){
							// TODO: fill in lip data struct
							if(mOnLipData) mOnLipData();
							//float * weightings = mLipData.prediction_data.blend_shape_weight;
							//printf("[Lip] frame: %d, time: %d, weightings %.2f\n", 		mLipData.frame_sequence, mLipData.timestamp, weightings[0]);
						}
					}
					FACE_TRACKER_SLEEP(mPeriod);
				}
			});
			mRunning = mThread != nullptr;
		}
		return mRunning;
	}

	FaceTracker& stop(){
		if(mThread != nullptr){
			mRunning = false;
			mThread->join();
            delete mThread;
            mThread = nullptr;
		}
		return *this;
	}

	FaceTracker& release(){
		using namespace ViveSR;
		if(!mRunning){
			anipal::Release(anipal::Eye::ANIPAL_TYPE_EYE);
			anipal::Release(anipal::Lip::ANIPAL_TYPE_LIP);
			mEyeTracking = STATUS_DISABLE;
			mLipTracking = STATUS_DISABLE;
		}
		return *this;
	}

private:
	enum{
		STATUS_DISABLE,
		STATUS_ENABLE,
		STATUS_INIT
	};

	std::function<void(void)> mOnEyeData;
	std::function<void(void)> mOnLipData;

	EyeData mEyeData;
	ViveSR::anipal::Lip::LipData mLipData;
    char mLipImage[800 * 400];
	std::thread * mThread = nullptr;
	float mPeriod = 5./1000;
	bool mRunning = false;
	char mEyeTracking = STATUS_DISABLE;
	char mLipTracking = STATUS_DISABLE;
	bool needsInit() const { return STATUS_ENABLE==mEyeTracking || STATUS_ENABLE==mLipTracking; }
	bool initGood() const { return STATUS_INIT==mEyeTracking || STATUS_INIT==mLipTracking; }
	static void fixCoord(float * v){
		// ViveSR is right-handed, but rotated around y so +z is forward and +x is left
		v[0]=-v[0];
		v[2]=-v[2];
	}
};


inline std::string to_string(const FaceTracker::EyeData& d){
	std::string s;
	s = s + "valid:" + (d.gazePosValid?" gazePos":"") + (d.gazeDirValid?" gazeDir":"") + (d.opennessValid?" openness":"") + (d.convergenceValid?" convergence":"") + (d.pupilPosValid?" pupilPos":"") + (d.pupilDiamValid?" pupilDiam":"") + "\n";
	return s;
}
