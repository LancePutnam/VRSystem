# VRSystem
VRSystem is meant to provide a simplified interface to a VR headset and controllers. It uses OpenGL and OpenVR as its backend. Beyond interfacing with an HMD and hand controllers, it features
* Minimal setup code
* Generic tracker support
* Hidden area mask
* Velocity-based vignetting
* Attachment to a parent pose (as a scene graph node)
* Custom render resolution
* FBO rendering with color texture attachment
* RGB camera feed

# Example Code
Here is a minimal code example illustrating how you might proceed to plug VRSystem into an existing app.

    class MyApp {
    public:
        VRSystem vr;

        void onSetup(){
            if(!vr.init()){
                printf("Error initializing VR\n");
            }
        }

        void onFrame(){
            // Poll for events from all active VR devices
            while(vr.pollEvent()){
                printf("%s %s\n", toString(ev.deviceType), toString(ev.type));
            }

            // Differential event handling
            if(vr.controller(vr.LEFT).buttonWentDown(vr.GRIP)){
            
            }

            // Bind FBOs and render to each eye
            vr.render([](){
                // Get view and projection matrices for current eye
                auto view = vr.view();
                auto proj = vr.projection();
               
                // Render scene...
            });
        }
    }
	
# License
Permissive BSD 3-clause license. See the `LICENSE` file for full details.