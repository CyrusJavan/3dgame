#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "ParticleEmitter.h"
#include "ParticleSystem.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"

class ofApp : public ofBaseApp{

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
        void initLightingAndMaterials();
        Box  meshBounds(const ofMesh &);
        float getAGL();
        void doCollisions();
        bool checkCollisions(ParticleSystem *, float, TreeNode&, int &);
        void performCollisions(ParticleSystem *, TreeNode&, int &, ofVec3f);
        bool cylinderContains(ofCylinderPrimitive cyl, ofVec3f point);
		
        Octree octree;
        vector<Octree> octrees;
        bool showOctree;
        bool showLeafNodes;
        ofEasyCam cam;
        ofxAssimpModelLoader mars, lander, scene;
        ParticleSystem* landerSystem; // Use a particle system with one particle for the lander
        ThrusterForce* thrust;
        ParticleEmitter* exhaust;
        ParticleEmitter* ballSpawner;
        ofLight light;
        ofImage backgroundImage;
        ofCamera *theCam = NULL;
        ofCamera topCam;
        ofEasyCam followCam;
        Box boundingBox;
        ofMesh marsMesh;
        ofTexture mTex, orangeTexture, appleTexture, peachTexture, limeTexture, plumTexture;
        float followCamAngle = 0.0;
        ofSoundPlayer rocketSound, catchSound, themeSong;
    
        int score = 0;
        ofCylinderPrimitive insideBarrel;
    
        float agl;
    
        bool bAltKeyDown;
        bool bCtrlKeyDown;
        bool bWireframe;
        bool bDisplayPoints;
    
        bool bBackgroundLoaded = false;
        bool bLanderLoaded = false;
};
