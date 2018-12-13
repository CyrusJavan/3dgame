#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    bAltKeyDown = false;
    bCtrlKeyDown = false;
    bLanderLoaded = false;
    
    cam.setDistance(4);
    cam.setNearClip(.1);
    cam.setFov(65.5);
    cam.setPosition(0, 10, 10);
    cam.lookAt(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    //cam.rotateDeg(180, 0, 0, 1);
    cam.disableMouseInput();
    
    topCam.setNearClip(.1);
    topCam.setFov(65.5);
    topCam.setPosition(0, 100, 0);
    topCam.lookAt(glm::vec3(0, 0, 0));
    
    followCam.setNearClip(.1);
    followCam.setFov(65.5);
    followCam.setPosition(0, 10, 0);
    
    // set current camera;
    //
    theCam = &cam;
    
    ofSetVerticalSync(true);
    ofEnableSmoothing();
    ofEnableDepthTest();
    
    // load BG image
    //
    bBackgroundLoaded = backgroundImage.load("images/sky.jpg");

    // setup rudimentary lighting
    //
    initLightingAndMaterials();
    
    // setup sounds
    rocketSound.load("sounds/rocket_thruster.wav");
    catchSound.load("sounds/baseball_pitch.wav");
    themeSong.load("sounds/mario_theme.mp3");
    themeSong.setLoop(true);
    themeSong.setVolume(.1);
    
    scene.loadModel("geo/mountains-tree.obj");
    scene.setRotation(0, 180, 0, 0, 1);
    scene.setScaleNormalization(false);
    for(int i=0; i < scene.getNumMeshes(); i ++){
        ofMesh sceneMesh = scene.getMesh(i);
        Octree octree = Octree();
        octree.create(sceneMesh, 6);
        octrees.push_back(octree);
    }
    
    //mars.loadModel("geo/mountain_highPoly_v2.obj");
    //mars.loadModel("geo/mountain_highPoly.obj");
    mars.loadModel("geo/mountain_highPoly_v2.obj");
    //mars.loadModel("geo/moon-low-v1.obj");
    //mars.loadModel("geo/moon-houdini.obj");
    //mars.loadModel("geo/moon-crater-v1.obj");
    mars.setRotation(0, 180, 0, 0, 1);

    mars.setScaleNormalization(false);
    cout << "# of meshes " << mars.getNumMeshes() << endl;
    
    marsMesh = mars.getMesh(0);
    
    
//    boundingBox = meshBounds(marsMesh);
//    boundingBox = Box(Vector3(-mars.getSceneMin().x,
//                              -mars.getSceneMin().y,
//                              mars.getSceneMin().z),
//                      Vector3(-mars.getSceneMax().x,
//                              -mars.getSceneMax().y,
//                              mars.getSceneMax().z));
//    boundingBox = Box(Vector3(mars.getSceneMin().x,
//                              mars.getSceneMin().y,
//                              mars.getSceneMin().z),
//                      Vector3(mars.getSceneMax().x,
//                              mars.getSceneMax().y,
//                              mars.getSceneMax().z));
    int start = ofGetElapsedTimeMillis();
    octree.create(marsMesh, 6);
    int end = ofGetElapsedTimeMillis();
    cout << "Octree built in " << end - start << " milliseconds" << endl;
    
    
    
    ofDisableArbTex();
    //ofLoadImage(mTex, "images/glow.png");
    ofLoadImage(mTex, "images/fire.jpg");
    ofLoadImage(orangeTexture, "images/orange.jpg");
    ofLoadImage(appleTexture, "images/apple.png");
    ofLoadImage(peachTexture, "images/peach.jpg");
    ofLoadImage(limeTexture, "images/lime.jpg");
    ofLoadImage(plumTexture, "images/plum.jpg");
    
    // load lander model
    //
    if (lander.loadModel("geo/barrel.obj")) {
        lander.setScaleNormalization(false);
        lander.setScale(.5, .5, .5);
        lander.setRotation(0, 180, 0, 0, 1);
        
        // Lander is represented as a single particle in the system
        // Initially there is just a turbulence force
        landerSystem = new ParticleSystem();
        Particle landerParticle;
        landerParticle.lifespan = -1;
        landerParticle.position = ofVec3f(0,10,0);
        landerSystem->add(landerParticle);
        
        insideBarrel.set(0.4, 0.4);

        // Maunually creating collision points
        vector<ofVec3f> corners;
        float h = 1.6;
        float w = 0.55;
        corners.push_back(ofVec3f( w,  10,      w));
        corners.push_back(ofVec3f(-w,  10,      w));
        corners.push_back(ofVec3f( w,  10,     -w));
        corners.push_back(ofVec3f(-w,  10,     -w));
        corners.push_back(ofVec3f( w,  10 + h,  w));
        corners.push_back(ofVec3f(-w,  10 + h,  w));
        corners.push_back(ofVec3f( w,  10 + h, -w));
        corners.push_back(ofVec3f(-w,  10 + h, -w));

        for (auto corner : corners){
            Particle p;
            p.lifespan = -1;
            p.position = corner;
            landerSystem->add(p);
        }

        // No turbulence for now, becauase it would break the multi point collision detection
        // landerSystem->addForce(new TurbulenceForce(ofVec3f(-0.1,-0.1,-0.1),
        //                                            ofVec3f( 0.1, 0.1, 0.1)));
        ofVec3f gravity = ofVec3f(0,-.1,0);
        landerSystem->addForce(new GravityForce(gravity));
        // thrust force will be updated by keyPressed
        // thrust force is responsible for moving the lander
        thrust = new ThrusterForce(ofVec3f(0,0,0));
        landerSystem->addForce(thrust);
        
        exhaust = new ParticleEmitter(new ParticleSystem());
        exhaust->type = DiscEmitter;
        exhaust->setGroupSize(50);
        exhaust->setLifespan(.3);
        exhaust->setParticleRadius(.01);
        exhaust->setRate(20);
        exhaust->setVelocity(ofVec3f(0,1,0));
        bLanderLoaded = true;
        
//        ofVec3f spawnerLocations[] =
//        {   ofVec3f(3.6,20,1.7),
//            ofVec3f(3.0,25,-3.4),
//            ofVec3f(0.1,20,7.3),
//            ofVec3f(-3.5,30,6.3),
//            ofVec3f(-7.3,30,3),
//            ofVec3f(-0.9,20,12.2),
//            ofVec3f(-9.3,40,13.5),
//        };
        
        ofVec3f spawnerLocations[] =
        {   ofVec3f(-9.86352, 26.6276, -7.06939),
            ofVec3f(-3.34461, 29.3954, -11.9546),
            ofVec3f( 0.464731, 24.1474, -5.56162),
            ofVec3f(4.59306, 21.3098, 15.3218),
            ofVec3f(-0.124639, 23.3434, 13.7385),
            ofVec3f( -11.4664, 27.666, 4.43373),
            ofVec3f(-2.12983, 16.343, 2.99508),
        };
        
        for (int i = 0; i < 7; i++){
            //load the falling balls from the sky
            ParticleSystem* ballSystem = new ParticleSystem();
            ballSystem->addForce(new GravityForce(ofVec3f(0,-1,0)));
            ParticleEmitter* ballSpawner = new ParticleEmitter(ballSystem);
            ballSpawner->init();
            ballSpawner->type = PlaneEmitter; // Emit randomly from a plane, MUST SET WIDTH AND HEIGHT
            ballSpawner->width = 2;
            ballSpawner->length = 2;
            //ballSpawner->setGroupSize(50);
            ballSpawner->setLifespan(20);
            ballSpawner->setParticleRadius(.2);
            ballSpawner->setRate(1);
            ballSpawner->drawEmitter = false;
            ballSpawner->setVelocity(ofVec3f(0,-1,0));
            ballSpawner->setPosition(spawnerLocations[i]);
            vector<ofTexture> fruitTextures = {orangeTexture, appleTexture, peachTexture, limeTexture, plumTexture};
            ballSpawner->setTextures(fruitTextures);
            //ballSpawner->setOneShot(true);
            ballSpawner->start();
            ballSpawners.push_back(ballSpawner);
            //ballSpawner->setOneShot(true);
            //ballSpawner->spawn(ofGetElapsedTimeMillis());
        }
    }
    else {
        cout << "Error: Can't load model" << "geo/lander.obj" << endl;
        ofExit(0);
    }
    themeSong.play();
}

//--------------------------------------------------------------
void ofApp::update(){
    doCollisions(); // Detect collisions and update velocity accordingly
    landerSystem->update();
    lander.setPosition(landerSystem->particles[0].position.x,
                       landerSystem->particles[0].position.y,
                       landerSystem->particles[0].position.z);
    exhaust->setPosition(ofVec3f(landerSystem->particles[0].position.x,
                       landerSystem->particles[0].position.y,
                       landerSystem->particles[0].position.z));
    insideBarrel.setPosition(landerSystem->particles[0].position.x,
                             landerSystem->particles[0].position.y + 0.5,
                             landerSystem->particles[0].position.z);
    exhaust->update();
    for (auto bs : ballSpawners){
        bs->update();
    }
    
    // Make the follow cam rotate around the lander
    followCamAngle += 0.005;
    ofVec3f landerPos = lander.getPosition();
    followCam.setPosition(ofVec3f(landerPos.x + 3 * sin(followCamAngle),
                                  landerPos.y + 2,
                                  landerPos.z + 3 * cos(followCamAngle)));
    followCam.lookAt(lander.getPosition(), ofVec3f(0,1,0));
    
    // Prevent exhaust particles from coming out the top of the lander
    if (landerSystem->particles[0].velocity.y < 0){
        exhaust->setVelocity(landerSystem->particles[0].velocity + ofVec3f(0,-5,0));
    }else {
        exhaust->setVelocity(ofVec3f(0,-5,0));
    }
    
    // Update the above ground level
    agl = getAGL();
    cam.lookAt(lander.getPosition(),ofVec3f(0,1,0));
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    if (bBackgroundLoaded) {
        ofPushMatrix();
        ofDisableDepthTest();
        ofSetColor(255);
        ofScale(.3, .3);
        backgroundImage.draw(-200, -100);
        ofEnableDepthTest();
        ofPopMatrix();
    }
    
    
    
    if(!gameStarted){
        ofDrawBitmapString("Unbarrelble: A 3D game", ofGetWindowWidth() / 2 - 125, ofGetWindowHeight() / 2 - 100);
        ofDrawBitmapString("V to start", ofGetWindowWidth() / 2 - 125, ofGetWindowHeight() / 2 - 80);
    }
    
    else if(gameStarted){
        
        theCam->begin();
        ofPushMatrix();
        
        if (bWireframe) {                    // wireframe mode  (include axis)
            ofDisableLighting();
            ofSetColor(ofColor::slateGray);
            if (bLanderLoaded) {
                lander.drawWireframe();
            }
        }
        else {
            ofEnableLighting();              // shaded mode
            mars.drawFaces();
            //mars.drawWireframe();
            //ofNoFill();
            //Octree::drawBox(boundingBox);
            //octree.drawLeafNodes();
            if (bLanderLoaded) {
                lander.drawFaces();
                scene.drawFaces();
                //lander.drawVertices();
                //lander.drawWireframe();
            }
        }
        
        // Draw the exhaust with a texture
        ofPushMatrix();
        mTex.bind();
        exhaust->draw();
        mTex.unbind();
        ofPopMatrix();
        
        ofFill();
        // draw falling balls
        for (auto bs : ballSpawners){
            bs->draw();
        }
        
        ofNoFill();
        // draw octrees
        //draw Octree
        if(showOctree){
            
            for(int i=0; i<octrees.size(); i++){
                octrees[i].draw(octrees[i].root, 4, 0);
            }
            //octree.draw(octree.root, 10, 0);
        }
        
        //draw leaf nodes
        if(!showOctree && showLeafNodes){
            for(int i=0; i<octrees.size(); i++){
                octrees[i].drawLeafNodes(octrees[i].root);
            }
            //        octree.drawLeafNodes(octree.root);
        }
        
        // draw the collision points for debugging
        for (auto p : landerSystem->particles){
            ofSetColor(255);
            ofDrawSphere(p.position.x, p.position.y, p.position.z, 0.01);
        }
        
        insideBarrel.draw();
        //barrelTree.drawLeafNodes();
        ofPopMatrix();
        theCam->end();
        
        // draw screen data
        //
        ofSetColor(ofColor::white);
        string str;
        str = "Frame Rate: " + std::to_string(ofGetFrameRate());
        ofDrawBitmapString(str, ofGetWindowWidth() - 170, 20);
        str = "AGL: " + std::to_string(agl);
        ofDrawBitmapString(str, ofGetWindowWidth() - 170, 40);
        str = "Score: " + std::to_string(score);
        ofDrawBitmapString(str, ofGetWindowWidth() - 170, 60);
        
        ofDrawBitmapString("ARROW KEYS to move, R to reset position, B to quit", 20, 20);
        ofDrawBitmapString("1 for normal perspective, 2 for follow cam, 3 for top down view", 20, 40);
        ofDrawBitmapString("O to un/draw Octree, L to un/draw Leaf Nodes", 20, 60);
        
        
    }

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    float speed = .3;
    switch (key) {
        case 'l':
            showLeafNodes = !showLeafNodes;
            break;
        case 'o':
            showOctree = !showOctree;
            break;
        case 'C':
        case 'c':
            if (cam.getMouseInputEnabled()) cam.disableMouseInput();
            else cam.enableMouseInput();
            break;
        case 'F':
        case 'f':
            ofToggleFullscreen();
            break;
        case 'H':
        case 'h':
            break;
        case 'P':
        case 'p':
            break;
        case 'r':
            //cam.reset();
            landerSystem->particles[0].position = ofVec3f(0,10,0);
            break;
        case 'b':
            gameStarted = false;
        case 't':
            break;
        case 'u':
            break;
        case 'v':
            gameStarted = true;
            //togglePointsDisplay();
            break;
        case 'V':
            break;
        case OF_KEY_F1:
        case '1':
            theCam = &cam;
            break;
        case OF_KEY_F2:
        case '2':
            theCam = &followCam;
            break;
        case OF_KEY_F3:
        case '3':
            theCam = &topCam;
            break;
        case 'w':
            thrust->add(ofVec3f(0,1,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case 'a':
            thrust->add(ofVec3f(1,0,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case 's':
            thrust->add(ofVec3f(0,-1,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case 'd':
            thrust->add(ofVec3f(-1,0,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case 'q':
            thrust->add(ofVec3f(0,0,1) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }

            break;
        case 'e':
            thrust->add(ofVec3f(0,0,-1) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case OF_KEY_ALT:
            cam.enableMouseInput();
            bAltKeyDown = true;
            break;
        case OF_KEY_CONTROL:
            bCtrlKeyDown = true;
            break;
        case OF_KEY_SHIFT:
            break;
        case OF_KEY_DEL:
            break;
        case OF_KEY_UP:
            if (bAltKeyDown)
                thrust->add(ofVec3f(0,0,1) * speed);
            else
                thrust->add(ofVec3f(0,1,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case OF_KEY_DOWN:
            if (bAltKeyDown)
                thrust->add(ofVec3f(0,0,-1) * speed);
            else
                thrust->add(ofVec3f(0,-1,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case OF_KEY_LEFT:
            thrust->add(ofVec3f(1,0,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        case OF_KEY_RIGHT:
            thrust->add(ofVec3f(-1,0,0) * speed);
            exhaust->start();
            if (!rocketSound.isPlaying()){
                rocketSound.play();
            }
            break;
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    switch (key) {
        case OF_KEY_ALT:
            cam.disableMouseInput();
            bAltKeyDown = false;
            break;
        case OF_KEY_CONTROL:
            bCtrlKeyDown = false;
            break;
        case OF_KEY_SHIFT:
            break;
        default:
            rocketSound.stop();
            thrust->set(ofVec3f(0,0,0));
            exhaust->stop();
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
//    ofVec3f mouse(mouseX, mouseY);
//    ofVec3f rayPoint = cam.screenToWorld(mouse);
//    ofVec3f rayDir = rayPoint - cam.getPosition();
//    rayDir.normalize();
//    Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
//                  Vector3(rayDir.x, rayDir.y, rayDir.z));
//
//    float startTime = ofGetElapsedTimeMillis();
//    for(int i=0; i<octrees.size(); i++){
//        if (octrees[i].intersect(ray, octrees[i].root, octrees[i].selectedNode)) {
//            //rayIntersected = true;
//            int pt;
//            pt = octrees[i].selectedNode.points[0];
//            ofVec3f intersectPoint = octrees[i].mesh.getVertex(pt);
//            cout << "(x,y,z): " << intersectPoint << endl;
    //        float endTime = ofGetElapsedTimeMillis();
    //        cout << endTime - startTime << " milliseconds to select correct leaf node with " << octree.selectedNode.points.size() << " points" << endl << endl;
//            }
//        }

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

bool ofApp::checkCollisions(ParticleSystem *ps, float delta, TreeNode& potential, int &returnIndex){
    for(int i=0; i<ps->particles.size(); i++){
        Particle p = ps->particles[i];
        for(int i=0; i < octrees.size(); i ++){
            if(octrees[i].collides(p, delta, potential, returnIndex)){
                return true;
            }
        }
//        if(octree.collides(p, delta, potential, returnIndex )){
//            return true;
//        }
    }
}

void ofApp::performCollisions(ParticleSystem *ps, TreeNode& potential, int & index, ofVec3f inputVelocity){
    ofVec3f normal = marsMesh.getNormal(potential.points[index]);
    float restitution = .3333;
    //ofVec3f reflect = inputVelocity - (((1 + restitution) * inputVelocity.dot(normal)) * normal);
    ofVec3f reflect = ((1 + restitution) * (-inputVelocity).dot(normal)) * normal;
    for (Particle& p : ps->particles){
        //cout << "old velocity " << p.velocity <<endl;
        //p.velocity = (reflect + normal).normalize().scale(p.velocity.length() *(2.0/3.0));
        p.velocity = reflect;
        //cout << "new velocity " << p.velocity <<endl;
    }
}

// Detect collisons between the lander and the surface
// Author: Cyrus & Vivian
void ofApp::doCollisions(){
    // Check if collided
    //bool collided = false;
    ofVec3f inputVelocity = landerSystem->particles[0].velocity;
    float delta = 5 * landerSystem->particles[0].velocity.length();
    TreeNode potential;
    int index;
    
//    for (int i = 0; i < landerSystem->particles.size(); i++){
//        Particle p = landerSystem->particles[i];
//        if ( octree.collides(p, delta, potential, index) ){
//            collided = true;
//            break;
//        }
//    }
    
    // check collisions for space craft
    
    if(checkCollisions(landerSystem, delta, potential, index)){
        //cout << "Barrel collision" << endl;
//        cout << "Barrel old velocity: " << landerSystem->particles[0].velocity <<endl;
        performCollisions(landerSystem, potential, index, inputVelocity);
//        cout << "Barrel new velocity " << landerSystem->particles[0].velocity <<endl;
    }
    
    TreeNode potential2;
    int index2;
    
    //delta = 100;
    // check collisions for falling balls
    for (auto bs : ballSpawners){
        for (Particle& ball : bs->sys->particles){
            ParticleSystem* tempSystem = new ParticleSystem();
            //ball.position = ofVec3f(-ball.position.x, -ball.position.y, ball.position.z);
            tempSystem->add(ball);
    //        tempSystem->particles[0].position = ofVec3f(-ball.position.x, -ball.position.y, ball.position.z);
            if(checkCollisions(tempSystem, delta, potential2, index2)){
                
                //cout << "Ball collision" << endl;
                //cout << "Ball old velocity " << ball.velocity <<endl;
                performCollisions(tempSystem, potential2, index2, ball.velocity);
                //cout << "Ball other new velocity" << tempSystem->particles[0].velocity << endl;
                //cout << "Ball new velocity " << ball.velocity <<endl;
                ball.velocity = tempSystem->particles[0].velocity;
            }
            //ball.position = ofVec3f(-ball.position.x, -ball.position.y, ball.position.z);
        }
    }
    
    // Check if a ball is inside the barrel, if it is, remove the ball and increment the score
    // A Ball is inside the barrell if it collides with any of the landerSystem Particles
    // TODO: Make sure ball cannot enter the barrel from the sides/bottom only the open top
    for (auto bs : ballSpawners){
        float minDistance = 5; // If a ball is within this distance to a landerSystem particle it is a collision
        for (auto ball = bs->sys->particles.begin(); ball != bs->sys->particles.end(); ){
            ofVec3f ballPos = ofVec3f((*ball).position.x,(*ball).position.y,(*ball).position.z);
            
            if (cylinderContains(insideBarrel, ballPos)){
                //cout << "Cylinder Collision!" << endl;
                score++;
                catchSound.play();
                ball = bs->sys->particles.erase(ball); // Remove the ball
            }
            else{
                ++ball;
            }
        }
    }
//    bool result = cylinderContains(insideBarrel, ofVec3f(-landerSystem->particles[0].position.x,
//                                            -landerSystem->particles[0].position.y,
//                                            landerSystem->particles[0].position.z));
//
//    if (collided){ // Do surface normal reflection
//        ofVec3f normal = marsMesh.getNormal(potential.points[index]);
//        // Refection vector calculation
//        ofVec3f reflect = inputVelocity - ((2 * inputVelocity.dot(normal)) * normal);
//        for (Particle& p : landerSystem->particles){
//            p.velocity = (reflect + normal).normalize().scale(p.velocity.length() * (2.0/3.0));
//        }
//    }
}


// Returns whether the cylinder contains the point
bool ofApp::cylinderContains(ofCylinderPrimitive cyl, ofVec3f point){
    //cout << "point y " << point.y << " cylinder.y " <<cyl.getPosition().y << endl;
    bool withinHeight = point.y < cyl.getPosition().y && cyl.getPosition().y - cyl.getHeight() < point.y;
    //cout << "WithinHeihgt  "<<withinHeight << endl;
    
    ofVec2f pointXZ(point.x,point.z);
    ofVec2f cylXZ(cyl.getPosition().x,cyl.getPosition().z);
    bool withinRadius = pointXZ.distance(cylXZ) < cyl.getRadius();
    //cout << "WithinRad  "<<withinRadius << endl;
    return  withinHeight && withinRadius;
}

// Finds the above ground level of the lander
// Author: Cyrus
float ofApp::getAGL(){
    // To find AGL shoot a ray downwards from the lander
    ofVec3f landerPos = lander.getPosition();
    Ray ray = Ray(Vector3(landerPos.x, landerPos.y, landerPos.z),
                  Vector3(0, 1, 0));
    TreeNode possible;
    
    if (octree.intersect(ray, octree.root, possible)){
        float sumDist = 0;
        for (int v : possible.points){
            ofVec3f point = marsMesh.getVertex(v);
            point = ofVec3f(-point.x,-point.y,point.z);
            sumDist += point.distance(landerPos);
        }
        // Draw the leaf node box that the ray intersected with, for debugging
        // Octree::drawBox(possible.box);
        return sumDist / possible.points.size();
    }
    else{
        cout << "Could not find AGL, ray did not intersect..." << endl;
        return -1;
    }
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh) {
    int n = mesh.getNumVertices();
    ofVec3f v = mesh.getVertex(0);
    ofVec3f max = v;
    ofVec3f min = v;
    for (int i = 1; i < n; i++) {
        ofVec3f v = mesh.getVertex(i);
        
        if (v.x > max.x) max.x = v.x;
        else if (v.x < min.x) min.x = v.x;
        
        if (v.y > max.y) max.y = v.y;
        else if (v.y < min.y) min.y = v.y;
        
        if (v.z > max.z) max.z = v.z;
        else if (v.z < min.z) min.z = v.z;
    }
    return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

void ofApp::initLightingAndMaterials() {
    
    static float ambient[] =
    { .5f, .5f, .5, 1.0f };
    static float diffuse[] =
    { .7f, .7f, .7f, 1.0f };
    
    static float position[] =
    {20.0, 20.0, 20.0, 0.0 };
    
    static float lmodel_ambient[] =
    { 1.0f, 1.0f, 1.0f, 1.0f };
    
    static float lmodel_twoside[] =
    { GL_TRUE };
    
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, position);
    
    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glShadeModel(GL_SMOOTH);
}
