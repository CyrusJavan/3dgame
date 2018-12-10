//  Kevin M. Smith - Basic Octree Class - CS134/235 4/18/18
//


#include "Octree.h"
 

// draw Octree (recursively)
//
void Octree::draw(TreeNode & node, int numLevels, int level) {
	if (level >= numLevels) return;
	drawBox(node.box);
	level++;
	for (int i = 0; i < node.children.size(); i++) {
		draw(node.children[i], numLevels, level);
	}
}

// draw only leaf Nodes
//
void Octree::drawLeafNodes(TreeNode & node) {
    if (node.children.size() == 0) {
        drawBox(node.box);
        return;
    }
    for (int i = 0; i < node.children.size(); i++) {
        drawLeafNodes(node.children[i]);
    }
}


//draw a box from a "Box" class  
//
void Octree::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box Octree::meshBounds(const ofMesh & mesh) {
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
	cout << "vertices: " << n << endl;
//	cout << "min: " << min << "max: " << max << endl;
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

// getMeshPointsInBox:  return an array of indices to points in mesh that are contained 
//                      inside the Box.  Return count of points found;
//
int Octree::getMeshPointsInBox(const ofMesh & mesh, const vector<int>& points,
	Box & box, vector<int> & pointsRtn)
{
    int count = 0;
    for (int i = 0; i < points.size(); i++) {
        ofVec3f v = mesh.getVertex(points[i]);
        if (box.inside(Vector3(v.x, v.y, v.z))) {
            count++;
            pointsRtn.push_back(points[i]);
        }
    }
    return count;
}



//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void Octree::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

void Octree::getIndices(const ofMesh & mesh, vector<int>& points){
    vector<int> indices(mesh.getNumIndices());
    // Init indices vector
    auto& vertices = mesh.getIndices();
    for (int i = 0; i < indices.size(); i++){
        indices[i] = vertices[i];
    }
    points = indices;
}

void Octree::create(const ofMesh & geo, int numLevels) {
	// initialize octree structure
	//
    mesh = geo;
    root = TreeNode();
    root.box = meshBounds(mesh);
    getIndices(mesh, root.points);
    subdivide(mesh, root, numLevels, 0);
}

void Octree::subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level) {
    // Recursive base case
    if (level == numLevels) return;
    // Create up to 8 children
    // Don't need to keep a child if it doesn't contain any points
    vector<Box> boxes;
    vector<TreeNode> potentialChildren(8);
    // Create the 8 subdivided boxes
    subDivideBox8(node.box, boxes);
    int i;
    // For each point, find the box it should go in
    for (int p : node.points){
        i = 0;
        for (Box& b : boxes){
            ofVec3f v = mesh.getVertex(p);
            if (b.inside(Vector3(-v.x,-v.y,v.z))){ // Point is inside the box
                potentialChildren[i].points.push_back(p);
                break;
            }
            i++;
        }
    }
    i = 0;
    for (Box b : boxes){
        potentialChildren[i].box = b;
        if (potentialChildren[i].points.size() != 0){
            node.children.push_back(potentialChildren[i]);
        }
        i++;
    }
    // Subdivide each of the children
    for (TreeNode& child : node.children)
    {
        subdivide(mesh, child, numLevels, level + 1);
    }
}
// For this to work properly, the leaf nodes of the octree must not have gaps
// between their boxes, or else a ray could shoot in between the boxes, and thus never
// intersect with them, even though it should have.
bool Octree::intersect(const Ray &ray, const TreeNode & node, TreeNode & nodeRtn) {
    // Base case
    if (node.children.size() == 0){
        nodeRtn = node;
        return true;
    }
    // Otherwise we need to recurse down
    bool anyIntersections = false;
    for (const TreeNode& child : node.children){
        if (child.box.intersect(ray, -1000, 1000)){
            anyIntersections = anyIntersections || intersect(ray, child, nodeRtn);
        }
    }
    return anyIntersections;
}

// Checks if the particle is within delta distance to any of the vertices in the
// octree
bool Octree::collides(Particle & p, float delta, TreeNode & nodeRtn,int & index){
    return collides(p, delta, root, nodeRtn, index);
}

bool Octree::collides(Particle & p, const float delta, TreeNode & node, TreeNode & nodeRtn, int& index){
    // At a leaf node and we are in that node
    // check if the average distance to the points is less than delta
    // Base case
    if (node.children.size() == 0){
        float min = 10000000;
        for (int i = 0; i < node.points.size(); i++){
            ofVec3f vert = mesh.getVertex(node.points[i]);
            vert = ofVec3f(-vert.x, -vert.y, vert.z);
            float d = vert.distance(ofVec3f(-p.position.x,-p.position.y,p.position.z));
            if (d < min){
                min = d;
                nodeRtn = node;
                index = i;
            }
        }
        if(min < delta){
            //cout << ofGetElapsedTimeMillis()<< " Octree:: Collided!" << endl;
            return true;
        }
        return false;
    }
    // Regular Case
    // Check if the particles collides with any of the children
    bool collided = false;
    for (TreeNode& child : node.children){
        if (child.box.inside(Vector3(-p.position.x,-p.position.y,p.position.z))){
            if (collides(p,delta,child,nodeRtn,index)){
                collided = true;
                break;
            }
        }
    }
    return collided;
}

