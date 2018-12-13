#pragma once
#include "ofMain.h"
#include "box.h"
#include "ray.h"
#include "Particle.h"


class TreeNode {
public:
	Box box;
	vector<int> points;
	vector<TreeNode> children;
};

class Octree {
public:
	
	void create(const ofMesh & mesh, int numLevels, Box boundingBox);
	void subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level);
	bool intersect(const Ray &, const TreeNode & node, TreeNode & nodeRtn);
    bool intersect(const Ray & r, TreeNode & nodeRtn){
        recursionCounter = 0;
        return intersect(r,root,nodeRtn);
    }
	void draw(TreeNode & node, int numLevels, int level);
	void draw(int numLevels, int level) {
		draw(root, numLevels, level);
	}
	void drawLeafNodes(TreeNode & node);
    void drawLeafNodes(){
        drawLeafNodes(root);
    }
    bool collides(Particle & p, float delta, TreeNode & nodeRtn, int & index);
    bool collides(Particle & p, const float delta, TreeNode & node, TreeNode & nodeRtn, int & index);
	static void drawBox(const Box &box);
	static Box meshBounds(const ofMesh &);
	int getMeshPointsInBox(const ofMesh &mesh, const vector<int> & points, Box & box, vector<int> & pointsRtn);
	void subDivideBox8(const Box &b, vector<Box> & boxList);
    void getIndices(const ofMesh &, vector<int>&);
    
    int recursionCounter = 0;
	ofMesh mesh;
	TreeNode root;
};
