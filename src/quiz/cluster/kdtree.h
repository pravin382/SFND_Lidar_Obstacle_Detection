/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>
#include <algorithm>    // std::sort
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
	void insertHelper(Node** node, uint depth, std::vector<float> data, int index) {
            if (*node == NULL) {
                *node = new Node(data,index);
            } else{
                /* // this is bit long. See below for shorter version
                if (depth%2) { // if odd, check y-coordinate
                    if(data[1] < ((*node)->point[1])) {
                        depth++;
                        insertHelper(&(*node)->left, depth, data, index);
                    } else {
                        depth++;
                        insertHelper(&(*node)->right, depth, data, index);				
                    }			
                } else {
                    if(data[0] < ((*node)->point[0])) {
                        depth++;
                        insertHelper(&(*node)->left, depth, data, index);
                    } else {
                        depth++;
                        insertHelper(&(*node)->right, depth, data, index);				
                    }	
                }*/
                int m = depth%data.size();
                if(data[m] < ((*node)->point[m])){
                    insertHelper(&(*node)->left, depth+1, data, index);
                } else {
                    insertHelper(&(*node)->right, depth+1, data, index);
                }
                    
            }
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertHelper(&root, 0, point, id);
                //std::cout << "inserted " << point[0] << " " << point[1] << " " << point[2] << std::endl; 

	}
        
        // check if it is neighbour
        bool isNeighbour3D(std::vector<float> target, std::vector<float> other_point, float distanceTol) {
            bool flag = false;
            if (((target[0] + distanceTol) > other_point[0]) && ((target[0] - distanceTol) < other_point[0]) && ((target[1] + distanceTol) > other_point[1]) && ((target[1] - distanceTol) < other_point[1]) &&  ((target[2] + distanceTol) > other_point[2]) && ((target[2] - distanceTol) < other_point[2])) {
                float dist = std::sqrt(std::pow((target[0] - other_point[0]),2) + std::pow((target[1] - other_point[1]),2) + std::pow((target[2] - other_point[2]),2));
                if (dist <= distanceTol) {
                    return true;
                }
            }
            return flag;
        }
        
        bool isNeighbour2D(std::vector<float> target, std::vector<float> other_point, float distanceTol) {
            bool flag = false;
            if (((target[0] + distanceTol) > other_point[0]) && ((target[0] - distanceTol) < other_point[0]) && ((target[1] + distanceTol) > other_point[1]) && ((target[1] - distanceTol) < other_point[1])) {
                float dist = std::sqrt(std::pow((target[0] - other_point[0]),2) + std::pow((target[1] - other_point[1]),2));
                if (dist <= distanceTol) {
                    return true;
                }
            }
            return flag;
        }
        
	void searchHelper(Node** node, uint depth,std::vector<float> target, float distanceTol,std::vector<int>& ids) {
            if(*node != NULL) {
                if (target.size() == 2) { 
                    if (isNeighbour2D(target,(*node)->point, distanceTol)) {
                        (ids).push_back((*node)->id);
                    }
                }
                if (target.size() == 3) { 
                    if (isNeighbour3D(target,(*node)->point, distanceTol)) {
                        (ids).push_back((*node)->id);
                    }
                } 
                // go to next node
                int i = depth % target.size();
                if ((target[i] + distanceTol) > (*node)->point[i]) {
                    searchHelper(&(*node)->right, depth+1,target,distanceTol, ids);
                } 
                if ((target[i] - distanceTol) < (*node)->point[i]) {
                    searchHelper(&(*node)->left, depth+1,target,distanceTol,ids);
                }
            }   
        }
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{  
            std::vector<int> ids;
            searchHelper(&(root),0,target,distanceTol,ids);
            std::sort (ids.begin(), ids.end());
            return ids;
	}
	

};




