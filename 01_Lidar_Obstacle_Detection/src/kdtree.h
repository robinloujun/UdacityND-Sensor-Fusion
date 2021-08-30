/* \author Aaron Brown */
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/types.h>
#include <stdexcept>
#include <vector>

// Structure to represent node of kd tree
template <typename PointT> 
struct Node {
  PointT point;
  int id;
  Node *left;
  Node *right;

  Node(PointT arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

template <typename PointT> 
struct KdTree {
  Node<PointT> *root;

  KdTree() : root(NULL) {}

  ~KdTree() { delete root; }

  void insertHelper(Node<PointT> **node, uint depth, PointT point, int id) {
    if (*node == nullptr) {
      *node = new Node<PointT>(point, id);
    } else {
      uint current_dim = depth % 3;
      switch (current_dim) {
      case 0: {
        if (point.x < (*node)->point.x)
          insertHelper(&((*node)->left), depth + 1, point, id);
        else
          insertHelper(&((*node)->right), depth + 1, point, id);
        break;
      }
      case 1: {
        if (point.y < (*node)->point.y)
          insertHelper(&((*node)->left), depth + 1, point, id);
        else
          insertHelper(&((*node)->right), depth + 1, point, id);
        break;
      }
      case 2: {
        if (point.z < (*node)->point.z)
          insertHelper(&((*node)->left), depth + 1, point, id);
        else
          insertHelper(&((*node)->right), depth + 1, point, id);
        break;
      }
      }
    }
  }

  void insert(PointT point, int id) {
    // insert a new point into the tree
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(PointT target, Node<PointT> *node, uint depth,
                    float distanceTol, std::vector<int> &ids) {
    if (node != NULL) {
      if (fabs(target.x - node->point.x) <= distanceTol &&
          fabs(target.y - node->point.y) <= distanceTol &&
          fabs(target.z - node->point.z) <= distanceTol) {
        float dist = pcl::euclideanDistance(node->point, target);

        if (dist <= distanceTol)
          ids.push_back(node->id);
      }

      uint current_dim = depth % 3;
      switch (current_dim) {
      case 0: {
        if ((target.x - distanceTol) < node->point.x)
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        if ((target.x + distanceTol) > node->point.x)
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        break;
      }
      case 1: {
        if ((target.y - distanceTol) < node->point.y)
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        if ((target.y + distanceTol) > node->point.y)
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        break;
      }
      case 2: {
        if ((target.z - distanceTol) < node->point.z)
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        if ((target.z + distanceTol) > node->point.z)
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        break;
      }
      }
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(PointT target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }
};
