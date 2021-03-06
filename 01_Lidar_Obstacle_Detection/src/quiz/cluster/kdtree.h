/* \author Aaron Brown */
// Quiz on implementing kd tree
#include "../../render/render.h"
#include <stdexcept>

template <typename T>
T distanceBetween(std::vector<T> &point_a, std::vector<T> &point_b) {
  if (point_a.size() != point_b.size())
    throw std::invalid_argument("received points of different dimensions");

  T sum = 0;
  for (int i = 0; i < point_a.size(); ++i) {
    sum += (point_a[i] - point_b[i]) * (point_a[i] - point_b[i]);
  }
  return sqrt(sum);
}

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

struct KdTree {
  int dimension;
  Node *root;

  KdTree(int dim) : root(NULL), dimension(dim) {}

  ~KdTree() { delete root; }

  void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {
    if (*node == nullptr) {
      *node = new Node(point, id);
    } else {
      uint current_dim = depth % dimension;
      if (point[current_dim] < (*node)->point[current_dim])
        insertHelper(&((*node)->left), depth + 1, point, id);
      else
        insertHelper(&((*node)->right), depth + 1, point, id);
    }
  }

  void insert(std::vector<float> point, int id) {
    // insert a new point into the tree
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(std::vector<float> target, Node *node, uint depth,
                    float distanceTol, std::vector<int> &ids) {
    if (node != NULL) {
      if (fabs(node->point[0] - target[0]) <= distanceTol &&
          fabs(node->point[1] - target[1]) <= distanceTol) {
        float dist = distanceBetween(node->point, target);

        if (dist <= distanceTol)
          ids.push_back(node->id);
      }

      uint current_dim = depth % dimension;
      if ((target[current_dim] - distanceTol) < node->point[current_dim])
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      if ((target[current_dim] + distanceTol) > node->point[current_dim])
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }
};
