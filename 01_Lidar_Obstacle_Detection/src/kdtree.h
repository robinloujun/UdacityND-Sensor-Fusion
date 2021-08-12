/* \author Aaron Brown */
#include <stdexcept>

// Structure to represent node of kd tree
template <typename PointT> struct Node {
  typename PointT point;
  int id;
  Node *left;
  Node *right;

  Node(typename PointT arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

template <typename PointT> struct KdTree {
  int dimension;
  Node *root;

  KdTree(int dim) : root(NULL), dimension(dim) {}

  ~KdTree() { delete root; }

  void insertHelper(Node **node, uint depth, typename PointT point, int id) {
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

  void insert(typename PointT point, int id) {
    // insert a new point into the tree
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(typename PointT target, Node *node, uint depth,
                    float distanceTol, std::vector<int> &ids) {
    if (node != NULL) {
      if (fabs(target.x - node->point.x) <= distanceTol &&
          fabs(target.y - node->point.y) <= distanceTol &&
          fabs(target.z - node->point.z) <= distanceTol) {
        float dist = pcl::euclideanDistance(node->point, target);

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
  std::vector<int> search(typename PointT target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }
};
