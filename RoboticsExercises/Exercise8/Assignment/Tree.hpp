#include <rw/math/Q.hpp>
#include <rw/core/Ptr.hpp>
#include <iostream>

using namespace rw::math;

struct Node
{
    using Ptr = rw::core::Ptr <Node>;
    Node (Q q) : q (q), parent (NULL) {}

    Q q;
    Node::Ptr parent;
};

class Tree
{
  public:
    Tree (Node::Ptr root) : _root (root) { _nodes.push_back (root); }

    void add_edge (Node::Ptr parent, Node::Ptr node)
    {
        node->parent = parent;
        _nodes.push_back (node);
    }

    Node::Ptr find_nearest_neighbor (Node::Ptr node)
    {
        double min_dist = (_nodes[0]->q - node->q).norm2 ();
        Node::Ptr nearest   = _root;

        for (Node::Ptr& n : _nodes) {
            double dist = (n->q - node->q).norm2 ();
            if (min_dist > dist) {
                min_dist = dist;
                nearest  = n;
            }
        }
        return nearest;
    }

    std::vector< Node::Ptr > get_route (Node::Ptr leaf)
    {
        std::vector< Node::Ptr > route;
        route.push_back (leaf);
        while (route.back ()->parent != NULL) {
            route.push_back (route.back ()->parent);
        }
        return route;
    }

  private:
    Node::Ptr _root;
    std::vector< Node::Ptr > _nodes;
};
