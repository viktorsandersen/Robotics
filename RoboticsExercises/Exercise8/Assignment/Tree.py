import numpy as np
import copy
import sdurw


class Node:
    def __init__(self, q):
        self.q = sdurw.Q(6, q[0], q[1], q[2], q[3], q[4], q[5])
        self.q_numpy = np.array([q[0], q[1], q[2], q[3], q[4], q[5]])
        self.parent = None

class Tree:
    def __init__(self, root: Node):
        self.root = root
        self.nodes = [root]
        root.parent = None
        self.goal_route = []

    def add_vertex(self, node: Node):
        self.nodes.append(node)

    def add_edge(self, parent, node: Node):
        node.parent = parent

    def find_nearest_neighbor(self, node: Node):
        min_dist = np.linalg.norm(self.nodes[0].q_numpy - node.q_numpy)
        nearest = self.root
        for i in self.nodes:
            dist = np.linalg.norm(node.q_numpy - i.q_numpy)
            if min_dist > dist:
                min_dist = dist
                nearest = i
        return nearest, min_dist

    def get_route(self, leaf):
        if leaf.parent is not None:
            self.get_route(leaf.parent)
        print(leaf.q_numpy)
        self.goal_route.append(leaf)

