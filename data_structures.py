from graph import compareNodes, getSameNode, checkinThis
from maps_utils import DistanceBetween
from graph import printNode


# Priority Queue Class
class PriorityQueue:
    """
    This class is Priority Queue.
    Look into python heapq for a similar usecase
    """

    def __init__(self):
        self.queue = []

    def insert_pq(self, cost, data_node):
        """
        Insert into the Priority queue

        :param      cost:       The cost
        :type       cost:       { type_description }
        :param      data_node:  The data node
        :type       data_node:  { type_description }
        """
        node_cost_combo = (cost, data_node)
        self.queue.append(node_cost_combo)

    def pop_pq(self):
        """
        Pops the element as per the rules of priority queue
        """
        try:
            max_idx = 0
            for i in range(len(self.queue)):
                if self.queue[i][0] < self.queue[max_idx][0]:
                    max_idx = i
            max_cost, max_cost_node = self.queue[max_idx]
            del self.queue[max_idx]
            return max_cost, max_cost_node
        except IndexError:
            exit()

    def len_pq(self):
        """
        Returns the length of the priority queue
        """
        # print('Length of queue : >> ', len(self.queue))
        return len(self.queue)

    def checkinPQ(self, node_to_check):
        """
        Checks for a node in the Priority Queue
        :param node_to_check: The node to check
        :type node_to_check: Node type
        :return: True or False
        :rtype: boolean
        """
        for all_tuples in self.queue:
            if compareNodes(all_tuples[1], node_to_check):
                return True
        return False


class Tree:
    """
    Tree data setructure. Used for RRT, RRT*
    """

    def __init__(self):
        self.tree = {}

    def setChild(self, parent_name, child_name):
        """
        Sets the child of a parent in the tree
        :param parent_name: Parent
        :type parent_name: Node
        :param child_name: Child
        :type child_name: Node
        :return: None
        :rtype: None
        """
        self.tree[parent_name] = child_name

    def setStart(self, parent_name, child_name):
        """
        Sets the start in the treee
        :param parent_name: Parent Node
        :type parent_name: Node
        :param child_name: Child Node
        :type child_name: Node
        :return: None
        :rtype: None
        """
        self.tree[parent_name] = child_name

    def getChildOfParent(self, parent_name):
        """
        Gets the child of the parent
        :param parent_name: Parent
        :type parent_name: Node
        :return: None
        :rtype: None
        """
        return self.tree[parent_name]

    def getTreeLength(self):
        """
        Returns the length of the tree
        :return: Length of the tree
        :rtype: Integer
        """
        return len(self.tree)

    def getNearestNode(self, node_to_check):
        """
        Amongst all the child nodes in the tree,
        return the node closest to the node_to_check
        :return: closest_tree_node
        :rtype: Node
        """

        min_dist = float('inf')
        for parent, child in self.tree.items():
            calc_distance = DistanceBetween(child, node_to_check)
            if calc_distance < min_dist:
                min_dist = calc_distance
                closest_tree_node = child

        return closest_tree_node
