import operator

import numpy as np
import networkx as nx

def extend(x_near, x_rand, eta):
    x_new = x_near + eta * (x_rand - x_near)
    return x_new

def sample(rm, goal, bias):
    # Sample random point from map
    if np.random.uniform() < bias:
        return goal
    rm.num_vertices = 1
    return rm.sample_vertices()[:1]

def rrt(rm, start, goal, bias=0.05, eta=0.5, max_iter=1000):
    """Compute a path from start to goal on a roadmap.

    Args:
        rm: Roadmap instance to plan on
        start, goal: integer labels for the start and goal states
        bias: probability to sample goal as next point when extending the tree
        eta: nearest neighbor tries to extend to the sample point by step size eta
        max_iter: maximum number of iterations to run

    Returns:
        plan_ids: a sequence of node labels, including the start and goal
        plan: a sequence of nodes, including the start and goal
    """

    # seeding (do not touch this)
    np.random.seed(0)

    if start not in rm.graph or goal not in rm.graph:
        msg = "Either start {} or goal {} is not in G"
        raise nx.NodeNotFound(msg.format(start, goal))

    # convert node indices to vertices
    start_config = rm.vertices[start][None]
    goal_config = rm.vertices[goal][None]

    # initialize the Tree
    tree = RRTTree(rm)

    # you might find the following functions useful:
    # sample, extend, rm.problem.check_state_validity, rm.problem.check_edge_validity, rm.problem.goal_criterion
    
    x_goal_id = None
    x_init_id = tree.AddVertex(start_config)

    for _ in range(max_iter):
        
        # 1. draw a new sample
        # 2. get the nearest neighbor from the tree
        # 3. connect the new sample to the nearest neighbor with step size eta
        # BEGIN QUESTION 5
        "*** REPLACE THIS LINE ***"
        raise NotImplementedError
        # END QUESTION 5

        # increase counter by 1
        rm.edges_evaluated += 1

        # 1. check if the new sample and edge are valid
        # 2. if so, add the sample and edge to the tree
        # 3. finally, check whether the goal has been reached and if so, terminate the search
        # BEGIN QUESTION 5
        "*** REPLACE THIS LINE ***"
        raise NotImplementedError
        # END QUESTION 5
        
    if x_goal_id is None:
        return np.array([]), []
    
    x_new_id = x_goal_id

    # create plan by moving backward from the node meeting the goal criteria to the start
    plan = tree.vertices[x_new_id]
    plan_ids = [x_new_id]
    while x_new_id is not x_init_id:
        x_new_id = tree.edges[x_new_id]
        plan = np.vstack((plan, tree.vertices[x_new_id]))
        plan_ids.append(x_new_id)
    
    # flip plan from goal -> start to start -> goal
    plan = np.flip(plan, axis=0)
    plan_ids = np.flip(np.array(plan_ids), axis=0).tolist()

    # set start and goal idx
    rm.start = plan_ids[0]
    rm.goal = plan_ids[-1]

    # rebuild graph for plotting
    rm.vertices = np.concatenate(tree.vertices, axis=0)
    rm.weighted_edges = np.stack([[u, v, rm.problem.compute_heuristic(tree.vertices[u][0], tree.vertices[v][0]).item()] for u, v in tree.edges.items()])
    rm.rebuild_graph()

    return plan_ids, plan


class RRTTree(object):
    
    def __init__(self, rm):
        
        self.rm = rm
        self.vertices = []
        self.costs = []
        self.edges = dict()

    def GetRootID(self):
        """ Return the ID of the root in the tree.
        """
        return 0

    def GetNearestVertex(self, config):
        """ Return the nearest state ID in the tree.
            
            @param config: Sampled configuration.
        """
        dists = []
        for v in self.vertices:
            dists.append(self.rm.problem.compute_heuristic(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
            
    def GetNNInRad(self, config, rad):
        ''' Return neighbors within ball of radius. Useful for RRT*

            @param config: Sampled configuration.
            @param rad ball radius
        '''
        rad = np.abs(rad)
        vids = []
        vertices = []
        for idx, v in enumerate(self.vertices):
            if self.rm.problem.compute_heuristic(config, v) < rad:
                vids.append(idx)
                vertices.append(v)

        return vids, vertices


    def AddVertex(self, config, cost=0):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices.append(config)
        self.costs.append(cost)
        return vid

    def AddEdge(self, sid, eid):
        '''
        Adds an edge in the tree.
        @param sid start state ID.
        @param eid end state ID.
        '''
        self.edges[eid] = sid