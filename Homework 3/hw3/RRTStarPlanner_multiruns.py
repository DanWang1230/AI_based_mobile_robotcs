import numpy as np
from RRTTree import RRTTree
import time

class RRTStarPlanner_multiruns(object):

    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 1000000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def Plan(self, start_config, goal_config, rad=50):
        # TODO: YOUR IMPLEMENTATION HERE

        plan_time = time.time()
        cost = 0

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        self.tree.edges[0] = None # parent of root node is none

        for _ in range(self.max_iter):
            rand_state = self.sample(goal_config)
            near_state_ID, near_state = self.tree.GetNearestVertex(rand_state)
            new_state = self.extend(near_state, rand_state)

            if self.env.edge_validity_checker(near_state, new_state):
                new_cost = self.tree.costs[near_state_ID] + self.env.compute_distance(near_state, new_state)
                new_state_ID = self.tree.AddVertex(new_state, new_cost)
                self.tree.AddEdge(near_state_ID, new_state_ID)

                vids, vertices = self.tree.GetNNInRad(new_state, rad)
                for i in range(len(vertices)):
                    if self.env.edge_validity_checker(vertices[i], new_state):
                        if self.tree.costs[vids[i]] + self.env.compute_distance(vertices[i], new_state) < self.tree.costs[new_state_ID]:
                            self.tree.costs[new_state_ID] = self.tree.costs[vids[i]] + self.env.compute_distance(vertices[i], new_state)
                            self.tree.AddEdge(vids[i], new_state_ID)

                for i in range(len(vertices)):
                    if self.env.edge_validity_checker(vertices[i], new_state):
                        if self.tree.costs[new_state_ID] + self.env.compute_distance(vertices[i], new_state) < self.tree.costs[vids[i]]:
                            self.tree.AddEdge(new_state_ID, vids[i])

                
                if self.env.goal_criterion(new_state, goal_config):
                    plan_time = time.time() - plan_time
                    cost = self.tree.costs[new_state_ID]
                    print("Cost: %f" % cost)
                    print("Planning Time: %ds" % plan_time)
                    currID = new_state_ID
                    plan = []
                    while currID is not None:
                        plan.append(self.tree.vertices[currID])
                        currID = self.tree.edges[currID]

                    plan.reverse()
                    return np.concatenate(plan, axis=1), cost, plan_time
        return None, 0, 0
        

    def extend(self, x_near, x_rand):
        # TODO: YOUR IMPLEMENTATION HERE
        if self.eta == 1:
            x_new = x_rand
        if self.eta == 0.5:
            x_new = (x_near + x_rand) / 2
        return x_new

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()
