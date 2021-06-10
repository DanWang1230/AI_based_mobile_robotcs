import numpy as np
from RRTTree import RRTTree
import time

class RRTPlannerNonholonomic_multiruns(object):

    def __init__(self, planning_env, bias=0.05, max_iter=1000000, num_control_samples=25):
        self.env = planning_env                 # Car Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                        # Goal Bias
        self.max_iter = max_iter                # Max Iterations
        self.num_control_samples = 25           # Number of controls to sample

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE

        plan_time = time.time()
        cost = 0

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        self.tree.edges[0] = None # parent of root node is none

        for _ in range(self.max_iter):
            rand_state = self.sample(goal_config)
            near_state_ID, near_state = self.tree.GetNearestVertex(rand_state)
            new_state, delta_t_new = self.extend(near_state, rand_state)
            
            new_cost = self.tree.costs[near_state_ID] + delta_t_new
            new_state_ID = self.tree.AddVertex(new_state, new_cost)
            self.tree.AddEdge(near_state_ID, new_state_ID)
            
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
        """ Extend method for non-holonomic RRT

            Generate n control samples, with n = self.num_control_samples
            Simulate trajectories with these control samples
            Compute the closest closest trajectory and return the resulting state (and cost)
        """
        # TODO: YOUR IMPLEMENTATION HERE
        x_sample_all = []
        delta_t_all = []
        for _ in range(self.num_control_samples):
            linear_vel, steer_angle = self.env.sample_action()
            x_sample, delta_t = self.env.simulate_car(x_near, x_rand, linear_vel, steer_angle)
            if x_sample is not None:
                x_sample_all.append(x_sample)
                delta_t_all.append(delta_t)
        
        x_new = x_sample_all[0]
        delta_t_new = delta_t_all[0]
        for i in range(1, len(x_sample_all)):
            if self.env.compute_distance(x_sample_all[i], x_rand) < self.env.compute_distance(x_new, x_rand):
                x_new = x_sample_all[i]
                delta_t_new = delta_t_all[i]
        return x_new, delta_t_new
            
    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()