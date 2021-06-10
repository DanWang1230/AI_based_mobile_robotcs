import numpy as np

class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon):
        self.env = planning_env
        self.nodes = {}
        self.epsilon = epsilon
        self.visited = np.zeros(self.env.map.shape)

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE 
        CLOSED = {}
        OPEN = {}
        Backlinks = {}
        OPEN[tuple(start_config.reshape(1,2)[0])] = self.epsilon * self.env.h(start_config) # g=0 for start state
        Backlinks[tuple(start_config.reshape(1,2)[0])] = None
        state_count = 0
        cost = 0
        gscore  = np.ones(self.env.map.shape) * 100000000000

        while len(OPEN) > 0:
            # get the state in "OPEN" with minimum f value and the index of the state
            minf =  min(OPEN.values())
            for key in OPEN:
                if OPEN[key] == minf:
                      cur_state = (key, minf)
                      break
            del OPEN[key]
            CLOSED[cur_state[0]] = cur_state[1] # insert state into CLOSED dict
            
            self.visited[int(cur_state[0][0])][int(cur_state[0][1])] = 1

            if self.env.goal_criterion(np.array(cur_state[0]).reshape(2,1), goal_config):
                cost = cur_state[1] # h is 0 for goal state
                print("States Expanded: %d" % state_count)
                print("Cost: %f" % cost)
                plan = []
                current = cur_state[0]
                while current is not None:
                    plan.append(np.array(current).reshape(2,1))
                    current = Backlinks[current]
                plan.reverse()
                return np.concatenate(plan, axis=1)

            state_count += 1

            for action in [[0, 1], [0, -1], [1, 1], [1, -1], [1, 0], [-1, 1], [-1, -1], [-1, 0]]:
                successor = tuple(map(sum, zip(cur_state[0], tuple(action))))
                successor_array = np.array(successor).reshape(2,1)
                if self.env.state_validity_checker(successor_array) and successor not in CLOSED:
                    gvalue = cur_state[1] + self.env.compute_distance(np.array(cur_state[0]).reshape(2,1), successor_array)\
                        - self.epsilon * self.env.h(np.array(cur_state[0]).reshape(2,1))
                    hvalue = self.epsilon * self.env.h(successor_array)

                    if gvalue < gscore[int(successor[0])][int(successor[1])]:
                        gscore[int(successor[0])][int(successor[1])] = gvalue
                        fvalue = gvalue + hvalue
                        OPEN[successor] =  fvalue
                        Backlinks[successor] = cur_state[0]
                    


        

