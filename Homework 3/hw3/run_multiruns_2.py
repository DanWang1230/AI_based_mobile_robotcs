import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from CarEnvironment import CarEnvironment
from AStarPlanner import AStarPlanner
from RRTPlanner import RRTPlanner
from RRTPlanner_multiruns import RRTPlanner_multiruns
from RRTStarPlanner import RRTStarPlanner
from RRTStarPlanner_multiruns import RRTStarPlanner_multiruns
from RRTPlannerNonholonomic import RRTPlannerNonholonomic

# def main(planning_env, planner, start, goal, argplan = 'astar'):

#     # Notify.
#     input('Press any key to begin planning...')

#     planning_env.init_visualizer()

#     # Plan.
#     plan, cost, plan_time = planner.Plan(start, goal)

    # # Visualize the final path.
    # tree = None
    # visited = None
    # if argplan != 'astar':
    #     tree = planner.tree
    # else:
    #     visited = planner.visited
    # planning_env.visualize_plan(plan, tree, visited)
    # plt.show()


if __name__ == "__main__":
    
    # parser = argparse.ArgumentParser(description='script for testing planners')

    # parser.add_argument('-m', '--map', type=str, default='map1.txt',
    #                     help='The environment to plan on')    
    # parser.add_argument('-p', '--planner', type=str, default='astar',
    #                     help='The planner to run (astar, rrt, rrtstar, nonholrrt)')
    # parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    # parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)
    # parser.add_argument('-eps', '--epsilon', type=float, default=1.0, help='Epsilon for A*')
    # parser.add_argument('-eta', '--eta', type=float, default=1.0, help='eta for RRT/RRT*')
    # parser.add_argument('-b', '--bias', type=float, default=0.05, help='Goal bias for RRT/RRT*')

    # args = parser.parse_args()

    # # First setup the environment and the robot.
    # dim = 3 if args.planner == 'nonholrrt' else 2
    # args.start = np.array(args.start).reshape(dim, 1)
    # args.goal = np.array(args.goal).reshape(dim, 1)
    # if args.planner == 'nonholrrt':
    # 	planning_env = CarEnvironment(args.map, args.start, args.goal)
    # else:
	    
    # # Next setup the planner
    # if args.planner == 'astar':
    #     planner = AStarPlanner(planning_env, args.epsilon)
    # elif args.planner == 'rrt':
        
    # elif args.planner == 'rrtstar':
    #     planner = RRTStarPlanner(planning_env, bias=args.bias, eta=args.eta)
    # elif args.planner == 'nonholrrt':
    # 	planner = RRTPlannerNonholonomic(planning_env, bias=args.bias)
    # else:
    #     print('Unknown planner option: %s' % args.planner)
    #     exit(0)

    map2 = 'map2.txt'
    start = np.array([321, 148]).reshape(2,1)
    goal = np.array([106, 202]).reshape(2,1)
    planning_env = MapEnvironment(map2, start, goal)
   
    
    costs = []
    plan_times = []

    # planning_env.init_visualizer()
    
    for i in range(10):
        planner = RRTStarPlanner_multiruns(planning_env, bias=0.2, eta=1) # RRTPlanner_multiruns for RRT
        plan, cost, plan_time = planner.Plan(start, goal)
        costs.append(cost)
        plan_times.append(plan_time)

    # # Visualize the final path.
    # tree = planner.tree
    # visited = None
    # planning_env.visualize_plan(plan, tree, visited)
    # plt.show()
    std_costs = np.std(costs)
    mean_costs = np.mean(costs)
    std_plan_times = np.std(plan_times)
    mean_plan_times = np.mean(plan_times)

    print(costs)
    print(mean_costs)
    print(std_costs)
    print(plan_times)
    print(mean_plan_times)
    print(std_plan_times)

    
