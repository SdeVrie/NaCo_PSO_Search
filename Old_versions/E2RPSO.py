import math
from enum import Enum

import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import matplotlib.animation as animation
import datetime
import numpy as np
import time
import random
import configparser
import E2RPSO_util
from Continuous_grid import  visualize_grid




# save data
ani_list = []  # trajectory info
Out_list = []
obs_list = []
for i in range(int(int(200) * int(200))):
    Out_list.append(100)


'''
function E2RPSO
    main function of PSO

    parameter:
        Side        : the map size
        Size        : num of robots
        Goal_num    : num of goal

    returns
        Iteration time, goal num found


'''

from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch


def E2RPSO(grid, T, step_size):
    # Side length of square map
    SIDE = 100
    Size = 6 #config.getint('E2RPSO','Size')
    Goal_num = 10 
    V_LIMIT = step_size
    # Max iteration times
    T = T
    
    
    C1 = 0.4
    # parameter for Gbest
    C2 = 0.9
    C3_index = 5
    # parameter for avoid obstancle
    C4_index = 10
    # Map area exploration rate increment
    K = 1

    C3 = C3_index * C2
    C4 = C4_index * SIDE

    gbest = np.inf
    gx = 0
    gy = 0
    t = 0.0  # iteration time

    # initial

    Out_list, agents, obs_list, goal_list_x, goal_list_y, goal_c, target_radius = E2RPSO_util.init_E2RPSO(grid)
    visualize_grid(grid, 0)

    c1 = C1
    c2 = C2

    

    # begin iteration
    while t < T:
        t = t + 1

        # update best_avg
        best_avg = E2RPSO_util.update_best_avg(agents)

        # print info
        # if can_print == 1:
        #     print("---------------{}---------------".format(t))
        #     print("g_best = {} {}".format(gx, gy))
        #     print("g_best = {}".format(gbest))

        # if find all target , return
        if len(goal_list_x) == 0:
            break

        # Traverse all robots
        for i, agent in enumerate(agents):
            # do avoid local best every 10 iteration
            if t % 10 == 0:
                c1 = C1
                c2 = C2
            c3 = 0
            r = agent
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            r3 = 1


            # end?
            if len(goal_list_x) == 0:
                break

            # update inertial component
            w = 0.9 - 0.5 * (1 - (min(r.pbest_p, r.pbest) / max(r.pbest_p, r.pbest))) + (
                        min(gbest, best_avg) / max(gbest, best_avg))


            # Judge whether the goal is reached and update info
            goal_list_x, goal_list_y, goal_c, gbest = E2RPSO_util.judge_allGoalReached(goal_list_x, goal_list_y, agent ,goal_c , gbest, grid, target_radius)

            # if find all target , return
            if len(goal_c) == 0:
                break

            # # if this robot reach a goal, stop
            # if (reach == 1):
            #     # print("reach of robot {}".format(i))
            #     agent.reach = 1
            #     continue

            # Find the farthest & emptiest area
            area_id, far_area_id = E2RPSO_util.find_FarthestAndEmptiestArea(r, Out_list , SIDE)


            # farthest & emptiest exploration rate increment - K
            Out_list[far_area_id] = Out_list[far_area_id] - K

            # Determine whether to enter local optimum or not, then set C3
            if Out_list[area_id] < -800:
                c2 = 0
                c1 = 0
                c3 = 10 * C3
            elif Out_list[area_id] < -100:
                c3 = C3
                c2 = 0
                c1 = 0
            # prepare for unit speed x,y component
            pu_x = ((far_area_id + 1) % 20) * 50
            pu_y = int((far_area_id) / 20 + 1) * 50
            # avoid obstacle and decide the avoiding obstacle point
            ob, po_x, po_y = E2RPSO_util.avoidObstacle(r, obs_list, agents, i)
            if ob == 1:
                c3 = C3
                pa_x = po_x
                pa_y = po_y
            else:
                pa_x = pu_x
                pa_y = pu_y

            # Unit speed for x and y component
            xp, yp, xg, yg = E2RPSO_util.Unit_speed(r, gx, gy)

            # update velocity , using E2RPSO
            v_x = w * r.Vx + r1 * c1 * xp + c2 * r2 * xg + c3 * r3 * (pa_x - r.x)
         
            v_y = w * r.Vy + r1 * c1 * yp + c2 * r2 * yg + c3 * r3 * (pa_y - r.y)
            # limit max velocity
            v_x, v_y = E2RPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)
            r.setVelocity(v_x, v_y)
            # Auxiliary judgment enters local optimum
            if t % 10 == 0:
                r.lastx = r.x + v_x
                r.lasty = r.y + v_y

            # update position
            r_x = r.x + v_x
            r_y = r.y + v_y
            r.setcoords(r_x, r_y)
            # print(r.x)
            # print(r.y)

            # deal root out of map
            if (r.x - r.half_size < 0 or r.y - r.half_size  < 0 or r.x + r.half_size  > SIDE or r.y + r.half_size  > SIDE):
                r.x = r.x - v_x
                r.y = r.y - v_y
                r.Vx = -r.Vx
                r.Vy = -r.Vy

            grid.pos_change()

            # update Pbest & Gbest
            gbest, gx, gy = E2RPSO_util.update_GbestPbest(r, gx, gy, gbest, goal_list_x, goal_list_y, agent, v_x, v_y, t)
            cell_size = 5  # or any other constant
            grid_cols = SIDE // cell_size
            x_idx = int(r.x // cell_size)
            y_idx = int(r.y // cell_size)
            index = y_idx * grid_cols + x_idx
            Out_list[index] -= 1
            
            grid.agents[i].setcoords(r.x, r.y)
            
        grid.pos_change()
        if t % 10 == 0:  
            visualize_grid(grid, t,name='E2RPSO')

        
    visualize_grid(grid, t, done=True, name='E2RPSO')
    return int(t), Goal_num - len(goal_list_x), grid.total_distance_covered()











