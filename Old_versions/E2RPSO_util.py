import datetime
import random
import math
import numpy as np
from Continuous_grid import Grid, Agent
'''
function : detect obstacle
    detect the obstacle within Safe distance

    input:
    x : x position
    y : y position
    Obstacle : Obstacle instance

    return:
    out = 1 : detect obstacle
    out = 0 : no obstacle
'''
def detect_ob(x, y, size, radius, Obstacle):
    out = 0
    
    ob_x, ob_y, width, height = Obstacle

    x_min = math.ceil((ob_x - width))
    x_max = math.ceil((ob_x + width))
    y_min = math.ceil((ob_y - height))
    y_max = math.ceil((ob_y + height))



    if (x_min -radius <= x + size <= x_max + radius and y_min-radius <= y + size <= y_max + radius) or(x_min -radius <= x - size <= x_max + radius and y_min-radius <= y - size <= y_max + radius):
        return 1
    return out



'''
function : detect robot
    detect the robot within Safe distance

    input:
    x : x position
    y : y position
    ob : robot instance

    return:
    out = 1 : detect robot
    out = 0 : no obstacle
'''
def detect_ob_self(x, y, radius, agent):
    out = 0
    
    x_a = agent.x
    y_a = agent.y
    size = agent.half_size

    x_min = math.ceil((x_a - size))
    x_max = math.ceil((x_a + size))
    y_min = math.ceil((y_a - size))
    y_max = math.ceil((y_a + size))

    if (x_min - radius <= x <= x_max + radius) and (y_min - radius <= y <= y_max + radius):
        out = 1
    return out



'''
function getFitness
    calculate the fitness value in this x,y position

    input: 
    gx      : x position of one goal
    gy      : y position of one goal
    x       : x positoin 
    y       : y position

    return:
    the Square root of distance between x,y and gx,gy
'''
def getFitness(gx, gy, x, y):
    dis = math.sqrt((gx - x) * (gx - x) + (gy - y) * (gy - y))

    return dis

# update Gbest Pbest
def update_GbestPbest(r, Gx, Gy, Gbest, goal_list_x, goal_list_y, agent, v_x, v_y, t):
    gx = Gx
    gy = Gy
    gbest = Gbest
    f = 0
    for ttt in range(len(goal_list_x)):
        tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], agent.x, agent.y)
        f += tmp_f

    if f <= r.pbest:
        r.pbest_p = r.pbest
        r.pbest = f
        r.pbest_avg = (r.pbest_avg * t + f) / (t + 1)
        r.px = r.x
        r.py = r.y
    if f <= gbest:
        gbest = f
        gx = r.px
        gy = r.py
    # if can_print == 1:
    #     print("position of {} is {}+{}={} {}+{}={}  length = {}".format(i, int(r.x - v_x), v_x, int(r.x),
    #                                                                     int(r.y - v_y), v_y, int(r.y),
    #                                                                     math.sqrt(v_x * v_x + v_y * v_y)))
    #     print("position of {} ".format(i))
    #     print("f = {}".format(f))
    #     print(goal_list_x)
    #     print(goal_list_y)
    return gbest, gx, gy

# limit max velocity
def Limit_maxVelocity(v_x, v_y, v_limit):
    if abs(v_x) > v_limit / 2 or abs(v_y) > v_limit / 2:
        k = v_x * v_x + v_y * v_y
        k = math.sqrt(k)
        v_x = v_x * v_limit / k
        v_y = v_y * v_limit / k
    return v_x, v_y

def Unit_speed(r, gx, gy):
    xp = r.px - r.x
    xg = gx - r.x
    yp = r.py - r.y
    yg = gy - r.y
    p_len = math.sqrt(xp ** 2 + yp ** 2)
    if p_len > 0:
        xp = xp / p_len
        yp = yp / p_len
    g_len = math.sqrt(xg ** 2 + yg ** 2)
    if g_len > 0:
        xg = xg / g_len
        yg = yg / g_len
    return xp, yp, xg, yg

# Avoid obstacle
def avoidObstacle(r, obs_list , agents, i):
    po_x = r.x
    po_y = r.y
    ob = 0
    for jj in range(len(obs_list)):
        if detect_ob(r.x, r.y,r.half_size, r.radius, obs_list[jj]) == 1:
            ob = 1
            break

    for j, agent in enumerate(agents):
        if j != i:
            if detect_ob_self(r.x, r.y, agent.half_size+1, agent) == 1:
                ob = 1
                break

    # x,y direction
    zf_index_x = 1
    zf_index_y = 1
    if r.Vx != 0:
        zf_index_x = r.Vx / abs(r.Vx)
    if r.Vy != 0:
        zf_index_y = r.Vy / abs(r.Vy)
    # 8 direction
    tmp_list1 = [3, 3, 0, 3, -3, 0, -3, -3]
    tmp_list2 = [3, 0, 3, -3, 3, -3, 0, -3]

    # find 1 best direction in 8 to avoid obstacle
    if ob == 1:
        c3 = 100000
        for ixx in range(len(tmp_list1)):
            ii = tmp_list1[ixx]
            ij = tmp_list2[ixx]

            ii = ii * zf_index_x
            ij = ij * zf_index_y

            ob11 = 0
            for jj in range(len(obs_list)):

                if detect_ob(r.x + ii, r.y + ij, r.half_size, r.radius, obs_list[jj]) == 1:
                    ob11 = 1
                    break

            for j, agent in enumerate(agents):
                if j !=i:
                    if detect_ob_self(r.x + ii, r.y + ij, agent.half_size+1, agent) == 1:
                        ob11 = 1
                        break
            if (ob11 == 0):
                po_x += ii
                po_y += ij
                break

        # if can_print == 1:
        #     print("ob {} {}".format(po_x, po_y))

    return ob, po_x, po_y


# Find the farthest & emptiest area
def find_FarthestAndEmptiestArea(r, Out_list , SIDE):
    # Get current robot's position area
    area_id = int(int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))
    far_area_id = 0
    tmp_num = -10000

    # Find the farthest & emptiest area
    for j in range(len(Out_list)):
        x_d = abs(area_id % 20 - j % 20)
        y_d = int(abs(area_id - j) / 20)
        dis = math.sqrt(x_d * x_d + y_d * y_d)

        if (Out_list[j] * dis > tmp_num):
            tmp_num = Out_list[j] * dis
            far_area_id = j
    return area_id, far_area_id


def judge_allGoalReached(goal_list_x, goal_list_y, agent ,goal_c, Gbest, grid, radius):
    gbest = Gbest
    for xi in range(len(goal_list_x)):
        if getFitness(goal_list_x[xi], goal_list_y[xi], agent.x, agent.y) < grid.agents[0].radius + grid.target_half_size + 3:
            goal_c[xi] = goal_c[xi] + 1
    for xi in range(len(goal_c)):
        if (goal_c[xi] >= 1 and len(goal_c) > 0 and xi < len(goal_c)):
            grid.remove_target(goal_list_x[xi], goal_list_y[xi], radius)
            del goal_list_x[xi]
            del goal_list_y[xi]
            del goal_c[xi]
            

            gbest = 100000
            break
    return goal_list_x , goal_list_y , goal_c , gbest


def update_best_avg(agents):
    best_avg = 0
    for agent in agents:
        r = agent
        best_avg = best_avg + r.pbest
    best_avg = best_avg / len(agents)
    return best_avg

def init_E2RPSO(grid):
    Out_list = []
    for i in range(int(int(200) * int(200))):
        Out_list.append(100)
    # init Map area exploration rate
    for i in range(int(int(grid.width / 20) * int(grid.height / 20))):
        Out_list.append(100)

    goal_c = []
    for _ in range(grid.nr_of_targets):
        goal_c.append(0)


    obs_list = grid.obstacles
    # init gx,gy,gbest
    gbest = np.inf
    gx = 0
    gy = 0
    t = 0.0  # iteration time

    goal_list_x = []
    goal_list_y = []
    target_radius = 0
    for x, y, radius in grid.targets:
        goal_list_x.append(x)
        goal_list_y.append(y)
        target_radius = radius

    # init Gbest Pbest
    for agent in grid.agents:
        f = 0
        for ttt in range(len(goal_list_x)):
            tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], agent.x, agent.y)
            f += tmp_f

        if f < agent.pbest:
            agent.pbest_p = f
            agent.pbest = f
            agent.pbest_avg = f
            agent.px = agent.x
            agent.py = agent.y
            if f < gbest:
                gbest = f
                gx = agent.px
                gy = agent.py


    return Out_list,grid.agents,obs_list,goal_list_x,goal_list_y,goal_c, target_radius

