import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from math import factorial          #求阶乘库

# 参数设置
KP = 5        # 目标点吸引系数
ETA = 100     # 障碍物排斥系数

OSCILLATIONS_DETECTION_LENGTH = 0.3

show_animation = True

def comb(n, k):
    return factorial(n) // (factorial(k) * factorial(n-k))

def get_bezier_curve(points):
    n = len(points) - 1
    return lambda t: sum(comb(n, i)*t**i * (1-t)**(n-i)*points[i] for i in range(n+1))

def evaluate_bezier(points, total):
    bezier = get_bezier_curve(points)
    new_points = np.array([bezier(t) for t in np.linspace(0, 1, total)])
    return new_points[:, 0], new_points[:, 1]

def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr, o_r):

    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):                                  #找到距离最近的障碍物
        d = np.hypot(x - ox[i], y - oy[i]) - o_r[i]
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid]) - o_r[minid]        #只考虑最近障碍物的作用

    if dq <= rr:                                   #超过阈值，则不产生作用
        if dq <= 0.1:
            dq = 0.01

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():

    motion = [[1, 0],   #每一步的八个方向
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]
    return motion

# def oscillations_detection(previous_ids, ix, iy):
#     previous_ids.append((ix, iy))

#     if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
#         previous_ids.popleft()

#     # check if contains any duplicates by copying into a set
#     previous_ids_set = set()
#     for index in previous_ids:
#         if index in previous_ids_set:
#             return True
#         else:
#             previous_ids_set.add(index)
#     return False


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr, o_r, xmin, xmax, ymin, ymax):

    
    
    # 计算人工势场
    xw = int((xmax - xmin) / reso)
    yw = int((ymax - xmin) / reso)

    pmap = [[0 for i in range(xw)] for i in range(yw)]
    flag = pmap

    for ix in range(xw):
        x = ix * reso + xmin

        for iy in range(yw):
            y = iy * reso + ymin
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr, o_r)
            uf = ug + uo
            pmap[ix][iy] = uf

    # 根据人工势场规划路径点
    d = np.hypot(sx - gx, sy - gy)      #计算出发点到目标点之间的距离，np.hypot 用于计算直角三角形斜边
    ix = round((sx - xmin) / reso)      #算出发点对应的刻度点（步长），并赋给当前位置点
    iy = round((sy - ymin) / reso)

    gix = round((gx - xmin) / reso)     #计算目标点对应的刻度点
    giy = round((gy - xmin) / reso)

    rx, ry = [sx], [sy]                 #用于保存轨迹点
    motion = get_motion_model()
    previous_ids = deque()              #双向队列

    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])         #对八个方向都进行计算

            if(flag[inx][iny]) != 1:
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny

        ix = minix
        iy = miniy
        flag[ix][iy] = 1
        xp = ix * reso + xmin
        yp = iy * reso + ymin
        d = np.hypot(gx - xp, gy - yp)      #计算当前点到目标点的距离
        rx.append(xp)
        ry.append(yp)


        # if oscillations_detection(previous_ids, ix, iy):
        #     print("Oscillation detected at ({},{})!".format(ix, iy))
        #     break

        if show_animation:
            plt.plot(xp, yp, ".b")
            plt.pause(0.01)

    rx.append(gx)
    ry.append(gy)

    print("Goal!!")
    return rx, ry

def draw_axis(sx,sy,gx, gy, ox, oy, xmin, xmax, ymin, ymax, o_r):
    fig = plt.figure(figsize=(5, 5))
    ax = fig.gca()

    ax.set_xticks(np.arange(xmin, xmax + 0.3, 0.3))
    ax.set_yticks(np.arange(ymin, ymax + 0.3, 0.3))
    for i in range(len(ox)):
        circle = plt.Circle((ox[i], oy[i]), o_r[i], color='k', fill=True)
        plt.gcf().gca().add_artist(circle)

    plt.plot(sx, sy, "*r")
    plt.plot(gx, gy, "pg")
    plt.grid()

def planning():
    
    print("potential_field_planning start")

    xmin, xmax = -1.5, 1.5                      #设置人工势场的边界（也是坐标轴范围）
    ymin, ymax = -1.5, 1.5

    sx, sy = -1.2, -1.2                         # 设置出发点
    gx, gy = 1.2, 1.2                           # 设置目标点
    grid_size = 0.02                            # 单步长度
    robot_radius = 0.3                         # 机器人尺寸，轨迹点与障碍物圆的最短距离

    ox = [0, 0.3]                   # 障碍物横坐标
    oy = [-0.6, 1.2]                 # 障碍物纵坐标

    obstacle_radius = [0.6, 0.5]      #障碍物半径

    if show_animation:
        draw_axis(sx, sy, gx, gy, ox, oy, xmin, xmax, ymin, ymax, obstacle_radius)
    
    # 返回路径点
    rx, ry = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius, obstacle_radius, xmin, xmax, ymin, ymax)

    points = np.array([[rx[0], ry[0]], [rx[1], ry[1]]])     #创建路径的二维数组

    for j in range(2, len(rx), 1):
        a = np.array([[rx[j], ry[j]]])
        points = np.concatenate((points, a), axis=0)

    x, y = points[:, 0], points[:, 1]
    bx, by = evaluate_bezier(points, len(rx))                    #平滑
    
    # cyaw = []
    # for i in range(len(bx)-1):
    #     if bx[i+1]-bx[i] == 0:
    #         if by[i+1]-by[i]>0:
    #             yaw = np.pi / 2.0
    #         else:
    #             yaw = -np.pi / 2.0
    #     if bx[i+1]-bx[i] > 0:
    #         yaw = math.atan((by[i+1]-by[i])/((bx[i+1])-bx[i]))
    #     if bx[i+1]-bx[i] < 0:
    #         yaw = math.atan((by[i+1]-by[i])/((bx[i+1])-bx[i])) + np.pi
    #         if yaw > np.pi:
    #             yaw = yaw - 2 * np.pi
    #     cyaw.append(yaw)
    # cyaw.append(cyaw[-1])
    diff_bx = bx[1:-1] - bx[0:-2]
    diff_by = by[1:-1] - by[0:-2]
    cyaw = np.arctan2(diff_bx, diff_by)
    cyaw = np.hstack((cyaw, cyaw[-1]))

    plt.plot(bx, by, 'g-')

    if show_animation:
        plt.show()
    
    return bx, by, cyaw

if __name__ == '__main__':
    print(__file__ + " start!!")
    planning()
    print(__file__ + " Done!!")
