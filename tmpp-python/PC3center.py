# -*- coding: utf-8 -*-

import numpy as np
from merge_point import Point_Merge
import math


def SphereEqu(Point, r, rate):
    P_num = Point.shape[1]
    y_max = np.max(Point[1, :])
    y_thred = y_max - rate * r
    for i in range(P_num - 1, -1, -1):
        if Point[1, i] < y_thred:
            Point = np.delete(Point, i, axis=1)

    Coefficient = np.concatenate((Point.T, np.ones((Point.shape[1], 1))), axis=1)
    I = np.eye(Coefficient.shape[1])
    eps = 1e-6
    value = -np.sum(np.power(Coefficient[:, :3], 2), axis=1)
    par = np.linalg.solve(np.dot(Coefficient.T, Coefficient) + eps * I, np.dot(Coefficient.T, value))
    Center = -par[0:3] / 2
    Radius = np.sqrt(np.sum(Center ** 2) - par[-1])

    return Center, Radius


def ParSet(TargType, Method):
    Par = {}
    Par['TargType'] = TargType
    Par['Method'] = Method
    # Par['Method'] = 4
    Par['rate'] = 1.1

    if TargType == 1:
        Par['TargSize_1'] = 0.6
        Par['TargSize_2'] = 0.6
    elif TargType == 2:
        Par['TargSize'] = 0.3
        Par['rThred'] = 0.1

    return Par


def Centering_3DPC(PointCloud, ParIn):
    # 若点云为空集
    Center = np.array([1e5, 1e5, 1e5])
    ParOut = 1e5

    # 若点云不为空集
    if PointCloud.shape[1] != 0:
        if ParIn['TargType'] == 1:  # 方型标靶
            if ParIn['Method'] == 1:  # 平均法
                Center = np.mean(PointCloud, axis=1)
            elif ParIn['Method'] == 2:  # 滑窗法
                # x
                Max = np.max(PointCloud[0, :])
                Min = np.min(PointCloud[0, :])
                if (Max - Min) <= ParIn['TargSize_1']:
                    Center[0] = np.mean([Min, Max])
                else:
                    delta = np.arange(0, Max - Min - ParIn['TargSize_1'], 0.01)
                    Count = np.inf * np.ones(delta.shape)
                    for j in range(len(delta)):
                        Count[j] = np.sum((PointCloud[0, :] >= (Min + delta[j])) & (
                        PointCloud[0, :] <= (Min + delta[j] + ParIn['TargSize_1'])))
                    idx = np.argmax(Count)
                    Center[0] = np.mean([Min + delta[idx], Min + delta[idx] + ParIn['TargSize_1']])

                # y
                Center[1] = np.mean(PointCloud[1, :])

                # z
                Max = np.max(PointCloud[2, :])
                Min = np.min(PointCloud[2, :])
                if (Max - Min) <= ParIn['TargSize_2']:
                    Center[2] = np.mean([Min, Max])
                else:
                    delta = np.arange(0, Max - Min - ParIn['TargSize_2'], 0.01)
                    Count = np.inf * np.ones(delta.shape)
                    for j in range(len(delta)):
                        Count[j] = np.sum((PointCloud[2, :] >= (Min + delta[j])) & (
                        PointCloud[2, :] <= (Min + delta[j] + ParIn['TargSize_2'])))
                    idx = np.argmax(Count)
                    Center[2] = np.mean([Min + delta[idx], Min + delta[idx] + ParIn['TargSize_2']])

        elif ParIn['TargType'] == 2:  # 球型标靶
            if ParIn['Method'] == 1:  # 平均法
                Center = np.mean(PointCloud, axis=1)
            elif ParIn['Method'] == 2:  # 球心法
                print('---------------method=2----------------')
                Center, ParOut = SphereEqu(PointCloud, ParIn['TargSize'] / 2, ParIn['rate'])
                # 如果半径估计不是很准，需要纠正y坐标  0.2 - 0.15 = 0.05 < 0.1
                delta_r = ParOut - ParIn['TargSize'] / 2
                if abs(delta_r) < ParIn['rThred']:
                    d = np.sqrt(np.sum(Center ** 2))
                    Center = (1 + delta_r / d) * Center
            elif ParIn['Method'] == 3:  # 滑窗法
                print('--------------------method=3-------------')
                # x
                Max = np.max(PointCloud[0, :])
                Min = np.min(PointCloud[0, :])
                if (Max - Min) <= ParIn['TargSize']:
                    Center[0] = np.mean([Min, Max])
                else:
                    delta = np.arange(0, Max - Min - ParIn['TargSize'], 0.01)
                    Count = np.inf * np.ones(delta.shape)
                    for j in range(len(delta)):
                        Count[j] = np.sum((PointCloud[0, :] >= (Min + delta[j])) & (
                        PointCloud[0, :] <= (Min + delta[j] + ParIn['TargSize'])))
                    idx = np.argmax(Count)
                    Center[0] = np.mean([Min + delta[idx], Min + delta[idx] + ParIn['TargSize']])

                # y
                Center[1] = np.max(PointCloud[1, :]) - ParIn['TargSize'] / 2

                # z
                Max = np.max(PointCloud[2, :])
                Min = np.min(PointCloud[2, :])
                if (Max - Min) <= ParIn['TargSize']:
                    Center[2] = np.mean([Min, Max])
                else:
                    delta = np.arange(0, Max - Min - ParIn['TargSize'], 0.01)
                    Count = np.inf * np.ones(delta.shape)
                    for j in range(len(delta)):
                        Count[j] = np.sum((PointCloud[2, :] >= (Min + delta[j])) & (
                        PointCloud[2, :] <= (Min + delta[j] + ParIn['TargSize'])))
                    idx = np.argmax(Count)
                    Center[2] = np.mean([Min + delta[idx], Min + delta[idx] + ParIn['TargSize']])

                x0, y0, z0 = Center[0], Center[1], Center[2]
                point0 = PointCloud[0, :]
                radius = math.sqrt((point0[0] - x0)**2 + (point0[1] - y0)**2 + (
                            point0[2] - z0)**2)
                ParOut = radius

            elif ParIn['Method'] == 4:  # 改进滑窗法
                print('------------------method = 4-------------')
                # x
                Max = np.max(PointCloud[0, :])
                Min = np.min(PointCloud[0, :])
                if (Max - Min) <= (ParIn['TargSize'] + 0.05):
                    Center[0] = np.mean([Min, Max])
                else:
                    delta = np.arange(0, Max - Min - ParIn['TargSize'], 0.01)
                    Count = np.inf * np.ones(delta.shape)
                    for j in range(len(delta)):
                        Count[j] = np.sum((PointCloud[0, :] >= (Min + delta[j])) & (
                        PointCloud[0, :] <= (Min + delta[j] + ParIn['TargSize'])))
                    idx = np.argmax(Count)
                    Center[0] = np.mean([Min + delta[idx], Min + delta[idx] + ParIn['TargSize']])
                # y
                xmax = np.max(PointCloud[0, :])
                xmin = np.min(PointCloud[0, :])
                half_r = abs(xmax - xmin) / 2.0
                # Center[1] = np.max(PointCloud[1, :]) - ParIn['TargSize'] / 2
                Center[1] = np.max(PointCloud[1, :]) - half_r
                # z
                initial_z_estimate = np.mean(PointCloud[2, :])
                center_x = Center[0]
                center_y = Center[1]
                r = 0.15

                center_z1 = cal_min_dist(center_x, center_y, r, PointCloud, initial_z_estimate)
                Center[2] = center_z1

                x0, y0, z0 = Center[0], Center[1], Center[2]
                point0 = PointCloud[0, :]
                radius = math.sqrt((point0[0] - x0)**2 + (point0[1] - y0)**2 + (
                            point0[2] - z0)**2)
                ParOut = radius

    return Center, ParOut



def cal_min_dist(center_x, center_y, r, points, mean_z):
    center_z = mean_z
    res = float('inf')
    # 设置起始点、终止点和步长
    start_value = -15
    end_value = 15
    step = 1
    tmp = mean_z

    # 使用 for 循环遍历
    for j in range(start_value, end_value + 1, step):
        scaled_j = j / 100
        # 在这里进行你的操作，例如显示当前值
        z_e = tmp + scaled_j
        total = 0
        for i in range(points.shape[1]):
            x = points[0, i]
            y = points[1, i]
            z = points[2, i]
            total += (np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2 + (z - z_e) ** 2) - 0.0225)
        if total < res:
            res = total
            center_z = z_e
    return center_z


def Centering(point1, point2, point3, x, y):
    FrameNum = 1  # 帧数
    yThred = 0.3 / (2 * math.tan(math.radians(2 / 3)))  # 判别距离 12.8m
    LidarEff = np.ones((1, 3))  # 判断能扫描到标靶的雷达个数
    TargType = 2  # 指定 TargType 变量的值

    if point1.shape[0] == 0:
        LidarEff[0][0] = 0
    if point2.shape[0] == 0:
        LidarEff[0][1] = 0
    if point3.shape[0] == 0:
        LidarEff[0][2] = 0
    LidarEff_Count = np.sum(LidarEff, axis=1)[0]

    pos_RoadO_LidarR_Uest = np.ones((3, FrameNum)) * 1e5
    r_Uest = np.zeros(FrameNum)

    for j in range(1, FrameNum + 1):
        pos_LidarPC_LidarR = Point_Merge(point1.T, point2.T, point3.T, x, y)
        if pos_LidarPC_LidarR.shape[0] == 0:
            pos_RoadO_LidarR_Uest[:, j - 1] = [1e5, 1e5, 1e5]
            break

        elif abs(np.min(pos_LidarPC_LidarR[1, :])) < yThred and LidarEff_Count > 1:
            Method = 2
        else:
            # Method = 3
            Method = 4

        ParIn = ParSet(TargType, Method)
        pos_RoadO_LidarR_Uest[:, j - 1], r_Uest[j - 1] = Centering_3DPC(pos_LidarPC_LidarR, ParIn)

        delta_r = r_Uest[j - 1] - ParIn['TargSize'] / 2
        if abs(delta_r) >= ParIn['rThred']:
            Method = 4
            ParIn = ParSet(TargType, Method)
            pos_RoadO_LidarR_Uest[:, j - 1], r_Uest[j - 1] = Centering_3DPC(pos_LidarPC_LidarR, ParIn)

    return pos_RoadO_LidarR_Uest


def Centering_new(pos_LidarPC_LidarR, LidarEff_Count):
    FrameNum = 1  # 帧数
    # yThred = 0.3 / (2 * math.tan(math.radians(2 / 3)))  # 判别距离 12.8m  缩小距离，滑窗法会
    yThred = 10
    # LidarEff = np.ones((1, 3))  # 判断能扫描到标靶的雷达个数
    TargType = 2  # 指定 TargType 变量的值
    pos_RoadO_LidarR_Uest = np.ones((3, FrameNum)) * 1e5
    r_Uest = np.zeros(FrameNum)

    for j in range(1, FrameNum + 1):
        if pos_LidarPC_LidarR.shape[0] == 0:
            pos_RoadO_LidarR_Uest[:, j - 1] = [1e5, 1e5, 1e5]
            break

        elif abs(np.min(pos_LidarPC_LidarR[1, :])) < yThred and LidarEff_Count > 1:
            Method = 2
        else:
            # Method = 3
            Method = 4

        ParIn = ParSet(TargType, Method)
        pos_RoadO_LidarR_Uest[:, j - 1], r_Uest[j - 1] = Centering_3DPC(pos_LidarPC_LidarR, ParIn)

        delta_r = r_Uest[j - 1] - ParIn['TargSize'] / 2
        if abs(delta_r) >= ParIn['rThred']:
            Method = 4
            ParIn = ParSet(TargType, Method)
            pos_RoadO_LidarR_Uest[:, j - 1], r_Uest[j - 1] = Centering_3DPC(pos_LidarPC_LidarR, ParIn)

    return pos_RoadO_LidarR_Uest