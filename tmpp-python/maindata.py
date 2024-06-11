###!/usr/bin/env python3
# -*- coding: utf-8 -*-3
# #tag: v0.1.8

import math
import threading
import pandas as pd
import time
import numpy as np
from datetime import datetime
import redis
from coordinate_trans_single import cal_output, euler_matrix
from PC3center import Centering_new
from controlcommunication import ControlCommunication
from getballdata import get_ball_data
import json
import csv
import os
import socket


file_path_xyz = "/home/user/tmpp/tmpp-python/xyzcoor.csv"
#filename_ = os.path.abspath('xyzcoor.csv')
data_xyz = [[0] for _ in range(3)]
with open(file_path_xyz, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data_xyz)

inslist = []
file_path = "/home/user/tmpp/tmpp-python/configer.json"
# file_path = 'configer.json'
absolute_path = os.path.abspath(file_path)
print(absolute_path)

with open(absolute_path, 'r') as json_file:
    data = json.load(json_file)

if_update = data['if_update_param']
# if_update = True

if if_update:
    with open(absolute_path, 'w') as json_file:
        json_file.write('')

    if_update_param = False
    angle_h_n = np.array([90.1225, -0.375, 0.405])
    jwg_biaoba = np.array([123.2000, 41.75823, 47.2])
    HeadToMWR = True
    init_state = True
    Lidar1_z = 2.068
    Version = 1
    Equipment_number = 2
    model = 0
    Type = 1
    angle_Car2Lidar_set = np.array([
        [7.34, -2.13, 1.37],
        [4.35, 5.76, 0.8],
        [0.24, -7.13, 0.54]
    ])
    y_th = 14.0
    pos_LidarO_Car = np.array([
        [0, 0, 0],
        [-1.09396155, 1.30818361, -0.30573671],
        [1.04366303, 1.73805063, -0.33470241]
    ])
    controlcommunication_switch = True
    CenterToB1 = np.array([0, -0.758, -2.086])  # 机心在载体坐标，z设为0
    Hexahedron = [[-1.8, 3.507, 0], [-1.8, 3.507, 2.086],
                  [1.8, 3.507, 0], [1.8, 3.507, 2.086],
                  [-1.35, -4.390, 0], [-1.35, -4.390, 2.086],
                  [1.35, -4.390, 0], [1.35, -4.390, 2.086]]
    RoadParameter = [5, 6]
    Reardist = 4.0
    pos_CarO_Car2 = np.array([[1.2120237], [1.62705657], [1.62720876]])
    x_addition = 0
    z_addition = 0

    existing_data = {}
    existing_data["if_update_param"] = if_update_param
    existing_data["angle_h_n"] = angle_h_n.tolist()
    existing_data["jwg_biaoba"] = jwg_biaoba.tolist()
    existing_data["HeadToMWR"] = HeadToMWR
    existing_data["init_state"] = init_state
    existing_data["Lidar1_z"] = Lidar1_z
    existing_data["Version"] = Version
    existing_data["Equipment_number"] = Equipment_number
    existing_data["model"] = model
    existing_data["Type"] = Type
    existing_data["angle_Car2Lidar_set"] = angle_Car2Lidar_set.tolist()
    existing_data["pos_LidarO_Car"] = pos_LidarO_Car.tolist()
    existing_data["controlcommunication_switch"] = controlcommunication_switch
    existing_data["centertob1"] = CenterToB1.tolist()
    existing_data["hexahedron"] = Hexahedron  # 这里测试是否能够保存
    existing_data["roadparams"] = RoadParameter
    existing_data["reardist"] = Reardist
    existing_data["carTocar2"] = pos_CarO_Car2.tolist()
    existing_data["y_thred"] = y_th
    existing_data["x_add"] = x_addition
    existing_data["z_add"] = z_addition

    with open(absolute_path, 'w') as json_file:
        json.dump(existing_data, json_file, indent=2)

    print("数据已添加到 JSON 文件：{json_file_path}")

    with open(absolute_path, 'r') as json_file:
        data = json.load(json_file)

    angle_h_n = np.array(data['angle_h_n'])
    jwg_biaoba = np.array(data['jwg_biaoba'])
    HeadToMWR = data['HeadToMWR']
    init_state = data['init_state']
    Lidar1_z = data['Lidar1_z']
    Version = data['Version']
    Equipemt_number = data['Equipment_number']
    model = data['model']
    Type = data['Type']
    angle = np.array(data['angle_Car2Lidar_set'])
    coor = np.array(data['pos_LidarO_Car']).T
    controlcommunication_switch = data['controlcommunication_switch']
    centertob1 = np.array(data['centertob1'])
    hexahedron_l = data['hexahedron']
    hexahedron = []
    for item in hexahedron_l:
        hexahedron.append(np.array(item))
    roadparams = data['roadparams']
    reardist = data['reardist']
    CarToCar2 = np.array(data['carTocar2'])
    y_thred = data['y_thred']
    x_addval = data['x_add']
    z_addval = data['z_add']
    print("JSON  配置文件参数已读取（已修改）")
else:
    with open(absolute_path, 'r') as json_file:
        data = json.load(json_file)

    # 现在，data 是一个包含 JSON 文件内容的字典
    angle_h_n = np.array(data['angle_h_n'])
    jwg_biaoba = np.array(data['jwg_biaoba'])
    HeadToMWR = data['HeadToMWR']
    init_state = data['init_state']
    Lidar1_z = data['Lidar1_z']
    Version = data['Version']
    Equipemt_number = data['Equipment_number']
    model = data['model']
    Type = data['Type']
    angle = np.array(data['angle_Car2Lidar_set'])
    coor = np.array(data['pos_LidarO_Car']).T
    controlcommunication_switch = data['controlcommunication_switch']
    centertob1 = np.array(data['centertob1'])
    hexahedron_l = data['hexahedron']
    hexahedron = []
    for item in hexahedron_l:
        hexahedron.append(np.array(item))
    roadparams = data['roadparams']
    reardist = data['reardist']
    CarToCar2 = np.array(data['carTocar2'])
    y_thred = data['y_thred']
    x_addval = data['x_add']
    z_addval = data['z_add']

    print("JSON  配置文件参数已读取（未修改）")
    print('starting..................')
anglelidar1 = angle[0]
print('lidar1 angle')
print(anglelidar1)
print('angle_h_n')
print(angle_h_n)

# 通讯初始化
C = ControlCommunication(Version, Equipemt_number, model, Type, centertob1, hexahedron, roadparams)

def insdata():
    print('get_insdata--------------------')
    while True:
        r = redis.Redis(host='127.0.0.1', port=6379)
        value = r.get('startTime')
        if value != None and value != b'0':
            print(value)
            data = json.loads(value)
            Starttime = int(data)
            Starttime = Starttime + 10000
            r.set('startTime', '0')
            break
    #print('udp_port----')
    UDP_IP = "127.0.0.1"  # 根据实际情况修改IP地址
    UDP_PORT = 19999  # 根据实际情况修改端口号
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    #print('udp_blind------')
    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
    PreDict = {}  # dd
    try:
        while True:
            #print('1')
            # 从CAN总线接收数据帧
            data_ins, addr = sock.recvfrom(128)
            #print('2')
            # 这里拆分data然后得到id和data的对应关系，
            can_id_ = int.from_bytes(data_ins[2:5], byteorder='big')  # big or little
            can_id = can_id_.to_bytes(3, byteorder='big').hex()
            data = data_ins[5:]
            if can_id == '000078':
                current_time = datetime.now()
                # 将时间转换为毫秒
                milliseconds = int(current_time.timestamp() * 1000)
                if abs(milliseconds - Starttime) < 200:
                    if Dict['time'] == '0':
                        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        Dict['time'] = formatted_time

                    combined_data = int.from_bytes([data[4], data[5]], byteorder='big')
                    horizontalAngleOfBody = combined_data * 0.01
                    Dict['horizontalAngleOfBody'] = horizontalAngleOfBody
                    combined_data1 = int.from_bytes([data[6], data[7]], byteorder='big')
                    pitchAngleOfBody = combined_data1 * 0.01 - 90
                    Dict['pitchAngleOfBody'] = pitchAngleOfBody
                    if len(Dict) == 7:
                        inslist.append(Dict)
                        PreDict = Dict  # dd
                        print(Dict)
                        Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
                        Starttime = Starttime + 1000
                elif milliseconds - Starttime > 200:  # dd
                    Starttime = Starttime + 1000
                    inslist.append(PreDict)
                    # InsData.put(PreDict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
            if can_id == '000079':
                current_time = datetime.now()
                # 将时间转换为毫秒
                milliseconds = int(current_time.timestamp() * 1000)
                if abs(milliseconds - Starttime) < 200:
                    if Dict['time'] == '0':
                        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        Dict['time'] = formatted_time

                    combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                    rollAngleOfBody = combined_data2 * 0.01 - 180
                    Dict['rollAngleOfBody'] = rollAngleOfBody
                    if len(Dict) == 7:
                        inslist.append(Dict)
                        PreDict = Dict  # dd
                        print(Dict)
                        Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
                        Starttime = Starttime + 1000
                elif milliseconds - Starttime > 200:  # dd
                    Starttime = Starttime + 1000
                    inslist.append(PreDict)
                    # InsData.put(PreDict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
            if can_id == '00007D':
                current_time = datetime.now()
                # 将时间转换为毫秒
                milliseconds = int(current_time.timestamp() * 1000)
                if abs(milliseconds - Starttime) < 200:
                    if Dict['time'] == '0':
                        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        Dict['time'] = formatted_time

                    # combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                    # rollAngleOfBody = combined_data2 * 0.01 - 180
                    # Dict['rollAngleOfBody'] = rollAngleOfBody
                    if len(Dict) == 7:
                        inslist.append(Dict)
                        PreDict = Dict  # dd
                        print(Dict)
                        Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
                        Starttime = Starttime + 1000
                elif milliseconds - Starttime > 200:  # dd
                    Starttime = Starttime + 1000
                    inslist.append(PreDict)
                    # InsData.put(PreDict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
            if can_id == '00008D':
                current_time = datetime.now()
                # 将时间转换为毫秒
                milliseconds = int(current_time.timestamp() * 1000)
                print(milliseconds - Starttime)
                if abs(milliseconds - Starttime) < 200:
                    if Dict['time'] == '0':
                        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        Dict['time'] = formatted_time

                    # combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                    # rollAngleOfBody = combined_data2 * 0.01 - 180
                    # Dict['rollAngleOfBody'] = rollAngleOfBody
                    if len(Dict) == 7:
                        inslist.append(Dict)
                        PreDict = Dict  # dd
                        print(Dict)
                        Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
                        Starttime = Starttime + 1000
                elif milliseconds - Starttime > 200:  # dd
                    Starttime = Starttime + 1000
                    inslist.append(PreDict)
                    # InsData.put(PreDict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
    except Exception as e:
        print("Exception:", e)


def read(body):
    Lidar3 = body['3LidarDatas']
    Lidar3 = pd.DataFrame(Lidar3)
    Lidar3 = Lidar3[[ 'x', 'y', 'z', 'id']]

    Lidar2 = body['2LidarDatas']
    Lidar2 = pd.DataFrame(Lidar2)
    Lidar2 = Lidar2[[ 'x', 'y', 'z', 'id']]
    # Lidar2.iloc[0]=0
    # Lidar2 = pd.DataFrame(columns=['distance', 'x', 'y', 'z', 'frameId', 'intensity', 'frameTime','hAngle'])
    Lidar1 = body['1LidarDatas']
    Lidar1 = pd.DataFrame(Lidar1)
    Lidar1 = Lidar1[['x', 'y', 'z', 'id']]
    # columns = ['longitude', 'latitude', 'height', 'horizontalAngleOfBody', 'pitchAngleOfBody', 'rollAngleOfBody']
    columns = ['rightCoor', 'frontCoor', 'insState', 'insRunTime', 'longitude', 'latitude', 'height', 'horizontalAngleOfBody', 'pitchAngleOfBody', 'rollAngleOfBody', 'xOfECD', 'yOfECD', 'zOfECD', 'xVOfBCD', 'yVOfBCD', 'zVOfBCD',
         'xAccOfBCD', 'yAccOfBCD', 'zAccOfBCD', 'xAVOfBCD', 'yAVOfTCD', 'zAVOfTCD']
    insdata = pd.DataFrame([[0] * len(columns)], columns=columns)
    return Lidar1, Lidar2, Lidar3, insdata

def cal_car2(pos_b_l, angle_h_n, angle_ins):
    pos_b_l2 = pos_b_l.reshape(3, 1)
    pos_b_l2 = - pos_b_l2
    C_n2h = euler_matrix(angle_h_n[::-1], flag=True)
    pos_b_l2 = np.dot(C_n2h.T, pos_b_l2)
    C_n2i = euler_matrix(angle_ins[::-1], flag=True)

    pos_b_l2 = np.dot(C_n2i, pos_b_l2)
    pos_b_l2 = pos_b_l2 + CarToCar2
    pos_b_l2 = np.dot(C_n2i.T, pos_b_l2)
    pos_b_l2 = np.dot(C_n2h, pos_b_l2)
    pos_b_l2 = - pos_b_l2
    return pos_b_l2

def main():
    centerk1, centerk0 = np.array([]), np.array([])
    predict_Lidar = pd.DataFrame(columns=['x', 'y', 'z'])
    Car2_road = pd.DataFrame(columns=['x', 'y', 'z'])
    carcenter = pd.DataFrame(columns=['x'])
    r = redis.Redis(host='127.0.0.1', port=6379)
    i = 0
    final_lidar_state = [1, 1, 1]
    tmp_lidar_state = [1, 1, 1]
    angle_body_ins = np.array([0, 0, 0])
    init_state = True

    while True:
        value = r.lpop('test4sensorDataQueue')
        if not value:
            print('lidar no data')
        if not inslist:
            print('ins no data')
            Insdata = {}
        else:
            Insdata = inslist.pop()
        if Insdata and value != None:
            print('Insdata')
            print(Insdata)
            start = time.time()

            body = json.loads(json.loads(value))
            Lidar1, Lidar2, Lidar3, insdata = read(body)
            insdata['longitude'] = Insdata['longitude']
            insdata['latitude'] = Insdata['latitude']
            insdata['height'] = Insdata['height']
            insdata['horizontalAngleOfBody'] = Insdata['horizontalAngleOfBody']
            insdata['pitchAngleOfBody'] = Insdata['pitchAngleOfBody']
            insdata['rollAngleOfBody'] = Insdata['rollAngleOfBody']

            last_lidar_state = tmp_lidar_state
            if Lidar1.empty or Lidar2.empty or Lidar3.empty:
                tmp_lidar_state = [0 if lidar.empty else 1 for lidar in [Lidar1, Lidar2, Lidar3]]

            for i in range(3):
                if last_lidar_state[i] == 0 and tmp_lidar_state[i] == 0:
                    final_lidar_state[i] = 0

            # 读取惯导 312
            angle_ins = insdata[['horizontalAngleOfBody', 'pitchAngleOfBody', 'rollAngleOfBody']]
            print('angle_ins')
            print(angle_ins)
            angle_ins = np.array(
                [float(angle_ins.iloc[0, 0]), float(angle_ins.iloc[0, 1]), float(angle_ins.iloc[0, 2])])
            print(angle_ins)
            data_ins = insdata
            data_ins = data_ins.head(1)


            # 需要设置第一帧全局处理，后续是需要读取xyzcoor的坐标值，
            # 以下保证读取到的坐标都是[0 0 0]形式。
            if init_state:
                pos_Car0_Road = np.array([0, 0, 0]).T
            else:
                car_coor_final = []
                with open('xyzcoor.csv', 'rt') as readfile:
                    for row in csv.reader(readfile):
                        car_coor_final.append(float(row[0]))
                pos_Car0_Road = np.asarray(car_coor_final).T
                print('Using last coor to get ball data.')
                print(pos_Car0_Road)

            leftwall_road = roadparams[0] / 2 + x_addval
            rightwall_road = roadparams[0] / 2 - x_addval
            pitchangle = float(angle_ins[1])
            pitchrad = math.radians(pitchangle)
            upwall_road = (0.3 + 0.15) * math.cos(pitchrad)
            down_road = roadparams[1] - upwall_road
            # upwall_road = 0.4
            # down_road = 2


            distance_Ball_Road = np.array([leftwall_road, rightwall_road, upwall_road, down_road]).T   # left right up down
            merge_cloud_point, lidar_num, tmp = get_ball_data(Lidar1, Lidar2, Lidar3, angle, angle_body_ins, angle_ins, angle_h_n,
                                        coor, pos_Car0_Road, distance_Ball_Road, init_state, centerk1, centerk0, Indentation=0.2, maxclass=5)
            merge_cloud_point = merge_cloud_point.T
            center = Centering_new(merge_cloud_point, lidar_num)
            init_state = tmp


            print('center')
            print(center)

            #if centerk0.size \== 0:
            #    centerk0 = center
            #    centerk1 = center
            #if centerk0.size != 0:
            #    centerk1 = centerk0
            #    centerk0 = center

            if int(center.flatten()[0]) != 100000:
                if centerk0.size == 0:
                    centerk0 = center
                    centerk1 = center
                else:
                    centerk1 = centerk0
                    centerk0 = center
                            #if centerk0.size == 0:                                                                                                      centerk0 = center                                                                                                       centerk1 = center                                                                                                   if centerk0.size != 0:                                                                                                      centerk1 = centerk0                                                                                                     centerk0 = center
                print('Can see ball!')
                center = pd.DataFrame(center).T
                final = pd.concat([data_ins, center], axis=1).iloc[:, 1:]  # 按列进行拼接，并且去掉第一列数据。
                # 坐标转换
                final = final.iloc[0, 3:]
                final = np.array(final)

                integer_array = []
                for string in final:
                    integer = float(string)
                    integer_array.append(integer)

                integer_array = pd.DataFrame(integer_array).values.T
                pos_b_l, out_ins = cal_output(integer_array, jwg_biaoba, angle_h_n, anglelidar1)
                print('pos_b_l')
                print(pos_b_l)

                pos_b_l2 = cal_car2(pos_b_l, angle_h_n, angle_ins)
                z_addval_road = z_addval * math.cos(pitchrad)

                car_coor_final = pos_b_l[0:3]   # (3, 1)
                with open('xyzcoor.csv', 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(car_coor_final)
                if controlcommunication_switch:
                    running_time = (time.time() - start) // 60
                    # car_coor, state, heatbeat, NeToRoad_angle, NeToIns_angle, RunningTime, Failure, HeadToMWR, reardist, Lidar1_z
                    can_data, carx = C.get_data(car_coor_final, '000', 0, angle_h_n, angle_ins,
                                                running_time, 1, 1, HeadToMWR, reardist, Lidar1_z, float(car_coor_final[1][0]),
                                                False, y_thred, x_addval, z_addval_road, final_lidar_state)
                    C.send_data_udp(can_data, time_interval=0.001)
                    # C.send_data_can(can_data, 0x0000F000, time_interval=0.001)
                    carcenter.loc[i] = carx
                    carcenter.to_csv('carcenter.csv')
            else:
                print('Can not see ball. Please move！！！')
                time.sleep(0.5)
                car_coor_final = []
                if init_state:
                    car_coor_final = np.array([[0], [0], [0]])
                else:
                    with open('xyzcoor.csv', 'rt') as readfile:
                        for row in csv.reader(readfile):
                            car_coor_final.append([float(row[0])])
                pos_b_l = np.asarray(car_coor_final)
                print('pos_b_l')
                print(pos_b_l)
                pos_b_l2 = cal_car2(pos_b_l, angle_h_n, angle_ins)

                pitchangle = float(angle_ins[1])
                pitchrad = math.radians(pitchangle)
                z_addval_road = z_addval * math.cos(pitchrad)

                if controlcommunication_switch:
                    running_time = (time.time() - start) // 60
                    # car_coor, state, heatbeat, NeToRoad_angle, NeToIns_angle, RunningTime, Failure, HeadToMWR, reardist, Lidar1_z
                    can_data, carx = C.get_data(car_coor_final, '000', 0, angle_h_n,
                                                angle_ins,
                                                running_time, 1, 1, HeadToMWR,
                                                reardist, Lidar1_z,
                                                float(car_coor_final[1][0]),
                                                True, y_thred, x_addval,
                                                z_addval_road, final_lidar_state)
                    C.send_data_udp(can_data, time_interval=0.01)
                    # C.send_data_can(can_data, 0x0000F000, time_interval=0.001)
                    carcenter.loc[i] = carx
                    carcenter.to_csv('carcenter.csv')
                # 这里需要处理看不见的情况


            # 保存雷达数据到predict_Lidar
            predict_Lidar.loc[i] = pos_b_l.flatten()
            predict_Lidar.to_csv('predict_Lidar.csv')

            r = redis.Redis(host='127.0.0.1', port=6379)
            my_dict = {
                    'angle_ins': angle_ins.tolist(),
                    'pos_b_l': pos_b_l.tolist()
                   }

            queue_name = 'my_queue'
            queue_length = r.llen(queue_name)
            if queue_length > 10:
                r.delete(queue_name)
            r.lpush('my_queue', json.dumps(my_dict))

            Car2_road.loc[i] = pos_b_l2.flatten()
            Car2_road.to_csv('car2_road.csv')
            i = i + 1
        else:
            time.sleep(3)
            print('no lidar data')
            continue

thread_1 = threading.Thread(target=insdata)
thread_2 = threading.Thread(target=main)

thread_1.start()
thread_2.start()

thread_1.join()
thread_2.join()


