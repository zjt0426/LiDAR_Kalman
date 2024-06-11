# coding=utf-8
import math
import numpy as np
from controlcommunication import ControlCommunication
import json
import os


def euler_matrix(angle, flag):
    # 这里的angle表示欧拉角的三个角度翻滚角、俯仰角以及偏航角。并且考虑正向YXZ输入顺序。
    # 规定惯导和东北天之间的夹角为：惯导输出的角度为东北天到惯导的角度。
    # flag :True--东北天到惯导或者巷道(zxy)    False--惯导到东北天角度
    a, b, c = angle[0], angle[1], angle[2]
    # Y
    a = math.radians(a)
    b = math.radians(b)
    c = math.radians(c)
    A = np.array([[np.cos(a), 0, -np.sin(a)],
                  [0, 1, 0],
                  [np.sin(a), 0, np.cos(a)]])
    # X
    B = np.array([[1, 0, 0],
                  [0, np.cos(b), np.sin(b)],
                  [0, -np.sin(b), np.cos(b)]])
    # Z
    C = np.array([[np.cos(c), np.sin(c), 0],
                  [-np.sin(c), np.cos(c), 0],
                  [0, 0, 1]])
    C_ = C.T
    A_ = A.T
    B_ = B.T
    return np.dot(np.dot(A, B), C) if flag else np.dot(np.dot(A_, B_), C_)







# 相对路径
file_path = "configer.json"

# 获取绝对路径
absolute_path = os.path.abspath(file_path)

print(absolute_path)



# file_path = '/home/user/tunnelPostion/tmpp-python/configer.json'
# file_path = r"D:\Desktop\tmpp\configer.json"

with open(absolute_path, 'r') as json_file:
    data = json.load(json_file)

if_update = data['if_update_param']
# if_update = True

if if_update:
    # json_file_path = '/home/user/tunnelPostion/tmpp-python/configer.json'
    # json_file_path = r"D:\Desktop\tmpp\configer.json"

    with open(absolute_path, 'w') as json_file:
        json_file.write('')

    if_update_param = False
    angle_h_n = np.array([90.1225, -0.375, 0.405])
    jwg_biaoba = np.array([123.2000, 41.75823, 47.2])
    HeadToMWR = False
    Lidar1_z = 2.068
    Version = 1
    Equipment_number = 1
    model = 0
    Type = 1
    angle_Car2Lidar_set = np.array([
        [7.34, -2.13, 1.37],
        [4.35, 5.76, 0.8],
        [0.24, -7.13, 0.54]
    ])
    y_th = 10.0
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
    Reardist = 16.0
    pos_CarO_Car2 = np.array([[1.2120237], [1.62705657], [1.62720876]])
    x_addition = 0
    z_addition = 0

    existing_data = {}
    existing_data["if_update_param"] = if_update_param
    existing_data["angle_h_n"] = angle_h_n.tolist()
    existing_data["jwg_biaoba"] = jwg_biaoba.tolist()
    existing_data["HeadToMWR"] = HeadToMWR
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

    # json_file_path = '/home/user/tunnelPostion/tmpp-python/configer.json'
    # json_file_path = r"D:\Desktop\tmpp\configer.json"

    with open(absolute_path, 'r') as json_file:
        data = json.load(json_file)

    angle_h_n = np.array(data['angle_h_n'])
    jwg_biaoba = np.array(data['jwg_biaoba'])
    HeadToMWR = data['HeadToMWR']
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
    # print(angle)

    # print(angle_h_n, jwg_biaoba, HeadToMWR, Lidar1_z, Version, Equipemt_number, model, Type)
    # print(angle, coor, controlcommunication_switch)
    print("JSON  配置文件参数已读取（已修改）")
else:
    # 指定 JSON 文件路径
    # json_file_path = '/home/user/tunnelPostion/tmpp-python/configer.json'
    # json_file_path = r"D:\Desktop\tmpp\configer.json"

    # 读取 JSON 文件
    with open(absolute_path, 'r') as json_file:
        data = json.load(json_file)

    # 现在，data 是一个包含 JSON 文件内容的字典
    angle_h_n = np.array(data['angle_h_n'])
    jwg_biaoba = np.array(data['jwg_biaoba'])
    HeadToMWR = data['HeadToMWR']
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
print(anglelidar1)
print('angle_h_n')
print(angle_h_n)


# pos_b_l = np.array([[0.90537], [2.50409], [-0.0722]])
# print(pos_b_l)

# 通讯初始化
C = ControlCommunication(Version, Equipemt_number, model, Type, centertob1, hexahedron, roadparams)  # Version, Equipemt_number, model, Type



import time
start = time.time()
time.sleep(5)

car_coor_final = np.array([[0.445], [3.361], [-0.270]])
# car_coor_final = pos_b_l[0:3]
print(car_coor_final)
angle_ins = np.array([358.48, 2.99, -1.21])
# angle_ins = np.array([359.04, -0.2475, 0.2975])


pitchangle = float(angle_ins[1])
pitchrad = math.radians(pitchangle)
z_addval_road = z_addval * math.cos(pitchrad)
print(z_addval_road)






if controlcommunication_switch:
    while True:
        running_time = (time.time() - start) // 60
        # 这里需要定义状态state怎么获得以及故障Failure，还有reardist，后向距离。心跳。

        # 就需要设计 故障情况，以及不同的故障类型。
        # car_coor, state, heatbeat, NeToRoad_angle, NeToIns_angle, RunningTime, Failure, HeadToMWR, reardist, Lidar1_z
        can_data, carx = C.get_data(car_coor_final, '000', 0, angle_h_n, angle_ins,
                                    running_time, 1, 1, HeadToMWR, reardist, Lidar1_z, car_coor_final[1],
                                    False, y_thred, x_addval, z_addval_road, [1, 1, 1])
        # C.send_data_can(can_data, 0x0000F000, time_interval=0.001)

        C.send_data_udp(can_data, time_interval=0.5)


