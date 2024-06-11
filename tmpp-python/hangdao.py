import pandas as pd
import numpy as np
import redis
import json
import os
import glob

angle_Car2Lidar_set = np.array([
    [-4.31, -0.31, 0.06],
    [3.13, 7.96, 1.88],
    [-16.35, -4.68, -3.38]
])
pos_LidarO_Car = np.array([
    [0, 0, 0],
    [-1.09396155, 1.30818361, -0.30573671],
    [1.04366303, 1.73805063, -0.33470241]
]).T


file_path = "/home/user/tmpp/tmpp-python/configer.json"
absolute_path = os.path.abspath(file_path)
print(absolute_path)

with open(absolute_path, 'r') as json_file:
    data = json.load(json_file)

angle_h_n = np.array(data['angle_h_n'])

def read(body):

    Lidar3 = body['3LidarDatas1']
    Lidar3 = pd.DataFrame(Lidar3)
    Lidar3 = Lidar3[[ 'x', 'y', 'z']]
    Lidar2 = body['2LidarDatas1']
    Lidar2 = pd.DataFrame(Lidar2)
    Lidar2 = Lidar2[[ 'x', 'y', 'z']]
    Lidar1 = body['1LidarDatas1']
    Lidar1 = pd.DataFrame(Lidar1)
    Lidar1 = Lidar1[['x', 'y', 'z']]
    # insdata = pd.DataFrame(data=[])
    # insdata = body['insdata']
    # insdata = pd.DataFrame(insdata, index=[0])
    # insdata = insdata[
    # columns = ['rightCoor', 'frontCoor', 'insState', 'insRunTime', 'longitude', 'latitude', 'height', 'horizontalAngleOfBody',
    #      'pitchAngleOfBody', 'rollAngleOfBody', 'xOfECD', 'yOfECD', 'zOfECD', 'xVOfBCD', 'yVOfBCD', 'zVOfBCD',
    #      'xAccOfBCD', 'yAccOfBCD', 'zAccOfBCD', 'xAVOfBCD', 'yAVOfTCD', 'zAVOfTCD']
    # insdata = pd.DataFrame([[0] * len(columns)], columns=columns)
    return Lidar1, Lidar2, Lidar3

def RotMat_N2Target(euler_angle_n2target):
    """
    由东北天到目标坐标系的旋转顺序：zxy（312）
    由目标坐标系到东北天的旋转顺序：yxz（213）
    采用角度制
    EulerAngle_N2Target (roll, pitch, yaw) （横滚角，俯仰角，偏航角） (y, x, z) (2, 1, 3)
    """

    roll = euler_angle_n2target[0]  # 横滚角roll  y 2
    pitch = euler_angle_n2target[1]  # 俯仰角pitch x 1
    yaw = euler_angle_n2target[2]  # 偏航角yaw   z 3

    cos_roll = np.cos(np.deg2rad(roll))
    sin_roll = np.sin(np.deg2rad(roll))

    cos_pitch = np.cos(np.deg2rad(pitch))
    sin_pitch = np.sin(np.deg2rad(pitch))

    cos_yaw = np.cos(np.deg2rad(yaw))
    sin_yaw = np.sin(np.deg2rad(yaw))

    RotMat_y = np.array([[cos_roll, 0, -sin_roll],
                         [0, 1, 0],
                         [sin_roll, 0, cos_roll]])

    RotMat_x = np.array([[1, 0, 0],
                         [0, cos_pitch, sin_pitch],
                         [0, -sin_pitch, cos_pitch]])

    RotMat_z = np.array([[cos_yaw, sin_yaw, 0],
                         [-sin_yaw, cos_yaw, 0],
                         [0, 0, 1]])

    rot_mat = np.dot(np.dot(RotMat_y, RotMat_x), RotMat_z)

    return rot_mat

def Point_Merge(pos_LidarPC_Lidar1, pos_LidarPC_Lidar2, pos_LidarPC_Lidar3, x, y):
    center, r = None, 0  # 设置默认值
    pos_LidarPC_LidarR = np.array([])
    angle_Car2Lidar_set = x
    pos_LidarO_Car = y
    print(9999)
    # angle_Car2Lidar_set = np.array([[-0.81, -1.11, -0.04],
    #                                 [1.22, -2.67, 0.65],
    #                                 [-0.54, -1.86, 0.66]])
    #
    # pos_LidarO_Car = np.array([[-0.321, 0.533284072, 0.124499108],
    #                            [0.778, 0.533884376, 0.125042265],
    #                            [0.331, -0.248713655, 0.272606905]]).T


    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))

        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_LidarR2, pos_LidarPC_LidarR3), axis=1)

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR3), axis=1)

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR2), axis=1)


    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = pos_LidarPC_LidarR3

    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = pos_LidarPC_LidarR2

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        pos_LidarPC_LidarR = pos_LidarPC_Lidar1

    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        pos_LidarPC_LidarR = np.array([])

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))

        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR2, pos_LidarPC_LidarR3), axis=1)
        # center, r = sphere_equation(pos_LidarPC_LidarR)

    return pos_LidarPC_LidarR

def delete(path):
    # 获取所有.csv文件的路径
    csv_files = glob.glob(os.path.join(path, '*.csv'))

    # 遍历并删除这些文件
    for file_path in csv_files:
        os.remove(file_path)
    # paths = os.listdir(path)
    # # paths = ['444.las', 'left.las', 'leftplane.las', 'origin.las', 'pointleft.las', 'pointright.las', 'quzao.las', 'right.las', 'rightplane.las', 'test.las', 'xiawu.csv']
    # print(paths)
    # fileLength = len(paths)
    # paths_ = paths[:fileLength - 2]
    # for path_ in paths_:
    #     filePath = path + path_
    #     if os.path.exists(filePath):
    #         os.remove(filePath)
    #     else:
    #         print("当前文件不存在，请检查文件路径……")
    #         continue

def main_(path):
    i = 0
    while True:
        r = redis.Redis(host='127.0.0.1', port=6379)
        value = r.lpop('hangdaoDataQueue')
        print('weijinru')
        if value != None:
            print('jinru')
            body = json.loads(json.loads(value))
            Lidar1, Lidar2, Lidar3 = read(body)
            # Lidar1.to_csv(path1 + str(i) + '_car2_road.csv', index=False)
            # Lidar2.to_csv(path2 + str(i) + '_car2_road.csv', index=False)
            # Lidar3.to_csv(path3 + str(i) + '_car2_road.csv', index=False)
            # insdata['longitude'] = Insdata['longitude']
            # insdata['latitude'] = Insdata['latitude']
            # insdata['height'] = Insdata['height']
            # insdata['horizontalAngleOfBody'] = Insdata['horizontalAngleOfBody']
            # insdata['pitchAngleOfBody'] = Insdata['pitchAngleOfBody']
            # insdata['rollAngleOfBody'] = Insdata['rollAngleOfBody']
            #angle_ins = insdata[['horizontalAngleOfBody', 'pitchAngleOfBody', 'rollAngleOfBody']]
            Lidar1data = Lidar1[['x', 'y', 'z']].values.T
            Lidar2data = Lidar2[['x', 'y', 'z']].values.T
            Lidar3data = Lidar3[['x', 'y', 'z']].values.T
            # insti1 = np.concatenate((np.array(Lidar1[['intensity']].values),np.array(Lidar2[['intensity']].values),np.array(Lidar3[['intensity']].values)),axis = 0)
            #frame = [Lidar1, Lidar2, Lidar3]
            #LidarR = pd.concat(Lidar1, Lidar2, Lidar3, ignore_index=True)
            #LidarR = pd.concat(frame)

            pos_LidarPC_LidarR = Point_Merge(Lidar1data, Lidar2data, Lidar3data, angle_Car2Lidar_set, pos_LidarO_Car)
            r = redis.Redis(host='127.0.0.1', port=6379)
            dict_str = r.rpop('my_queue')
            print('kaishixuanhuan')
            while dict_str == None:
                dict_str = r.rpop('my_queue')
            print('tuichuhunxuan')
            my_dict = json.loads(dict_str)
            angle_ins = np.array(my_dict['angle_ins'])
            pos_b_l = np.array(my_dict['pos_b_l'])
            #坐标系转换
            C_l2b = RotMat_N2Target(np.flip(angle_Car2Lidar_set[0,:]))
            pos_b = np.dot(C_l2b.T, pos_LidarPC_LidarR) + pos_LidarO_Car[:, 0].reshape(3, 1)
            # 传入的惯导和东北天欧拉角的顺序是ZXY，这样取反解析的时候才会得到YXZC
            C_i2n = RotMat_N2Target(angle_ins[::-1])
            C_n2h = RotMat_N2Target(angle_h_n[::-1])
            pos_n = np.dot(np.dot(C_n2h, C_i2n.T), pos_b) + pos_b_l
            pos_n = pos_n.T
            # pos_n = np.column_stack((pos_n,insti1))
            #pos_n.to_csv(path + str(i) + '_car2_road.csv')
            np.savetxt(path + str(i) + '_car2_road.csv',pos_n,delimiter=',')
            # pos_n = pd.Series(pos_n[:, 0])
            # LidarR.loc[:, 'x'] = pos_n
            # pos_n = pd.Series(pos_n[:, 1])
            # LidarR.loc[:, 'y'] = pos_n
            # pos_n = pd.Series(pos_n[:, 2])
            # LidarR.loc[:, 'z'] = pos_n
            # LidarR.to_csv(path + str(i) + '_car2_road.csv', index=False)
            print("开始保存")
            i = i + 1
            if i == 10:
                delete(path)
                # delete(path1)
                # delete(path2)
                # delete(path3)
                i = 0
r = redis.Redis(host='127.0.0.1', port=6379)
r.delete('hangdaoDataQueue')
path = '/home/user/roadway_detect/data/'
# path1 = '/home/user/roadway_detect/data/1/'
# path2 = '/home/user/roadway_detect/data/2/'
# path3 = '/home/user/roadway_detect/data/3/'
main_(path)


