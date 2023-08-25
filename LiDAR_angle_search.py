import time
import pandas as pd
import numpy as np
import os

TargType = 1
TestNum_set = [10, 14]


def RotMat_N2Target(EulerAngle_N2Target):
    # print(EulerAngle_N2Target)
    roll = np.deg2rad(EulerAngle_N2Target[0])  # 横滚角roll  y 2
    pitch = np.deg2rad(EulerAngle_N2Target[1])  # 俯仰角pitch x 1
    yaw = np.deg2rad(EulerAngle_N2Target[2])  # 偏航角yaw   z 3

    RotMat_y = np.array([[np.cos(roll), 0, -np.sin(roll)],
                         [0, 1, 0],
                         [np.sin(roll), 0, np.cos(roll)]])

    RotMat_x = np.array([[1, 0, 0],
                         [0, np.cos(pitch), np.sin(pitch)],
                         [0, -np.sin(pitch), np.cos(pitch)]])

    RotMat_z = np.array([[np.cos(yaw), np.sin(yaw), 0],
                         [-np.sin(yaw), np.cos(yaw), 0],
                         [0, 0, 1]])

    RotMat = np.dot(RotMat_y, np.dot(RotMat_x, RotMat_z))

    return RotMat


t0 = time.time()
for LidarR in range(1, 3):
    print('-------')
    print(LidarR)
    for TestType in range(1, 3):  # 标定 or 验证
        print('++++++')
        print(TestType)
        TestNum = TestNum_set[TestType - 1]

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/东北天坐标系转向巷道坐标系的欧拉角.xlsx'
        angle_N2Road = pd.read_excel(Path, header=None).values
        angle_N2Road = angle_N2Road[-1][::-1]
        RotMat_N2Road = RotMat_N2Target(angle_N2Road)


        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/惯性导航/东北天坐标系转向惯导坐标系的欧拉角.xlsx'
        angle_N2Ins = pd.read_excel(Path, header=None).values


        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/标定点位在巷道坐标系的坐标.xlsx'
        pos_TestO_Road = pd.read_excel(Path, header=None).values.T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/载体原点在巷道坐标系的坐标.xlsx'
        pos_CarRO_Road = pd.read_excel(Path, header=None).values.T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/巷道原点在雷达坐标系的坐标（估计）_' + str(LidarR) + '.xlsx'
        pos_RoadO_Lidar_est = pd.read_excel(Path, header=None).values.T

        RotMat_z_180 = np.array([[np.cos(np.deg2rad(180)), np.sin(np.deg2rad(180)), 0],
                        [-np.sin(np.deg2rad(180)), np.cos(np.deg2rad(180)), 0],
                        [0, 0, 1]])

        if LidarR == 2:
            pos_RoadO_Lidar_est = np.dot(RotMat_z_180, pos_RoadO_Lidar_est)

        pos_CarO_Road_est = np.zeros((3, TestNum))
        pos_CarO_Road = np.zeros((3, TestNum))
        # print(pos_CarO_Road)

        angle_Car2Ins = 0
        RotMat_Car2Ins = np.array([[np.cos(np.deg2rad(angle_Car2Ins)), np.sin(np.deg2rad(angle_Car2Ins)), 0],
                          [-np.sin(np.deg2rad(angle_Car2Ins)), np.cos(np.deg2rad(angle_Car2Ins)), 0],
                          [0, 0, 1]])

        pos_InsO_Car = np.array([0.0895, 0.5555, 0]).T
        pos_CarO_Car2 = np.array([-0.0130, 0.0960, 0]).T
        pos_Car2O_TestN = np.array([0, 0, 0.5315]).T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/雷达原点在载体坐标系的坐标（抬头后）.xlsx'
        pos_LidarO_Car = pd.read_excel(Path, header=None).values.T

        AngleRange_Car2Lidar_x = np.arange(-3, 0, 0.01)
        AngleRange_Car2Lidar_y = np.arange(-1, 0, 0.01)
        AngleRange_Car2Lidar_z = np.arange(-5, -1, 0.01)
        # res = np.inf((len(AngleRange_Car2Lidar_x), len(AngleRange_Car2Lidar_y), len(AngleRange_Car2Lidar_z)))
        res = np.full((len(AngleRange_Car2Lidar_z), len(AngleRange_Car2Lidar_x), len(AngleRange_Car2Lidar_y)), np.inf)
        # print(res.shape)
        i_min, j_min, k_min = 0, 0, 0
        tmp = float('inf')
        for i in range(len(AngleRange_Car2Lidar_x)):
            print(i)
            for j in range(len(AngleRange_Car2Lidar_y)):
                for k in range(len(AngleRange_Car2Lidar_z)):
                    angle_Car2Lidar = [AngleRange_Car2Lidar_z[k], AngleRange_Car2Lidar_x[i], AngleRange_Car2Lidar_y[j]]
                    RotMat_Car2Lidar = RotMat_N2Target(np.flip(angle_Car2Lidar))
                    for l in range(TestNum):
                        RotMat_N2Ins = RotMat_N2Target(np.flip(angle_N2Ins[l, :]))
                        pos_RoadO_Car_est = pos_LidarO_Car[:, LidarR-1] + np.dot(RotMat_Car2Lidar.T,
                                                                               pos_RoadO_Lidar_est[:, l])
                        pos_RoadO_CarN_est = np.dot(np.dot(RotMat_N2Ins.T, RotMat_Car2Ins), pos_RoadO_Car_est)
                        pos_RoadO_CarRoad_est = np.dot(RotMat_N2Road, pos_RoadO_CarN_est)
                        pos_CarO_Road_est[:, l] = -pos_RoadO_CarRoad_est
                        pos_TestO_RoadN = np.dot(RotMat_N2Road.T, pos_TestO_Road[:, l])
                        pos_RoadO_Car2N = -pos_TestO_RoadN - pos_Car2O_TestN
                        pos_RoadO_Car = np.dot(np.dot(RotMat_Car2Ins.T, RotMat_N2Ins), pos_RoadO_Car2N) - pos_CarO_Car2
                        pos_RoadO_CarN = np.dot(np.dot(RotMat_N2Ins.T, RotMat_Car2Ins), pos_RoadO_Car)
                        pos_RoadO_CarRoad = np.dot(RotMat_N2Road, pos_RoadO_CarN)
                        # print(pos_RoadO_CarRoad)
                        pos_CarO_Road[:, l] = -pos_RoadO_CarRoad

                    error = pos_CarO_Road - pos_CarO_Road_est
                    # print(i, j, k)
                    res[k, i, j] = np.linalg.norm(error, 'fro') / TestNum
                    if res[k, i, j] < tmp:
                        tmp = res[k, i, j]
                        i_min, j_min, k_min = i, j, k
                    else:
                        continue

        # idx_min = np.unravel_index(np.argmin(res), res.shape)
        # i_min, j_min, k_min = idx_min
        # print(idx_min)
        if TestType == 1:
            angle_Car2Lidar_best_1 = np.array(
                [AngleRange_Car2Lidar_z[i_min], AngleRange_Car2Lidar_x[j_min], AngleRange_Car2Lidar_y[k_min]])
        elif TestType == 2:
            angle_Car2Lidar_best_2 = np.array(
                [AngleRange_Car2Lidar_z[i_min], AngleRange_Car2Lidar_x[j_min], AngleRange_Car2Lidar_y[k_min]])
        # print('--------')
        # print(i_min, j_min, k_min)

    for TestType in range(1, 3):

        angle_Car2Lidar_set = np.array([[0, 0, 0], angle_Car2Lidar_best_1, angle_Car2Lidar_best_2])
        # print(angle_Car2Lidar_set)
        TestNum = TestNum_set[TestType - 1]
        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/东北天坐标系转向巷道坐标系的欧拉角.xlsx'
        angle_N2Road = pd.read_excel(Path, header=None).values
        angle_N2Road = angle_N2Road[-1][::-1]
        RotMat_N2Road = RotMat_N2Target(angle_N2Road)

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(
            TargType) + '/惯性导航/东北天坐标系转向惯导坐标系的欧拉角.xlsx'
        angle_N2Ins = pd.read_excel(Path, header=None).values

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/标定点位在巷道坐标系的坐标.xlsx'
        pos_TestO_Road = pd.read_excel(Path, header=None).values.T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/载体原点在巷道坐标系的坐标.xlsx'
        pos_CarRO_Road = pd.read_excel(Path, header=None).values.T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(
            TargType) + '/巷道原点在雷达坐标系的坐标（估计）_' + str(LidarR) + '.xlsx'
        pos_RoadO_Lidar_est = pd.read_excel(Path, header=None).values.T

        RotMat_z_180 = np.array([[np.cos(np.deg2rad(180)), np.sin(np.deg2rad(180)), 0],
                                 [-np.sin(np.deg2rad(180)), np.cos(np.deg2rad(180)), 0],
                                 [0, 0, 1]])

        if LidarR == 2:
            pos_RoadO_Lidar_est = np.dot(RotMat_z_180, pos_RoadO_Lidar_est)

        pos_CarO_Road_est = np.zeros((3, TestNum))
        pos_CarO_Road = np.zeros((3, TestNum))

        angle_Car2Ins = 0
        RotMat_Car2Ins = np.array([[np.cos(np.deg2rad(angle_Car2Ins)), np.sin(np.deg2rad(angle_Car2Ins)), 0],
                                   [-np.sin(np.deg2rad(angle_Car2Ins)), np.cos(np.deg2rad(angle_Car2Ins)), 0],
                                   [0, 0, 1]])

        pos_InsO_Car = np.array([0.0895, 0.5555, 0]).T
        pos_CarO_Car2 = np.array([-0.0130, 0.0960, 0]).T
        pos_Car2O_TestN = np.array([0, 0, 0.5315]).T

        Path = '多雷达静态_0801/测试数据_' + str(TestType) + '/标靶_' + str(TargType) + '/雷达原点在载体坐标系的坐标（抬头后）.xlsx'
        pos_LidarO_Car = pd.read_excel(Path, header=None).values.T



        Path = f'多雷达静态_0801\\测试数据_{TestType}\\标靶_{TargType}\\雷达偏角分析_{TestType}_0823.xlsx'
        writer = pd.ExcelWriter(Path, engine='xlsxwriter')
        for i in range(3):
            angle_Car2Lidar = angle_Car2Lidar_set[i, :]
            RotMat_Car2Lidar = RotMat_N2Target(np.flip(angle_Car2Lidar))

            for l in range(TestNum):
                RotMat_N2Ins = RotMat_N2Target(np.flip(angle_N2Ins[l, :]))
                pos_RoadO_Car_est = pos_LidarO_Car[:, LidarR-1] + np.dot(np.transpose(RotMat_Car2Lidar),
                                                                       pos_RoadO_Lidar_est[:, l])
                pos_RoadO_CarN_est = np.dot(np.dot(np.transpose(RotMat_N2Ins), RotMat_Car2Ins), pos_RoadO_Car_est)
                pos_RoadO_CarRoad_est = np.dot(RotMat_N2Road, pos_RoadO_CarN_est)
                pos_CarO_Road_est[:, l] = -pos_RoadO_CarRoad_est
                pos_TestO_RoadN = np.dot(np.transpose(RotMat_N2Road), pos_TestO_Road[:, l])
                pos_RoadO_Car2N = -pos_TestO_RoadN - pos_Car2O_TestN
                pos_RoadO_Car = np.dot(np.dot(np.transpose(RotMat_Car2Ins), RotMat_N2Ins),
                                       pos_RoadO_Car2N) - pos_CarO_Car2
                pos_RoadO_CarN = np.dot(np.dot(np.transpose(RotMat_N2Ins), RotMat_Car2Ins), pos_RoadO_Car)
                pos_RoadO_CarRoad = np.dot(RotMat_N2Road, pos_RoadO_CarN)
                pos_CarO_Road[:, l] = -pos_RoadO_CarRoad

            error = pos_CarO_Road - pos_CarO_Road_est
            SheetName = f'Lidar_{LidarR}'
            ColDiff = i * (TestNum+5)
            Range1 = f'B{2+ColDiff}:D{2+ColDiff}'
            Range2 = f'B{5+ColDiff}:D{4+TestNum+ColDiff}'
            Range3 = f'E{5+ColDiff}:G{4+TestNum+ColDiff}'
            Range4 = f'H{5+ColDiff}:J{4+TestNum+ColDiff}'
            Path = f'多雷达静态_0801/测试数据_{TestType}/标靶_{TargType}/雷达偏角分析_{TestType}_0825.xlsx'

            if os.path.exists(Path):
                writer = pd.ExcelWriter(Path, mode='a', engine='openpyxl', if_sheet_exists='overlay')
                df1 = pd.DataFrame(angle_Car2Lidar)
                df1.to_excel(writer, sheet_name=SheetName, startrow=int(2 + ColDiff), header=False, index=False)
                df2 = pd.DataFrame(pos_CarO_Road.T)
                df2.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), header=False, index=False)
                df3 = pd.DataFrame(pos_CarO_Road_est.T)
                df3.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), startcol=4, header=False,
                             index=False)
                df4 = pd.DataFrame(error.T)
                df4.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), startcol=7, header=False,
                             index=False)
                writer._save()
            else:
                with pd.ExcelWriter(Path) as writer:
                    df1 = pd.DataFrame(angle_Car2Lidar)
                    df1.to_excel(writer, sheet_name=SheetName, startrow=int(2 + ColDiff), header=False, index=False)
                    df2 = pd.DataFrame(pos_CarO_Road.T)
                    df2.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), header=False, index=False)
                    df3 = pd.DataFrame(pos_CarO_Road_est.T)
                    df3.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), startcol=4, header=False,
                                 index=False)
                    df4 = pd.DataFrame(error.T)
                    df4.to_excel(writer, sheet_name=SheetName, startrow=int(5 + ColDiff), startcol=7, header=False,
                                 index=False)
t1 = time.time()
print(t1-t0)