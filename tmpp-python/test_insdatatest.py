# coding=utf-8
# import queue
import socket
import time

# InsData = queue.Queue()
inslist = []


def insdata():
    # 创建CAN总线对象
    UDP_IP = "127.0.0.1"  # 根据实际情况修改IP地址
    UDP_PORT = 19999  # 根据实际情况修改端口号

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    

    print('1')
    # global InsData
    # 设置需要监听的帧ID列表
    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
    try:
        while True:
            print('2')
            # 从CAN总线接收数据帧
            data_ins, addr = sock.recvfrom(128)
            print('3')
            # 这里拆分data然后得到id和data的对应关系，
            can_id_ = int.from_bytes(data_ins[2:5], byteorder='big')    # big or little
            can_id = can_id_.to_bytes(3, byteorder='big').hex()
            data = data_ins[5:]

            print(can_id)
            print(type(can_id))
            print(data)
            
            print(can_id == '000078')
            if can_id == '000078':
                combined_data = int.from_bytes([data[4], data[5]], byteorder='big')
                horizontalAngleOfBody = combined_data * 0.01
                Dict['horizontalAngleOfBody'] = horizontalAngleOfBody
                combined_data1 = int.from_bytes([data[6], data[7]], byteorder='big')
                pitchAngleOfBody = combined_data1 * 0.01 - 90
                Dict['pitchAngleOfBody'] = pitchAngleOfBody
                print(Dict)
                    # print(len(Dict))
                if len(Dict) == 7:
                    inslist.append(Dict)
                    # InsData.put(Dict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
            if can_id == '000079':
                combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                rollAngleOfBody = combined_data2 * 0.01 - 180
                Dict['rollAngleOfBody'] = rollAngleOfBody
                if len(Dict) == 7:
                    inslist.append(Dict)
                    # InsData.put(Dict)
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
                    print(Dict)
            if can_id == '00007D':
                print(can_id)
                    # combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                    # rollAngleOfBody = combined_data2 * 0.01 - 180
                    # Dict['rollAngleOfBody'] = rollAngleOfBody
                if len(Dict) == 7:
                    inslist.append(Dict)
                    # PreDict = Dict  # dd
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
            if can_id == '00008D':
                    # combined_data2 = int.from_bytes([data[0], data[1]], byteorder='big')
                    # rollAngleOfBody = combined_data2 * 0.01 - 180
                    # Dict['rollAngleOfBody'] = rollAngleOfBody
                print(can_id)
                if len(Dict) == 7:
                    inslist.append(Dict)
                    # PreDict = Dict  # dd
                    print(Dict)
                    Dict = {'time': '0', 'latitude': 41.75823, 'longitude': 123.2000, 'height': 47.2}
    except Exception as e:
        # 异常处理代码
        print("Exception:", e)


if __name__ == '__main__':
    while True:
        print('start')
        insdata()

        time.sleep(0.2)
