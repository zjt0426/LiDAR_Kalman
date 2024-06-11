import socket

UDP_IP = "127.0.0.1"  # 根据实际情况修改IP地址
UDP_PORT = 8000  # 根据实际情况修改端口号

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))


radar1_data = None
radar2_data = None
count = 0  # 计数器
return_data = {}

while True:
    
    print('11111')
    data, addr = sock.recvfrom(8)  # 接收8个字节的数据
    print('2222')
    id = data[0]  # 第一个字节为id
    print(id)
    status = data[1]  # 第二个字节为状态，0表示掉线，1表示在线
    distance = int.from_bytes(data[2:6], byteorder='big')  # 第三到第六个字节为距离，使用big-endian字节序转换为整数
    reflection = int.from_bytes(data[6:8], byteorder='big')  # 最后两个字节为反射强度

    if id in [1, 2]:  # 判断id字段是否为1或2
        count += 1
        if id == 1:
            # return_data[id] = id
            # return_data[status] = status
            radar1_data = (distance, status, reflection)
        elif id == 2:
            radar2_data = (distance, status, reflection)

        print("ID:", id)
        print("Status:", status)  # 打印状态值
        print("Distance:", distance/10000.0)
        print("Reflection:", reflection)

    if count >= 2:  # 已获取前两个雷达数据
        break

print("Radar 1 data:", radar1_data)
print("Radar 2 data:", radar2_data)
