import socket
import time

robot_ip = '192.168.2.84'      ### main::  '192.168.2.84'
robot_port = 6666

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_address = (robot_ip, robot_port)

#
theString = ""
message1 = 0     # left 0+, right 180-
message2 = 0   # left 0+, right 180-
theString = theString + str(message1) + "," + str(message2) + ","
theString = "0,0,0,90,0,0,90,0,0,90,0,0,90,0,"

for i in range(10) :
    sock.sendto(theString.encode(), s_address)

    # try to receive reply
    try:
        data, server = sock.recvfrom(1024)  #
        print(f"Reply: {data.decode()}")
    except socket.timeout:
        print("Timeout. Did not receive reply.")

    # if i % 2 == 0:
    #     theString = "32, 22, 32, 32, 32, 32, 32, 32, 32,"
    # elif i % 2 == 1:
    #     theString = "30, 20, 30, 30, 30, 30, 30, 30, 30,"
        
    message1 = message1
    message2 = message2

    #theString = "" + str(message1) + "," + str(message2) + ","
    
    print(theString)

    time.sleep(0.02)  # 


sock.close()
