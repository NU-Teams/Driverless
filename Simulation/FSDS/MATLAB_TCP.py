# This will pass over info to ANY other programming languages via TCP/IP (server)
# by: Kwan Nok Mak
# Date: 13/9/20

import sys
import os

import socket

## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(__file__))
import fsds

#connect to airsim server
client = fsds.FSDSClient()
client.confirmConnection()
client.enableApiControl(True)

# connect to matlab
print('Waiting for connection...')
TCP_IP = '127.0.0.1'
TCP_PORT = 12201
BUFFER_SIZE = 128
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
server, addr = s.accept()
print('Connection address:', addr)
print('Start listening now...')

while 1:
    #pass data to matlab
    data = server.recv(BUFFER_SIZE)
    if data == b'UPDATE]': #if received update request
        print('Update request received')
        
        #gather IMU + GPS + LIDAR info
        gps = client.getGpsData(gps_name='Gps', vehicle_name='FSCar')
        imu = client.getImuData(imu_name = 'Imu', vehicle_name = 'FSCar')
        lidardata = client.getLidarData(lidar_name = 'Lidar1')
        
        #pack into byte array packet
        server.sendall(bytes(str([\
        #IMU
        imu.time_stamp, imu.orientation.x_val, imu.orientation.y_val, imu.orientation.z_val,\
        imu.angular_velocity.x_val, imu.angular_velocity.y_val,imu.angular_velocity.z_val,\
        imu.linear_acceleration.x_val,imu.linear_acceleration.y_val,imu.linear_acceleration.z_val,\
        #GPS
        gps.time_stamp,gps.gnss.time_utc,\
        gps.gnss.geo_point.latitude,gps.gnss.geo_point.longitude,gps.gnss.geo_point.altitude,\
        gps.gnss.eph,gps.gnss.epv,\
        gps.gnss.velocity.x_val,gps.gnss.velocity.y_val,gps.gnss.velocity.z_val,\
        #LIDAR
        lidardata.time_stamp,lidardata.point_cloud]), 'utf8'))
        
        #gather and send camera pixel info
        [image] = client.simGetImages([fsds.ImageRequest(camera_name = 'cam1', image_type = fsds.ImageType.Scene, pixels_as_float = False, compress = False)], vehicle_name = 'FSCar')
        server.sendall(image.image_data_uint8)
        
    if data == b'CTRL]': 
        #parse command
        cmd=server.recv(BUFFER_SIZE)
        #cmd_arry=str(cmd).replace("]'","")
        #cmd_arry.replace("b'[",'')
        cmd_arry=str(cmd).split(" ")
        cmd_arry[2]=cmd_arry[2].replace("]'","")
        cmd_arry[0]=cmd_arry[0].replace("b'","")
        #cmd_arry=[x for x in cmd]
        print('ctrl',cmd_arry)
        car_controls = fsds.CarControls()
        car_controls.throttle = float(cmd_arry[0])
        car_controls.steering = float(cmd_arry[1])
        car_controls.brake = float(cmd_arry[2])
        client.setCarControls(car_controls)
        
server.close()



