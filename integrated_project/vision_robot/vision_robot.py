'''
Set appropriate cronjob @reboot and execute bash command to run this program at computer start
'''
import cv2
import cv2.aruco as aruco
import math
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
from threading import Thread

# define scan aruco function: this is a threading function
def scan_aruco(camera_port: int, diagonal_length_threshold: float):
    # open camera handle (port, cam driver api)
    cam = cv2.VideoCapture(camera_port, cv2.CAP_ANY)
    # main loop
    while True:
        # capture a frame from camera
        success, img = cam.read()
        # detect all aruco markers in frame
        aruco_found = findArucoMarkers(img)
        # filter aruco found
        if len(arucofound[0]):
            for bbox, id in zip(arucofound[0], arucofound[1]):
                id = int(id[0])
                # target detected, send correct serial command to navigation robot
                if not memory.get(id) and diagonal_length(bbox) > diagonal_length_threshold:
                    if id >= 0 and id <= 9 and camera_port == 0:
                        serial_command(commands.get('cam0_friend_detected'))
                    elif id >= 0 and id <= 9 and camera_port == 1:
                        serial_command(commands.get('cam1_friend_detected'))
                    elif id >= 10 and id <= 19 and camera_port == 0:
                        serial_command(commands.get('cam0_enemy_detected'))
                    elif id >= 10 and id <= 19 and camera_port == 1:
                        serial_command(commands.get('cam1_enemy_detected'))
                    # update memory
                    memory[id] = True
                    break # save time
        cv2.waitKey(1)
    # close camera handle
    cam.release()

# define aruco detection algorithm
def detect_aruco(img, marker_size = 6, total_markers = 250):
    # convert to grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # obtain aruco dictionary
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    aruco_dict = aruco.Dictionary_get(key)
    aruco_param = aruco.DetectorParameters_create()
    # detect aruco markers within image
    bboxs, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters = aruco_param)
    # return results as tuple
    return (bboxs, ids)

# define diagonal length calculation of bounding box of detected aruco marker
def diagonal_length(bbox) -> float:
    point_a = bbox[0][0]
    point_b = bbox[0][2]
    return math.sqrt(abs(point_b[0] - point_a[0])**2 + abs(point_b[1] - point_a[1])**2)

# define serial command protocol
def serial_command(command: int):
    send_size = link.tx_obj(command)
    link.send(send_size)
    sleep(1)

# create memory for aruco markers: 0-19
memory = dict()
for i in range(20):
    memory[i] = False

# define serial commands
commands = {
    "cam0_friend_detected": 0,
    "cam0_enemy_detected": 1,
    "cam1_friend_detected": 2,
    "cam1_enemy_detected": 3,
}

# define serial USB port
usb_port = '/dev/ttyACM0'

# initiate serial communication
link = txfer.SerialTransfer(usb_port)
link.open()
sleep(5)

# define threads for 2 cameras (Python concurrency, I/O bound task, threading)
cam0_thread = Thread(target = scan_aruco, arags = (0,))
cam1_thread = Thread(target = scan_aruco, arags = (1, 200,))

# start threads for 2 cameras
cam0_thread.start()
cam1_thread.start()
