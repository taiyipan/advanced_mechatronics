'''
Set appropriate cronjob @reboot and execute bash command to run this program at computer start
'''
import cv2
import cv2.aruco as aruco
import math
from time import sleep
import RPi.GPIO as GPIO

# global variables
FRIEND_STATE_PIN = 36
ENEMY_STATE_PIN = 38

# define scan aruco function
def scan_aruco(camera_port = 0, diagonal_length_threshold = 150):
    # open camera handle (port, cam driver api)
    cam = cv2.VideoCapture(camera_port, cv2.CAP_ANY)
    # main loop
    while True:
        # capture a frame from camera
        success, img = cam.read()
        # flip image vertically
        img = cv2.rotate(img, cv2.ROTATE_180)
        # detect all aruco markers in frame
        aruco_found = detect_aruco(img)
        # filter aruco found
        if len(aruco_found[0]):
            for bbox, id in zip(aruco_found[0], aruco_found[1]):
                id = int(id[0])
                print(diagonal_length(bbox))
                # target detected, send correct serial command to navigation robot
                if not memory.get(id) and diagonal_length(bbox) > diagonal_length_threshold:
                    if id >= 0 and id <= 9 and camera_port == 0:
                        io_command(commands.get('cam0_friend_detected'))
                    elif id >= 10 and id <= 19 and camera_port == 0:
                        io_command(commands.get('cam0_enemy_detected'))
                    # update memory
                    memory[id] = True
                    break # save time
        cv2.waitKey(1)
        # listen for arduino reset
        # arduino_reset()
    # close camera handle
    cam.release()

# define aruco detection algorithm
def detect_aruco(img, marker_size = 6, total_markers = 250):
    # convert to grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # obtain aruco dictionary
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
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
def io_command(command: int):
    print('Sending io command: {}'.format(command))
    if command == 0:
        GPIO.output(FRIEND_STATE_PIN, GPIO.HIGH)
        sleep(1)
        GPIO.output(FRIEND_STATE_PIN, GPIO.LOW)
    elif command == 1:
        GPIO.output(ENEMY_STATE_PIN, GPIO.HIGH)
        sleep(1)
        GPIO.output(ENEMY_STATE_PIN, GPIO.LOW)

# create memory for aruco markers: 0-19
memory = dict()
for i in range(20):
    memory[i] = False

# define serial commands
commands = {
    "cam0_friend_detected": 0,
    "cam0_enemy_detected": 1
}

try:
    print('Start')
    # set GPIO mode and declare pin modes
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(FRIEND_STATE_PIN, GPIO.OUT)
    GPIO.setup(ENEMY_STATE_PIN, GPIO.OUT)
    # start
    scan_aruco()
except:
    print('Exception')
finally:
    print('GPIO Cleanup')
    GPIO.cleanup() # reset GPIO
