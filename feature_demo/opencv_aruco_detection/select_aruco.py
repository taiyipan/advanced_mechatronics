import cv2
import cv2.aruco as aruco
import math

def findArucoMarkers(img, markerSize = 6, totalMarkers = 250, draw = True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs, ids)
    return [bboxs, ids]

def diagonal_distance(bbox) -> float:
    point_a = bbox[0][0]
    point_b = bbox[0][2]
    return math.sqrt(abs(point_b[0] - point_a[0])**2 + abs(point_b[1] - point_a[1])**2)

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    arucofound = findArucoMarkers(img)
     # loop through all the markers and augment each one
    if len(arucofound[0]) != 0:
        for bbox, id in zip(arucofound[0], arucofound[1]):
            print(id)
            print(diagonal_distance(bbox))
    cv2.imshow('img', img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()








































#
