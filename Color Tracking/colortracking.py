import cv2
import picamera
import numpy as np
import time
from gopigo import *
import math
from collections import namedtuple

im_temp = 0
pts_src = np.empty((0,2),dtype=np.int32)

Point = namedtuple("pt", "x y")
KeyPoint = namedtuple("KeyPoint", "size pt")

set_speed(55)

def mouse_handler(event, x, y, flags, param):
    global im_temp, pts_src
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(im_temp,(x,y), 3, (0,255,255), 5, cv2.LINE_AA)
        cv2.imshow("Stat Computation", im_temp)
        if len(pts_src) < 2:
            pts_src = np.append(pts_src, [(x,y)],axis=0)

def compute_statistics(frame):
    global im_temp, pts_src

    cv2.namedWindow('Stat Computation', cv2.WINDOW_NORMAL)

    im_temp = frame.copy()
    cv2.setMouseCallback('Stat Computation', mouse_handler)


    # Picking rectangle to determine colors
    while(len(pts_src) < 2):
        cv2.imshow('Stat Computation', im_temp)
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    # draws rectangle over portion we are interested in
    cv2.rectangle(im_temp, tuple(pts_src[0,:]), tuple(pts_src[1,:]), (255,255,0))
    cv2.imshow('Stat Computation', im_temp)

    # Points seem to be inverted. Pts_src seems to be (y,x)
    # Take only the part of the image inside of the reatangle
    selected_img = frame[pts_src[0][1]:pts_src[1][1], pts_src[0][0]:pts_src[1][0]]
    ##cv2.imshow("Selected Region", selected_img)
    # Convert into HSV instead of RGB
    selected_img_hsv = cv2.cvtColor(selected_img, cv2.COLOR_BGR2HSV)
    ##cv2.imshow("Selected Region", selected_img_hsv)

    # get the range of colors we want to track
    hsv_split = cv2.split(selected_img_hsv)
    mm = [(np.amin(hsv_split[i]), np.amax(hsv_split[i])) for i in  range(3)]

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return (np.array([mm[i][0] for i in range(3)]),
           np.array([mm[i][1] for i in range(3)]))


def thresh_and_binarize(frame, low, high):
    ''' Threshold and binarize an image. Then find blob and calculate
    centroid and area. Return centroid and area'''

    cv2.namedWindow("Centroid of Largest Blob", cv2.WINDOW_NORMAL)

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, low, high)

    masked = cv2.bitwise_and(frame, frame, mask=mask)

    kernel = np.ones((3,3), np.uint8)
    # dilation = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, kernel)
    # dilation = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
    # Found that only using dilation is best operation for now

    dilation = cv2.dilate(mask, kernel, iterations = 10)
    cv2.imshow('mod', dilation)


    # Need to set all other filters to False
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByInertia = False
    params.filterByConvexity = False

    #print(params.filterByColor, params.filterByArea, params.filterByCircularity, params.filterByInertia, params.filterByConvexity)
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(dilation)

    if (len(keypoints) == 0):
        return None
    biggest = max(keypoints, key=lambda x:x.size)
    print("Size: {k.size}\nPosition: {k.pt}".format(k = biggest))

    cv2.circle(frame,(int(biggest.pt[0]),int(biggest.pt[1])), 3, (255,0,255), 5, cv2.LINE_AA)
    cv2.imshow("Centroid of Largest Blob", frame)
    return(biggest)

# trun right 1 encoder in the same position
def my_turn_right():
    enc_tgt(1,0,1)
    right_rot()
    while enc_read(0) < 1:
        pass

# turn left 1 encoder in the same position
def my_turn_left():
    enc_tgt(1,0,1)
    left_rot()
    while enc_read(0) < 1:
        pass

def move_forward(distance = 5):
    SPEED = 80
    set_speed(SPEED)

    BASE = 18
    ENCODER_PER_CM = 18 / 20.42
    revolutions = int(distance * ENCODER_PER_CM)
    enc_tgt(1, 1, revolutions)
    fwd()

def move_backward(distance = 5):
    SPEED = 80
    set_speed(SPEED)

    BASE = 18
    ENCODER_PER_CM = 18 / 20.42
    revolutions = int(distance * ENCODER_PER_CM)
    enc_tgt(1, 1, revolutions)
    bwd()

def adjust_accordingly(initial_blob, blob):
    '''
    blob.size will give you size
    blob.pt[0] is the x value
    blob.pt[1] is the y value
    '''
    # Object is out of bounds, no clue what to do, so just stop
    # print blob.size
    # print initial_blob.size
    if blob == None:
        return

        # turn left or right
    if abs(blob.pt[0] - initial_blob.pt[0]) > initial_blob.size/4:
        if blob.pt[0] > initial_blob.pt[0]:
            my_turn_right()
            time.sleep(0.5)
        if blob.pt[0] < initial_blob.pt[0]:
            my_turn_left()
            time.sleep(0.5)
            
    # move forward or backward
    if abs(blob.size - initial_blob.size) > 12:
        if blob.size > initial_blob.size:
            move_backward()
            time.sleep(0.5)
        elif blob.size < initial_blob.size:
            move_forward()
            time.sleep(0.5)


def main():
    # for PiCamera
    camera = picamera.PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 24
    time.sleep(2)
    frame = np.empty((320 * 240 * 3), dtype=np.uint8)
    camera.capture(frame, 'bgr')
    frame = frame.reshape((240, 320, 3))
    frame = cv2.blur(frame, (5,5))

    # cap = cv2.VideoCapture(0)
    # time.sleep(0.5)

    # ret, image = cap.read()

    low_high = compute_statistics(frame)

    low_high[0][1] = low_high[0][1] - 20
    low_high[0][2] = low_high[0][2] - 20
    low_high[1][1] = low_high[1][1] + 20
    low_high[1][2] = low_high[1][2] + 20
    # low_high[0][1] = 50
    # low_high[0][2] = 50
    # low_high[1][1] = 255
    # low_high[1][2] = 255

    reading = thresh_and_binarize(frame, low_high[0], low_high[1])
    initial_blob = KeyPoint(reading.size, reading.pt)

    #greenLower = (29, 86, 6)
    #greenUpper = (64, 255, 255)
    # while (cap.isOpened()):
    initial = time.clock()
    blobs = []
    while True:
        # equivalent to  ret, frame = cap.read()
        frame = np.empty((320 * 240 * 3), dtype=np.uint8)
        camera.capture(frame, 'bgr')
        frame = frame.reshape((240, 320, 3))
        frame = cv2.blur(frame, (5, 5))
        blob = thresh_and_binarize(frame, low_high[0], low_high[1])
        if blob is not None:
            blobs.append(blob)
        # Sometimes blob==None when no centroid found, it is passed to adjust_accordingly and then nothing should happen
        # When this happens the video will appear to freeze because we do not update the image output with cv2.imshow in thresh_and_binarize

        # This number determines length of sleep, MUST BE EVEN
        ELEMENTS = 4
        if len(blobs) == ELEMENTS:
            # blob_size = sorted(blobs, key=lambda k: k.size)[int(math.ceil(ELEMENTS/2))]
            # blob_point = sorted(blobs, key=lambda k: k.pt)[int(math.ceil(ELEMENTS/2))]
            blob_size = sorted(blobs, key=lambda k: k.size)[ELEMENTS/2]
            blob_point = sorted(blobs, key=lambda k: k.pt)[ELEMENTS/2]
            blob = KeyPoint(blob_size.size, blob_point.pt)
            adjust_accordingly(initial_blob, blob)
            blobs = []

        # To close the windows, press 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

        # This is slowing down video capture
        #time.sleep(1)

        # Used to calculate duration of image capture
        # current = time.clock()
        # print("Time Elapsed: {}".format(current-initial))
        # initial = current


    # cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
