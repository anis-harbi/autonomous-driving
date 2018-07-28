import picamera
import cv2
import numpy as np
import os
import time
import math

from gopigo import *

WIDTH = 320
HEIGHT = 240
SIZE = (WIDTH, HEIGHT)
TOTOAL_AREA = 76800
SPEED = 45

im_temp = 0
pts_src = np.empty((0,2),dtype=np.int32)

SAVED_TRANSFORM = 'transform.npy'
CAMERA_DIST = 18

"""
values of yellow and orange are adjusted according to the pictures taken by our
gopigo, not from the test images given by TAs
"""
YELLOW_MIN = np.array([25, 150, 46])
YELLOW_MAX = np.array([40, 255, 230])
ORANGE_MIN = np.array([6, 168, 234])
ORANGE_MAX = np.array([15, 192, 247])
HOUGH_THRESHOLD = 50

class Capturer(object):
    def __init__(self):
        resolution = (320, 240)
        self.camera = picamera.PiCamera(resolution=resolution, framerate=24)

    def capture_img(self):
        frame = np.empty((320 * 240 * 3), dtype=np.uint8)
        self.camera.capture(frame, 'bgr')
        return (frame.reshape(240, 320, 3))

def mouse_handler(event, x, y, flags, param):
    global im_temp, pts_src
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(im_temp,(x,y), 3, (0,255,255), 5, cv2.LINE_AA)
        cv2.imshow("transform finder", im_temp)
        if len(pts_src) < 4:
            pts_src = np.append(pts_src, [(x,y)],axis=0)

def get_transform(im_source, im_roadway):
    global im_temp, pts_src
    im_temp = im_source
    cv2.namedWindow("transform finder")
    cv2.setMouseCallback('transform finder', mouse_handler)

    while(len(pts_src) < 4):
        cv2.imshow('transform finder', im_source)
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    print(pts_src)

    s = im_roadway.shape


    pts_roadway = np.array([[0, 0], [s[0], 0], [s[0], s[1]], [0, s[1]]], dtype=np.int32)

    transform, status = cv2.findHomography(pts_src, pts_roadway)
    np.save(SAVED_TRANSFORM, transform)

    print(transform)

    return (transform)

def find_cameratohomo(transform):
    bottom_pixel = np.array([160, 240, 1])
    correspon = np.dot(transform, bottom_pixel)
    correspon = correspon / correspon[-1]
    y = correspon[1] - 240
    return y * 0.1

def thresh_and_binarize(frame, low, high):
    ''' Threshold and binarize an image. Then find blob and calculate
    centroid and area. Return centroid and area'''

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, low, high)

    kernel = np.ones((3,3), np.uint8)

    mask = cv2.dilate(mask, kernel, iterations = 6)
    mask = cv2.erode(mask, kernel, iterations = 6)

    return mask


def canny_and_hough(mask, threshold):
    """
    Input: mask of image from function thresh_and_binarize
    procedure: first implement canny edge detector, then implement Hough line
    Output: array of paramaters of lines, for each pair of parameters in
            lines[i], rho = lines[i][0], theta = lines[i][1]
    warnning: may need more effort to adjust parameters of hough lines
    """
    edges = cv2.Canny(mask,100,200)
    lines = cv2.HoughLines(edges,1,np.pi/180,threshold)
    if lines is None:
        return None
    lines_size = lines.shape
    lines = np.reshape(lines,[lines_size[0],lines_size[2]])
    arr =  lines[:2]
    arr[:,1] = map(math.degrees, arr[:,1])
    return map(tuple, arr)

def rotate_right_small():
    set_speed(SPEED)
    right_rot()
    time.sleep(0.1)
    stop()

def rotate_left_small():
    set_speed(SPEED)
    left_rot()
    time.sleep(0.1)
    stop()

def my_turn_right():
    set_speed(SPEED)
    enc_tgt(1,0,1)
    right_rot()
    while enc_read(0) < 1:
        pass

def my_turn_left():
    set_speed(SPEED)
    enc_tgt(1,0,1)
    left_rot()
    while enc_read(0) < 1:
        pass

def move_forward(distance = 5):
    set_speed(SPEED)
    BASE = 18
    ENCODER_PER_CM = 18 / 20.42
    revolutions = int(distance * ENCODER_PER_CM)
    enc_tgt(1, 1, revolutions)
    fwd()

def compute_distance(rho_theta_list, camera_distance):
    rho = rho_theta_list[0][0]
    theta = rho_theta_list[0][1]

    # 0.1 cm per pixel
    back_dist = camera_distance / 0.1

    d = (160 * math.cos(math.radians(theta)) + (back_dist + 240) * math.sin(math.radians(theta)) - rho) * 0.1
    return d

def main():
    c = Capturer()
    capture_img = c.capture_img
    img = capture_img()
    cv2.namedWindow('initial', cv2.WINDOW_NORMAL)
    cv2.moveWindow('initial', 20, 20)
    cv2.imshow('initial', img)

    im_roadway = np.empty(SIZE, dtype=np.uint8)
    im_source = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    transform = np.load(SAVED_TRANSFORM) if os.path.isfile(SAVED_TRANSFORM) else get_transform(im_source, im_roadway)

    key = cv2.waitKey(1) & 0xFF == ord('q')
    needToTurnRight = False
    needToTurnLeft = False
    while not key:
        img = capture_img()
        cv2.line(img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 0, 255))
        im_roadway = cv2.warpPerspective(img, transform, SIZE)
        cv2.imshow('initial', im_roadway)

        yellow_mask = thresh_and_binarize(im_roadway, YELLOW_MIN, YELLOW_MAX)
        line_parameters = canny_and_hough(yellow_mask, HOUGH_THRESHOLD)

        # if yellow line is missed, rotate to find yellow line
        while line_parameters is None:
            if needToTurnRight == True:
                rotate_right_small()
                img = capture_img()
                cv2.line(img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 0, 255))
                im_roadway = cv2.warpPerspective(img, transform, SIZE)
                cv2.imshow('initial', im_roadway)
                yellow_mask = thresh_and_binarize(im_roadway, YELLOW_MIN, YELLOW_MAX)
                line_parameters = canny_and_hough(yellow_mask, HOUGH_THRESHOLD)
            if needToTurnLeft == True:
                rotate_left_small()
                img = capture_img()
                cv2.line(img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 0, 255))
                im_roadway = cv2.warpPerspective(img, transform, SIZE)
                cv2.imshow('initial', im_roadway)
                yellow_mask = thresh_and_binarize(im_roadway, YELLOW_MIN, YELLOW_MAX)
                line_parameters = canny_and_hough(yellow_mask, HOUGH_THRESHOLD)

        # calculate the distance
        dist = compute_distance(line_parameters, CAMERA_DIST)
        # calculate hough line theta
        if line_parameters[0][1] > 90:
            hough_theta = line_parameters[0][1] - 90
        else:
            hough_theta = line_parameters[0][1] + 90
        # record current hough line position w.r.t magenta line
        if hough_theta > 90:
            needToTurnRight = True
            needToTurnLeft = False
        elif hough_theta == 90:
            needToTurnLeft = False
            needToTurnRight = False
        else:
            needToTurnLeft = True
            needToTurnRight = False
        # judge if we need to adjust direction
        if abs(hough_theta - 90) > 1:
            if hough_theta > 90:
                rotate_right_small()
            else:
                rotate_left_small()

        # judge if we see the orange stop line
        orange_mask = thresh_and_binarize(im_roadway, ORANGE_MIN, ORANGE_MAX)
        orange_mask = orange_mask / 255
        total_area = sum(sum(orange_mask))

        # print some values
        print(hough_theta)
        print('area', total_area)
        print("distance computed: {}".format(dist))
        # sees the orange line
        if total_area > 2000:
            move_forward(40)
            time.sleep(2)
            # rotate until yellow line is seen again
            # first rotate 15 encoders
            set_speed(SPEED)
            enc_tgt(1,0,15)
            left_rot()
            while enc_read(0) < 15:
                pass
            # then rotate a small amount each time until yellow line is seen again
            find_yellow = False
            img = capture_img()
            cv2.line(img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 0, 255))
            im_roadway = cv2.warpPerspective(img, transform, SIZE)
            cv2.imshow('initial', im_roadway)
            yellow_mask = thresh_and_binarize(im_roadway, YELLOW_MIN, YELLOW_MAX)
            yellow_mask /= 255
            total_area = sum(sum(yellow_mask))
            if total_area > 2000:
                find_yellow = True
            else:
                find_yellow = False

            while find_yellow != True:
                rotate_left_small()
                find_yellow = False
                img = capture_img()
                cv2.line(img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 0, 255))
                im_roadway = cv2.warpPerspective(img, transform, SIZE)
                cv2.imshow('initial', im_roadway)
                yellow_mask = thresh_and_binarize(im_roadway, YELLOW_MIN, YELLOW_MAX)
                yellow_mask /= 255
                total_area = sum(sum(yellow_mask))
                if total_area > 2000:
                    find_yellow = True
                else:
                    find_yellow = False
        # does not see the orange line
        else:
            move_forward(5)
            time.sleep(1)

        key = cv2.waitKey(1) & 0xFF == ord('q')

if __name__ == "__main__":
    main()
