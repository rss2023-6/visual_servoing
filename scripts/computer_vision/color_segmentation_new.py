from __future__ import division
import cv2
import numpy as np
import pdb
import math
import rospy

WIDTH = 400
HEIGHT = 300
SCALE = 100
CENTER = WIDTH // 2

def homography_matrix():
    PTS_IMAGE_PLANE =   [[180, 172],
                [315,226],
                [331, 170],
                [494, 159],
                [432, 210],
                [217, 220], 
                #FROM homography1
                [303, 367],
                [547, 361]]

    PTS_GROUND_PLANE = [[110.0, 50.0],
                [30.0, 0.0],
                [110, 0.0],
                [110.0, -50.0],
                [30.0, -10.0],
                [30.0, 10.0],
                #FROM HOMOGRAPHY1
                [10.0, 0.0],
                [10.0, -8.0]]

    METERS_PER_INCH = 0.0254

    if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
        rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

    np_pts_ground = np.array(PTS_GROUND_PLANE)
    np_pts_ground = np_pts_ground * METERS_PER_INCH
    np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

    np_pts_ground = np_pts_ground * SCALE
    np_pts_ground[:, :, 1] += CENTER
    np_pts_ground = np_pts_ground[:, :, ::-1]

    np_pts_image = np.array(PTS_IMAGE_PLANE)
    np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

    h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    return h

def transform_image(img):
    height, width, channels = img.shape
    height_cutoff = int(height*0.39)

    img[:int(height_cutoff), 0:width] = 0

    img = cv2.warpPerspective(img, homography_matrix(), (WIDTH, HEIGHT))

    plt.figure()
    plt.imshow(img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

    plt.figure()
    plt.imshow(thresh)

    linesP = cv2.HoughLinesP(thresh, rho=1, theta=np.pi/180, threshold=50, minLineLength=150, maxLineGap=10)
    # print(linesP)

    lines_arr = np.array(linesP)

    strong_lines = []
    x_intercepts = []
    strong_lines2 = []

    if(not type(linesP) is np.ndarray):
      return None, None, None

    # Iterate over each line
    for line in lines_arr:
        x1, y1, x2, y2 = line[0][0], line[0][1], line[0][2], line[0][3]

        if x1 == x2:
          x2+=1
        
        if abs (y1 - y2) < 80:
          continue
        
        m = (1.0*y2 - 1.0*y1) / (1.0*x2 - 1.0*x1)
        b = y1*1.0 - 1.0*m * x1
        intercept_with_top = -b / m
        convert_to_world = (intercept_with_top*1.0 - CENTER*1.0)/(SCALE*1.0)

        if abs(m) < 1:
          continue
        
        strong_lines.append([[m,b]])
        strong_lines2.append([[x1, y1, x2, y2]])
        x_intercepts.append(convert_to_world)
    
    return np.array(strong_lines), x_intercepts, strong_lines2

def get_lane_position(strong_lines, x_intercepts):
  x_intercepts = np.sort(x_intercepts)
  thresh = 0.15
  
  filter_intercepts = [x_intercepts[0]]

  for i in range(1, len(x_intercepts)):
    if(abs(x_intercepts[i] - x_intercepts[i-1]) >= thresh):
      filter_intercepts.append(x_intercepts[i])
  
  # print("FI", filter_intercepts)
  
  n = 0
  sum = 0
  angles = []
  for line in strong_lines:
    #get angles of each line in image
    m, b = line[0]
    angle = np.arctan(-1 * m)
    if(angle < 0):
      if(np.abs(angle + np.pi/2) < 0.7):
        angle = angle + np.pi
    angles.append(angle)
  
  avg_angle = np.mean(angles)
  avg_angle = -1.0 * (np.pi/2 - avg_angle)
  intercepts = filter_intercepts

  if(len(intercepts) == 2):
    left = intercepts[0]
    right = intercepts[1]
  else:
    found = False
    for i in range(len(intercepts) - 1):
      if((intercepts[i+1] > 0) != (np.sign(intercepts[i]) > 0)):
        left = intercepts[i]
        right = intercepts[i+1]
        found = True
    
    if not found:
      N = len(intercepts)
      if(intercepts[N-1] < 0):
        left = intercepts[N-1]
        right = intercepts[N-1] + 1.2
      else:
        right = intercepts[0]
        left = right - 1.2
  return avg_angle, left, right