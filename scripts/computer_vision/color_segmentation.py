import cv2
import numpy as np
import pdb
import math

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def get_slope(pt1,pt2):
    try:
        return (pt1[1]-pt2[1])/(pt1[0]-pt2[0])
    except:
        print("undefined")
        return None
    
def get_length(pt1, pt2):
    return np.sqrt((pt1[1]-pt2[1])**2+(pt1[0]-pt2[0])**2)

def get_intersect(a1, a2, b1, b2):
    """ 
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return (int(x/z), int(y/z))

def angle_bisector_equation(p1, p2, q1, q2):
    """
    Find the equation of the angle bisector between two lines given two points on each line.

    Parameters:
    p1 (tuple): (x, y) coordinates of the first point on the first line
    p2 (tuple): (x, y) coordinates of the second point on the first line
    q1 (tuple): (x, y) coordinates of the first point on the second line
    q2 (tuple): (x, y) coordinates of the second point on the second line

    Returns:
    tuple: a tuple containing the slope and y-intercept of the angle bisector
    """
    m1 = get_slope(p1,p2)
    b1 = p1[1] - (m1 * p1[0])
    m2 = get_slope(q1, q2)
    b2 = q1[1] - (m2 * q1[0])
    
    A1 = -m1
    B1 = 1
    C1 = -b1 
    
    A2 = -m2
    B2 = 1
    C2 = -b2

    sqrt1 = math.sqrt(A1**2+B1**2)
    sqrt2= math.sqrt(A2**2+B2**2)
    denom = (B2*sqrt1-B1*sqrt2)
    
    # new_slope = (A1*math.sqrt(A2**2+B2**2) - A2*math.sqrt(A1**2+B1**2))/(B2*math.sqrt(A1**2+B1**2)-B1*math.sqrt(A2**2+B2**2))
    # new_yintercept = (C1*math.sqrt(A2**2+B2**2) - C2*math.sqrt(A1**2+B1**2))/(B2*math.sqrt(A1**2+B1**2)-B1*math.sqrt(A2**2+B2**2))
    new_slope = (A1*sqrt2 - A2*sqrt1)/denom
    new_yintercept = (C1*sqrt2 - C2*sqrt1)/denom
    return new_slope, new_yintercept

def best_lines_bisector_line(fd_linesp, shape):
    x_max,y_max = shape[1],shape[0]
    ####testing
    all_lines = np.array(fd_linesp)
    start_pts = all_lines[:, :2]
    end_pts = all_lines[:, 2:]
    
    # Compute angles for all lines
    angles = np.arctan2(end_pts[:, 1] - start_pts[:, 1], end_pts[:, 0] - start_pts[:, 0])
    sort_ind = np.argsort(angles)
    sorted_lines = all_lines[sort_ind]
    
    # Find lines with minimum and maximum angles
    pos_idx = np.where(angles[sort_ind]>0.2)#np.argmin(angles) #right
    neg_idx = np.where(angles[sort_ind]<-0.2)#np.argmax(angles) #left
    
    # Get start and end points of the two lines
    A,B = (0-0.001,y_max),(0,0) #left side
    C,D = (x_max+0.001,y_max),(x_max,0) #right side
    if np.size(pos_idx, axis=None): #if no positive slopes, use right image edge
        med_pos = round(np.median(pos_idx))
#         print("median pos index:", med_pos)
        C,D = (sorted_lines[med_pos][0],sorted_lines[med_pos][1]),(sorted_lines[med_pos][2],sorted_lines[med_pos][3])

    if np.size(neg_idx, axis=None):
        neg_pos = round(np.median(neg_idx))
        A,B = (sorted_lines[neg_pos][0],sorted_lines[neg_pos][1]),(sorted_lines[neg_pos][2],sorted_lines[neg_pos][3])
    
    # Compute intersection point of the two lines
    intersection_pt = get_intersect(A, B, C, D)
    
    slope, y_intercept = angle_bisector_equation(A, B, C, D)
    avg_y = y_max #(B[0] + D[0]) / 2
    a_x = (avg_y - y_intercept) / slope
    end_intersection_pt = (int(a_x),int(avg_y))#(int(avg_y), int(a_x))


    # quad_slope, quad_y_intercept = angle_bisector_equation(A,B,intersection_pt, end_intersection_pt)
#     a_quad_x = (avg_y - quad_y_intercept) / quad_slope
#     end_quad_intersection_pt = (int(a_quad_x),int(avg_y))

    # print(intersection_pt, end_intersection_pt)
    return intersection_pt, end_intersection_pt

def cd_color_segmentation(img, template, visualize =True):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        point: that is some look ahead distance on the angle bisector line between the two track lines
    """
    ########## YOUR CODE STARTS HERE ##########
    ### Cut off anything not the floor
    start_point1 = (0, 0) #top rectangle 
    end_point1 = (676, 150) #top rectangle: just edit y_coordinate for line_follower

    thickness = -1 
    color = (0, 0, 0) # Black color in BGR
    img_line= cv2.rectangle(img, start_point1, end_point1, color, thickness)


    ### change image to HSV and detect white 
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,200])
    upper_white = np.array([179,255,255])

    mask = cv2.inRange(img_hsv, lower_white, upper_white)  
    masked_img = cv2.bitwise_and(img_hsv, img_hsv, mask=mask).astype('uint8')

    ### Remove any small pieces 
    hsv_image = masked_img#cv2.cvtColor(img_masked2, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv_image)
    ret, th1 = cv2.threshold(h,180,255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    kernel = np.ones((1,1), dtype = "uint8")/9
    bilateral = cv2.bilateralFilter(th1, 9 , 75, 75)
    erosion = cv2.erode(bilateral, kernel, iterations = 1)

        ##finding the area of all connected white pixels in the image
    pixel_components, output, stats, centroids =cv2.connectedComponentsWithStats(erosion, connectivity=8)
    area = stats[1:, -1]
    pixel_components = pixel_components - 1
    min_size = 50
    img2 = np.zeros((output.shape))

        ##Removing the small white pixel area below the minimum size
    for i in range(0, pixel_components):
        if area[i] >= min_size:
            img2[output == i + 1] = 255

    # try:
    ### get hough transform and filter by slope and length 
    src = np.uint8(img2)
    src = cv2.GaussianBlur(src, (5, 5), 0)
    dst = cv2.Canny(src, 50, 200, None, 3)

    minLineLength = 70
    maxLineGap = 10
    linesP = cv2.HoughLinesP(dst, 1, np.pi/180, threshold=50, minLineLength=minLineLength, maxLineGap=maxLineGap)
    filtered_linesp = []

    if linesP is not None:
        for line in linesP:
            x1, y1, x2, y2 = line[0]
            if abs(y2 - y1) > 0.2 * abs(x2 - x1) and get_length((x1, y1), (x2, y2)) >= minLineLength:
                filtered_linesp.append([x1, y1, x2, y2])

    intersection_pt, end_intersection_pt = best_lines_bisector_line(filtered_linesp, img.shape)
    avg_pt = int((intersection_pt[0]+end_intersection_pt[0])/2), int((intersection_pt[1]+end_intersection_pt[1])/2)

    if visualize:
        cv2.line(img, intersection_pt, end_intersection_pt, (0,200,255), 3, cv2.LINE_AA)
        cv2.circle(img, avg_pt, radius=5, color=(225, 0, 255), thickness=-1)
        image_print(img)
                
    return avg_pt
    # except:
    # 	return None


if __name__ == '__main__':
    _img = cv2.imread("/Users/kristinezheng/racecar_docker/home/racecar_ws/src/final_challenge2023/track_img/c5.png")

    cd_color_segmentation(_img, "", True)