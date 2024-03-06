import numpy as np
import cv2 as cv
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import math
from PID import TAMU_Controller #Class used for PID controller, going to pass it the lines in the contours

global image

def get_contours(im_pth):
    #opening the image and grayscaling it
    global image
    image = cv.imread(im_pth)
    assert image is not None, "file could not be read, check with os.path.exists()"
    imgray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(imgray, 127, 255, 0)

    #Using find_contours to find the edges
    contours, heirarchy = cv.findContours(thresh,  cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    indices = [0, 4] #Typically, the only indice we'd care about is 0 or -1, but due to the hole in A, we have to finagle a bit
    contours = np.asarray(contours)[np.isin(heirarchy, indices)[:, :, -1][0]]
    
    # Displaying the contours
    # cv.drawContours(image, contours, -1, (0,255,0), 3)
    # cv.imshow("contour", image)
    # cv.waitKey()
    return contours

def adjustContours(perimeter, bounds = [-5, 5, -5, 2.87]):
    # perimeter = np.asarray(perimeter, dtype = object)
    # print(np.min(perimeter[:, 0, 0]))
    #Finding current bounds, have to do it in this method because it is a ragged array.
    minX = np.inf
    minY = np.inf
    maxX = 0
    maxY = 0
    for contour in perimeter:
        xMin = np.min(contour[:, 0, 0])
        yMin = np.min(contour[:, 0, 1])
        xMax = np.max(contour[:, 0, 0])
        yMax = np.max(contour[:, 0, 1])
        if xMin<minX:
            minX =  xMin
        if yMin<minY:
            minY = yMin
        if yMax>maxY:
            maxY = yMax
        if xMax>maxX:
            maxX = xMax

    #Performing the scaling and translation transformation
    adjX = maxX - minX
    adjY = maxY - minY
    out = []
    for contour in perimeter:
        contour = np.asarray(contour, dtype=float)
        contour[:, 0, 0] -=minX
        contour[:, 0, 1] -=minY

        contour[:, 0, 0] *=((bounds[1]-bounds[0])/adjX)
        contour[:, 0, 1] *=((bounds[3]-bounds[2])/adjY)

        contour[:, 0, 0] +=bounds[0]
        contour[:, 0, 1] +=bounds[2]
        out.append(contour)

    return out

def approximatePoly(contours):
    global image
    epsilon = .005
    perimeter = []
    for contour in contours:
        perimeter.append(cv.approxPolyDP(contour, epsilon, True))
    print("Initial", perimeter[-1].shape, perimeter[-1].dtype)
    #Displaying
    # for contour in perimeter:
    #     color = [random.randint(0,255) for col in range(3)]
    #     cv.polylines(image, [contour], True, color, 2)
    # cv.imshow("Contour with straight lines", image)
    # cv.waitKey(0)
    return perimeter        

def PIDController(perimeter, origin = 0.0):
    """Taking in lines, this will create an object to navigate through all the points provided. It will turn the pen on as it is tracing.

    Args:
        lines (_type_): array of contours, contains the line segments. 
        origin (float, optional): _description_. Defaults to 0.0. Dont know if this is necessary yet.
    """
    rclpy.init(args=args)
    controller = TAMU_Controller()

    for contour in perimeter:
        initial = True
        controller.set_pen(False)
        for point in contour:
            # print("Point: ", point)
            controller.move_point(point[0])
            if initial:
                initial = False
                controller.set_pen(True)

    #TODO: Go through each contour, patching the edges together after turning the pen off. 
    rclpu.shutdown()
    pass

if __name__ == "__main__":
    #Step One: Get the contours
    contours = get_contours('./img.jpg')

    #Step Two: Break the contours into lines (approxPolyDP)
    perimeter = approximatePoly(contours)

    #Step Three: Map these contours to real-values (bounding box method)
    adjusted = adjustContours(perimeter, bounds = [-5, 5, -5, 2.87])

    #Step Four: Iterate through each of these contours and trace them using a PID controller. 
    PIDController(adjusted, origin = 0)