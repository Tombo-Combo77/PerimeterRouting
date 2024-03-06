import numpy as np
import cv2 as cv
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import SetBool
import math
#from PID import TAMU_Controller #Class used for PID controller, going to pass it the lines in the contours

global image

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self, error):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = error

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

class TAMU_Controller(Node):
    def __init__(self):
        super().__init__('shape_drawer')
        # Movement and feedback
        #TODO: Setup the subscriber properly.
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, "/pose", self.pose_callback, 10) #What is the topic for returning the information? #What is the  datatype for Pose's location?

        # Setting PID controller parameters
        self.angle_PID = PID()
        self.distance_PID = PID()

        self.angle_PID.setKp(1.4)
        self.angle_PID.setKi(0)
        self.angle_PID.setKd(0)

        self.distance_PID.setKp(1.4)
        self.distance_PID.setKi(0)
        self.distance_PID.setKd(0)

        self.msg = Twist()

        #Default Parameters
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_angle = 0

        # Initialize the service client to set pen
        self.set_pen_client = self.create_client(SetBool, 'set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen service not available, waiting again...')

    def angular_controller(self):
        
        self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))

        self.xr = self.R*math.cos(self.current_angle)
        self.yr = self.R*math.sin(self.current_angle)

        self.xim = self.current_pose_x + self.xr
        self.yim = self.current_pose_y + self.yr

        self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))

        if self.xim > self.goal_x:

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        else:
            self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        
        while self.alpha>0.005: 
            self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))

            self.xr = self.R*math.cos(self.current_angle)
            self.yr = self.R*math.sin(self.current_angle)

            self.xim = self.current_pose_x + self.xr
            self.yim = self.current_pose_y + self.yr

            self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))
            
            if self.xim > self.goal_x:

                self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
            
            else:
                
                self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

            self.PID_angle = self.angle_PID.update(self.alpha)

            self.msg.angular.y = self.PID_angle

            self.pub.publish(self.msg)
            
    def distance_controller(self):
        distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))
        #self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))
        while distance > 0.15:

            distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))

            PID_distance_X = self.distance_PID.update(self.goal_x - self.current_pose_x)
            PID_distance_Y = self.distance_PID.update(self.goal_y - self.current_pose_y)

            self.msg.linear.x = PID_distance_X
            self.msg.linear.y = PID_distance_Y

            self.pub.publish(self.msg)

    
    # def move_contour(self, contour):
    #     #Need to iterate through every set of points in this contour
    #     #Its a bunch of line segments, so there might need to be some adjustment for the corners to avoid rounding them off. 
    #     for point in contour:
    #         self.goal_x = point[:, 0]
    #         self.goal_y = point[:, 1]
    #         #NOTE: Might need to make it so that the angle lines up before the distance controller does anything
    #         #TODO: Make it so the angular controller lines up before movement
    #         self.angular_controller()
    #         self.distance_controller()

    def move_point(self, point):
        self.goal_x = point[0]
        self.goal_y = point[1]
        self.angular_controller()
        self.distance_controller()

    def pose_callback(self, data):
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta

    # Helper functions
    def set_pen(self, value):
        # Call the set_pen service to activate/deactivate pen
        req = SetBool.Request()
        req.data = value
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Pen set to: %r' % future.result().message)
        else:
            self.get_logger().error('Failed to call set_pen service')
        
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
    rclpy.init()
    controller = TAMU_Controller()

    for contour in perimeter:
        initial = True
        controller.set_pen(False)
        for point in contour:
            print("Point: ", point)
            controller.move_point(point[0])
            if initial:
                initial = False
                controller.set_pen(True)

    #TODO: Go through each contour, patching the edges together after turning the pen off. 
    rclpu.shutdown()
    pass

def main():
    #Step One: Get the contours
    contours = get_contours('/home/tcous/ros2_humble/src/TAMU_ctrl/TAMU_ctrl/img.jpg')
    #contours = get_contours('./img.jpg')

    #Step Two: Break the contours into lines (approxPolyDP)
    perimeter = approximatePoly(contours)

    #Step Three: Map these contours to real-values (bounding box method)
    adjusted = adjustContours(perimeter, bounds = [-5, 5, -5, 2.87])

    #Step Four: Iterate through each of these contours and trace them using a PID controller. 
    PIDController(adjusted, origin = 0)

if __name__ == "__main__":
    main()
