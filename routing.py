import numpy as np
import cv2 as cv
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import SetBool
import math
import time

global image


class PID():
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint

        self.prev_error = 0.0
        self.integral = 0

    # NOTE: Should move this to a decorator
    def set_point(self, setpoint):
        self.setpoint = setpoint

    def calculate_control(self, current_value):
        error = self.setpoint - current_value

        self.integral += error
        derivative = error - self.prev_error

        control_effort = self.kp * error + self.ki * \
            self.integral + self.kd * derivative

        self.prev_error = error

        self.integral = max(-1000, min(self.integral, 1000)
                            )  # bounds on the integral
        return control_effort

    # For linear movement, we take into account both X and Y
    def calculate_control2D(self, current_value):
        # Have to do L1 norm to maintain directionality
        error = (self.setpoint[0] - current_value[0]) + \
            (self.setpoint[1] - current_value[1])

        self.integral += error
        derivative = error - self.prev_error

        control_effort = self.kp * error + self.ki * \
            self.integral + self.kd * derivative

        self.prev_error = error

        self.integral = max(-100, min(self.integral, 100)
                            )  # bounds on the integral
        return control_effort

class TAMU_Controller(Node):
    def __init__(self):
        super().__init__('shape_drawer')
        # Movement and feedback
        # TODO: Setup the subscriber properly.
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # What is the topic for returning the information? #What is the  datatype for Pose's location?
        self.sub = self.create_subscription(
            Pose2D, "/pose", self.pose_callback, 10)

        # Setting PID controller parameters
        self.angle_PID = PID(kp=.5, ki=.00, kd=.00, setpoint=0)
        self.linear_PID = PID(kp=.5, ki=.00, kd=.01, setpoint=0)

        # Setting bounds on the upper limits of the controller outputs
        self.max_rad = 17
        self.max_vel = 10

        self.msg = Twist()

        # Default Parameters
        self.current_pose = Pose2D()
        self.current_pose.x = 0.0
        self.current_pose.y = 0.0
        self.current_pose.theta = 0.0

        # Initialize the service client to set pen
        self.set_pen_client = self.create_client(SetBool, 'set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen service not available, waiting again...')

    def stop(self):
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)

    def _angle(self, point):
        self.stop()
        rclpy.spin_once(self)
        angle = (math.atan2(
            (point[1]-self.current_pose.y), (point[0]-self.current_pose.x)))
        
        #Testing that the angle is in the proper direction
        # l2Ang = np.sqrt((np.cos(angle)+self.current_pose.x-point[0])**2 + (np.sin(angle)+self.current_pose.y-point[1])**2)
        # l2Base = np.sqrt((np.cos(self.current_pose.theta)+self.current_pose.x-point[0])**2 + (np.sin(self.current_pose.theta)+self.current_pose.y-point[1])**2)
        # if l2Ang>l2Base:
        #     angle-=math.pi
        self.angle_PID.set_point(angle)
        print("ANGLE SET POINT: ", angle)
        controlArr = np.full([10], np.inf)  # Moving average filter
        idx = 0
        while np.abs(np.mean(controlArr)) > .01:
            rclpy.spin_once(self)
            control = self.angle_PID.calculate_control(self.current_pose.theta)

            self.msg.angular.z = float(
                max(-self.max_rad, min(control, self.max_rad)))
            self.pub.publish(self.msg)
            # print("Angular CONTROL: ", control, " Set Point: ", self.angle_PID.setpoint,
            #       " Pose: ", self.current_pose, " Message: ", self.msg)
            time.sleep(.05)  # Pose updates at 10 Hz
            controlArr[idx] = control
            idx += 1
            if idx >= 10:
                idx = 0
        self.stop()

    def _linear(self, point):
        self.stop()
        print("Linear Point: ", point)
        self.linear_PID.set_point(point[0])
        controlArr = np.full([10], np.inf)
        idx = 0
        timeout = 100  # iterations before angle gets readjusted. To compensate for drift
        while np.abs(np.mean(controlArr)) > .01:
            rclpy.spin_once(self)
            # control = self.linear_PID.calculate_control2D([self.current_pose.x, self.current_pose.y])
            control = self.linear_PID.calculate_control(self.current_pose.x)
            control = float(max(-self.max_vel, min(control, self.max_vel)))
            self.msg.linear.x = np.sign(np.cos(self.current_pose.theta))*control #accounting for facing the opposite direction
            # self.msg.linear.y = control #This only uses
            self.pub.publish(self.msg)
            print("Linear CONTROL: ", control, " Set Point: ",
                  self.linear_PID.setpoint, " Pose: ", self.current_pose)
            time.sleep(.05)  # Pose updates at 10 Hz
            controlArr[idx] = control
            idx += 1
            if idx >= 10:
                idx = 0
            timeout -= 1
            if timeout == 0:
                self._angle(point)
                timeout = 100
        self.stop()

    def move_point(self, point):
        self._angle(point)
        self._linear(point)

    def pose_callback(self, data):
        # print("Pose Updated: ", data)
        self.current_pose = data

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
    # opening the image and grayscaling it
    global image
    image = cv.imread(im_pth)
    assert image is not None, "file could not be read, check with os.path.exists()"
    imgray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(imgray, 127, 255, 0)

    # Using find_contours to find the edges
    contours, heirarchy = cv.findContours(
        thresh,  cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # Typically, the only indice we'd care about is 0 or -1, but due to the hole in A, we have to finagle a bit
    indices = [0, 4]
    contours = np.asarray(contours)[np.isin(heirarchy, indices)[:, :, -1][0]]

    # Displaying the contours
    # cv.drawContours(image, contours, -1, (0,255,0), 3)
    # cv.imshow("contour", image)
    # cv.waitKey()
    return contours

def adjustContours(perimeter, bounds=[-5, 5, -5, 2.87]):
    # perimeter = np.asarray(perimeter, dtype = object)
    # print(np.min(perimeter[:, 0, 0]))
    # Finding current bounds, have to do it in this method because it is a ragged array.
    minX = np.inf
    minY = np.inf
    maxX = 0
    maxY = 0
    for contour in perimeter:
        xMin = np.min(contour[:, 0, 0])
        yMin = np.min(contour[:, 0, 1])
        xMax = np.max(contour[:, 0, 0])
        yMax = np.max(contour[:, 0, 1])
        if xMin < minX:
            minX = xMin
        if yMin < minY:
            minY = yMin
        if yMax > maxY:
            maxY = yMax
        if xMax > maxX:
            maxX = xMax

    # Performing the scaling and translation transformation
    adjX = maxX - minX
    adjY = maxY - minY
    out = []
    for contour in perimeter:
        contour = np.asarray(contour, dtype=float)
        contour[:, 0, 0] -= minX
        contour[:, 0, 1] -= minY

        contour[:, 0, 0] *= ((bounds[1]-bounds[0])/adjX)
        contour[:, 0, 1] *= ((bounds[3]-bounds[2])/adjY)

        contour[:, 0, 0] += bounds[0]
        contour[:, 0, 1] += bounds[2]
        out.append(contour)

    return out

def approximatePoly(contours):
    global image
    epsilon = .01
    perimeter = []
    for contour in contours:
        perimeter.append(cv.approxPolyDP(contour, epsilon, True))
    # print("Initial", perimeter[-1].shape, perimeter[-1].dtype, perimeter[-2].shape)
    # Displaying
    # for contour in perimeter:
    #     color = [random.randint(0,255) for col in range(3)]
    #     cv.polylines(image, [contour], True, color, 2)
    # cv.imshow("Contour with straight lines", image)
    # cv.waitKey(0)
    return perimeter


def PIDController(perimeter, origin=0.0):
    """Taking in lines, this will create an object to navigate through all the points provided. It will turn the pen on as it is tracing.

    Args:
        lines (_type_): array of contours, contains the line segments. 
        origin (float, optional): _description_. Defaults to 0.0. Dont know if this is necessary yet.
    """
    rclpy.init()
    controller = TAMU_Controller()
    # rclpy.spin(controller)
    for contour in perimeter:
        initial = True
        controller.set_pen(False)
        for point in contour:
            # print("Point: ", point)
            controller.move_point(point[0])
            if initial:
                initial = False
                controller.set_pen(True)
        controller.move_point(contour[0][0])

    # TODO: Go through each contour, patching the edges together after turning the pen off.
    rclpy.shutdown()
    pass


def main():
    # Step One: Get the contours
    contours = get_contours(
        '/home/tcous/ros2_humble/src/TAMU_ctrl/TAMU_ctrl/img.jpg')
    # contours = get_contours('./img.jpg')

    # Step Two: Break the contours into lines (approxPolyDP)
    perimeter = approximatePoly(contours)

    # Step Three: Map these contours to real-values (bounding box method)
    #adjusted = adjustContours(perimeter, bounds=[-5, 5, -5, 2.87])
    adjusted = perimeter
    # Step Four: Iterate through each of these contours and trace them using a PID controller.
    PIDController(adjusted, origin=0)


if __name__ == "__main__":
    main()
