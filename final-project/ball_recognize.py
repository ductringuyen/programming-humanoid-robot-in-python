''' FINAL PROJECT
* Robot be able to detect ball in the soccer field using OpenCV

* With forward_kinematics enable the robot to approach to ball and kick the ball

'''


from naoqi import ALProxy
import os
import sys
import cv2
import numpy as np
import pickle
import matplotlib.pyplot as plot


class BallDetectionAgent(SparkAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(BallDetectionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.ball = 'b_w_ball'
        self.ball_classifier = pickle.load(open('../classifier', 'rb')) #Load classifier


    def think(self, perception):
        self.ball = self.recognize_ball(perception)
        return super(BallDetectionAgent, self).think(perception)

    ''' Finish this '''
    def recognize_ball(self, perception):
        ball = 'b_w_ball'

        dataset = []

        dataset += perception.imu
        dataset = numpy.array(dataset).reshape(1,-1)

    vision = ALProxy('RobocupVision', 'localhost', 3100)

    cameraID = 0
    data = vision.getBGR24Image(cameraId)
    image = np.fromstring(data, dtype = np.uint8).reshape((480, 640, 3))

    rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plot.imshow(rgb_img)
