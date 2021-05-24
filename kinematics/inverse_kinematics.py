'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
import math


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        e = 0.01
        joint_angles = []
        lambda_ = 0.002
        # YOUR CODE HERE
        chain = {}

        for name in self.chains[effector_name]:
            chain[name] = self.perception.joint[name]

        matrix_pos = [0] * len(self.chains[effector_name])
        for i,name in enumerate(self.chains[effector_name]):
            matrix_pos[i] = self.transforms[name]

        while(np.linalg.norm(transform - matrix_pos[-1]) > e):
            dx = self.jacobianTrafo(effector_name, transform, matrix_pos)

            d_theta = np.dot(np.dot(J, np.linalg.pinv(np.dot(J.T, J))), e.T) * lambda_
            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(d_theta)[i]
                matrix_pos[i] = self.transform[name]
            if  np.linalg.norm(d_theta) < 1e-3:
                break


        return joint_angles

    #decompose a transformationmatrix
    def get_values(self, matrix):

        tx = np.arctan2(matrix[2,1], matrix[2,2])
        ty = np.arctan2(-matrix[0,2], np.sqrt(pow(matrix[2, 1], 2) + pow(matrix[2, 2], 2)))
        tz = np.arctan2(matrix[1,0], matrix[0,0])

        return {'tx': tx, 'ty': ty, 'tz': tz}

    def jacobianTrafo(self, effector_name, transform, matrix_pos):
        Jx = self.jacobianHelper(effector_name, transform, matrix_pos)
        V = transform - matrix_pos[-1]
        dx = np.dot(Jx, V)

        return dx


    def jacobianHelper(self, effector_name, slot, matrix_pos):

        Jx = [0] * len(matrix_pos)
        i = 0
        for matrix in matrix_pos:
            val = self.get_values(matrix)
            vec = [val['tx'], val['ty'], val['tz']]

            vec = np.cross(vec, np.subtract([slot[0,3], slot[1,3], slot[2,3]], [matrix[0,3], matrix[1,3], matrix[2,3]]))
            Jx[i] = vec
            i += 1

        print(Jx)
        return Jx


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        names = []
        times = []
        keys = []
        self.forward_kinematics(self.perception.joint)
        angles = self.inverse_kinematics(effector_name, transform)
        print(angles)

        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([1.0, 3.0])
            keys.append([[angles[joint] - 0.01, [3, 0, 0], [3, 0, 0]], [angles[joint], [3, 0, 0], [3, 0, 0]]])
        self.keyframes = (names, times, keys) # the result joint angles have to fill in


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
