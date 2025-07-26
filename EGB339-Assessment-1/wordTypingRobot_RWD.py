import math
import numpy as np
from coppeliaRobot import CoppeliaRobot
import scipy.optimize
import time

def Keyboardcoords(orig):
    angle = np.pi/2
    Tr = np.array([
            [ np.cos(angle), -np.sin(angle), 0,  247.5],
            [ np.sin(angle), np.cos(angle) , 0, -77.25],
            [ 0            , 0             , 1,    0],
            [ 0            , 0             , 0,    1]
            ], dtype=np.float64)
    
    keycoord = np.dot(Tr,orig)
    final = np.array([keycoord[0], keycoord[1], keycoord[2]])
    return final
 
letterCoords = {'q': Keyboardcoords(np.array([6.75,28.75,20,1])), 
            'w': Keyboardcoords(np.array([20.25,28.75,20,1])), 
            'e': Keyboardcoords(np.array([33.75,28.75,20,1])), 
            'r': Keyboardcoords(np.array([47.25,28.75,20,1])), 
            't': Keyboardcoords(np.array([60.75,28.75,20,1])), 
            'y': Keyboardcoords(np.array([74.25,28.75,20,1])), 
            'u': Keyboardcoords(np.array([87.75,28.75,20,1])), 
            'i': Keyboardcoords(np.array([101.25,28.75,20,1])), 
            'o': Keyboardcoords(np.array([114.75,28.75,20,1])), 
            'p': Keyboardcoords(np.array([128.25,28.75,20,1])),
            'a': Keyboardcoords(np.array([13.5,17.25,20,1])), 
            's': Keyboardcoords(np.array([27,17.25,20,1])), 
            'd': Keyboardcoords(np.array([40.5,17.25,20,1])), 
            'f': Keyboardcoords(np.array([54,17.25,20,1])), 
            'g': Keyboardcoords(np.array([67.5,17.25,20,1])), 
            'h': Keyboardcoords(np.array([81,17.25,20,1])), 
            'j': Keyboardcoords(np.array([94.5,17.25,20,1])), 
            'k': Keyboardcoords(np.array([108,17.25,20,1])), 
            'l': Keyboardcoords(np.array([121.5,17.25,20,1])),
            'z': Keyboardcoords(np.array([29.25,5.75,20,1])), 
            'x': Keyboardcoords(np.array([42.75,5.75,20,1])), 
            'c': Keyboardcoords(np.array([56.25,5.75,20,1])), 
            'v': Keyboardcoords(np.array([69.75,5.75,20,1])), 
            'b': Keyboardcoords(np.array([83.25,5.75,20,1])), 
            'n': Keyboardcoords(np.array([96.75,5.75,20,1])), 
            'm': Keyboardcoords(np.array([110.25,5.75,20,1])), 
}
coordinates = []

# joint lengths in mm
L0 = 138
L1 = 135
L2 = 147
L3 = 60
L4 = -70


def wordTypingRobot(robotObj, word: str):
    """
    Type out a word on the screen using the Dobot Robot.
    Note: You must not change the name of this file or this function.

    Parameters
    ----------
    robotObj
        Dobot object; see fakeRobot.py for the API
        You can pass in a FakeRobot or CoppeliaRobot object for testing
    word
        Word to type out

    """
    print(f"I was asked to type: {word}")

    # move arm to initial position 
    robot.move_arm(0, 0, 0)
    moveToKBDLocation()

    for letter in word.lower():
        if letter in letterCoords:
            coords = letterCoords[letter]
            coordinates.append(coords) # will contain all of the coordinates the robot will need to move to in an array

            moveToLocation(coords)
            time.sleep(1)
            press_key_simulation(coords)
            time.sleep(0.5)

    move_To_enter() 
    robot.move_arm(0, 0, 0)


def moveToKBDLocation():
    KBD_Coords = np.array([247.5, -77.25, 20])
    joint_angles = inverse_kinematics(KBD_Coords)
    robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])

def moveToLocation(coords):
    joint_angles = inverse_kinematics(coords)
    robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])

def press_key_simulation(coords):
    """
    Simulate pressing a key. It moves the robot down by a small distance
    and then moves it back up.
    """
    # Adjust the Z-coordinate (lower by 10mm to simulate key press)
    coords[2] -= 20  
    joint_angles = inverse_kinematics(coords)
    robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
    time.sleep(0.2)  # delay to simulate pressing
    
    # Move back to original position
    coords[2] += 20  
    joint_angles = inverse_kinematics(coords)
    robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])

def move_To_enter():
    Original_enterCoords = np.array([137.75, 17.25, 20, 1])
    enterCoords = Keyboardcoords(Original_enterCoords)
    joint_angles = inverse_kinematics(enterCoords)

    robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
    time.sleep(0.5)
    press_key_simulation(enterCoords)

def rot_y(theta):
    mtrx = np.array([[np.cos(theta), 0, np.sin(theta), 0], 
                     [0, 1, 0, 0], 
                     [-(np.sin(theta)), 0, np.cos(theta), 0], 
                     [0, 0, 0, 1]], dtype=np.float64)

    return mtrx

def rot_z(theta):
    mtrx = np.array([[np.cos(theta), -(np.sin(theta)), 0, 0], 
                     [np.sin(theta), np.cos(theta), 0, 0], 
                     [0, 0, 1, 0], 
                     [0, 0, 0, 1]], dtype=np.float64)
    
    return mtrx

def Translate(x, y, z):
    mtrx = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    return mtrx

def forward_kinematics_q(q):
    T1 = Translate(0, 0, L0)
    T2 = Translate(0, 0, L1)
    T3 = Translate(L2, 0, 0)
    T4 = Translate(L3, 0, L4)
    
    R1 = rot_z(q[0])
    R2 = rot_y(q[1])
    R3 = rot_y(q[2])
    R4 = rot_y(q[3])

    T = R1 @ T1 @ R2 @ T2 @ R3 @ T3 @ R4 @ T4
    
    p = T[:3, 3]
    
    return p


def joint_mapping(th):
    """
    Map the physical joint angles to the kinematic joint angles

    Parameters
    ----------
    th
        A numpy array array of shape (3,) of physical joint angles
        [theta1, theta2, theta3] in radians

    Returns
    -------
    q
        A numpy array of shape (4,) of kinematic joint angles [q1, q2, q3, q4]
        in radians
    """
    theta1, theta2, theta3 = th
    q1 = theta1
    q2 = theta2
    q3 = theta3 - theta2
    q4 = theta3
    return np.array([q1, q2, q3, -q4], dtype=np.float64)


def forward_kinematics(theta):
    """
    Calculate the forward kinematics of the robot for a given set of
    physical joint angles

    Parameters
    ----------
    theta
        A numpy array of shape (3,) of physical joint angles [theta1, theta2, theta3]
        in radians

    Returns
    -------
    p
        A numpy array of shape (3,) of the position of the end effector in the world
        frame in millimeters

    """
    q = joint_mapping(theta)
    return forward_kinematics_q(q)

def distance(p1, p2):
    """
    Compute the Euclidean distance between two 3D points p1 and p2.

    Parameters
    ----------
    p1 : numpy array of shape (3,)
        The coordinates of the first point.
    p2 : numpy array of shape (3,)
        The coordinates of the second point.

    Returns
    -------
    float
        The Euclidean distance between p1 and p2.
    """
    return np.linalg.norm(p1 - p2)

def cost(theta, pstar):
    """
    Cost function for inverse kinematics

    Parameters
    ----------
    theta
        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)
    pstar
        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)

    Returns
    -------
    c
        A scalar value cost which is the Euclidean distance between
        forward kinematics and pstar

    """
    p = forward_kinematics(theta)
    return distance(p, pstar)

def inverse_kinematics(pstar):
    """
    Inverse kinematics using optimisation

    Parameters
    ----------
    pstar
        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)

    Returns
    -------
    theta
        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)
    """
    initial_guess = np.array([0, 0, 0])
    theta = scipy.optimize.fmin(func=cost, x0=initial_guess, args=(pstar,))
    print(np.degrees(theta))
    return theta

if __name__ == "__main__":
    robot = CoppeliaRobot()  # Create an instance of the robot
    word = input('Introduce a word: ')
    wordTypingRobot(robot, word)
