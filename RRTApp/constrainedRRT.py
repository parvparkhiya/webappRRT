#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import RRTApp.lib.kdtree as kdtree
from RRTApp.OccupancyGrid import OccupancyGrid
from RRTApp.OccupancyGrid import plotObstacles
import RRTApp.trajectoryRollout as tRollout

class ConRRTNode:
    def __init__(self, x, y, theta, parentNode=None, control=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parentNode = parentNode
        self.control = control
        self.children = []

    def addChild(self, child):
        self.children.append(child)


class KDTNode:
    def __init__(self, node):
        self.coords = (node.x, node.y, node.theta)
        self.data = node

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.data)


def module2pi(angle):
    newAngle = angle
    if angle > np.pi:
        newAngle = angle - (2 * np.pi)
    elif angle < -np.pi:
        newAngle = angle + (2 * np.pi)
    return newAngle


class ConstrainedRRT:
    def __init__(self, startPose, goalPose, stepLength=1.0, arenaSize=(20, 20), conThr=0.5, goalBiasFactor=4, ogHandle=None, numRollout = 10):
        self.start = startPose
        self.goal = goalPose
        self.stepLength = stepLength
        self.arenaSize = arenaSize
        self.conThr = conThr
        self.goalBiasFactor = goalBiasFactor
        self.ogHandle = ogHandle

        self.numRollout = numRollout
        self.collisionSamplingFactor = 10
        self.enableReverse = True

        self.root = ConRRTNode(startPose[0], startPose[1], startPose[2])
        self.nodeList = [self.root]

        self.kdt = kdtree.create(dimensions=3)
        self.kdt.add(KDTNode(self.root))
        self.goalNode = None

    def setOccupancyGrid(self, ogHandle):
        self.ogHandle = ogHandle

    def checkConvergence(self, node):
        dist = self.getDist(node.x, node.y, self.goal[0], self.goal[1])
        if dist <= self.conThr:
            self.goalNode = node
            return True
        else:
            return False

    def sampleNode(self, visFlag=False):
        if np.random.randint(self.goalBiasFactor) == 0:
            sampleX = self.goal[0] + (2*np.random.rand()) - 1.0
            sampleY = self.goal[1] + (2*np.random.rand()) - 1.0
            sampleTheta = module2pi(self.goal[2] + (0.5*np.random.rand()) - 0.25)
        else:
            sampleX = (np.random.rand() * (self.arenaSize[0])) - (self.arenaSize[0] / 2.0)
            sampleY = (np.random.rand() * (self.arenaSize[1])) - (self.arenaSize[1] / 2.0)
            sampleTheta = module2pi((np.random.rand() * 2*np.pi) - np.pi)


        closestNode = self.kdtNearestNeighbour(sampleX, sampleY, sampleTheta)

        pose, control = self.getBestRollOut(closestNode, ConRRTNode(sampleX, sampleY, sampleTheta))

        if pose is not None:
            newNode = ConRRTNode(pose[0], pose[1], pose[2], closestNode, control)

            closestNode.addChild(newNode)
            self.kdt.add(KDTNode(newNode))
            self.nodeList.append(newNode)
            if visFlag:
                self.updateVisualization(closestNode, newNode)

            return True
        else:
            return False

    def sampleStepLength(self, enableReverse=False):
        tStepLength = np.random.normal(self.stepLength, 0.25)
        if tStepLength < 0.25:
            tStepLength = 0.25
        elif tStepLength > (self.stepLength + 0.25):
            tStepLength = self.stepLength + 0.25

        if enableReverse:
            if np.random.randint(2) == 0:
                tStepLength = tStepLength * (-1.0)

        return tStepLength

    @staticmethod
    def sampleSteeringAngle(count, steeringLimit=(-0.5, 0.5), randomFlag=False):
        if randomFlag:
            steeringAngles = (np.random.rand(count) * (steeringLimit[1] - steeringLimit[0])) + steeringLimit[0]
        else:
            steeringAngles = np.linspace(steeringLimit[0], steeringLimit[1], count)

        return steeringAngles

    def rollOutTrajectory(self, baseNode, goalNode, control):
        pose = tRollout.moveBicycleOnArc([baseNode.x, baseNode.y, baseNode.theta], steeringAngle=control[0], arcLength=control[1])
        distance = self.get3DDist(pose[0], pose[1], pose[2], goalNode.x, goalNode.y, goalNode.theta)

        if self.validateEdge(baseNode, control):
            return pose, distance
        else:
            return None, None

    def getBestRollOut(self, baseNode, goalNode):
        steeringAngles = self.sampleSteeringAngle(self.numRollout, randomFlag=True)

        bestControl = [0.0, self.sampleStepLength()]
        bestPose, minDistance = self.rollOutTrajectory(baseNode, goalNode, bestControl)
        for angle in steeringAngles:
            tControl = [angle, self.sampleStepLength(self.enableReverse)]
            tPose, tDistance = self.rollOutTrajectory(baseNode, goalNode, tControl)

            if (minDistance is None) or (tDistance is not None and tDistance < minDistance):
                minDistance = tDistance
                bestPose = tPose
                bestControl = tControl

        return bestPose, bestControl

    def validateNode(self, node):
        if self.ogHandle is not None:
            point1 = (node.x, node.y)
            point2 = (node.parentNode.x, node.parentNode.y)
            return not self.ogHandle.obstacleAtEdge(point1, point2)
        else:
            return True

    def validateEdge(self, node, control):
        if self.ogHandle is not None:
            arcLengthSample = np.linspace(0, control[1], self.collisionSamplingFactor)
            for tLength in arcLengthSample:
                newPose = tRollout.moveBicycleOnArc([node.x, node.y, node.theta], steeringAngle=control[0], arcLength=tLength)
                # print(newPose, " ", control)
                if self.ogHandle.obstacleAtPoint(newPose):
                    return False
            return True
        else:
            return True

    def kdtNearestNeighbour(self, sampleX, sampleY, sampleTheta):
        kdn = self.kdt.search_nn([sampleX, sampleY, sampleTheta])
        return (kdn[0]).data.data

    def getNearestNeighbour(self, sampleX, sampleY):
        closestNode = None
        closestNodeDistance = None
        for node in self.nodeList:
            dist = self.getDist(node.x, node.y, sampleX, sampleY)
            if (closestNode is None) or (dist < closestNodeDistance):
                closestNode = node
                closestNodeDistance = dist
        return closestNode

    def addNewNode(self, visualizeFlag):
        while True:
            if self.sampleNode(visualizeFlag):
                break

        return self.nodeList[-1]

    def generateRRT(self, visualizeFlag=False):

        if self.ogHandle is not None:
            if self.ogHandle.obstacleAtPoint(self.start):
                print("Obstacle at start pose")
                return False
            if self.ogHandle.obstacleAtPoint(self.goal):
                print ("Obstacle at goal Pose")
                return False

        if visualizeFlag:
            self.initVisualization()

        convergeFlag = False

        while not convergeFlag:
            newNode = self.addNewNode(visualizeFlag)
            convergeFlag = self.checkConvergence(newNode)

        return True

    def getPath(self):
        if self.goalNode is None:
            exit("Goal not found using RRT graph")
        else:
            pathNodeList = [self.goalNode]
            node = self.goalNode
            while node.parentNode is not None:
                node = node.parentNode
                pathNodeList.append(node)

            pathNodeList.reverse()
            return pathNodeList

    def initVisualization(self):
        plt.ion()
        plt.plot(self.root.x, self.root.y, 'bo')
        plt.plot(self.goal[0], self.goal[1], 'ro')
        plt.xlim([-self.arenaSize[0] / 2, self.arenaSize[0] / 2])
        plt.ylim([-self.arenaSize[1] / 2, self.arenaSize[1] / 2])
        if self.ogHandle is not None:
            plotObstacles(self.ogHandle.obstaclesData)
        plt.show()

    @staticmethod
    def updateVisualization(parentNode, childNode):
        plt.plot([parentNode.x, childNode.x], [parentNode.y, childNode.y], 'k')
        plt.plot(childNode.x, childNode.y, 'ko')
        plt.draw()
        plt.pause(0.0001)

    @staticmethod
    def getDist(x1, y1, x2, y2):
        return np.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

    @staticmethod
    def get3DDist(x1, y1, z1, x2, y2, z2):
        return  np.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(module2pi(z1 - z2), 2))

def plotConstrainedRRT(node, subSamplingFactor=10):
    for child in node.children:
        steeringAngle, arcLength = child.control
        arcLengthSample = np.linspace(0, arcLength, subSamplingFactor)
        curve = []
        for tLength in arcLengthSample:
            newPose = tRollout.moveBicycleOnArc([node.x, node.y, node.theta], steeringAngle, arcLength=tLength)
            curve.append(newPose)
        curve = np.asarray(curve)
        plt.plot(curve[:, 0], curve[:, 1], 'k')
        # plt.plot(child.x, child.y, 'ko')
        plotConstrainedRRT(child)

def getSmoothCurve(pathNodeList,subSamplingFactor=10):
    curve = []
    pose = [pathNodeList[0].x, pathNodeList[0].y, pathNodeList[0].theta]
    for node in pathNodeList[1:]:
        steeringAngle, arcLength = node.control
        arcLengthSample = np.linspace(0, arcLength, subSamplingFactor)
        for tLength in arcLengthSample:
            newPose = tRollout.moveBicycleOnArc(pose, steeringAngle, arcLength=tLength)
            curve.append(newPose)
        pose = curve[-1]

    return np.asarray(curve)


def drawArrow(nodes):
    arrowLength = 0.3
    for node in nodes:
        plt.arrow(node.x, node.y, arrowLength*np.cos(node.theta), arrowLength*np.sin(node.theta), color="blue", head_width=0.1, linewidth=2.0)


def visualizeTree(rootNode, goalPose=None, pathNodeList=None, obstaclesData=None, arenaSize=(20, 20)):
    plt.figure()
    plt.plot(rootNode.x, rootNode.y, 'bo')

    if goalPose is not None:
        plt.plot(goalPose[0], goalPose[1], 'ro')

    if obstaclesData is not None:
        plotObstacles(obstaclesData)

    plotConstrainedRRT(rootNode)

    if pathNodeList is not None:
        curve = getSmoothCurve(pathNodeList)
        plt.plot(curve[:, 0], curve[:, 1], 'r')
        drawArrow(pathNodeList)

    plt.xlim(-arenaSize[0] / 2, arenaSize[0] / 2)
    plt.ylim(-arenaSize[0] / 2, arenaSize[0] / 2)


def runConstrainedRRT(start=None, goal=None, stepLength=0.5, goalBiasFactor=5, arenaSize=(20, 20)):
    
    if goal is None:
        gx = (np.random.rand()*(arenaSize[0])) - (arenaSize[0]/2.0)
        gy = (np.random.rand()*(arenaSize[1])) - (arenaSize[1]/2.0)
        gtheta = (np.random.rand()*2*np.pi) - np.pi
        goal = [gx, gy, gtheta]

    if start is None:
        sx = (np.random.rand()*(arenaSize[0])) - (arenaSize[0]/2.0)
        sy = (np.random.rand()*(arenaSize[1])) - (arenaSize[1]/2.0)
        stheta = (np.random.rand()*2*np.pi) - np.pi
        start = [sx, sy, stheta]

    try:
        rrtHandle = ConstrainedRRT(startPose=start, goalPose=goal, stepLength=stepLength, arenaSize=arenaSize, goalBiasFactor=goalBiasFactor)
        ogHandle = OccupancyGrid(arenaSize=arenaSize, customFilePath="RRTApp/data/obstacle.data")

        rrtHandle.setOccupancyGrid(ogHandle)
        rrtHandle.generateRRT(visualizeFlag=False)
        path = rrtHandle.getPath()

        print('Node count in RRT:', len(rrtHandle.nodeList))
        visualizeTree(rrtHandle.root, goal, path, ogHandle.obstaclesData, arenaSize)
        plt.savefig("RRTApp/static/rrt.png", bbox_inches='tight')

        return start, goal, True
    except:
        return start, goal, False



if __name__ == '__main__':
    goal = (-7.5, -8, 0.0)
    start = (0.0, 0.0, np.pi/2)
    stepLength = 0.5
    arenaSize = (20, 20)
    goalBiasFactor = 5

    rrtHandle = ConstrainedRRT(startPose=start, goalPose=goal, stepLength=stepLength, arenaSize=arenaSize, goalBiasFactor=goalBiasFactor)
    ogHandle = OccupancyGrid.OccupancyGrid(arenaSize=arenaSize)

    rrtHandle.setOccupancyGrid(ogHandle)
    rrtHandle.generateRRT(visualizeFlag=False)
    path = rrtHandle.getPath()

    print('Node count in RRT:', len(rrtHandle.nodeList))
    visualizeTree(rrtHandle.root, goal, path, ogHandle.obstaclesData, arenaSize)
