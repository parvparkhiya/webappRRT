#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import RRTApp.lib.kdtree as kdtree
from RRTApp.OccupancyGrid import OccupancyGrid
from RRTApp.OccupancyGrid import plotObstacles

class RRTNode:
    def __init__(self, x, y, parentNode):
        self.x = x
        self.y = y
        self.parentNode = parentNode
        self.children = []

    def addChild(self, child):
        self.children.append(child)


class KDTNode:
    def __init__(self, node):
        self.coords = (node.x, node.y)
        self.data = node

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.data)


class RRT:
    def __init__(self, startPose, goalPose, stepLength=1.0, arenaSize=(20, 20), conThr=0.5, goalBiasFactor=4, ogHandle=None):
        self.start = startPose
        self.goal = goalPose
        self.stepLength = stepLength
        self.arenaSize = arenaSize
        self.conThr = conThr
        self.goalBiasFactor = goalBiasFactor
        self.ogHandle = ogHandle

        self.root = RRTNode(startPose[0], startPose[1], None)
        self.nodeList = [self.root]

        self.kdt = kdtree.create(dimensions=2)
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
        else:
            sampleX = (np.random.rand() * (self.arenaSize[0])) - (self.arenaSize[0] / 2.0)
            sampleY = (np.random.rand() * (self.arenaSize[1])) - (self.arenaSize[1] / 2.0)

        tStepLength = np.random.normal(self.stepLength, 0.25)
        if (tStepLength < 0.25):
            tStepLength = 0.25
        elif (tStepLength > self.stepLength + 0.25):
            tStepLength = self.stepLength + 0.25
        # closestNode = self.getNearestNeighbour(sampleX, sampleY)
        closestNode = self.kdtNearestNeighbour(sampleX, sampleY)

        theta = np.arctan2(sampleY - closestNode.y, sampleX - closestNode.x)
        newNode = RRTNode(closestNode.x + (tStepLength * np.cos(theta)),
                          closestNode.y + (tStepLength * np.sin(theta)), closestNode)

        if self.validateNode(newNode):
            closestNode.addChild(newNode)
            self.kdt.add(KDTNode(newNode))
            self.nodeList.append(newNode)
            if visFlag:
                self.updateVisualization(closestNode, newNode)

            return True
        else:
            return False

    def validateNode(self, node):
        if self.ogHandle is not None:
            point1 = (node.x, node.y)
            point2 = (node.parentNode.x, node.parentNode.y)
            return not self.ogHandle.obstacleAtEdge(point1, point2)
        else:
            return True

    def kdtNearestNeighbour(self, sampleX, sampleY):
        kdn = self.kdt.search_nn([sampleX, sampleY])
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


def plotRRT(node):
    for child in node.children:
        plt.plot([node.x, child.x], [node.y, child.y], 'k')
        # plt.plot(child.x, child.y, 'ko')
        plotRRT(child)


def plotPath(pathNodeList):
    xCorr = []
    yCorr = []

    for node in pathNodeList:
        xCorr.append(node.x)
        yCorr.append(node.y)

    plt.plot(xCorr, yCorr, 'r')

def visualizeTree(rootNode, goalPose=None, pathNodeList=None, obstaclesData=None, arenaSize=(20, 20)):
    plt.figure()
    plt.plot(rootNode.x, rootNode.y, 'bo')

    if goalPose is not None:
        plt.plot(goalPose[0], goalPose[1], 'ro')

    if obstaclesData is not None:
        plotObstacles(obstaclesData)

    plotRRT(rootNode)

    if pathNodeList is not None:
        plotPath(pathNodeList)

    plt.xlim(-arenaSize[0] / 2, arenaSize[0] / 2)
    plt.ylim(-arenaSize[0] / 2, arenaSize[0] / 2)

def runRRT(start=(0.0, 0.0), goal=(-7.5, -8.0), stepLength=0.5, goalBiasFactor=5, arenaSize=(20, 20)):
    rrtHandle = RRT(startPose=start, goalPose=goal, stepLength=stepLength, arenaSize=arenaSize, goalBiasFactor=goalBiasFactor)
    ogHandle = OccupancyGrid(arenaSize=arenaSize, customFilePath="RRTApp/data/obstacle.data")

    rrtHandle.setOccupancyGrid(ogHandle)
    rrtHandle.generateRRT(visualizeFlag=False)
    path = rrtHandle.getPath()

    print('Node count in RRT:', len(rrtHandle.nodeList))
    visualizeTree(rrtHandle.root, goal, path, ogHandle.obstaclesData, arenaSize)
    plt.savefig("RRTApp/static/rrt.png")


if __name__ == '__main__':
    runRRT()