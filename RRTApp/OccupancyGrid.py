#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import scipy.ndimage as ndimage

class OccupancyGrid:
    def __init__(self, arenaSize=(20, 20), resolution=10, dilationFactor=3, customFilePath=None):
        self.arenaSize = (arenaSize[0] + 5, arenaSize[1] + 5)
        self.resolution = resolution
        self.dilationFactor = dilationFactor
        self.customFilePath = customFilePath

        self.obstaclesData = self.getObstaclesData()
        self.grid = self.createGrid(self.obstaclesData)

    def obstacleAtEdge(self, startPoint, endPoint, subSampling=None):
        if subSampling is None:
            subSampling = self.resolution

        tStartPoint = self.transformPoint(startPoint)
        tEndPoint = self.transformPoint(endPoint)
        points = self.getLinearInterpolationPoints(tStartPoint, tEndPoint, subSampling)

        for i in range(points[0].shape[0]):
            tx = points[0][i]
            ty = points[1][i]
            if self.grid[np.round(tx).astype('int'), np.round(ty).astype('int')]:
                return True

        return False

    def obstacleAtPoint(self, point):
        tPoint = self.transformPoint(point)
        tPoint = np.round(tPoint).astype('int')
        return self.grid[tPoint[0], tPoint[1]]

    def createGrid(self, obstacleData):
        grid = np.zeros((self.arenaSize[1]*self.resolution, self.arenaSize[0]*self.resolution), dtype=np.bool)
        for i in range(self.obstaclesData.shape[0]):
            tx, ty = obstacleData[i, 1], obstacleData[i, 2]
            sx, sy = obstacleData[i, 3], obstacleData[i, 4]
            theta = obstacleData[i, 5]

            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            p1 = np.matmul(np.array([-sx/2.0, sy/2.0]), R.T) + np.array([tx, ty])
            p2 = np.matmul(np.array([sx/2.0, sy/2.0]), R.T) + np.array([tx, ty])
            p3 = np.matmul(np.array([sx/2.0, -sy/2.0]), R.T) + np.array([tx, ty])
            p4 = np.matmul(np.array([-sx/2.0, -sy/2.0]), R.T) + np.array([tx, ty])
            p1 = self.transformPoint(p1)
            p2 = self.transformPoint(p2)
            p3 = self.transformPoint(p3)
            p4 = self.transformPoint(p4)

            points = self.getPointsInsideRectangle(p1, p2, p3, p4, self.resolution*5)
            grid[points[0], points[1]] = np.ones(points[0].shape[0], dtype=np.bool)

        gridDilated = ndimage.binary_dilation(grid, iterations=self.dilationFactor)
        return gridDilated

    def getPointsInsideRectangle(self, p1, p2, p3, p4, subSamplePoints):
        startPointList = self.getLinearInterpolationPoints(p1, p4, subSamplePoints)
        endPointList = self.getLinearInterpolationPoints(p2, p3, subSamplePoints)

        points = [np.array(()).astype('int'), np.array(()).astype('int')]

        for i in range(startPointList[0].shape[0]):
            startPoint = (startPointList[0][i], startPointList[1][i])
            endPoint = (endPointList[0][i], endPointList[1][i])
            newPoints = self.getLinearInterpolationPoints(startPoint, endPoint, subSamplePoints)
            points[0] = np.hstack((points[0], (np.round(newPoints[0]).astype('int'))))
            points[1] = np.hstack((points[1], (np.round(newPoints[1]).astype('int'))))

        return points

    @staticmethod
    def getLinearInterpolationPoints(startPoint, endPoint, numPoints):
        xCorr = np.linspace(startPoint[0], endPoint[0], numPoints)
        yCorr = np.linspace(startPoint[1], endPoint[1], numPoints)

        return [xCorr, yCorr]

    def transformPoint(self, point):
        xCorr = ((self.arenaSize[1]/2.0) - point[1]) * self.resolution
        yCorr = ((self.arenaSize[0]/2.0) + point[0]) * self.resolution

        return np.array([xCorr, yCorr])

    def getObstaclesFilePath(self):
        return self.customFilePath

    def getObstaclesData(self):
        obstaclesFile = open(self.getObstaclesFilePath(), "r")
        obstaclesData = obstaclesFile.read()
        obstaclesData = np.asarray(obstaclesData.split(), 'float')
        obstaclesData = np.resize(obstaclesData, ((int)(obstaclesData.shape[0]/6), 6))
        return obstaclesData

def plotObstacles(obstacleData):

    for i in range(obstacleData.shape[0]):
        tx, ty = obstacleData[i, 1], obstacleData[i, 2]
        sx, sy = obstacleData[i, 3], obstacleData[i, 4]
        theta = obstacleData[i, 5]

        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        p = np.matmul(np.array([-sx / 2.0, -sy / 2.0]), R.T)
        rect = patches.Rectangle((p[0] + tx, p[1] + ty), sx, sy, angle=np.rad2deg(theta))
        plt.gca().add_patch(rect)

if __name__ == '__main__':
    # Obstacle data file format: id x y sizeX sizeY theta
    ogHandle = OccupancyGrid()
    point = (-7.0, -7.0)
    print("Obstacle At:",point, ":", ogHandle.obstacleAtPoint(point))