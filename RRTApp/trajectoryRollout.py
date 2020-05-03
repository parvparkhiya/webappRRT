import numpy as np
import matplotlib.pyplot as plt

def moveBicycleOnArc(centerPose, steeringAngle, arcAngle = None, arcLength = None, vehicleLength = 0.335):

    if steeringAngle == 0.0:
        if arcLength is None:
            arcLength = 1.0
        newX = centerPose[0] + (np.cos(centerPose[2]) * arcLength)
        newY = centerPose[1] + (np.sin(centerPose[2]) * arcLength)
        return [newX, newY, centerPose[2]]
    else:
        backWheelXY = moveOnLine(centerPose[0:2], centerPose[2] + np.pi, vehicleLength / 2.0)
        newBackWheelPose = moveOnArc([backWheelXY[0], backWheelXY[1], centerPose[2]], steeringAngle, arcAngle, arcLength, vehicleLength)
        newCenterXY = moveOnLine(newBackWheelPose[0:2], newBackWheelPose[2], vehicleLength / 2.0)

        return [newCenterXY[0], newCenterXY[1], newBackWheelPose[2]]

def moveOnArc(pose, steeringAngle, arcAngle = None, arcLength = None, vehicleLength = 0.335):
    arcRadius = vehicleLength/np.tan(abs(steeringAngle))
    if arcAngle is None:
        arcAngle = arcLength/arcRadius

    if steeringAngle > 0.0:
        arcCenterXY= moveOnLine([pose[0], pose[1]], pose[2] + np.pi/2.0, arcRadius)
        newPoseXY = moveOnLine(arcCenterXY, pose[2] - np.pi/2.0 + arcAngle, arcRadius)
        newTheta = pose[2] + arcAngle
    else:
        arcCenterXY= moveOnLine([pose[0], pose[1]], pose[2] - np.pi/2.0, arcRadius)
        newPoseXY = moveOnLine(arcCenterXY, pose[2] + np.pi/2.0 - arcAngle, arcRadius)
        newTheta = pose[2] - arcAngle
    newPose = [newPoseXY[0], newPoseXY[1], newTheta]

    return newPose

def moveOnLine(pose, theta, length):
    x = pose[0] + (length * np.cos(theta))
    y = pose[1] + (length * np.sin(theta))
    return [x, y]

if __name__ == "__main__":
    vehiclePose = np.array((1.0, -3.2, 0))
    steeringAngle = np.deg2rad(25)
    arcAngleList = np.linspace(0, 2*np.pi, 100)
    poseList = []
    for arcAngle in arcAngleList:
        poseList.append(moveBicycleOnArc(vehiclePose, steeringAngle, arcAngle))

    steeringAngle = np.deg2rad(-15)

    for arcAngle in arcAngleList:
        poseList.append(moveBicycleOnArc(vehiclePose, steeringAngle, arcAngle))

    poses = np.asarray(poseList)
    plt.plot(poses[:, 0], poses[:, 1])
    plt.show()
