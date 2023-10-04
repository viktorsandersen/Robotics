import sdurw
import sdurw_proximitystrategies
import numpy as np

def getConfigurations(nameGoal, nameTcp, robot, wc, state):
    # Get, make and print name of frames
    robotName = robot.getName()
    nameRobotBase = robotName + "." + "Base"
    nameRobotTcp = robotName + "." + "TCP"

    # Find frames and check for existence
    frameGoal = wc.findFrame(nameGoal)
    frameTcp = wc.findFrame(nameTcp)
    frameRobotBase = wc.findFrame(nameRobotBase)
    frameRobotTcp = wc.findFrame(nameRobotTcp)
    if frameGoal == None or frameTcp == None or frameRobotBase == None or frameRobotTcp == None:
        print(" ALL FRAMES NOT FOUND:")
        print(" Found \"", nameGoal, "\": ", "NO!" if frameGoal == None else "YES!")
        print(" Found \"", nameTcp, "\": ", "NO!" if frameTcp == None else "YES!")
        print(" Found \"", nameRobotBase, "\": ", "NO!" if frameRobotBase == None else "YES!")
        print(" Found \"", nameRobotTcp, "\": ", "NO!" if frameRobotTcp == None else "YES!")

    # Make "helper" transformations
    frameBaseTGoal = sdurw.Kinematics.frameTframe(frameRobotBase, frameGoal, state)
    frameTcpTRobotTcp = sdurw.Kinematics.frameTframe(frameTcp, frameRobotTcp, state)

    # get grasp frame in robot tool frame
    targetAt = frameBaseTGoal * frameTcpTRobotTcp
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot, state)
    return closedFormSovler.solve(targetAt, state)



if __name__ == "__main__":
    #load workcell

    # find relevant frames
    cylinderFrame = wc.findMovableFrame("Cylinder")
    if cylinderFrame == None:
        raise Exception("COULD not find movable frame Cylinder ... check model")

    # find UR device
    robotUR5 = wc.findSerialDevice("UR5")
    if robotUR5.isNull():
        raise Exception("COULD not find device UR5 ... check model")

    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())


    # get the default state
    state = wc.getDefaultState()
    collisionFreeSolutions = []

    # sample the solutions in a loop(360 deg around rollAngle) with resolution of 1 deg.
    # Use the function MoveTo to move the manipulator,
    # record the state


    solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state)

        # loop and check the solutions if there is a colision
            if not detector.inCollision(state, res1):
                collisionFreeSolutions.append(solutions[i]) # save it
                break # we only need one


    # print the number of colision free solutions


    # visualize them
    tStatePath = sdurw.PathTimedState()
    time=0
    for i in range(len(collisionFreeSolutions)):
        robotUR5.setQ(collisionFreeSolutions[i], state)
        tStatePath.push_back(sdurw.TimedState(time,state))
        time+=0.01

    sdurw.PathLoader.storeTimedStatePath(wc.deref(),tStatePath,"./scene/visu.rwplay")


