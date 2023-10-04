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
    wc = sdurw.WorkCellLoaderFactory.load("./scene/Scene.wc.xml")
    if wc.isNull():
        raise Exception("COULD NOT LOAD scene... check path!")

    # find relevant frames
    cylinderFrame = wc.findMovableFrame("Cylinder")
    if cylinderFrame == None:
        raise Exception("COULD not find movable frame Cylinder ... check model")

    robotUR5 = wc.findSerialDevice("UR5")
    if robotUR5.isNull():
        raise Exception("COULD not find device UR5 ... check model")

    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())


    # get the default state
    state = wc.getDefaultState()
    collisionFreeSolutions = []

    for rollAngle in range(360): # for every degree around the roll axis

        cylinderFrame.setTransform(sdurw.Transform3D(cylinderFrame.getTransform(state).P(), sdurw.RPY(np.deg2rad(rollAngle),0,0)), state)


        solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state)

        for i in range(len(solutions)):
            # set the robot in that configuration and check if it is in collision
            robotUR5.setQ(solutions[i], state)
            res1 = sdurw.ProximityData()
            if not detector.inCollision(state, res1):
                collisionFreeSolutions.append(solutions[i]) # save it
                break # we only need one


    print("Current position of the robot vs object to be grasped has: ", len(collisionFreeSolutions), " collision-free inverse kinematics solutions!")


    # visualize them
    tStatePath = sdurw.PathTimedState()
    time=0
    for i in range(len(collisionFreeSolutions)):
        robotUR5.setQ(collisionFreeSolutions[i], state)
        tStatePath.push_back(sdurw.TimedState(time,state))
        time+=0.01

    sdurw.PathLoader.storeTimedStatePath(wc.deref(),tStatePath,"./scene/visu.rwplay")


