import sdurw
import sdurw_proximitystrategies
import sdurw_pathplanners
import sdurw_kinematics
import sdurw_proximity
import sdurw_models
import numpy as np
import time as timer

MAXTIME=60.

def checkCollisions(device, state, detector, q):
    testState = sdurw.State.clone(state) # state.clone()
    data = sdurw.CollisionDetectorQueryResult()
    device.setQ(q, testState)
    colFrom = detector.inCollision(testState, data)
    if (colFrom):
        print("Configuration in collision: ", q)
        return False

    return True

if __name__ == "__main__":


    for extend in np.arange(0.02,1,0.02):
        for trial in range(3):
            wcFile = "./Kr16WallWorkCell/Scene.wc.xml"
            deviceName = "KukaKr16"
            sdurw.Math.seed()

            wc = sdurw.WorkCellLoaderFactory.load(wcFile)
            # Find the tool and bottle frame
            tool_frame = 
            bottle_frame = 

            device =wc.findDevice(deviceName)
            if device == None:
                print("Device: ", deviceName, " not found!")
                exit()

            # Get the state
            state = None

            from_ = sdurw.Q(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573)
            to = sdurw.Q(6,1.571, 0.006, 0.030, 0.153, 0.762, 4.490)
            # Set Q to the initial state and grip the bottle frame with gripFrame
            detector = sdurw_proximity.ownedPtr(sdurw_proximity.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory.makeDefaultCollisionStrategy()))
            constraint = sdurw.PlannerConstraint.make(detector, device, state)
            
            sampler = sdurw.QSampler.makeConstrained(sdurw.QSampler.makeUniform(device), constraint.getQConstraintPtr())
            metric = sdurw.MetricFactory.makeEuclideanQ()
            planner = sdurw_pathplanners.RRTPlanner.makeQToQPlanner(constraint, sampler, metric, extend, sdurw_pathplanners.RRTPlanner.RRTConnect)

            if not checkCollisions(device, state, detector, from_):
                exit()
            if not checkCollisions(device, state, detector, to):
                exit()


            path = sdurw.PathQ()
            start = timer.time()
            # Query the planner to find a trajectory between the configurations
            end = timer.time()
            planning_time = end - start
            distance = 0.


            print("Path of length ", path.size(), " found in ", planning_time, " seconds.")
            if (planning_time >= MAXTIME):
                print("Notice: max time of ", MAXTIME, " seconds reached.")
            # Visualize the movement with an rwplay file

            print(trial)


