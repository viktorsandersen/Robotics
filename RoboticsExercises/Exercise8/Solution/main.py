import sdurw
import sdurw_kinematics
import sdurw_math
import sdurw_proximitystrategies
from Tree import Tree, Node
import numpy as np
from transform3d import Transform

def compute_task_error(q_s, q_near):

    #(C, Tt0) ← RETRIEVE CONSTRAINT(qs, qnear);
    C = np.zeros((6,6))
    C[3, 3] = 1
    C[4, 4] = 1
    C[5, 5] = 1
    
    #0Te ← FORWARD KINEMATICS(qs);
    base_frame = wc.findFrame("UR-6-85-5-A.Base")
    bottle_frame = wc.findFrame("Bottle")
    device.setQ(sdurw.Q(6, q_s[0], q_s[1], q_s[2], q_s[3], q_s[4], q_s[5]), state)

    #tTe ← tT0 0Te;
    base_t_task = Transform(rpy=(np.pi/2, 0, -np.pi/2))
    base_t_bottle = sdurw_kinematics.Kinematics_frameTframe(base_frame, bottle_frame, state)
    base_t_bottle = transform_to_numpy(base_t_bottle)
    base_t_bottle = Transform(matrix=base_t_bottle)
    task_t_bottle = base_t_task.inv @ base_t_bottle

    #∆x ← TASK COORDINATES(Tt);
    dx = task_t_bottle
    dx = np.concatenate([dx.p, dx.rpy])

    # ∆xerr ← C∆x
    dx_err = C @ dx

    # return ∆xerr ;
    return np.linalg.norm(dx_err)


def rgd_new_config(q_s, q_near, eps=2e-3):
    # i ← 0; j ← 0;
    i = 0
    I = 1000
    j = 0
    J = 100

    #∆xerr ← COMPUTE TASK ERROR(qs, qnear );
    dx_err = compute_task_error(q_s, q_near)

    #while i < I and j < J and |∆xerr | > eps
    while i < I and j < J and np.linalg.norm(dx_err) > eps:

        #do i ← i + 1; j ← j + 1;
        i = i + 1
        j = j + 1

        # q`s = qs + RANDOM DISPLACEMENT(dmax);
        q_s_ = q_s + np.random.uniform(low=-0.0005, high=0.0005, size=6)

        #∆x′err ← COMPUTE TASK ERROR(qs, qnear );
        dx_err_ = compute_task_error(q_s_, q_near)

        #if ∆xerr < ∆xerr
        if dx_err_ < dx_err:

            #then j ← 0; qs = qs; ∆xerr = ∆xerr 
            j = 0
            q_s[:] = q_s_
            dx_err = dx_err_

        # if ∆xerr ≤ eps
        if dx_err <= eps:

            # then if IN COLLISION(qs)
            device.setQ(sdurw.Q(6, q_s[0], q_s[1], q_s[2], q_s[3], q_s[4], q_s[5]), state)
            res1 = sdurw.ProximityData()
            if detector.inCollision(state, res1):
                # then return false;
                return False
            else:
                #else return true;
                return True
    #return false
    return False


def task_constrained_rrt(q_init, q_goal, dt):

    #  T .init(qinit);
    T = Tree(q_init)

    A = 1e6
    joint_max = np.deg2rad(np.array([180, 90, 180, 90, 180, 180]))
    joint_min = np.deg2rad(np.array([-180, -270, -180, -270, -180, -180]))

    #  for a = 1 to A
    for a in range(int(A)):
        # do qrand ← RANDOM CONFIG;
        q_rand = Node(np.random.uniform(low=joint_min, high=joint_max))

        #qnear ← NEAREST NEIGHBOR(qrand, T );
        q_near, dist = T.find_nearest_neighbor(q_rand)

        #qdir ← (qrand − qnear )/|qrand − qnear |;
        q_dir = (q_rand.q_numpy - q_near.q_numpy) / np.linalg.norm(q_rand.q_numpy - q_near.q_numpy)

        #qs = qnear + qdir ∆t;
        q_s = q_near.q_numpy + q_dir * dt

        # if *CONSTRAINED* NEW CONFIG(qs, qnear )
        if rgd_new_config(q_s, q_near):

            #then T . add vertex (qs);
            q_s = Node(q_s)
            T.add_vertex(q_s)

            #T . add edge (qnear , qs);
            T.add_edge(q_near, q_s)

            # Make it exit early when the solution is close enough
            dist_t_goal = np.linalg.norm(q_s.q_numpy - q_goal.q_numpy)
            if dist_t_goal < dt:
                T.add_vertex(q_goal)
                T.add_edge(q_s, q_goal)
                print("Found a route")
                break
            else:
                # Progress Info
                print("Distance to goal:", dist_t_goal, q_s.q)

    return T

def transform_to_numpy(transform):
    t = np.eye(4)
    t[0, 0] = transform[0, 0]
    t[0, 1] = transform[0, 1]
    t[0, 2] = transform[0, 2]
    t[0, 3] = transform[0, 3]
    t[1, 0] = transform[1, 0]
    t[1, 1] = transform[1, 1]
    t[1, 2] = transform[1, 2]
    t[1, 3] = transform[1, 3]
    t[2, 0] = transform[2, 0]
    t[2, 1] = transform[2, 1]
    t[2, 2] = transform[2, 2]
    t[2, 3] = transform[2, 3]
    return t


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
    frameBaseTGoal = sdurw_kinematics.Kinematics_frameTframe(frameRobotBase, frameGoal, state)
    frameTcpTRobotTcp = sdurw_kinematics.Kinematics_frameTframe(frameTcp, frameRobotTcp, state)

    # get grasp frame in robot tool frame
    targetAt = frameBaseTGoal * frameTcpTRobotTcp
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot.cptr(), state)
    return closedFormSovler.solve(targetAt, state)


if __name__ == "__main__":
    wcFile = "/home/kalor/Workspace/roviexercises/RoboticsExercises/Exercise8/Solution/WorkCellObstacle/Scene.wc.xml"
    deviceName = "UR-6-85-5-A"
    sdurw.Math.seed()

    wc = sdurw.WorkCellLoaderFactory.load(wcFile)
    tool_frame = wc.findFrame("GraspTCP")
    bottle_frame = wc.findFrame("Bottle")

    device = wc.findSerialDevice(deviceName)
    if device == None:
        print("Device: ", deviceName, " not found!")
        exit()

    state = wc.getDefaultState()

    solutions = getConfigurations("Bottle", "GraspTCP", device, wc, state)
    collisionFreeSolution = None
    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())

    for i in range(len(solutions)):
        # set the robot in that configuration and check if it is in collision
        device.setQ(solutions[i], state)
        res1 = sdurw.ProximityData()
        if not detector.inCollision(state, res1):
            collisionFreeSolution = solutions[i]  # save it
            break  # we only need one
    from_ = collisionFreeSolution
    print("From:", from_)
    device.setQ(from_, state)
    sdurw_kinematics.Kinematics_gripFrame(bottle_frame, tool_frame, state)

    solutions = getConfigurations("BottleGoal", "Bottle", device, wc, state)
    for i in range(len(solutions)):
        # set the robot in that configuration and check if it is in collision
        device.setQ(solutions[i], state)
        res1 = sdurw.ProximityData()
        if not detector.inCollision(state, res1):
            collisionFreeSolution = solutions[i]  # save it
            break  # we only need one
    to = collisionFreeSolution
    print("To:", to)

    from_ = Node(from_)
    to = Node(to)

    qs= sdurw_math.Q( 1.31400662e+00, -3.73399115e+00,  1.69481309e+00, -4.21934625e+00, 2.86882643e+00,  3.07075477e-03);
    qnear= sdurw_math.Q(1.32439, -3.74499, 1.7031, -4.24889, 2.89531, -0.00768951);

    qnear = Node(qnear);

    compute_task_error(qs,qnear);
    exit()

    tree = task_constrained_rrt(from_, to, 0.05)

    tree.get_route(to)
    # visualize them
    tStatePath = sdurw.PathTimedState()
    time = 0
    for i in tree.goal_route:
        device.setQ(i.q, state)
        tStatePath.push_back(sdurw.TimedState(time, state))
        time += 0.01

    tStatePath.save("./visu.rwplay", wc)





