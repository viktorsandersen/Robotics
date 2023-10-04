import sdurw
import numpy as np


def forwardKinematics(trefs, idx, q):
    if len(trefs) != q.size():
        raise ("The number of local transformations must be equal to the length of the configuration vector")

    baseTi = sdurw.Transform3D()

    for i in range(idx):
        Tz = sdurw.Transform3D(sdurw.RPY(q[i], 0, 0).toRotation3D())
        baseTi = baseTi * trefs[i] * Tz

    return baseTi


def generateTrefs():
    # Create Tref vector
    # Joint 0: < RPY > 0 0 0 < / RPY > < Pos > 0 0 0 < / Pos >
    V0 = sdurw.Vector3D(0, 0, 0)
    R0 = sdurw.RPY(0, 0, 0)
    T0 = sdurw.Transform3D(V0, R0.toRotation3D())

    # Joint 1: < RPY > 90 0 90 < / RPY > < Pos > 0 0 0.08920 < / Pos >
    V1 = sdurw.Vector3D(0, 0, 0.08920)
    R1 = sdurw.RPY(np.deg2rad(90), 0, np.deg2rad(90))
    T1 = sdurw.Transform3D(V1, R1.toRotation3D())

    # Joint 2: < RPY > 270 0 0 < / RPY > < Pos > 0 0.425 0 < / Pos >
    V2 = sdurw.Vector3D(0, 0.425, 0)
    R2 = sdurw.RPY(np.deg2rad(270), 0, 0)
    T2 = sdurw.Transform3D(V2, R2.toRotation3D())

    # Joint 3: < RPY > 0 0 0 < / RPY > < Pos > -0.39243 0 0 < / Pos >
    V3 = sdurw.Vector3D(-0.39243, 0, 0)
    R3 = sdurw.RPY(0, 0, 0)
    T3 = sdurw.Transform3D(V3, R3.toRotation3D())

    # Joint 4: < RPY > 270 0 90 < / RPY > < Pos > 0 0 0.109 < / Pos >
    V4 = sdurw.Vector3D(0, 0, 0.109)
    R4 = sdurw.RPY(np.deg2rad(270), 0, np.deg2rad(90))
    T4 = sdurw.Transform3D(V4, R4.toRotation3D())

    # Joint 5: < RPY > 0 0 270 < / RPY > < Pos > 0 0 0.093 < / Pos >
    V5 = sdurw.Vector3D(0, 0, 0.093)
    R5 = sdurw.RPY(0, 0, np.deg2rad(270))
    T5 = sdurw.Transform3D(V5, R5.toRotation3D())

    return [T0, T1, T2, T3, T4, T5]


if __name__ == "__main__":
    # Manual version
    # Create Tref vector
    Trefs = generateTrefs()

    # The transformation from Joint 5 to the TCP
    # TCP: < RPY > 270 0 0 < / RPY > < Pos > 0 0 0.082 < / Pos >
    VTCP = sdurw.Vector3D(0, 0, 0.082)
    RTCP = sdurw.RPY(np.deg2rad(270), 0, 0)
    TTCP = sdurw.Transform3D(VTCP, RTCP.toRotation3D())

    # The configuration to calculate the forward dynamics for
    q = sdurw.Q(6, 0.859, 0.208, -0.825, -0.746, -1.632, 1.527)
    baseTtcp_manual = forwardKinematics(Trefs, 6, q) * TTCP

    print("Manual: \n", baseTtcp_manual.P(), " ", sdurw.RPY(baseTtcp_manual.R()))

    # RobWork version using the result from Programming Exercise 3.3
    # Load RW workcell and compare
    # Load workcell and get device
    workcell_path = "/home/jeppe/phd_jeppe_langaa/teaching/rovi/introduction_to_exercises/lab3/UR5WorkCellFinal/Scene.wc.xml"
    device_name = "UR-6-85-5-A"

    wc = sdurw.WorkCellLoaderFactory.load(workcell_path)
    if wc.isNull():
        print("WorkCell is not found")
        exit()
        
    device = wc.findDevice(device_name)
    if device.isNull():
        print("Device " + device_name + " was not found!")


    # Set robot state, q
    state = wc.getDefaultState()
    device.setQ(q, state)

    # Compute baseTtcp
    tcp_frame = wc.findFrame(device_name + ".TCP")
    if tcp_frame == None:
        print("TCP frame not found!")

    baseTtool_rw = device.baseTframe(tcp_frame, state)

    print("RW:\n", baseTtool_rw.P(), " ", sdurw.RPY(baseTtool_rw.R()))
