import sdurw
import numpy as np

# This function is the solution to Programming Exercise 3.4
def forwardKinematics(trefs, idx, q):
    if len(trefs) != q.size():
        raise Exception("The number of local transformations must be equal to the length of the configuration vector")

    # Initialize empty transform to contain the desired transformation
    baseTi = sdurw.Transform3D()

    # Fill baseTi
    for i in range(idx):
        # Calculate Tz as per Equation 3.18
        Tz = sdurw.Transform3D(sdurw.RPY(q[i], 0, 0).toRotation3D())
        # Calculate (i-1)Ti (Equation after 3.21 in the notes)
        Ti = trefs[i]*Tz
        # This line implements Equation 3.21
        baseTi = baseTi*Ti

    return baseTi

# In order to calculate the Jacobian for the inverse kinematics we need the the
# transformation baseTi for all joints in the robot and also the transformation
# baseTtool
def calculateJacobian(tvec, baseTtool):
    # Calculate A and B
    A = []
    B = []
    for i, T in enumerate(tvec):
        # Create a column of the position part of the Jacobian. This line is
        # Equation 4.4 from the Robotics notes
        dummy1 = np.array([T.R().getCol(2).asNumpy()[0, 0], T.R().getCol(2).asNumpy()[1, 0], T.R().getCol(2).asNumpy()[2, 0]])
        dummy2 = np.array([(baseTtool.P() - T.P()).asNumpy()[0, 0], (baseTtool.P() - T.P()).asNumpy()[1, 0], (baseTtool.P() - T.P()).asNumpy()[2, 0]])
        dummy2 = np.cross(dummy1, dummy2)
        A.append(sdurw.Vector3D(dummy2[0], dummy2[1], dummy2[2]))
        print(T)
        # Create a column of the rotation part of the Jacobian. This line is
        # Equation 4.6 from the Robotics notes.
        B.append(T.R().getCol(2))
    print("A", A)
    print("B", B)

    # Fill the Jacobian matrix with A and B
    J = sdurw.Jacobian.zero(6, len(tvec))
    for i in range(len(tvec)):
        # The addPosition function fills column i with the vector A[i] to the position part of the
        # Jacobian starting at row 0 of the position part. Our Jacobian is 6x3 with a 3x3 position part
        # and a 3x3 rotation part. The addRotation functions the same way just with the rotation part
        # of the Jacobian.
        J.addPosition(A[i], 0, i)
        J.addRotation(B[i], 0, i)

    return J

if __name__ == "__main__":
    # Test is example 4.2.1 from the book
    # Initialize configuration
    q = sdurw.Q(3, 0, -np.pi/6, np.pi/6)

    # Create Trefs
    L = 3
    a2 = 2
    a3 = 2
    trefs = []

    # Base -> 1
    v1 = sdurw.Vector3D(0,0,L)
    r1 = sdurw.RPY(0,0,0)
    trefs.append(sdurw.Transform3D(v1, r1.toRotation3D()))

    # 1 -> 2
    v2 = sdurw.Vector3D(0,0,0)
    r2 = sdurw.RPY(0,0,np.pi/2)
    trefs.append(sdurw.Transform3D(v2, r2.toRotation3D()))

    # 2 -> 3
    v3 = sdurw.Vector3D(0,a2,0)
    r3 = sdurw.RPY(0,0,0)
    trefs.append(sdurw.Transform3D(v3, r3.toRotation3D()))

    # Calculate transforms
    # We need all the baseTi's to have access to baseZi and basePi in
    # Equations 4.4 and 4.6 from the notes.
    Tvec = []
    for i in range(len(trefs)):
        Tvec.append(forwardKinematics(trefs, i, q))

    # Fill in the transformation iTtool
    # 3 -> TCP
    v4 = sdurw.Vector3D(a3,0,0)
    r4 = sdurw.RPY(0,0,0)
    Ttool = sdurw.Transform3D(v4, r4.toRotation3D())
    baseTtcp = Tvec[-1]*Ttool

    # Create Jacobian
    J = calculateJacobian(Tvec, baseTtcp)

    print(J)

