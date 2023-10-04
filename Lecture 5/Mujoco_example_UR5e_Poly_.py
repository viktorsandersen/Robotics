
import time
from threading import Thread, Lock
import mujoco
import mujoco.viewer
import numpy as np
import roboticstoolbox as rtb
import time
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

class Test:

  def __init__(self):
    self.m = mujoco.MjModel.from_xml_path('C:/Users/vikto/OneDrive/Desktop/Robotteknologi/7. Semester noter/Robotics/Lecture 3/Example_code/Example_code/Ur5_robot/scene.xml')
    self.d = mujoco.MjData(self.m)
    self.joints = [0,0,0,0,0,0,0]
    self.q0=[0 , -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2,0]
    self.dt = 1/100

    self.Jpos = [
    [-1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 1.5707963267948966, 0],
    [-0.78539816, -1.1693706 ,  1.25663706, -1.11701072, -1.88495559, 0],
    [-0.08726646, -1.90240888,  1.95476876, -0.97738438, -0.54105207,0],
    [1.91986218, -1.01229097,  1.72787596, -2.58308729, -1.65806279,0],
    ]

    # Universal Robot UR5e kiematics parameters 

    self.robot = rtb.DHRobot(
        [ 
            rtb.RevoluteDH(d=0.1625, alpha = np.pi/2),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a=-0.3922),
            rtb.RevoluteDH(d=0.1333, alpha=np.pi/2),
            rtb.RevoluteDH(d=0.0997, alpha=-np.pi/2),
            rtb.RevoluteDH(d=0.0996)
        ], name="UR5e"
                    )
    
  def getState(self):
    ## State of the simulater robot 
    qState=[]    
    for i in range(0, 6):
      qState.append(float(self.d.joint(f"joint{i+1}").qpos)) 
    return qState


  def getEEState(self):
    return self.d.sensordata
    
  def launch_mujoco(self):
  

    with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
      # Close the viewer automatically after 30 wall-seconds.
      start = time.time()
      while viewer.is_running():
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        # with self.jointLock:
        mujoco.mj_step(self.m, self.d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
          viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.d.time % 2)

        with self.jointLock:
          if self.sendPositions:
            for i in range(0, 6):
              self.d.actuator(f"actuator{i+1}").ctrl = self.joints[i]
              # self.d.joint(f"joint{i+1}").qpos = self.joints[i]
            self.sendPositions = False

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)
  
  def sendJoint(self,join_values):
    with self.jointLock:
      for i in range(0, 6):
        self.joints[i] = join_values[i]
      self.sendPositions = True

  def quintic_poly_trj(self, via, t):
    # t number of samples in the trajectory
    Traj = np.array([])
    samp = t/self.dt
    tm = np.linspace(0,t, int(samp))

    for i in range(np.size(via,0)-1):
        q=rtb.mtraj(rtb.quintic,via[i], via[i+1], tm)
        if i == 0:
            Traj = q.q
        else:
            Traj = np.concatenate((Traj,q.q), axis=0)

    return Traj


  def joint_trj_via(self, via, t): #piecewise polynomial
    # t number of samples in the trajectory
    Traj = np.array([])
    samp = t/self.dt
    dq0 = np.ones(6)*0
    dqf = np.ones(6)*0
    dqv = np.ones(6)*1
    tm = np.linspace(0,t, int(samp))

    for i in range(np.size(via,0)-1):
        if i == 0:
            q = rtb.jtraj(via[i], via[i+1], tm, dq0, dqv)
            Traj = q.q
        elif i == np.size(via,0)-1:
            q = rtb.jtraj(via[i], via[i+1], tm, dqv, dqf)
            Traj = np.concatenate((Traj,q.q), axis=0)    
        else:
            q = rtb.jtraj(via[i], via[i+1], tm, dqv, dqv)
            Traj = np.concatenate((Traj,q.q), axis=0)

    return Traj


  def derivatives(self, Traj):
    # calculate the derivatives
    dq = np.array([]) 
    ddq = np.array([]) 
    dddq = np.array([]) 
    dq = np.gradient(Traj)
    ddq= np.gradient(dq[0])
    dddq= np.gradient(ddq[0])
        
    return dq[0], ddq[0], dddq[0]



  def start(self):
    self.jointLock = Lock()
    self.sendPositions = False
    mujoco_thrd = Thread(target=self.launch_mujoco, daemon=True)
    mujoco_thrd.start()
    print("Send robot to init state")
    input()
    # set state of the robot
    self.sendJoint(self.q0)

    input("Press any key generate trajectory")
     # specifie interpolation time for the segments
    t=0.5
    # calculate trajectory Quintiv polynom trajectory using the provided function
    Traj = Test.quintic_poly_trj(self, self.Jpos, t)


    # calculate derivatives of the generated trajectory 
    dq,ddq,dddq = self.derivatives(Traj)

    ## Plot results 
    fig, ax = plt.subplots(4)
    ax[0].plot(Traj)
    ax[0].set_title('Joint positions')
    ax[1].plot(dq)
    ax[1].set_title('Joint velocity')
    ax[2].plot(ddq)
    ax[2].set_title('Joint acceleration')
    ax[3].plot(dddq)
    ax[3].set_title('Joint jerk')
    ## Plot results 


    # send trajectory to the sim robot robot 
    for i in Traj:
      self.sendJoint(i)
      time.sleep(self.dt)
    input('.... Continue ...')


    # Calculate trajectory with mstraj()
    ttq = np.array(self.Jpos)
    dmax= np.ones((1,6))*5
    # call mstrj() function
    rq = rtb.mstraj(ttq, self.dt, 0.1, dmax)


    # calculate derivatives
    dq1,ddq1,dddq1 = self.derivatives(rq.q)
    ## Plot results 
    fig, ax = plt.subplots(4)
    ax[0].plot(Traj)
    ax[0].set_title('Joint positions mstraj - multi-segment multi-axis trajectory')
    ax[1].plot(dq1)
    ax[1].set_title('Joint velocity')
    ax[2].plot(ddq1)
    ax[2].set_title('Joint acceleration')
    ax[3].plot(dddq1)
    ax[3].set_title('Joint jerk')
   

    # send trajectory to the robot 
    for i in rq.q:
      self.sendJoint(i)
      time.sleep(self.dt)
    input('.... Continue ...')

    # calculate the trajectory using jraj() with via points 

    Jtrj = Test.joint_trj_via(self, self.Jpos, t)
    # calculate derivatives
    dq2,ddq2,dddq2 = self.derivatives(Jtrj)
    ## Plot results 
    fig, ax = plt.subplots(4)
    ax[0].plot(Traj)
    ax[0].set_title('Joint positions - joint_trj_via - piecewise polynomial')
    ax[1].plot(dq2)
    ax[1].set_title('Joint velocity')
    ax[2].plot(ddq2)
    ax[2].set_title('Joint acceleration')
    ax[3].plot(dddq2)
    ax[3].set_title('Joint jerk')

     # send trajectory to the robot 
    for i in Jtrj:
      self.sendJoint(i)
      time.sleep(self.dt)
    input('.... Continue ...')


    # Show plots 
    plt.show()
    

if __name__ == "__main__":
  Test().start() 