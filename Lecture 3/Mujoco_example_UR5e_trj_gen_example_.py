
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
    self.m = mujoco.MjModel.from_xml_path('Ur5_robot/scene.xml')
    self.d = mujoco.MjData(self.m)
    self.joints = [0,0,0,0,0,0,0]
    self.q0=[0 , -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2,0]
    self.dt = 1/100

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

  def start(self):
    self.jointLock = Lock()
    self.sendPositions = False
    mujoco_thrd = Thread(target=self.launch_mujoco, daemon=True)
    mujoco_thrd.start()
    print("Send robot to init state")
    input()
    # set state of the robot
    self.sendJoint(self.q0)

    print("Press any key generate trajectory")
    input()
    # read current robot configuration
    qn =self.getState()

    # conctruct the carthesian trajectories 

    # calculate joint configurations using IK

    qik = self.robot.ikine_LM(Trj, q0=self.getState())
    # caclualte velocities
    Tq = np.gradient(qik.q)

    ## plot resulting  position trajectory
    plt.figure(1)
 
    ## plot resulting velocity trajectory

    ## send trajectory to simulated robot 

    for i in qik.q:
      self.sendJoint(i)
      time.sleep(self.dt)

    ## generate joint space trjaectories

    Cur= self.getState()

    print("Modify the the configuration of the manipulator with the sliders and plan to the new configuration")
    input()
    
    dest = self.getState()
    
    #calculate a joint rajectory based on the current state of the manipulator and the user modified state of the manipulator in 100 samples
    qtrj = 

    plt.figure(3)
    # plot interpolated Joint poses
    plt.plot(qtrj.qd)
    plt.title('joint velocity')

    plt.figure(4)
    plt.title('joint positions')
    # # plot interpolated joint velocities
    plt.plot(qtrj.q)

    ## send robot to init state 
    self.sendJoint(Cur)
    print("moving to init position")
    time.sleep(self.dt)

    for k in qtrj.q:
      self.sendJoint(k)
      time.sleep(self.dt)

    # send trajectory to robot sim

    plt.show()
    

if __name__ == "__main__":
  Test().start() 