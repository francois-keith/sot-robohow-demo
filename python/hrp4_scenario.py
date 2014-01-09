#import roslib
#import rospy
import numpy
from math import radians

#from dynamic_graph.sot.core.meta_tasks_kine import *
# Create the robot romeo.
from dynamic_graph.sot.hrp4.sot_hrp4_controller import *
from dynamic_graph.sot.hrp4.prologue import *

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *

# Create a simple kinematic solver.
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize

from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_generic import MetaTaskGeneric

################################################################################
# Create the FeatureExpressionGraph and the corresponding task
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.expression_graph.gripper import Gripper



# --- PG ---------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix, Inverse_of_matrixHomo, Multiply_of_matrixHomo, Stack_of_vector, PoseRollPitchYawToMatrixHomo, MatrixHomoToPoseRollPitchYaw, Multiply_matrixHomo_vector
from dynamic_graph.sot.dynamics import Dynamic
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector
from dynamic_graph.sot.core.matrix_util import matrixToTuple
# from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d, Task, TaskPD
from dynamic_graph.sot.core import FeaturePosture



from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.core import feature_vector3
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.utils.attime import attime,ALWAYS,refset,sigset
from numpy import *



from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
#headMcam=array([[0.0,0.0,1.0,0.081],[1.,0.0,0.0,0.072],[0.0,1.,0.0,0.031],[0.0,0.0,0.0,1.0]])
headMcam=array([[0.0,0.0,1.0,0],[1.,0.0,0.0,0],[0.0,1.,0.0,0],[0.0,0.0,0.0,1.0]])
# headMcam = dot(headMcam,rotate('x',10*pi/180))


# --- FOV ---
taskFoV = MetaTaskVisualPoint('FoV',robot.dynamic,'head','gaze')
taskFoV.opmodif = matrixToTuple(headMcam)

taskFoV.task=TaskInequality('taskFoVineq')
taskFoV.task.add(taskFoV.feature.name)
[Xmax,Ymax]=[0.38,0.28]
taskFoV.task.referenceInf.value = (-Xmax,-Ymax)    # Xmin, Ymin
taskFoV.task.referenceSup.value = (Xmax,Ymax)  # Xmax, Ymax
taskFoV.task.dt.value=5e-3
taskFoV.task.controlGain.value=0.01
taskFoV.featureDes.xy.value = (0,0)
rightWristPosition = MatrixHomoToPose('rightWristPosition')
plug(robot.frames['rightGripper'].position, rightWristPosition.sin)

# taskFoV.goto3D((0,-1.1,0.9))
#rightWristPosition.sout.recompute(1)
#taskFoV.goto3D(rightWristPosition.sout.value)
plug(rightWristPosition.sout, taskFoV.target)



class Scenario:
  solver = None
  angleBottleZ = None
  taskAngleBottle = None
  r_gripper_angle = None
  taskGripperXY =None
  taskGripperZ =None
  taskPlanBottleY =None
  taskTips = None
  stepIndex = 0
	featureDict = {}

  def __init__(self, robot):
    ros = Ros(robot)
    solver = initialize ( robot, SolverKine )
    self.solver = solver
    self.stepIndex = 0


    # --- TASK POSTURE --------------------------------------------------
    # set a default position for the joints.
    robot.features['featurePosition'] = FeaturePosture('featurePosition')
    plug(robot.device.state,robot.features['featurePosition'].state)
    robotDim = len(robot.dynamic.velocity.value)
    robot.features['featurePosition'].posture.value = robot.halfSitting

    if robot.device.name == 'HRP2LAAS' or \
       robot.device.name == 'HRP2JRL':
      postureTaskDofs = [ False,False,False,False,False,False, \
                          False,False,False,False,False,False, \
                          True,True,True,True, \
                          True,True,True,True,True,True,True, \
                          True,True,True,True,True,True,True ]
    elif robot.device.name == 'HRP4LIRMM':
      # Right Leg, Left leg, chest, right arm, left arm
      postureTaskDofs = [False]*6 +  [False]*6 + [True]*4 + [False]*9 + [True]*9
    elif robot.device.name == 'ROMEO':
      # chest, left/right arms, left/right legs
      postureTaskDofs = [True]*5 + [True]*7 + [True]*7 + [False]*7 + [False]*7
    else:
      print "/!\\ walking.py: The robot " +robot.device.name+ " is unknown."
      print "  Default posture task froze all the dofs"
      postureTaskDofs=[True] * (robot.dimension-6)

    for dof,isEnabled in enumerate(postureTaskDofs):
      robot.features['featurePosition'].selectDof(dof+6,isEnabled)

    robot.tasks['robot_task_position']=Task('robot_task_position')
    robot.tasks['robot_task_position'].add('featurePosition')
    # featurePosition.selec.value = toFlags((6,24))


    gainPosition = GainAdaptive('gainPosition')
    gainPosition.set(0.1,0.1,125e3)
    gainPosition.gain.value = 5
    plug(robot.tasks['robot_task_position'].error,gainPosition.error)
    plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)


    # --- CONTACTS
    # define contactLF and contactRF
    for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
        contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
        contact.feature.frame('desired')
        contact.gain.setConstant(10)
        contact.keep()
        locals()['contact'+name] = contact
    # ---- TASKS -------------------------------------------------------------------

#    self.solver.push(robot.tasks['robot_task_position'])

    # operational point used
    ground_z    = VersorElement('ground_z', robot, 'ground', versor = (0,0,1))
    r_gripper_x = VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) )

    (taskAngleBottle, angleBottleZ) = createTaskAndFeature('bottleZ', r_gripper_x, ground_z, 'angle')
    angleBottleZ.reference.value = radians(90)

    # # # # # # # 
    ground_plane = PlaneElement('ground_plane', robot, 'ground', normal = (0,0,1))
    r_gripper_y  = VersorElement('r_gripper_y', robot, 'rightGripper', versor = (0,1,0) )

    (taskPlanBottleY, planBottleY) = createTaskAndFeature('bottleY', ground_plane, r_gripper_y, 'angle')
    planBottleY.reference.value = 0


    ################################ #######################""
    ## position right hand above target
    heightZ       = PointElement('heightZ', robot, 'ground', position = (0,0,1))
    r_gripperZpos = PointElement('r_gripperZpos', robot, 'rightGripper')

    (taskGripperZ, positionZ) =   createTaskAndFeaturePointToPoint('positionZ', r_gripperZpos, heightZ)

    positionZ.selec.value ='100'
    positionZ.reference.value = (0,)


    #######################################################""
    ## position leftHand op above right hand
    ## -pi/8 << dot(bottle_normal, World_Z_axis) << pi/8

    posXY       = PointElement('posXY',       robot, 'ground')
    r_gripperXY = PointElement('r_gripperXY', robot, 'rightGripper')

    (taskGripperXY, positionXY) = createTaskAndFeaturePointToPoint('positionXY', r_gripperXY, posXY, False)

    positionXY.selec.value = '011'
    positionXY.reference.value = (-0.3,0.1)
    taskGripperXY.referenceInf.value = (-0.05,-0.05)
    taskGripperXY.referenceSup.value = ( 0.05, 0.05)


    # define a task for the orientation of the fingertips : parallel to the handle
    # line / line constraint
    #tips = FeatureVersorToVersor('tips')

    ground_x       = VersorElement('ground_x', robot, 'ground', versor = (1,0,0))
    (taskTips, tips) = createTaskAndFeature('tips', ground_x, r_gripper_y, 'angle')
    tips.reference.value = 2.5


    r_gripper_angle = Gripper('r_gripper_angle', robot, 29, 2)


#    taskComIneq.task = TaskInequality('taskComIneq')
#    taskComIneq.task.add(robot.features[].feature.name)
#    taskComIneq.task.referenceInf.value = (-10,)    # Xmin, Ymin
#    taskComIneq.task.referenceSup.value = (0.20,)    # Xmin, Ymin
#    taskComIneq.task.dt.value=dt
#    taskComIneq.task.controlGain.value = 0.9



    ## TODO ...
    self.angleBottleZ = angleBottleZ
    self.taskAngleBottle = taskAngleBottle
    self.r_gripper_angle = r_gripper_angle
    self.taskGripperXY =taskGripperXY
    self.taskGripperZ =taskGripperZ
    self.taskPlanBottleY =taskPlanBottleY
    self.taskTips =taskTips
    robot.featureComDes.errorIN.value = (0.05, 0.0, 0.8)

  ## sequencing...
  def pour(self, angle=45):
    self.angleBottleZ.reference.value = radians(angle)


  # graps
  def _step1(self):
    self.solver.push(self.taskAngleBottle)
    self.solver.push(self.r_gripper_angle.task)
    self.r_gripper_angle.featureDes.errorIN.value = (1,0)

  # close the gripper
  def _step2(self):
    self.r_gripper_angle.close()
		#todo: estimate the position of the bottle neck

  # bent the bottle a little
  def _step2a(self):
    print " " # self.pour(80)
 
  # go above the glass.
  def _step3(self):
    print "Start pouring"
    self.solver.push(self.taskGripperXY)
    self.solver.push(self.taskGripperZ)
    self.solver.push(self.taskPlanBottleY)
    self.solver.push(self.taskTips)
    s.solver.push(taskFoV.task)

  # pour a little
  def _step4(self):
    self.pour()

  # pour ...
  def _step5(self):
    self.pour(10)

  def _step6(self):
    self.solver.remove(self.taskTips)
    self.solver.remove(self.taskPlanBottleY)
    self.solver.remove(self.taskGripperZ)
    self.solver.remove(self.taskGripperXY)

  def step(self):
    if(self.stepIndex == 0):
      self._step1()
    elif(self.stepIndex == 1):
      self._step2()
    elif(self.stepIndex == 2):
      self._step2a()
    elif(self.stepIndex == 3):
      self._step3()
    elif(self.stepIndex == 4):
      self._step4()
    elif(self.stepIndex == 5):
      self._step5()
    elif(self.stepIndex == 6):
      self.pour(90)
    self.stepIndex = self.stepIndex + 1
    print self.stepIndex

s = Scenario(robot)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)




taskComIneq = TaskInequality('taskComIneq')
taskComIneq.add(robot.featureCom.name)
taskComIneq.referenceInf.value = (-0.05,-0.02,-0.05)    # Xmin, Ymin
taskComIneq.referenceSup.value = ( 0.05, 0.02,0.05)    # Xmin, Ymin
taskComIneq.dt.value=0.005
taskComIneq.controlGain.value = 0.9
s.solver.sot.remove('robot_task_com')
#s.solver.push(taskComIneq)

def initWaistCoMTasks(robot):
  # Controlling also the yaw.
  robot.waist.selec.value = '111111'

  robot.tasks ['waist'].controlGain.value = 200


initWaistCoMTasks(robot)
s.solver.push(robot.tasks ['waist'])

go
