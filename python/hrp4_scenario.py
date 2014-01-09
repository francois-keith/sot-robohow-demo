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
def createFoVTasks(robot):
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
  return taskFoV.task

solver = initialize ( robot, SolverKine )


from numpy.linalg import norm

""" Return the norm of the error of a task """
def getError(task):
	return norm(array(task.error.value))


class Scenario:
  solver = None
  stepIndex = 0

  """ dictionnary of expressions """
  expressions = {}

  """ dictionnary of features (in the sense of sot features) """
  features = {}

  """ dictionnary of tasks (or constraints) """
  tasks = {}

  criticalTask = None

  r_gripper_angle = None

  taskRH = None # TODO

  def __init__(self, robot, solver):
    ros = Ros(robot)
    self.stepIndex = 0
    self.solver = solver

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

    # Constrain the rotation of the bottle for the pouring task : 
    # 90* => the Z axis of the world and the Z axis of the bottle are colinear
    #  0* => the bottle is horizontal
    self.expressions['ground_z']    = VersorElement('ground_z', robot, 'ground', versor = (0,0,1))
    self.expressions['r_gripper_x'] = VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) )
    self.createTask('bottleZ', 'r_gripper_x', 'ground_z', 'angle', lowerBound = (radians(90)), upperBound = (radians(90)))

    # Constrain the rotation of the gripper to keep the hand horizontal 
    self.expressions['ground_plane'] = PlaneElement('ground_plane', robot, 'ground', normal = (0,0,1))
    self.expressions['r_gripper_y']  = VersorElement('r_gripper_y', robot, 'rightGripper', versor = (0,1,0) )
    self.createTask('bottleY', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0), upperBound = (0))


    ################################ #######################
    ## height of the bottle above the target
    self.expressions['heightZ']       = PointElement('heightZ', robot, 'ground', position = (0,0,1))
    self.expressions['r_gripperZpos'] = PointElement('r_gripperZpos', robot, 'rightGripper')
#    self.createTask('positionZ', 'r_gripperZpos', 'heightZ', 'distance', lowerBound = (0,), upperBound = (0,))
    (self.tasks['positionZ'], self.features['positionZ']) = \
      createTaskAndFeaturePointToPoint('positionZ', \
        self.expressions['r_gripperZpos'], self.expressions['heightZ'])
    self.features['positionZ'].selec.value ='100' #TODO
    self.features['positionZ'].reference.value = (0,)

    #######################################################
    ## position of the bottle above the target.
    ## inequality task: we want the bottle to be above the recipient
		# TODO
    self.expressions['posXY']       = PointElement('posXY',       robot, 'ground')
    self.expressions['r_gripperXY'] = PointElement('r_gripperXY', robot, 'rightGripper')
    (self.tasks['positionXY'], self.features['positionXY']) = \
      createTaskAndFeaturePointToPoint('positionXY', \
      self.expressions['posXY'], self.expressions['r_gripperXY'], False)

    self.features['positionXY'].selec.value = '011'
    self.features['positionXY'].reference.value = (0.3, -0.1)
    self.tasks['positionXY'].referenceInf.value = (-0.05,-0.05)
    self.tasks['positionXY'].referenceSup.value = ( 0.05, 0.05)

    #######################################################
    # define a task for the orientation of the fingertips : parallel to the handle
    # line / line constraint
    #tips = FeatureVersorToVersor('tips')
    self.expressions['ground_x'] = VersorElement('ground_x', robot, 'ground', versor = (1,0,0))
    self.createTask('tips', 'ground_x', 'r_gripper_y', 'angle', lowerBound = (2.5), upperBound = (2.5))

    self.r_gripper_angle = Gripper('r_gripper_angle', robot, 29, 2)


		## ....
    robot.featureComDes.errorIN.value = (0.05, 0.0, 0.8)


    ## position task of the hand
    # ---- TASK GRIP ---
    # Defines a task for the right hand.
    self.taskRH=MetaTaskKine6d('rh',robot.dynamic,'right-wrist','right-wrist')
    self.taskRH.feature.frame('desired')
#    target=(0.15, -0.2,0.9) #TODO
    target=(0.25,-0.2,0.9, -1.5, -1.3, 1.3)
    gotoNd(self.taskRH,target,'111111',(4.9,0.9,0.01,0.9))
    self.tasks['taskRH'] = self.taskRH.task

  """ create a task """
  def createTask(self, name, expr1, expr2, taskType, lowerBound, upperBound, gain=1):
    (task, feature) = \
      createTaskAndFeature(name, self.expressions[expr1], 
       self.expressions[expr2],\
       taskType, lowerBound == upperBound)
    
    if(lowerBound == upperBound):
      feature.reference.value = lowerBound
    else:
      feature.reference.value = lowerBound * 0
      task.referenceInf.value = lowerBound
      task.referenceSup.value = upperBound
    task.gain = gain

    # setting the desired position.
    self.tasks[name] = task
    self.features[name] = feature


    # create FoV task
    self.tasks['FoV'] = createFoVTasks(robot)



  ## sequencing...
  def pour(self, angle=45):
    self.features['bottleZ'].reference.value = radians(angle)


  # graps
  def _step1(self):
    self.solver.push(self.tasks['bottleZ'])
    self.solver.push(self.r_gripper_angle.task)
    self.r_gripper_angle.featureDes.errorIN.value = (1,0)

    # update the criticalTask
    self.criticalTask = self.tasks['bottleZ']

  # close the gripper
  def _step2(self):
    self.r_gripper_angle.close()
    # update the criticalTask
    self.criticalTask = self.r_gripper_angle.task

    #todo: estimate the position of the bottle neck

  # bent the bottle a little
  def _step2a(self):
    print " " # self.pour(80)
 
  # go above the glass.
  def _step3(self):
    print "Start pouring"
    self.solver.push(self.tasks['positionXY'])
    self.solver.push(self.tasks['positionZ'])
    self.solver.push(self.tasks['bottleY'])
#    self.solver.push(self.tasks['tips'])
    self.solver.push(self.tasks['FoV'])

    # update the criticalTask
#    self.criticalTask = (self.tasks['r_gripperXY'], self.tasks['r_gripperZ'], \
#      self.tasks['bottleY'], self.tasks['tips'])
    self.criticalTask = self.tasks['positionZ']

  # pour a little
  def _step4(self):
    self.pour()
    self.criticalTask = self.tasks['bottleZ']

  # pour ...
  def _step5(self):
    self.pour(20)

  def _step6(self):
    self.solver.remove(self.tasks['FoV'])
    self.solver.remove(self.tasks['tips'])
    self.solver.remove(self.tasks['bottleY'])
    self.solver.remove(self.tasks['positionZ'])
    self.solver.remove(self.tasks['positionXY'])
    self.solver.push(self.tasks['taskRH'])
    self.pour(90)

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
      self._step6()
    self.stepIndex = self.stepIndex + 1
    print self.stepIndex

  def runDummySequencer(self):
    if self.criticalTask == None:
      return 
    err = getError(self.criticalTask)
#    if(err < 0.001):
#       self.step()



s = Scenario(robot, solver)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)
    s.runDummySequencer()

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
