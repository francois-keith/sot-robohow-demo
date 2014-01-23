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
solver = initialize ( robot, SolverKine )

from dynamic_graph.sot.core.meta_tasks_kine import *

################################################################################
# Create the FeatureExpressionGraph and the corresponding task
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.functions import *

from dynamic_graph.sot.robohow.tools import *
from dynamic_graph.sot.robohow.gripper import Gripper
from dynamic_graph.sot.robohow.cylinder_pouring import CylinderPouring
from dynamic_graph.sot.expression_graph.types import BaseElement


### Initialization of the Universe

# define the properties of a bottle containing liquid
#TODO: externalize
def createBottle():
  cp = CylinderPouring('cp')
  cp.setHeight(0.19)
  cp.setRadius(0.0325)
  cp.setVolume(0.5)
  return cp


# TODO : externalize
def estimateBottleFrame():
  BaseElement.frames['bottle'] = ((1,0,0,0.3), (0,1,0,-0.27), (0,0,1,1), (0,0,0,1))

def estimateCupFrame():
  BaseElement.frames['cup'] = ((1,0,0,0.3), (0,1,0,0.), (0,0,1,1), (0,0,0,1))

# ...
def estimateBottleFrameInHand(robot):
  # consider the difference of position between the bottle and the hand.
  # starting from that, define the robot (frame) corresponding to the top
  # of the bottle.
  bungFrame = OpPointModifier('bung')
  #TODO find the correct Z, position of the bung in the frame of the hand
  bungFrame.setTransformation(((1,0,0,0.0), (0,1,0,0.0), (0,0,1,-0.10), (0,0,0,1)))
  plug(robot.frames['rightGripper'].position, bungFrame.positionIN)
  plug(robot.frames['rightGripper'].jacobian, bungFrame.jacobianIN)
#  frame.position.recompute(bungFrame.position.time + 1)
#  frame.jacobian.recompute(bungFrame.jacobian.time + 1)
  robot.frames['bung'] = bungFrame


bottle = createBottle()
estimateBottleFrame()
estimateCupFrame()
estimateBottleFrameInHand(robot)

## FSM
from collections import namedtuple
Step = namedtuple('step', ['add', 'rm'])


""" ... """
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

  # the list of the steps. 
  # Each step contains 
  #  a list of tasks to add, classified by order of prioirity.
  #  a list of tasks to remove
  steps = []

  def __init__(self, robot, solver):
    ros = Ros(robot)
    self.stepIndex = -1
    self.solver = solver


    # ---- Humanoid specific tasks ---------------------------------------------
    # remove the com task, fix the waist instead
    robot.waist.selec.value = '111111'
    robot.tasks ['waist'].controlGain.value = 200
    solver.sot.remove('robot_task_com')
    solver.push(robot.tasks ['waist'])


    # --- CONTACTS
    # define contactLF and contactRF
    for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
        contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
        contact.feature.frame('desired')
        contact.gain.setConstant(10)
        contact.keep()
        locals()['contact'+name] = contact


    # --- TASK POSTURE --------------------------------------------------
    # set a default position for the joints. 
    robot.features['featurePosition'] = FeaturePosture('featurePosition')
    plug(robot.device.state,robot.features['featurePosition'].state)
    robotDim = len(robot.dynamic.velocity.value)
    robot.features['featurePosition'].posture.value = robot.halfSitting

    # 'HRP4LIRMM':
    # Right Leg, Left leg, chest, right arm, left arm
    postureTaskDofs = [False]*6 +  [False]*6 + [False]*4 \
        + [False]*1 + [True]*1 + [False]*7  \
        + [False]*1 + [True]*1 + [False]*7 

    for dof,isEnabled in enumerate(postureTaskDofs):
      robot.features['featurePosition'].selectDof(dof+6,isEnabled)
      
    robot.tasks['robot_task_position']=TaskInequality('robot_task_position')
    robot.tasks['robot_task_position'].add('featurePosition')
    # featurePosition.selec.value = toFlags((6,24))

    robot.tasks['robot_task_position'].referenceInf.value = (-1.6, 0.02)    # Xmin, Ymin
    robot.tasks['robot_task_position'].referenceSup.value = (-0.02, 1.6)    # Xmin, Ymin
    robot.tasks['robot_task_position'].dt.value= 0.005
    robot.tasks['robot_task_position'].controlGain.value = 0.009

    solver.push(robot.tasks['robot_task_position'])

    # ---- TASKS corresponding to the grasping of the bottle ---
    
    # 1st feature: Plane (x,y) _|_ bottleZ
    self.addExpressions(VersorElement('r_gripper_z', robot, 'rightGripper', versor = (0,0,-1) ))
    self.addExpressions(VersorElement('bottle_z', robot, 'bottle', versor = (0,0,1) ) )
    self.createTask('angle-gripperZ_bottleZ', 'r_gripper_z', 'bottle_z', 'angle', lowerBound = 0, upperBound = 0)


#    self.addExpressions(VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) ))
#    self.addExpressions(VersorElement('bottle_x', robot, 'bottle', versor = (1,0,0) ) )
#    self.createTask('angle-gripperX_bottleX', 'r_gripper_x', 'bottle_x', 'distance', lowerBound = 0, upperBound = 0)



# imo = Inverse_of_matrixHomo ('imo')
# mmh = Multiply_of_matrixHomo('mmh')
# mhpo = MatrixHomoToPose('mhpo')

# mhpo1 = MatrixHomoToPose('mhpo1')
# mhpo2 = MatrixHomoToPose('mhpo2')

# normedDirection = Normalization_of_vector('normedDirection')

#    # 2nd feature: axis gripper_center - bottle // gripper_x
#    plug(robot.frames['rightGripper'].position, imo.sin)
#
#    # multiply by the position of the bottle
#    plug(robot.frames['rightGripper'].position, mmh.sin1)
#    mmh.sin2.value = BaseElement.frames['bottle']
#
#    # # axis
#    plug(mmh.sout, mhpo.sin)
#
#    plug(mhpo.sout, normedDirection.vector)
#
#    plug(robot.frames['rightGripper'].position, mhpo1.sin)
#    mhpo2.sin.value = BaseElement.frames['bottle']
#    sov = Substract_of_vector('sov')
#    plug(mhpo1.sout, sov.sin2)
#    plug(mhpo2.sout, sov.sin1)
#    plug(sov.sout, normedDirection.vector)


#    self.expressions['r_gripper_x'] = VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) )
#    self.expressions['line_gripper_bottle'] = VersorElement('line_gripper_bottle', robot, 'rightGripper', versor = normedDirection.normedVector )
#    self.createTask('gripperX_axis', 'line_gripper_bottle', 'r_gripper_x', 'angle', lowerBound = radians(0), upperBound = radians(0))


#    self.expressions['r_gripper_z'] = VersorElement('r_gripper_z', robot, 'rightGripper', versor = (0,0,1) )
#    self.expressions['bottle_z']    = LineElement('bottle_z', robot, 'bottle', versor = (0,0,1) )
#    self.createTask('angle-gripperZ_bottleZ', 'r_gripper_z', 'bottle_z', 'angle', lowerBound = 0, upperBound = 0)



    # 2rd task: height Z == height bottle
    self.addExpressions(PointElement('r_gripper', robot, 'rightGripper'))
    self.addExpressions(PointElement('bottle', robot, 'bottle'))
#    self.createTask('position-gripper_bottle', 'r_gripper', 'bottle', 'distance', lowerBound = (radians(0)), upperBound = (radians(0)),)

    (self.tasks['position-gripper_bottle'], self.features['position-gripper_bottle']) = \
      createTaskAndFeaturePointToPoint('position-gripper_bottle', \
        self.expressions['r_gripper'], self.expressions['bottle'])
    self.features['position-gripper_bottle'].selec.value ='111' #TODO
    self.features['position-gripper_bottle'].reference.value = (0.1, 0, 0)


#    (self.tasks['position-gripper_bottle'], self.features['position-gripper_bottle']) = \
#      createTaskAndFeaturePointToPoint('position-gripper_bottle', \
#        self.expressions['r_gripper'], self.expressions['bottle'])
#    self.features['position-gripper_bottle'].selec.value ='100' #TODO
#    self.features['position-gripper_bottle'].reference.value = (0,)


#    (self.tasks['distance-gripperX_bottleX'], self.features['distance-gripperX_bottleX']) = \
#      createTaskAndFeaturePointToPoint('distance-gripperX_bottleX', \
#        self.expressions['r_gripper'], self.expressions['bottle'])
#    self.features['distance-gripperX_bottleX'].selec.value ='010' #TODO
#    self.features['distance-gripperX_bottleX'].reference.value = (0,)

    # steps = {('add'={}, 'rm'={})}
    # (4th task: distance ?)

#    self.steps.append(Step(add=['gripperX_axis'], rm=[]))
#    self.steps.append(Step(add=['angle-gripperZ_bottleZ'], rm=[]))
    self.steps.append(Step(add=['position-gripper_bottle', 'angle-gripperZ_bottleZ'], rm=[]))
#    self.steps.append(Step(add=['gripperX_axis'], rm=[]))


    # ---- TASKS corresponding the manipulation of the bottle ---

    # Constrain the rotation of the bottle for the pouring task : 
    # 90* => the Z axis of the world and the Z axis of the bottle are colinear
    #  0* => the bottle is horizontal
    self.addExpressions(VersorElement('ground_z', robot, 'ground', versor = (0,0,1)))
    self.addExpressions(VersorElement('bung_x', robot, 'bung', versor = (1,0,0) ))
    self.createTask('angle-pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (radians(90)), upperBound = (radians(90)))

    # Constrain the rotation of the gripper to keep the hand horizontal 
    self.addExpressions(PlaneElement('ground_plane', robot, 'ground', normal = (0,0,1)))
    self.addExpressions(VersorElement('r_gripper_y', robot, 'rightGripper', versor = (0,1,0) ))
    self.createTask('angle-gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0), upperBound = (0))

    # Distance bottle / r_hand
    self.createTask('distance-bottle_gripper', 'r_gripper', 'bottle', 'distance', lowerBound = (0), upperBound = (0))

    ################################ #######################
    ## height of the bottle above the target
    self.addExpressions(PointElement('cup', robot, 'cup'))
    self.addExpressions(PointElement('bung', robot, 'bung'))
    (self.tasks['position-bung_Z'], self.features['position-bung_Z']) = \
      createTaskAndFeaturePointToPoint('position-bung_Z', \
        self.expressions['bung'], self.expressions['cup'])
    self.features['position-bung_Z'].selec.value ='100' #TODO
    self.features['position-bung_Z'].reference.value = (-0.05,)


    #######################################################
    ## position of the bottle above the target.
    ## inequality task: we want the bottle to be above the recipient
    #(self.tasks['position-rg_XY'], self.features['position-rg_XY']) = \
    #  createTaskAndFeaturePointToPoint('position-rg_XY', \
    #  self.expressions['cup'], self.expressions['r_gripper'], False)

    #self.features['position-rg_XY'].selec.value = '011'
    #self.features['position-rg_XY'].reference.value = (0, 0)
    #self.tasks['position-rg_XY'].referenceInf.value = (0.03, 0.03)
    #self.tasks['position-rg_XY'].referenceSup.value = (100, 100)
    self.createTask('position-rg_XY', 'cup', 'r_gripper', 'distance', lowerBound = (0.02,), upperBound = (100,))


    #######################################################
    ## position of the bottle above the target.
    ## inequality task: we want the bottle to be above the recipient
    (self.tasks['position-bung_XY'], self.features['position-bung_XY']) = \
      createTaskAndFeaturePointToPoint('position-bung_XY', \
      self.expressions['cup'], self.expressions['bung'], False)

    self.features['position-bung_XY'].selec.value = '011'
    self.features['position-bung_XY'].reference.value = (0, 0)
    self.tasks['position-bung_XY'].referenceInf.value = (-0.025,-0.025)
    self.tasks['position-bung_XY'].referenceSup.value = ( 0.025, 0.025)


    #######################################################
    # define a task for the orientation of the fingertips : parallel to the handle
    # line / line constraint
    #tips = FeatureVersorToVersor('tips')
    self.addExpressions(VersorElement('ground_x', robot, 'ground', versor = (1,0,0)))
    self.createTask('tips', 'ground_x', 'r_gripper_y', 'angle', lowerBound = (2.5), upperBound = (2.5))

    self.r_gripper_angle = Gripper('r_gripper-opening', robot, 29, 2)

    ## position task of the hand
    # ---- TASK GRIP ---
    # Defines a task for the right hand.
    self.taskRH=MetaTaskKine6d('rh',robot.dynamic,'right-wrist','right-wrist')
    plug(robot.frames['rightGripper'].position,self.taskRH.feature.signal('position'))
    plug(robot.frames['rightGripper'].jacobian,self.taskRH.feature.signal('Jq'))
    self.taskRH.feature.frame('desired')
#    target=(0.15, -0.2,0.9) #TODO
    target=(0.3,-0.2,0.9, -1.5, -1.3, 1.3)
    gotoNd(self.taskRH,target,'111111',(4.9,0.9,0.01,0.9))
    self.tasks['taskRH'] = self.taskRH.task# mhpo = MatrixHomoToPose('mhpo')


    # create FoV task
    self.tasks['FoV'] = createFoVTasks(robot)


  """ insert an expression into the associated dictionnary """
  def addExpressions(self, expr):
    if expr.name in self.expressions:
      print expr.name + " already exists"
    self.expressions[expr.name] = expr


  """ create a task """
  def createTask(self, name, expr1, expr2, taskType, lowerBound, upperBound, gain=1):
    (task, feature) = \
      createTaskAndFeature(name, self.expressions[expr1], 
       self.expressions[expr2],\
       taskType, lowerBound == upperBound)    
    if(lowerBound == upperBound):
      feature.reference.value = lowerBound
    else:
      feature.reference.value = 0
      task.referenceInf.value = lowerBound
      task.referenceSup.value = upperBound
    task.gain = gain

    # setting the desired position.
    self.tasks[name] = task
    self.features[name] = feature

  def pourRaw(self, angle=45):
    self.features['angle-pouring'].reference.value = radians(angle)
    
  ## shortcut for the pouring taks
  def pour(self, vol):
    plug(self.features['angle-pouring'].error, bottle.angle)
    bottle.volume.recompute(robot.device.state.time)
    bottle.pour(vol)
    self.features['angle-pouring'].reference.value = bottle.targetAngle.value

  def applyStep(self, solver, step):
    for name in step.add:
      solver.push(self.tasks[name])
    for name in step.add:
      solver.remove(self.tasks[name])

  # graps
  def _step0(self):
    print "going in front of the bottle"
#    self.steps.append(Step(add=['position-gripper_bottle', 'angle-gripperZ_bottleZ'], rm=[]))
    self.criticalTask = self.tasks['position-gripper_bottle']
    self.seqStep()

  def _step1(self):
    self.solver.push(self.r_gripper_angle.task)
    self.r_gripper_angle.featureDes.errorIN.value = (1,0)
    
    # update the criticalTask
    self.criticalTask = self.r_gripper_angle.task
#self.tasks['angle-pouring']

  # close the gripper
  def _step2(self):
    print "going to the bottle"
    # Add a task to go to the bottle
    self.solver.push(self.tasks['distance-bottle_gripper'])
    self.solver.remove(self.tasks['position-gripper_bottle'])
    self.criticalTask = self.tasks['distance-bottle_gripper']

  # bent the bottle a little
  def _step2a(self):
    print "grasping"
    self.r_gripper_angle.featureDes.errorIN.value = (1,0.4)
#    self.r_gripper_angle.close()
    # update the criticalTask
    self.criticalTask = self.r_gripper_angle.task

    # replace the task controlling the orientation of the bottle by the pouring one.
    self.solver.remove(self.tasks['angle-gripperZ_bottleZ'])
#    self.solver.remove(self.tasks['distance-gripperX_bottleX'])
    self.solver.push(self.tasks['angle-pouring'])


    #todo: estimate the position of the bottle neck
 
  # go above the glass.
  def _step3(self):
    print "Start pouring"
    self.solver.remove(self.tasks['distance-bottle_gripper'])
    self.solver.push(self.tasks['position-bung_Z'])
    self.solver.push(self.tasks['position-rg_XY'])
    self.solver.push(self.tasks['position-bung_XY'])
    self.solver.push(self.tasks['angle-gripperY_in_ground_plane'])
#    self.solver.push(self.tasks['tips'])
    self.solver.sot.down('robot_task_angle-pouring')
    self.solver.sot.down('robot_task_angle-pouring')
    self.solver.sot.down('robot_task_angle-pouring')
    self.solver.sot.down('robot_task_angle-pouring')
    self.solver.push(self.tasks['FoV'])

    # update the criticalTask
#    self.criticalTask = (self.tasks['r_gripperXY'], self.tasks['r_gripperZ'], \
#      self.tasks['angle-gripperY_in_ground_plane'], self.tasks['tips'])
    self.criticalTask = self.tasks['position-bung_Z']

  # pour a little
  def _step4(self):
    print "Pouring more"
    self.pour(0.1)
    self.criticalTask = self.tasks['angle-pouring']

  # pour ...
  def _step5(self):
    print "And more"
    self.pourRaw(25)
    self.criticalTask = self.tasks['angle-pouring']

  def _step6(self):
    self.solver.remove(self.tasks['FoV'])
#    self.solver.remove(self.tasks['tips'])
#    self.solver.remove(self.tasks['angle-gripperY_in_ground_plane'])
    self.solver.remove(self.tasks['position-bung_Z'])
    self.solver.remove(self.tasks['position-bung_XY'])
#    self.solver.remove(self.tasks['angle-pouring'])
    self.features['angle-pouring'].reference.value = radians(85)
    self.solver.push(self.tasks['taskRH'])
    self.solver.sot.up(s.tasks['taskRH'].name)

    mhpo = MatrixHomoToPose('mhpo')
    plug(robot.rightWrist.position, mhpo.sin)
    mhpo.sout.recompute(robot.rightWrist.position.time)
    target=mhpo.sout.value
    gotoNd(self.taskRH,target,'111',(4.9,0.9,0.01,0.9))

    self.criticalTask = self.tasks['taskRH']
    


  def _step7(self):
    self.solver.remove(self.tasks['position-rg_XY'])
    self.solver.remove(self.tasks['angle-gripperY_in_ground_plane'])
    self.solver.remove(self.tasks['angle-pouring'])
    self.solver.remove(self.tasks['taskRH'])

    self.solver.push(self.tasks['distance-bottle_gripper'])
    self.solver.push(self.tasks['angle-gripperZ_bottleZ'])
    self.criticalTask = self.tasks['distance-bottle_gripper']

  def _step8(self):
    self.r_gripper_angle.featureDes.errorIN.value = (1,0)
    self.criticalTask = self.r_gripper_angle.task

  def _step9(self):
    self.solver.remove(self.r_gripper_angle.task)
    self.criticalTask = None

  def step(self):
    if(self.stepIndex == -1):
      self._step0()
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
    elif(self.stepIndex == 7):
      self._step7()
    elif(self.stepIndex == 8):
      self._step8()
    elif(self.stepIndex == 9):
      self._step9()
    self.stepIndex = self.stepIndex + 1
    print self.stepIndex
    print solver


  def seqStep(self):
    if self.stepIndex < len(self.steps):
      for name in self.steps[self.stepIndex].rm:
        self.solver.remove(self.tasks[name])
      for name in self.steps[self.stepIndex].add:
        self.solver.push(self.tasks[name])
#      self.stepIndex = self.stepIndex + 1


  def runDummySequencer(self):
    if self.criticalTask == None:
      return 
    err = getError(self.criticalTask)
    if(err < 0.001):
       self.step()

s = Scenario(robot, solver)

