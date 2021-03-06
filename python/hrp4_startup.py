# execfile('/home/fkeith/software/georg/install/lib/python2.7/site-packages/dynamic_graph/sot/robohow/hrp4_startup.py')

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.robohow.tools import *
from dynamic_graph.sot.robohow.gripper import Gripper
from dynamic_graph.sot.robohow.cylinder_pouring import CylinderPouring
from dynamic_graph.sot.expression_graph.types import BaseElement
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.robohow.superviser import *


# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.dyninv import SolverKine

from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot, SolverKine )

superviser = Superviser(robot, solver, ros.rosPublish)

# Additional frames.
robot.frames['l_gripper'] = robot.frames['leftGripper']
robot.frames['r_gripper'] = robot.frames['rightGripper']
robot.expressions={}

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
#  BaseElement.frames['bottle'] = ((1,0,0,0.5), (0,1,0,-0.27), (0,0,1,0.7), (0,0,0,1))

def estimateCupFrame():
  BaseElement.frames['cup'] = ((1,0,0,0.3), (0,1,0,0.), (0,0,1,1), (0,0,0,1))
#  BaseElement.frames['cup'] = ((1,0,0,0.5), (0,1,0,0.), (0,0,1,0.7), (0,0,0,1))

# ...
def estimateBottleFrameInHand(robot):
  # consider the difference of position between the bottle and the hand.
  # starting from that, define the robot (frame) corresponding to the top
  # of the bottle.
  bungFrame = OpPointModifier('bung')
  #TODO find the correct Z, position of the bung in the frame of the hand
  bungFrame.setTransformation(((1,0,0,0.0), (0,1,0,0.0), (0,0,1,0.10), (0,0,0,1)))
  plug(robot.frames['rightGripper'].position, bungFrame.positionIN)
  plug(robot.frames['rightGripper'].jacobian, bungFrame.jacobianIN)
#  frame.position.recompute(bungFrame.position.time + 1)
#  frame.jacobian.recompute(bungFrame.jacobian.time + 1)
  robot.frames['bung'] = bungFrame


def initPostureTask(robot):
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
    postureTaskDofs = [False]*6 +  [False]*6 + [True]*4 + [True]*9 + [True]*9
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
  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)


bottle = createBottle()
estimateBottleFrame()
estimateCupFrame()
estimateBottleFrameInHand(robot)


