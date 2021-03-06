#from dynamic_graph.sot.core.meta_tasks_kine import *
# Create the robot romeo.
from dynamic_graph.sot.hrp4.sot_hrp4_controller import *
from dynamic_graph.sot.hrp4.prologue import *

from numpy import eye, array
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint

from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver, createCenterOfMassFeatureAndTask, createOperationalPointFeatureAndTask, initializeSignals


# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
from dynamic_graph.sot.dyninv import SolverKine
solver = initialize ( robot, SolverKine )

robot.frames['l_gripper'] = robot.frames['leftGripper']
robot.frames['r_gripper'] = robot.frames['rightGripper']

#robot.frames['r_gripper'] = robot.frames['rightGripper']


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3

@loopInThread
def inc():
    robot.device.increment(dt)
#    s.runDummySequencer()

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

go

def initPostureTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints. 
  robot.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,robot.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurePosition'].posture.value = robot.halfSitting

  postureTaskDofs = [True]*(len(robot.halfSitting))

  for dof,isEnabled in enumerate(postureTaskDofs):
    robot.features['featurePosition'].selectDof(dof,isEnabled)
    
  robot.tasks['robot_task_position']=Task('robot_task_position')
  robot.tasks['robot_task_position'].add('featurePosition')
  # featurePosition.selec.value = toFlags((6,24))

  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)


initPostureTask(robot)

solver.sot.clear()
solver.push(robot.tasks['robot_task_position'])

mqLW = MatrixHomoToPoseQuaternion('mqLW')
plug(robot.dynamic.signal('left-wrist'), mqLW.sin)

mqLG = MatrixHomoToPoseQuaternion('mqLG')
plug(robot.frames['leftGripper'].position, mqLG.sin)

# allows to change only one joint and to test 
def testJoint(robot, index, angle):
  ndof=len(robot.halfSitting)
  robot.features['featurePosition'].posture.value = (0,)*index + (angle,) + (0,)*(ndof-index)
  robot.dynamic.signal('chest').recompute(robot.device.state.time)
  mqLW.sout.recompute(robot.device.state.time)
  mqLG.sout.recompute(robot.device.state.time)
  print "  robot.dynamic.signal('left-wrist').value"
  print robot.dynamic.signal('left-wrist').value
  print "  mqLW.sout "
  print mqLW.sout.value
  print "  robot.frames['leftGripper'].position.value"
  print robot.frames['leftGripper'].position.value
  print "  mqLG.sout "
  print mqLG.sout.value 



