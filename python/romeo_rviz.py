#from dynamic_graph.sot.core.meta_tasks_kine import *
# Create the robot romeo.
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from numpy import *

# Create the robot romeo.
from dynamic_graph.sot.romeo.robot import *
robot = Robot('robot', device=RobotSimu('romeo'))

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
from dynamic_graph.sot.dyninv import SolverKine
solver = initialize ( robot, SolverKine )


from dynamic_graph.sot.robohow.hrp4_scenario import *
s = Scenario(robot, solver)

#-------------------------------------------------------------------------------
#----- RVIZ --------------------------------------------------------------------
#-------------------------------------------------------------------------------


mqRW = MatrixHomoToPoseQuaternion('mqRW')
plug(robot.dynamic.signal('right-wrist'), mqRW.sin)

mqRG = MatrixHomoToPoseQuaternion('mqRG')
plug(robot.frames['rightGripper'].position, mqRG.sin)

mqLW = MatrixHomoToPoseQuaternion('mqLW')
plug(robot.dynamic.signal('left-wrist'), mqLW.sin)

mqLG = MatrixHomoToPoseQuaternion('mqLG')
plug(robot.frames['leftGripper'].position, mqLG.sin)

# allows to change only one joint and to test 
def updateQuaternion(robot):
#  ndof=len(robot.halfSitting)
#  robot.features['featurePosition'].posture.value = (0,)*index + (angle,) + (0,)*(ndof-index)

  mqRW.sout.recompute(robot.device.state.time)
  mqRG.sout.recompute(robot.device.state.time)
#  print "  robot.dynamic.signal('right-wrist').value"
#  print robot.dynamic.signal('right-wrist').value
#  print "  mqRW.sout "
#  print mqRW.sout.value
#  print " "
#  print " "
#  print "  robot.frames['rightGripper'].position.value"
#  print robot.frames['rightGripper'].position.value
#  print "  mqRG.sout "
#  print mqRG.sout.value 
#  print " "
#  print " "


  mqLW.sout.recompute(robot.device.state.time)
  mqLG.sout.recompute(robot.device.state.time)
#  print "  robot.dynamic.signal('left-wrist').value"
#  print robot.dynamic.signal('left-wrist').value
#  print "  mqLW.sout "
#  print mqLW.sout.value
#  print " "
#  print " "
#  print "  robot.frames['leftGripper'].position.value"
#  print robot.frames['leftGripper'].position.value
#  print "  mqLG.sout "
#  print mqLG.sout.value 
#  print " "
#  print " "

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3

@loopInThread
def inc():
    robot.device.increment(dt)
    updateQuaternion(robot)
#    s.runDummySequencer()

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

go

