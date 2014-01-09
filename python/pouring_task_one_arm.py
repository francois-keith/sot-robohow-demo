import roslib
import rospy
import numpy
import math


from dynamic_graph.sot.core.meta_tasks_kine import *

## Create the robot romeo.
from dynamic_graph.sot.romeo.robot import *
robot = Robot('romeo', device=RobotSimu('romeo'))

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot, SolverKine )

from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_generic import MetaTaskGeneric
	
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


# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact
# ---- TASKS -------------------------------------------------------------------



################################################################################
# Create the FeatureExpressionGraph and the corresponding task
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.functions import *


#######################################################""
## Angle of the "bottle" normal wrt world
ground_z    = VersorElement('ground_z', robot, 'ground', versor = (0,0,1))
r_gripper_x = VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) )

(taskAngleBottle, angleBottleZ) = createTaskAndFeature('bottleZ', r_gripper_x, ground_z, 'angle')

angleBottleZ.reference.value = math.radians(90)

#######################################################""
## The bottle can only rotate around its Y axis
ground_plane = PlaneElement('ground_plane', robot, 'ground', normal = (0,0,1))
r_gripper_y  = VersorElement('r_gripper_y', robot, 'rightGripper', \
	versor = (0,1,0) )

(taskPlanBottleY, planBottleY) = createTaskAndFeature('bottleY', ground_plane, r_gripper_y, 'angle')
planBottleY.reference.value = 0


#######################################################""
## position of the end effector (Z)
heightZ       = PointElement('heightZ', robot, 'ground', position = (0,0,0.8))
r_gripperZpos = PointElement('r_gripperZpos', robot, 'rightGripper')

(taskPositionZ, positionZ) = \
	createFeaturePointToPoint('positionZ', r_gripperZpos, heightZ)

positionZ.selec.value ='100'
positionZ.reference.value = (0,)


#######################################################""
## position of the end effector XY
posXY       = PointElement('posXY',       robot, 'ground')
r_gripperXY = PointElement('r_gripperXY', robot, 'rightGripper')

(taskPositionXY, positionXY) = \
	createFeaturePointToPoint('positionXY', r_gripperXY, posXY)

positionXY.selec.value = '011'
positionXY.reference.value = (-0.3,0.1)

# fill the SoT

solver.push(taskPositionXY)
solver.push(taskPositionZ)
solver.push(taskPlanBottleY)
solver.push(taskAngleBottle)

## sequencing...
def pour(angle=115):
  angleBottleZ.reference.value = math.radians(angle)

