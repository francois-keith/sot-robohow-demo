#import roslib
#import rospy
import numpy
from math import radians

# Create the robot romeo.
#from dynamic_graph.sot.hrp4.sot_hrp4_controller import *
#from dynamic_graph.sot.hrp4.prologue import *

from dynamic_graph.sot.core.meta_tasks_kine import *

# Create the robot romeo.
from dynamic_graph.sot.hrp4.sot_hrp4_controller import *
from dynamic_graph.sot.hrp4.prologue import *

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize


# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *

from dynamic_graph.sot.core.meta_task_generic import MetaTaskGeneric

################################################################################
# Create the FeatureExpressionGraph and the corresponding task
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.expression_graph.gripper import Gripper


class Scenario:
	solver = None
	angleBottleZ = None
	taskAngleBottle = None
	taskAngleCup = None

	l_gripper_angle = None
	r_gripper_angle = None

	taskGripperXY =None
	taskGripperZ =None
	taskPlanBottleY =None
	taskTips = None

	def __init__(self, robot):
		ros = Ros(robot)
		solver = initialize ( robot, SolverKine )
		self.solver = solver

		# --- CONTACTS
		# define contactLF and contactRF
		for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
				contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
				contact.feature.frame('desired')
				contact.gain.setConstant(10)
				contact.keep()
				locals()['contact'+name] = contact
		# ---- TASKS -------------------------------------------------------------------


		# operational point used
		ground_z    = VersorElement('ground_z', robot, 'ground', versor = (0,0,1))
		r_gripper_x = VersorElement('r_gripper_x', robot, 'rightGripper', versor = (1,0,0) )

		(taskAngleBottle, angleBottleZ) = createTaskAndFeature('bottleZ', r_gripper_x, ground_z, 'angle')
		angleBottleZ.reference.value = radians(90)



		l_gripper_x = VersorElement('r_gripper_x', robot, 'leftGripper', versor = (1,0,0) )

		(taskAngleCup, angleBottleCup) = createTaskAndFeature('cupZ', l_gripper_x, ground_z, 'angle')
		angleBottleCup.reference.value = radians(90)

		# # # # # # # 
		ground_plane = PlaneElement('ground_plane', robot, 'ground', normal = (0,0,1))
		r_gripper_y  = VersorElement('r_gripper_y', robot, 'rightGripper', versor = (0,1,0) )

		(taskPlanBottleY, planBottleY) = createTaskAndFeature('bottleY', ground_plane, r_gripper_y, 'angle')
		planBottleY.reference.value = 0


		################################ #######################""
		## position right hand above target
		# heightZ       = PointElement('heightZ', robot, 'ground', position = (0,0,1))
		l_gripperZpos = PointElement('l_gripperZpos', robot, 'leftGripper')
		r_gripperZpos = PointElement('r_gripperZpos', robot, 'rightGripper')

		(taskGripperZ, positionZ) = 	createFeaturePointToPoint('positionZ', r_gripperZpos, l_gripperZpos)

		positionZ.selec.value ='100'
		positionZ.reference.value = (-0.1,)


		#######################################################""
		## position leftHand op above right hand
		## -pi/8 << dot(bottle_normal, World_Z_axis) << pi/8

		#posXY       = PointElement('posXY',       robot, 'ground')
		l_gripperXY = PointElement('l_gripperXY', robot, 'leftGripper')
		r_gripperXY = PointElement('r_gripperXY', robot, 'rightGripper')

		(taskGripperXY, positionXY) = createFeaturePointToPoint('positionXY', l_gripperXY, r_gripperXY)

		positionXY.selec.value = '011'
		positionXY.reference.value = (0,0)


		# define a task for the orientation of the fingertips : parallel to the handle
		# line / line constraint
		#tips = FeatureVersorToVersor('tips')

		ground_x       = VersorElement('ground_x', robot, 'ground', versor = (1,0,0))
		(taskTips, tips) = createTaskAndFeature('tips', ground_x, r_gripper_y, 'angle')
		tips.reference.value = 2.5


		l_gripper_angle = Gripper('r_gripper_angle', robot, 38, 2)
		r_gripper_angle = Gripper('r_gripper_angle', robot, 29, 2)




		## TODO ...
		self.angleBottleZ = angleBottleZ
		self.taskAngleBottle = taskAngleBottle
		self.taskAngleCup = taskAngleCup
		self.l_gripper_angle = l_gripper_angle
		self.r_gripper_angle = r_gripper_angle
		self.taskGripperXY =taskGripperXY
		self.taskGripperZ =taskGripperZ
		self.taskPlanBottleY =taskPlanBottleY
		self.taskTips =taskTips


	## sequencing...
	def pour(self, angle=45):
		self.angleBottleZ.reference.value = radians(angle)


	# graps
	def step1a(self):
		self.solver.push(self.taskAngleBottle)
		self.solver.push(self.r_gripper_angle.task)

	def step1b(self):
		self.solver.push(self.taskAngleCup)
		self.solver.push(self.l_gripper_angle.task)

	# move away
	def step2a(self):
		self.r_gripper_angle.close()

	def step2b(self):
		self.l_gripper_angle.close()

	def step3(self):
		self.solver.push(self.taskGripperXY)
		self.solver.push(self.taskGripperZ)
		self.solver.push(self.taskPlanBottleY)
		self.solver.push(self.taskTips)

	def step4(self):
		self.pour()

	def step5(self):
		self.pour(90)

#	def step6(self):
#		solver.remove(taskTips)
#		solver.remove(taskPlanBottleY)
#		solver.remove(taskGripperZ)
#		solver.remove(taskGripperXY)

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



go

