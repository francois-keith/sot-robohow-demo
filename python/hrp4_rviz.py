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


from dynamic_graph.sot.robohow.hrp4_scenario import *
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
#    s.runDummySequencer()

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

go

