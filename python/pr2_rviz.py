# 1. Init robot, ros binding, solver
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph import plug

# creates the robot.
robot = Pr2('pr2', device=RobotSimu('pr2'))
plug(robot.device.state, robot.dynamic.position)

# publish to ros
from dynamic_graph.ros import *
ros = Ros(robot)

# Use kine solver (with inequalities)
from dynamic_graph.sot.dyninv import SolverKine
solver = initialize(robot, SolverKine)

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

