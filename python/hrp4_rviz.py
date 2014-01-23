from dynamic_graph.sot.robohow.hrp4_scenario import *

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

go

