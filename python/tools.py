
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



from numpy.linalg import norm

""" Return the norm of the error of a task """
def getError(task):
	return norm(array(task.error.value))


# --- FOV ---
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
def createFoVTasks(robot):
  headMcam=array([[0.0,0.0,1.0,0],[1.,0.0,0.0,0],[0.0,1.,0.0,0],[0.0,0.0,0.0,1.0]])

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


""" create an inequality task for the COM """ 
def createComInequalityTask(robot):
  taskComIneq = TaskInequality('taskComIneq')
  taskComIneq.add(robot.featureCom.name)
  taskComIneq.referenceInf.value = (-0.05,-0.02,-0.05)    # Xmin, Ymin
  taskComIneq.referenceSup.value = ( 0.05, 0.02,0.05)    # Xmin, Ymin
  taskComIneq.dt.value=0.005
  taskComIneq.controlGain.value = 0.9



# message to create a task:
# creates the feature and the tasks

# get the feedback of the task
# either create a dedicated action to add the task
# either run the feedback request at a given frequence.


