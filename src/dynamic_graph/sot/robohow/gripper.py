from dynamic_graph.sot.core.meta_tasks_kine import *

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

"""
A wrapper class for the gripper
"""
class Gripper: 

  feature = None
  featureDes = None
  task = None

  """
  name:
  robot the robot
  index: index of the first gripper value in the state vector
  dim: dimension of the gripper
  """
  def __init__(self, name, robot, indexes):
    dim=len(indexes)
    self.feature = FeatureGeneric('feature'+name)
    self.featureDes = FeatureGeneric('featureDes'+name)
    self.feature.setReference('featureDes'+name)
    self.featureDes.errorIN.value = (0,) * dim;

    # create jacobian.
    jacobianGripper = eye(dim,robot.dimension) * 0;
    position=0
    for index in indexes:
      jacobianGripper[position][index] = 1;
      ++position
#    jacobianGripper[1][index+1] = 1;
    self.feature.jacobianIN.value = jacobianGripper

    # only selec some dofs
    selecRightGripper = Selec_of_vector('selecRightGripper')
    selecRightGripper.selec(index, index+dim)
    plug(robot.dynamic.position, selecRightGripper.sin)
    plug(selecRightGripper.sout, self.feature.errorIN)

    # 2\ Define the task. Associate to the task the position feature.
    self.task = Task('task'+name)
    self.task.add('feature'+name)
    self.task.controlGain.value = 1

  """
  ... hard coded for hrp4. #TODO
  """
  def open(self):
    pos = self.featureDes.errorIN.value
    self.featureDes.errorIN.value=(pos[0], 0)

  """
  ... hard coded for hrp4. #TODO
  """
  def close(self):
    pos = self.featureDes.errorIN.value
    self.featureDes.errorIN.value=(pos[0], 1)

  def set(self, position):
    initpos = self.featureDes.errorIN.value 
    self.featureDes.errorIN.value = position

