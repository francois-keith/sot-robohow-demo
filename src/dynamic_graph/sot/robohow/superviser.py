#TODO: define the list of symbols exported.
from dynamic_graph import plug

"""
Publish the error of the associated task.
publisher: the rosPublish entity
taskname:  the task name 
"""
def startPublishingError(robot,publisher,taskname):
  if publisher == None:
    return

  # verify that the task exists in the robot database
  if taskname in robot.tasks:
    sig_name=taskname+'_error'
    publisher.add('vector', sig_name,'/sot/'+taskname+'_error')
    plug(robot.tasks[taskname].error, publisher.signal(sig_name))


def stopPublishingError(publisher, taskname):
  if publisher == None:
    return

  sig_name=taskname+'_error'
  try:
    # unplug the signal.
    publisher.signal(sig_name).unplug()
    publisher.rm(sig_name)
  except:
    print "signal " + sig_name + " not found"





""" The goal of the supervisor class is to ... """
class Superviser:
  robot  = None
  solver = None
  publisher = None
  desiredStack = [] # desired state of the stack of tasks

  def __init__(self, robot, solver, publisher):
    self.robot = robot
    self.solver = solver
    self.publisher = publisher

  def getCurrentStack(self):
    taskList = self.solver.sot.getTaskList()
    currentStack = taskList.split('|')
    return currentStack

  """clear the list of desired task"""
  def clear(self):
    self.desiredStack = [];

  """add a task at the end of desired stack""";
  def push(self, name):
    self.desiredStack.append(name);


  """display the state of the desired stack"""
  def dispCurrent(self):
    for name in self.getCurrentStack():
      print name

  def dispDesired(self):
    for name in self.desiredStack:
      print name


  """replace the content of the stack by the desired list of task"""
  def update(self):
    # builds the list of operations required
    index = 0
    currentStack = self.getCurrentStack()
    while (index < len(self.desiredStack)):
      # identical task
      if len(currentStack) > index and currentStack[index] == self.desiredStack[index]:
        index = index + 1
      # the task exists in b, but not at the same place
      elif self.desiredStack[index] in currentStack:
        # one task has been removed
        if len(currentStack) > index and not currentStack[index] in self.desiredStack:
          taskname = currentStack[index]
          stopPublishingError(self.publisher, taskname)
          self.solver.sot.remove(taskname)
          currentStack.remove(taskname)

        # the priority of the task desiredStack_[index] has changed
        else:
          taskname = self.desiredStack[index]
          index2=currentStack.index(task)
          for i in range(index, index2):
            self.solver.sot.up(taskname)
          currentStack.remove(taskname)
          currentStack.insert(index,taskname)
          index = index + 1

      # the task is not in the real stack: add it
      else:
        taskname = self.desiredStack[index]
        self.solver.sot.push(taskname)
        startPublishingError(self.robot,self.publisher,taskname)

        for i in range(index, len(currentStack)):
          self.solver.sot.up(taskname)
        currentStack.insert(index,taskname)
        index = index+1
    
    # Now, we should have current = [desiredStack_, some tasks], we remove the remaining tasks.
    while (index < len(currentStack)):
      taskname = currentStack[index]
      stopPublishingError(self.publisher, taskname)
      self.solver.sot.remove(taskname)
      currentStack.remove(taskname)

