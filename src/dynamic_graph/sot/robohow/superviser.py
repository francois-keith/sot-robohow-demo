#TODO: define the list of symbols exported.
from dynamic_graph import plug

"""
Publish the error of the associated task.
"""
def startPublishingError(robot,publisher,taskname):
  if publisher == None:
    return
  #print ' *** publishing ', taskname

  # verify that the task exists in the robot database
  if taskname in robot.tasks:
    sig_name=taskname+'_error'
    print (" *** publisher.add('vector', sig_name,'/sot/'"+taskname+"'_error')")
    publisher.add('vector', sig_name,'/sot/'+taskname+'_error')
    plug(robot.tasks[taskname].error, publisher.signal(sig_name))
    #print ' *** success publishing ', taskname


def stopPublishingError(publisher, taskname):
  if publisher == None:
    return

  sig_name=taskname+'_error'
  try:
    print (" *** publisher.signal("+sig_name+").unplug()")
    # unplug the signal. This will make the after command crash
    publisher.signal(sig_name).unplug()
    print ("publisher.rm("+sig_name+")")
    publisher.rm(sig_name)
  except:
    print "signal " + sig_name + " not found"





""" The goal of the supervisor class is to ... """
class Superviser:
  robot  = None
  solver = None
  publisher = None
  desiredStack = [] # desired state of the stack of tasks
  currentStack = [] # current state of the stack of tasks

  def __init__(self, robot, solver, publisher):
    self.robot = robot
    self.solver = solver
#    self.publisher = publisher


  """synchronize the current stack with the actual stack of tasks """
  def synchronize(self):
    self.solver.sot.clear()
    self.currentStack = [];

  """clear the list of desired task"""
  def clear(self):
    self.desiredStack = [];

  """add a task at the end of desired stack""";
  def push(self, name):
    #print ' *** self.desiredStack.append  ', name
    self.desiredStack.append(name);


  """display the state of the desired stack"""
  def dispCurrent(self):
    for name in self.currentStack:
      print name

  def dispDesired(self):
    for name in self.desiredStack:
      print name


  """replace the content of the stack by the desired list of task"""
  def update(self):
    print " *** update"

    # builds the list of operations required
    index = 0
    while (index < len(self.desiredStack)):
      # identical task
      if len(self.currentStack) > index and self.currentStack[index] == self.desiredStack[index]:
        index = index + 1
      # the task exists in b, but not at the same place
      elif self.desiredStack[index] in self.currentStack:
        # one task has been removed
        if len(self.currentStack) > index and not self.currentStack[index] in self.desiredStack:
          taskname = self.currentStack[index]
          stopPublishingError(self.publisher, taskname)
          #print ' *** self.solver.sot.remove ', taskname
          self.solver.sot.remove(taskname)
          #print ' *** end self.solver.sot.remove ', taskname
          self.currentStack.remove(taskname)

        # the priority of the task desiredStack_[index] has changed
        else:
          taskname = self.desiredStack[index]
          index2=currentStack.index(task)
          for i in range(index, index2):
            #print ' *** self.solver.sot.up ', taskname
            self.solver.sot.up(taskname)
            #print ' *** end self.solver.sot.up ', taskname
          self.currentStack.remove(taskname)
          self.currentStack.insert(index,taskname)
          index = index + 1

      # the task is not in the real stack: add it
      else:
        taskname = self.desiredStack[index]
        #print ' *** self.solver.sot.push ', taskname
        self.solver.sot.push(taskname)
        #print ' *** end self.solver.sot.push ', taskname
        startPublishingError(self.robot,self.publisher,taskname)

        for i in range(index, len(self.currentStack)):
          #print ' *** self.solver.sot.up ', taskname
          self.solver.sot.up(taskname)
          #print ' *** end self.solver.sot.up ', taskname
        self.currentStack.insert(index,taskname)
        index = index+1
    
    # Now, we should have current = [desiredStack_, some tasks], we remove the remaining tasks.
    while (index < len(self.currentStack)):
      taskname = self.currentStack[index]
      stopPublishingError(self.publisher, taskname)
      #print ' *** self.solver.sot.remove ', taskname
      self.solver.sot.remove(taskname)
      #print ' *** end self.solver.sot.remove ', taskname
      self.currentStack.remove(taskname)


  #print compareStacks([],[])  
  #print compareStacks(['a'],[])  
  #print compareStacks(['a', 'b'],[])
  #print compareStacks(['a', 'b'],['b'])  
  #print compareStacks(['a', 'b'],['a'])  
  #print compareStacks(['a', 'b'],['a', 'c','b'])  
  #print compareStacks(['a', 'b'],['a', 'c','d'])  

