/*
 */

#include "superviser.h"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <jrl/mal/matrixabstractlayer.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

int find(const std::vector<std::string> &vec, const std::string &elmt)
{
  for(unsigned i=0;i<vec.size(); ++i)
    if(vec[i] == elmt)
      return i;
  return -1;
}

void disp(const std::vector<std::string> &vec, const std::string &name)
{
  std::cout << "  " << name << std::endl;
  for(unsigned i=0;i<vec.size(); ++i)
    std::cout << vec[i] << ", ";
  std::cout << std::endl;
}

void insert(std::vector<std::string> &vec, unsigned index, const std::string &elmt)
{
  std::vector<std::string>::iterator it = vec.begin();
  for(unsigned i=0; i < index; ++i)
    ++it;
  vec.insert(it,elmt);
}

void erase(std::vector<std::string> &vec, std::string elmt)
{
  for(std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); )
  {
    if(*it == elmt)
    {
      vec.erase(it);
      it = vec.begin();
    }
    else
      ++it;
  }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Superviser,"Superviser");

Superviser::
Superviser( const std::string & name )
  :Entity(name)
{
  sotDEBUGIN(5);
  initCommands();
  sotDEBUGOUT(5);
}


Superviser::
~Superviser( void )
{
}

void Superviser::clear()
{
  desiredStack_.clear();
}

void Superviser::push(const std::string & name)
{
  desiredStack_.push_back(name);
  return ;
}

void Superviser::update()
{
  std::vector<std::string> currentStack;
  unsigned i = 0;
  if(sot_ == 0x0)
  {
    std::cerr << "No solver bound, aborting" << std::endl;
    return;
  }

  while (sot_->getTask(i) != "")
  {
    currentStack.push_back(sot_->getTask(i));
    ++i;
  }

  if (currentStack == desiredStack_)
    return;

  //builds the list of operations required
  int index = 0;
  while (index < desiredStack_.size())
  {
    // identical task
    if (currentStack.size() > index && currentStack[index] == desiredStack_[index])
    {
      ++index;
    }
    //the task exists in b, but not at the same place
    else if (find(currentStack, desiredStack_[index]) != -1)
    {
      // one task has been removed
      if (currentStack.size() > index && find(desiredStack_, currentStack[index]) != -1)
      {
        sot_->removeByTaskName(currentStack[index]);
        erase(currentStack, currentStack[index]);
      }

      // the priority of the task a[index] has changed
      else
      {
        int index2 = find(currentStack, desiredStack_[index]);
        for(int i=index; i<index2; ++i)
          sot_->upByTaskName(desiredStack_[index]);
        erase(currentStack, desiredStack_[index]);
        insert(currentStack,index,desiredStack_[index]);
        ++index;
      }
    }
    // the task is not in b
    else
    {
      sot_->pushByTaskName(desiredStack_[index]);
      for(i=index; i<currentStack.size(); ++i)
      {
        sot_->upByTaskName(desiredStack_[index]);
      }
      insert(currentStack,index,desiredStack_[index]);
      ++index;
    }
  }
  
  // Now, we should have b = [a, some tasks], we remove the remaining tasks.
  while (index < currentStack.size())
  {
    sot_->removeByTaskName(currentStack[index]);
    erase(currentStack, currentStack[index]);
  }
}


void Superviser::display()
{
  if (sot_)
    std::cout << "+-SOT set    "     << std::endl;
  else
    std::cout << "+-SOT not set  "     << std::endl;
  for (unsigned i =0; i<desiredStack_.size(); ++i)
    std::cout << "| " << desiredStack_[i] <<std::endl;
}

void Superviser::
addSolverFromName( const std::string & solverName )
{
  sot_ = dynamic_cast< dg::sot::dyninv::SolverKine * > (&(dg::PoolStorage::getInstance()->getEntity(solverName)));
  if( sot_  == 0x0)
    std::cerr << "Unable to link with the solver" << std::endl;
}

void Superviser::initCommands()
{
  namespace dc = ::dynamicgraph::command;
  addCommand("clear",
    dc::makeCommandVoid0(*this,&Superviser::clear,
    "clear the list of desired task"));

  addCommand("push",
    dc::makeCommandVoid1(*this,&Superviser::push,
    "add a task at the list of desired stack"));

  addCommand("update",
    dc::makeCommandVoid0(*this,&Superviser::update,
    "replace the content of the stack by the desired list of task"));

  addCommand("disp",
    dc::makeCommandVoid0(*this,&Superviser::display,
    "display"));

  addCommand("setSoT",
    dc::makeCommandVoid1(*this,&Superviser::addSolverFromName,
    "set the solver"));
}


