/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic.
 * sot-dynamic is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "circle_motion.h"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

#include <jrl/mal/matrixabstractlayer.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CircleMotion,"CircleMotion");

CircleMotion::
CircleMotion( const std::string & name )
  :Entity(name)

  ,desiredHand( boost::bind(&CircleMotion::computeHand,this,_1,_2),
                    qSIN,
               "sotCircleMotion("+name+")::output(Vector)::com" )

  ,qSIN(NULL,"sotCircleMotion("+name
                             +")::input()::q")
{
  sotDEBUGIN(5);

  signalRegistration(qSIN);
  signalRegistration(desiredHand);

  sotDEBUGOUT(5);
}


CircleMotion::
~CircleMotion( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
MatrixHomogeneous& CircleMotion::
computeHand(MatrixHomogeneous& res,
	       const int& time )
{
  sotDEBUGIN(15);
  double t = time;
  qSIN(time);

  res.setIdentity();
  res.elementAt(0,3) = 0.4+0.2*sin(t * 0.005);
  res.elementAt(1,3) = 0.2+0.2*cos(t * 0.005);
  res.elementAt(2,3) = 0.7;

  sotDEBUGOUT(15);
  return res;
}
