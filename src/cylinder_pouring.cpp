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

#include "cylinder_pouring.h"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <jrl/mal/matrixabstractlayer.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CylinderPouring,"CylinderPouring");

// use boost definition of pi
#include <boost/math/constants/constants.hpp>
using boost::math::constants::pi;

CylinderPouring::
CylinderPouring( const std::string & name )
  :Entity(name)
  , volumeDesSIN(NULL,"sotCylinderPouring("+name+")::input()::volume")
  , angle( boost::bind(&CylinderPouring::computeTheta,this,_1,_2),
                  volumeDesSIN,
               "sotCylinderPouring("+name+")::output(Vector)::angle" )

  , volume_(0)
  , height_(0)
  , radius_(0)
{
  sotDEBUGIN(5);

  signalRegistration(volumeDesSIN);
  signalRegistration(angle);

  initCommands();
  sotDEBUGOUT(5);
}


CylinderPouring::
~CylinderPouring( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */



// The volume remaining in an inclined container is:
// V = V_cylinder + V_cone
// ==> V = (h - h2) pi r² + pi r² h2/3, with h2 = 2r tan (theta)

double &
CylinderPouring::computeTheta(double& res, const int& time )
{
  sotDEBUGIN(15);
//  double angle = angleSIN(time);
  double volumeDes = volumeDesSIN(time);

  // compute the remaining quantity of liquid
  double volumeRemaining = std::max (volume_ - volumeDes, 0.);

  // compute the desired delta theta
  double theta;

  // the end of the bottle : the content is formulated as a cone only
  if (volumeRemaining <= 2./3.* pi<double>() * radius_ * radius_ * height_)
  {
    // compute the radius of the cone
    double r_mini = sqrt(3*volumeRemaining / (2 * pi<double>() * height_));
    theta = atan2(height_, 2 * r_mini);
  }
  // the content can be formulated as a cone + a cylinder
  else
  {
    double tanTheta = 3/(2*radius_) * (height_ - volumeRemaining / (pi<double>() * radius_ * radius_) );
    theta = atan (tanTheta);
  }

  res = theta;

  sotDEBUGOUT(15);
  return res;
}


void CylinderPouring::setHeight(const double & h)
{ height_ = h; }

void CylinderPouring::setRadius(const double & r)
{ radius_ = r; }

void CylinderPouring::setVolume(const double & v)
{ volume_ = std::max(0., std::min(v, pi<double>() * radius_ * radius_ * height_)); }

void CylinderPouring::initCommands()
{
  namespace dc = ::dynamicgraph::command;
  addCommand("setHeight",
    dc::makeCommandVoid1(*this,&CylinderPouring::setHeight,
    "set the height of the container"));

  addCommand("setRadius",
    dc::makeCommandVoid1(*this,&CylinderPouring::setRadius,
    "set the radius of the container"));

  addCommand("setVolume",
    dc::makeCommandVoid1(*this,&CylinderPouring::setVolume,
    "set volume of liquid contained in the container"));
}


