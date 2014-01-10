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
  , angleSIN(NULL,"sotCylinderPouring("+name+")::input(vector)::angle")
  , volumeSOUT( boost::bind(&CylinderPouring::computeRemainingVolume,this,_1,_2),
                  angleSIN,
               "sotCylinderPouring("+name+")::output(double)::volume" )
  , thetaTargetSOUT(NULL,"sotCylinderPouring("+name+")::output(double)::targetAngle")
  , volume_(0)
  , height_(0)
  , radius_(0)
{
  sotDEBUGIN(5);

  signalRegistration(angleSIN << volumeSOUT << thetaTargetSOUT);

  initCommands();
  sotDEBUGOUT(5);
}


CylinderPouring::
~CylinderPouring( void )
{
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */



// The volume remaining in an inclined container is:
// V = V_cylinder + V_cone
// ==> V = (h - h2) pi r² + pi r² h2/3, with h2 = 2r tan (theta)
double CylinderPouring::computeAngleToPour(double volumeDes)
{
  sotDEBUGIN(15);

  // compute the remaining quantity of liquid
  double volumeRemaining = std::max (volume_ - volumeDes, 0.) * 1e-3;

  // compute the desired delta theta
  double theta;

  // the end of the bottle : the content is formulated as a cone only
  if (volumeRemaining <= 1/3.* pi<double>() * radius_ * radius_ * height_)
  {
    // compute the radius of the cone
    double r_mini = sqrt(3*volumeRemaining / (pi<double>() * height_));
    theta = atan2(height_, 2 * r_mini);
  }
  // the content can be formulated as a cone + a cylinder
  else
  {
    double pirr =  pi<double>() * radius_ * radius_;
    double tanTheta = 3./(4.*radius_) * (height_ - volumeRemaining / pirr);
    theta = atan (tanTheta);
  }

  return theta;
}

double CylinderPouring::computeRemainingVolume(double angle) const
{
  double remainingVolume;

  // Starting from thetaLimit, the volume of the bottle can be discribed by a cone.
  double tanTheta = tan (angle);
  if (tanTheta > height_ /(2*radius_))
  {
    // compute the radius of the cone
    double r_mini = height_ / (2*tanTheta);
    remainingVolume = r_mini * r_mini * (pi<double>() * height_) / 3;
  }

  // the content can be formulated as a cone + a cylinder
  else
  {
    double pirr =  pi<double>() * radius_ * radius_;
    remainingVolume = ((height_ - (4.*radius_)/3. * tanTheta) * pirr);
  }

  // put it into litres
  remainingVolume *= 1e3;
  remainingVolume = std::min(remainingVolume, volume_);
  return remainingVolume;
}

double& CylinderPouring::computeRemainingVolume(double&  res, const int & time)
{
  sotDEBUGIN(15);
  double theta = angleSIN(time)(0);
  res = volume_ = computeRemainingVolume(theta);
  return res;
}

// The maximal volume than can be contained in the cylinder (Litre)
double CylinderPouring::computeMaxVolume() const
{
  return pi<double>() * radius_ * radius_ * height_ * 1e3;
}

void CylinderPouring::setHeight(const double & h)
{ height_ = h; }

void CylinderPouring::setRadius(const double & r)
{ radius_ = r; }

void CylinderPouring::setVolume(const double & v)
{ volume_ = std::max(0., std::min(v, computeMaxVolume())); }

void CylinderPouring::display()
{
  std::cout << " height: " << height_ << ", radius: " << radius_
     << ", Volume: (" << volume_ << " / " << computeMaxVolume() << ") L " << std::endl;
}

void CylinderPouring::pour(const double & v)
{
  double volumeAvaiblable = std::max(0., std::min(v, computeMaxVolume()));
  thetaTargetSOUT.setConstant(computeAngleToPour(volumeAvaiblable));
  thetaTargetSOUT.setReady();

  return ;
}

void CylinderPouring::test2()
{
  for(unsigned i=0; i<=50; ++i)
  {
    double vol = i *0.01;
    double angle = computeAngleToPour(vol);
    double vol2 =  computeRemainingVolume(angle);
    std::cout << volume_ << " desired " << (volume_ - vol) << " angle " << angle << " volCheck " << vol2 << std::endl;
  }
  return;
}

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

  addCommand("pour",
    dc::makeCommandVoid1(*this,&CylinderPouring::pour,
    "Computes the required inclinaison angle to pour the given volume(L)"));

  addCommand("display",
    dc::makeCommandVoid0(*this,&CylinderPouring::display,
    "Display the info of the Cylinder"));
  addCommand("test",
    dc::makeCommandVoid0(*this,&CylinderPouring::test2,
    "Test..."));
}


