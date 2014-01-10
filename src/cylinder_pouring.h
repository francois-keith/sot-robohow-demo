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

#ifndef __SOT_CYLINDER_POURING_H__
#define __SOT_CYLINDER_POURING_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/matrix-homogeneous.hh>

/* STD */
#include <string>


namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! \brief this class implements a cylinder container.
*		based on the desired volume of liquid to be poured, this computes the 
*		corresponding angle.
*/

class CylinderPouring
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */

  CylinderPouring( const std::string& name );
  virtual ~CylinderPouring( void );

 public: /* --- SIGNAL --- */

  // Angle between the normal of the cylinder and the gound z axis
  dg::SignalPtr<ml::Vector,int> angleSIN;

  // volume remaining in the bottle
  // max (current volume in the bottle, volume possible due to inclination)
  // Litre
  dg::SignalTimeDependent<double,int> volumeSOUT;

  // required angle to pour a given quantity of liquid
  dg::SignalPtr<double,int> thetaTargetSOUT;

 public: /* --- FUNCTIONS --- */
  double & computeTheta( double& res, const int& time );

  //
  double computeRemainingVolume(double angle) const;
  double& computeRemainingVolume(double&  res, const int & time);
  double computeAngleToPour(double volumeDes);

  void pour(const double & volume);
  void test2 ();

  void printVolume();

  double computeMaxVolume() const;
  void setHeight(const double & h);
  void setRadius(const double & r);
  void setVolume(const double & v);
  void initCommands();

  void display();
 private:

  // volume of liquid.
  double volume_;

  // height
  double height_;

  double radius_;
};


} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_ANGLE_ESTIMATOR_H__

