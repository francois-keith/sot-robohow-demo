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

#ifndef __SOT_COM_ESTIMATOR_H__
#define __SOT_COM_ESTIMATOR_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (com_estimator_EXPORTS)
#    define SOTCircleMotion_EXPORT __declspec(dllexport)
#  else
#    define SOTCircleMotion_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCircleMotion_EXPORT
#endif

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

class SOTCircleMotion_EXPORT CircleMotion
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */

  CircleMotion( const std::string& name );
  virtual ~CircleMotion( void );

 public: /* --- SIGNAL --- */

  dg::SignalTimeDependent<MatrixHomogeneous,int> desiredHand;
  dg::SignalPtr<ml::Vector,int> qSIN;

 public: /* --- FUNCTIONS --- */
  MatrixHomogeneous& computeHand( MatrixHomogeneous& res, const int& time );
};


} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_ANGLE_ESTIMATOR_H__

