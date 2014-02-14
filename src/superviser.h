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

#ifndef __SOT_SUPERVISER_H__
#define __SOT_SUPERVISER_H__

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
#include <sot-dyninv/solver-kine.h>

/* STD */
#include <list>
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

class Superviser
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */

  Superviser( const std::string& name );
  virtual ~Superviser( void );

 public: /* --- FUNCTIONS --- */

  void clear();
  void push(const std::string & name);
  void update();
  void initCommands();
  void display();

  void addSolverFromName( const std::string & solverName );

 private:
  dg::sot::dyninv::SolverKine * sot_;
  std::vector<std::string> desiredStack_;
};


} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_ANGLE_ESTIMATOR_H__

