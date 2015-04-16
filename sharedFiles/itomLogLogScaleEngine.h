/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
   This file is part of itom.

   itom is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   itom is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef ITOMLOGLOGSCALEENGINE
#define ITOMLOGLOGSCALEENGINE

#include <qwt_scale_engine.h>
//#include "qwt_global.h"
#include <qwt_transform.h>

/*!
  \brief A scale engine for logarithmic scales

  The step size is measured in *decades*
  and the major step size will be adjusted to fit the pattern
  \f$\left\{ 1,2,3,5\right\} \cdot 10^{n}\f$, where n is a natural number
  including zero.

  \warning the step size as well as the margins are measured in *decades*.
*/

class ItomLogLogScaleEngine: public QwtScaleEngine
{
public:
    ItomLogLogScaleEngine( uint base = 10 );
    virtual ~ItomLogLogScaleEngine();

    virtual void autoScale( int maxSteps,
        double &x1, double &x2, double &stepSize ) const;

    virtual QwtScaleDiv divideScale( double x1, double x2,
        int numMajorSteps, int numMinorSteps,
        double stepSize = 0.0 ) const;

protected:
    QwtInterval align( const QwtInterval&, double stepSize ) const;

    void buildTicks(
        const QwtInterval &, double stepSize, int maxMinSteps,
        QList<double> ticks[QwtScaleDiv::NTickTypes] ) const;

    QList<double> buildMajorTicks(
        const QwtInterval &interval, double stepSize ) const;

    void buildMinorTicks( const QList<double>& majorTicks,
        int maxMinorSteps, double stepSize,
        QList<double> &minorTicks, QList<double> &mediumTicks ) const;
};

class ItomTransform: public QwtTransform
{
public:
    ItomTransform();
    virtual ~ItomTransform();

    /*!
       Modify value to be a valid value for the transformation.
       The default implementation does nothing.
     */
    virtual double bounded( double value ) const;

    /*!
        Transformation function

        \param value Value
        \return Modified value

        \sa invTransform()
     */
    virtual double transform( double value ) const = 0;

    /*!
        Inverse transformation function

        \param value Value
        \return Modified value

        \sa transform()
     */
    virtual double invTransform( double value ) const = 0;

    //! Virtualized copy operation
    virtual ItomTransform *copy() const = 0;
};

/*!
   \brief Null transformation

   QwtNullTransform returns the values unmodified.
   
 */
class ItomNullTransform: public ItomTransform
{
public:
    ItomNullTransform();
    virtual ~ItomNullTransform();

    virtual double transform( double value ) const;
    virtual double invTransform( double value ) const;

    virtual ItomTransform *copy() const;
};
/*!
   \brief Logarithmic transformation

   QwtLogTransform modifies the values using log() and exp().

   \note In the calculations of QwtScaleMap the base of the log function
         has no effect on the mapping. So QwtLogTransform can be used 
         for log2(), log10() or any other logarithmic scale.
 */
class ItomLogLogTransform: public ItomTransform
{   
public:
    ItomLogLogTransform();
    virtual ~ItomLogLogTransform();
    
    virtual double transform( double value ) const;
    virtual double invTransform( double value ) const;

    virtual double bounded( double value ) const;

    virtual ItomTransform *copy() const;

#if QT_VERSION >= 0x050400
    static const double LogMin;
    static const double LogMax;
#else
    QT_STATIC_CONST double LogMin;
    QT_STATIC_CONST double LogMax;
#endif
};

/*!
   \brief A transformation using pow()

   QwtPowerTransform preserves the sign of a value. 
   F.e. a transformation with a factor of 2
   transforms a value of -3 to -9 and v.v. Thus QwtPowerTransform
   can be used for scales including negative values.
 */
class ItomPowerTransform: public ItomTransform
{
public:
    ItomPowerTransform( double exponent );
    virtual ~ItomPowerTransform();

    virtual double transform( double value ) const;
    virtual double invTransform( double value ) const;

    virtual ItomTransform *copy() const;

private:
    const double d_exponent;
};

#endif