/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
   Universitaet Stuttgart, Germany

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

#include "itomColorMap.h"
#include "qwt_interval.h"
#include "qwt_math.h"
#include <limits>
#include <qmath.h>
#include <qnumeric.h>
#include <qvector.h>


//----------------------------------------------------------------------------------------
class ItomColorMap::ColorStops
{
public:
    ColorStops() : d_doAlpha(false)
    {
        d_stops.reserve(256);
    }

    void insert(double pos, const QColor& color);
    QRgb rgb(ItomColorMap::Mode, double pos) const;

    QVector<double> stops() const;

private:
    class ColorStop
    {
    public:
        ColorStop() : pos(0.0), rgb(0){};

        ColorStop(double p, const QColor& c) : pos(p), rgb(c.rgba())
        {
            r = qRed(rgb);
            g = qGreen(rgb);
            b = qBlue(rgb);
            a = qAlpha(rgb);

            /*
            when mapping a value to rgb we will have to calcualate:
            - const int v = int( ( s1.v0 + ratio * s1.vStep ) + 0.5 );

            Thus adding 0.5 ( for rounding ) can be done in advance
            */
            r0 = r + 0.5;
            g0 = g + 0.5;
            b0 = b + 0.5;
            a0 = a + 0.5;

            rStep = gStep = bStep = aStep = 0.0;
            posStep = 0.0;
        }

        void updateSteps(const ColorStop& nextStop)
        {
            rStep = nextStop.r - r;
            gStep = nextStop.g - g;
            bStep = nextStop.b - b;
            aStep = nextStop.a - a;
            posStep = nextStop.pos - pos;
        }

        double pos;
        QRgb rgb;
        int r, g, b, a;

        // precalculated values
        double rStep, gStep, bStep, aStep;
        double r0, g0, b0, a0;
        double posStep;
    };

    inline int findUpper(double pos) const;
    QVector<ColorStop> d_stops;
    bool d_doAlpha;
};

//----------------------------------------------------------------------------------------
void ItomColorMap::ColorStops::insert(double pos, const QColor& color)
{
    // Lookups need to be very fast, insertions are not so important.
    // Anyway, a balanced tree is what we need here. TODO ...

    if (pos < 0.0 || pos > 1.0)
        return;

    int index;
    if (d_stops.size() == 0)
    {
        index = 0;
        d_stops.resize(1);
    }
    else
    {
        index = findUpper(pos);
        if (index == d_stops.size() || qAbs(d_stops[index].pos - pos) >= 0.001)
        {
            d_stops.resize(d_stops.size() + 1);
            for (int i = d_stops.size() - 1; i > index; i--)
                d_stops[i] = d_stops[i - 1];
        }
    }

    d_stops[index] = ColorStop(pos, color);
    if (color.alpha() != 255)
        d_doAlpha = true;

    if (index > 0)
        d_stops[index - 1].updateSteps(d_stops[index]);

    if (index < d_stops.size() - 1)
        d_stops[index].updateSteps(d_stops[index + 1]);
}

//----------------------------------------------------------------------------------------
inline QVector<double> ItomColorMap::ColorStops::stops() const
{
    QVector<double> positions(d_stops.size());
    for (int i = 0; i < d_stops.size(); i++)
        positions[i] = d_stops[i].pos;
    return positions;
}

//----------------------------------------------------------------------------------------
inline int ItomColorMap::ColorStops::findUpper(double pos) const
{
    int index = 0;
    int n = d_stops.size();

    const ColorStop* stops = d_stops.data();

    while (n > 0)
    {
        const int half = n >> 1;
        const int middle = index + half;

        if (stops[middle].pos <= pos)
        {
            index = middle + 1;
            n -= half + 1;
        }
        else
            n = half;
    }

    return index;
}

//----------------------------------------------------------------------------------------
inline QRgb ItomColorMap::ColorStops::rgb(ItomColorMap::Mode mode, double pos) const
{
    // 03/09/17 ck, retrieving d_stops.size() - 1 might crash
    if (d_stops.size() < 1)
        return qRgb(128, 128, 128);

    if (pos <= 0.0)
        return d_stops[0].rgb;
    if (pos >= 1.0)
    {
        return d_stops[d_stops.size() - 1].rgb;
    }

    int index = findUpper(pos);
    // 03/09/17 ck, we got a crash here, when pos was a #nan
    if (index == 0)
        index = 1;

    if (mode == FixedColors)
    {
        return d_stops[index - 1].rgb;
    }
    else
    {
        const ColorStop& s1 = d_stops[index - 1];

        const double ratio = (pos - s1.pos) / (s1.posStep);

        const int r = int(s1.r0 + ratio * s1.rStep);
        const int g = int(s1.g0 + ratio * s1.gStep);
        const int b = int(s1.b0 + ratio * s1.bStep);

        if (d_doAlpha)
        {
            if (s1.aStep)
            {
                const int a = int(s1.a0 + ratio * s1.aStep);
                return qRgba(r, g, b, a);
            }
            else
            {
                return qRgba(r, g, b, s1.a);
            }
        }
        else
        {
            return qRgb(r, g, b);
        }
    }
}


class ItomColorMap::PrivateData
{
public:
    ColorStops colorStops;
    ItomColorMap::Mode mode;
    ItomQwtPlotEnums::ScaleEngine scale;
    double invlog2;
    double invlog10;
    double invlog16;
};


//-----------------------------------------------------------------------
/*!
Build a color map with two stops at 0.0 and 1.0. The color
at 0.0 is Qt::blue, at 1.0 it is Qt::yellow.

\param format Preferred format of the color map
*/
ItomColorMap::ItomColorMap(
    const ItomQwtPlotEnums::ScaleEngine& scale /*= ItomQwtPlotEnums::Linear*/,
    QwtColorMap::Format format /*= QwtColorMap::RGB*/) :
    QwtColorMap(format)
{
    d_data = new PrivateData;
    d_data->mode = ScaledColors;
    d_data->scale = scale;

    d_data->invlog2 = 1.0 / qLn(2.0);
    d_data->invlog10 = 1.0 / qLn(10.0);
    d_data->invlog16 = 1.0 / qLn(16.0);

    setColorInterval(Qt::blue, Qt::yellow);
}

//-----------------------------------------------------------------------
/*!
Build a color map with two stops at 0.0 and 1.0.

\param color1 Color used for the minimum value of the value interval
\param color2 Color used for the maximum value of the value interval
\param format Preferred format for the color map
*/
ItomColorMap::ItomColorMap(
    const QColor& color1,
    const QColor& color2,
    const ItomQwtPlotEnums::ScaleEngine& scale /*= ItomQwtPlotEnums::Linear*/,
    QwtColorMap::Format format /*= QwtColorMap::RGB*/) :
    QwtColorMap(format)
{
    d_data = new PrivateData;
    d_data->mode = ScaledColors;
    d_data->scale = scale;

    d_data->invlog2 = 1.0 / qLn(2.0);
    d_data->invlog10 = 1.0 / qLn(10.0);
    d_data->invlog16 = 1.0 / qLn(16.0);

    setColorInterval(color1, color2);
}

//-----------------------------------------------------------------------
//! Destructor
ItomColorMap::~ItomColorMap()
{
    delete d_data;
}

//-----------------------------------------------------------------------
void ItomColorMap::setScaleEngine(const ItomQwtPlotEnums::ScaleEngine& scale)
{
    d_data->scale = scale;
}

//-----------------------------------------------------------------------
ItomQwtPlotEnums::ScaleEngine ItomColorMap::scaleEngine() const
{
    return d_data->scale;
}

/*!
\brief Set the mode of the color map

FixedColors means the color is calculated from the next lower
color stop. ScaledColors means the color is calculated
by interpolating the colors of the adjacent stops.

\sa mode()
*/
void ItomColorMap::setMode(Mode mode)
{
    d_data->mode = mode;
}

/*!
\return Mode of the color map
\sa setMode()
*/
ItomColorMap::Mode ItomColorMap::mode() const
{
    return d_data->mode;
}

/*!
Set the color range

Add stops at 0.0 and 1.0.

\param color1 Color used for the minimum value of the value interval
\param color2 Color used for the maximum value of the value interval

\sa color1(), color2()
*/
void ItomColorMap::setColorInterval(const QColor& color1, const QColor& color2)
{
    d_data->colorStops = ColorStops();
    d_data->colorStops.insert(0.0, color1);
    d_data->colorStops.insert(1.0, color2);
}

/*!
Add a color stop

The value has to be in the range [0.0, 1.0].
F.e. a stop at position 17.0 for a range [10.0,20.0] must be
passed as: (17.0 - 10.0) / (20.0 - 10.0)

\param value Value between [0.0, 1.0]
\param color Color stop
*/
void ItomColorMap::addColorStop(double value, const QColor& color)
{
    if (value >= 0.0 && value <= 1.0)
        d_data->colorStops.insert(value, color);
}

/*!
\return Positions of color stops in increasing order
*/
QVector<double> ItomColorMap::colorStops() const
{
    return d_data->colorStops.stops();
}

/*!
\return the first color of the color range
\sa setColorInterval()
*/
QColor ItomColorMap::color1() const
{
    return QColor(d_data->colorStops.rgb(d_data->mode, 0.0));
}

/*!
\return the second color of the color range
\sa setColorInterval()
*/
QColor ItomColorMap::color2() const
{
    return QColor(d_data->colorStops.rgb(d_data->mode, 1.0));
}

/*!
Map a value of a given interval into a RGB value

\param interval Range for all values
\param value Value to map into a RGB value

\return RGB value for value
*/
QRgb ItomColorMap::rgb(const QwtInterval& interval, double value) const
{
    if (qIsNaN(value))
        return 0u;

    if (interval.width() <= 0.0)
        return 0u;

    double minimum = interval.minValue();
    double maximum = interval.maxValue();

    // in logarithmic scales no value <= 0 is allowed
    if (d_data->scale != ItomQwtPlotEnums::Linear)
    {
        if ((int)d_data->scale < (int)ItomQwtPlotEnums::LogLog2)
        {
            if (minimum <= 0)
            {
                minimum = std::numeric_limits<double>::epsilon();
            }
            if (value <= 0 || maximum <= 0)
            {
                return 0u;
            }
        }
        else
        {
            if (minimum <= 1)
            {
                minimum = 1 + std::numeric_limits<double>::epsilon();
            }
            if (value <= 1 || maximum <= 1)
            {
                return 0u;
            }
        }
    }

    switch (d_data->scale)
    {
    case ItomQwtPlotEnums::Log2:
        minimum = qLn(minimum) * d_data->invlog2;
        maximum = qLn(maximum) * d_data->invlog2;
        value = qLn(value) * d_data->invlog2;
        break;
    case ItomQwtPlotEnums::Log10:
        minimum = qLn(minimum) * d_data->invlog10;
        maximum = qLn(maximum) * d_data->invlog10;
        value = qLn(value) * d_data->invlog10;
        break;
    case ItomQwtPlotEnums::Log16:
        minimum = qLn(minimum) * d_data->invlog16;
        maximum = qLn(maximum) * d_data->invlog16;
        value = qLn(value) * d_data->invlog16;
        break;
    case ItomQwtPlotEnums::LogLog2:
        minimum = qLn((qLn(minimum) * d_data->invlog2) * d_data->invlog2);
        maximum = qLn((qLn(maximum) * d_data->invlog2) * d_data->invlog2);
        value = qLn((qLn(value) * d_data->invlog2) * d_data->invlog2);
        break;
    case ItomQwtPlotEnums::LogLog10:
        minimum = qLn((qLn(minimum) * d_data->invlog10) * d_data->invlog10);
        maximum = qLn((qLn(maximum) * d_data->invlog10) * d_data->invlog10);
        value = qLn((qLn(value) * d_data->invlog10) * d_data->invlog10);
        break;
    case ItomQwtPlotEnums::LogLog16:
        minimum = qLn((qLn(minimum) * d_data->invlog16) * d_data->invlog16);
        maximum = qLn((qLn(maximum) * d_data->invlog16) * d_data->invlog16);
        value = qLn((qLn(value) * d_data->invlog16) * d_data->invlog16);
        break;
    }

    const double ratio = (value - minimum) / (maximum - minimum);

    return d_data->colorStops.rgb(d_data->mode, ratio);
}

/*!
   \brief Map a value of a given interval into a color index

   \param numColors Number of colors
   \param interval Range for all values
   \param value Value to map into a color index

   \return Index, between 0 and numColors - 1, or -1 for an invalid value
 */
uint ItomColorMap::colorIndex(int numColors, const QwtInterval& interval, double value) const
{
    if (qIsNaN(value) || interval.width() <= 0.0)
        return 0;

    double minimum = interval.minValue();
    double maximum = interval.maxValue();

    // in logarithmic scales no value <= 0 is allowed
    if (d_data->scale != ItomQwtPlotEnums::Linear)
    {
        if ((int)d_data->scale < (int)ItomQwtPlotEnums::LogLog2)
        {
            if (minimum <= 0)
            {
                minimum = std::numeric_limits<double>::epsilon();
            }
            if (value <= 0 || maximum <= 0)
            {
                return 0;
            }
        }
        else
        {
            if (minimum <= 1)
            {
                minimum = 1 + std::numeric_limits<double>::epsilon();
            }
            if (value <= 1 || maximum <= 1)
            {
                return 0;
            }
        }
    }

    if (value <= interval.minValue())
        return 0;

    const int maxIndex = numColors - 1;
    if (value >= interval.maxValue())
        return maxIndex;

    switch (d_data->scale)
    {
    case ItomQwtPlotEnums::Log2:
        minimum = qLn(minimum) * d_data->invlog2;
        maximum = qLn(maximum) * d_data->invlog2;
        value = qLn(value) * d_data->invlog2;
        break;
    case ItomQwtPlotEnums::Log10:
        minimum = qLn(minimum) * d_data->invlog10;
        maximum = qLn(maximum) * d_data->invlog10;
        value = qLn(value) * d_data->invlog10;
        break;
    case ItomQwtPlotEnums::Log16:
        minimum = qLn(minimum) * d_data->invlog16;
        maximum = qLn(maximum) * d_data->invlog16;
        value = qLn(value) * d_data->invlog16;
        break;
    case ItomQwtPlotEnums::LogLog2:
        minimum = qLn((qLn(minimum) * d_data->invlog2) * d_data->invlog2);
        maximum = qLn((qLn(maximum) * d_data->invlog2) * d_data->invlog2);
        value = qLn((qLn(value) * d_data->invlog2) * d_data->invlog2);
        break;
    case ItomQwtPlotEnums::LogLog10:
        minimum = qLn((qLn(minimum) * d_data->invlog10) * d_data->invlog10);
        maximum = qLn((qLn(maximum) * d_data->invlog10) * d_data->invlog10);
        value = qLn((qLn(value) * d_data->invlog10) * d_data->invlog10);
        break;
    case ItomQwtPlotEnums::LogLog16:
        minimum = qLn((qLn(minimum) * d_data->invlog16) * d_data->invlog16);
        maximum = qLn((qLn(maximum) * d_data->invlog16) * d_data->invlog16);
        value = qLn((qLn(value) * d_data->invlog16) * d_data->invlog16);
        break;
    }

    const double ratio = (value - minimum) / (maximum - minimum);

    uint index;
    if (d_data->mode == FixedColors)
    {
        index = static_cast<uint>(ratio * maxIndex); // always floor
    }
    else
    {
        index = static_cast<uint>(ratio * maxIndex + 0.5);
    }

    return index;
}


//-------------------------------------------------------------------------------------------
/*!
   Build and return a color map of 256 colors

   The color table is needed for rendering indexed images in combination
   with using colorIndex().

   \return A color table, that can be used for a QImage
 */
QVector<QRgb> ItomColorMap::colorTable256() const
{
    QVector<QRgb> table(256);

    switch (d_data->scale)
    {
    case ItomQwtPlotEnums::Log2:
    case ItomQwtPlotEnums::Log10:
    case ItomQwtPlotEnums::Log16:
    case ItomQwtPlotEnums::LogLog2:
    case ItomQwtPlotEnums::LogLog10:
    case ItomQwtPlotEnums::LogLog16: {
        const QwtInterval interval(std::exp(0.0), std::exp(256.0 / 10.0));
        double ratio = 0.0;

        for (int i = 0; i < 256; i++)
        {
            ratio = static_cast<double>(i) / 256.0;
            table[i] = d_data->colorStops.rgb(d_data->mode, ratio);
        }
    }
    break;
    default: {
        const QwtInterval interval(0, 255);

        for (int i = 0; i < 256; i++)
        {
            table[i] = rgb(interval, i);
        }
    }
    break;
    }

    return table;
}

//-------------------------------------------------------------------------------------------
/*!
   Build and return a color map of arbitrary number of colors

   The color table is needed for rendering indexed images in combination
   with using colorIndex().

   \param numColors Number of colors
   \return A color table
 */
QVector<QRgb> ItomColorMap::colorTable(int numColors) const
{
    QVector<QRgb> table(numColors);

    switch (d_data->scale)
    {
    case ItomQwtPlotEnums::Log2:
    case ItomQwtPlotEnums::Log10:
    case ItomQwtPlotEnums::Log16:
    case ItomQwtPlotEnums::LogLog2:
    case ItomQwtPlotEnums::LogLog10:
    case ItomQwtPlotEnums::LogLog16: {
        const QwtInterval interval(std::exp(0.0), std::exp(1.0));
        double ratio = 0.0;

        for (int i = 0; i < numColors; i++)
        {
            ratio = static_cast<double>(i) / numColors;
            table[i] = d_data->colorStops.rgb(d_data->mode, ratio);
        }
    }
    break;
    default: {
        const QwtInterval interval(0.0, 1.0);

        const double step = 1.0 / (numColors - 1);
        for (int i = 0; i < numColors; i++)
        {
            table[i] = rgb(interval, step * i);
        }
    }
    break;
    }

    return table;
}
