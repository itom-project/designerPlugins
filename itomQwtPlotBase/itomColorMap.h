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

#ifndef ITOMCOLORMAP_H
#define ITOMCOLORMAP_H

#include "itomQwtPlotBase.h"

#include <qwt_color_map.h>

#include "itomQwtPlotEnums.h"

class ITOMQWTPLOTBASE_EXPORT ItomColorMap : public QwtColorMap
{
public:
	/*!
	Mode of color map
	\sa setMode(), mode()
	*/
	enum Mode
	{
		//! Return the color from the next lower color stop
		FixedColors,

		//! Interpolating the colors of the adjacent stops.
		ScaledColors
	};

	ItomColorMap(const ItomQwtPlotEnums::ScaleEngine &scale = ItomQwtPlotEnums::Linear, QwtColorMap::Format = QwtColorMap::RGB);
	ItomColorMap(const QColor &color1, const QColor &color2, const ItomQwtPlotEnums::ScaleEngine &scale = ItomQwtPlotEnums::Linear, QwtColorMap::Format = QwtColorMap::RGB);

	virtual ~ItomColorMap();

	void setScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scale);
	ItomQwtPlotEnums::ScaleEngine scaleEngine() const;

	void setMode(Mode);
	Mode mode() const;

	void setColorInterval(const QColor &color1, const QColor &color2);
	void addColorStop(double value, const QColor&);
	QVector<double> colorStops() const;

	QColor color1() const;
	QColor color2() const;

	virtual QRgb rgb(const QwtInterval &, double value) const;
	virtual uint colorIndex(int numColors,
        const QwtInterval& interval, double value) const;
    virtual QVector<QRgb> colorTable256() const;
    virtual QVector< QRgb > colorTable(int numColors) const;

	class ColorStops;

private:
	// Disabled copy constructor and operator=
	ItomColorMap(const ItomColorMap &);
	ItomColorMap &operator=(const ItomColorMap &);

	class PrivateData;
	PrivateData *d_data;
};

#endif