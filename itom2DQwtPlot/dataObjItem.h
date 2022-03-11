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


#ifndef ITO_PLOT_SPECTROGRAM_H
#define ITO_PLOT_SPECTROGRAM_H

#include <qwt_plot_spectrogram.h>
#include <qelapsedtimer.h>
#include "common/sharedStructuresGraphics.h"



class DataObjItem: public QwtPlotSpectrogram
{
public:

    explicit DataObjItem( const QString &title = QString() );
    virtual ~DataObjItem();
    virtual QPen contourPen(double level) const;
    void setContourPalette(const ito::ItomPalette &palette);


protected:
    QImage renderImage(const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &area, const QSize &imageSize ) const;
    void renderTile(const QwtScaleMap &xMap, const QwtScaleMap &yMap, const char dataTypeFlag, const QRect &tile, QImage *image ) const;

    int m_counter;

private:
    ito::ItomPalette m_pContourPalette;

};

#endif
