/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
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

#ifndef ITOMQWTPLOT_H
#define ITOMQWTPLOT_H

#include <qwt_plot.h>

class QMenu;
class ItomPlotZoomer;
class ItomPlotMagnifier;
class QwtPlotPanner;
class QContextMenuEvent;
class QResizeEvent;

class ItomQwtPlot : public QwtPlot
{
    Q_OBJECT

public:
    explicit ItomQwtPlot(QMenu *contextMenu, QWidget * parent = NULL);
    virtual ~ItomQwtPlot();

    bool showContextMenu() const { return m_showContextMenu; }
    void setShowContextMenu(bool value) { m_showContextMenu = value;  }
    
    bool keepAspectRatio() const { return m_keepAspectRatio; }
    void setKeepAspectRatio(bool keep);

    void setVisible(bool visible);


public Q_SLOTS:


protected:
    void loadStyles();
    ItomPlotZoomer *zoomer() const;
    QwtPlotPanner *panner() const;
    void configRescaler();

    /*
    @param doReplot forces a replot of the content
    @param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
    */
    virtual void updateScaleValues(bool doReplot = true, bool doZoomBase = true) = 0; //to be overwritten in every plot.

    virtual void contextMenuEvent(QContextMenuEvent * event);
    virtual void resizeEvent(QResizeEvent * event);

private:
    QMenu *m_pContextMenu;
    bool m_showContextMenu;

    ItomPlotZoomer *m_pZoomer;
    ItomPlotMagnifier *m_pMagnifier;
    QwtPlotPanner *m_pPanner;

    bool m_keepAspectRatio;
    bool m_firstTimeVisible; /*!< true if this plot becomes visible for the first time */

signals:
    void statusBarClear();
    void statusBarMessage(const QString &message, int timeout = 0);
};

#endif //ITOMQWTPLOT_H