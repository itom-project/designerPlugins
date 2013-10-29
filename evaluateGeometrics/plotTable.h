/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO), 
   Universität Stuttgart, Germany 
 
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

#ifndef PLOTTABLE_H
#define PLOTTABLE_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresPrimitives.h"
#include "DataObject/dataobj.h"

#include <qtabwidget.h>
#include <qtablewidget.h>
#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qmenu.h>

class EvaluateGeomatricsFigure;

struct InternalInfo
{
    bool m_autoTitle;
    QString m_title;
    bool m_autoAxisLabel;
    QString m_axisLabel;
    bool m_autoValueLabel;
    QString m_valueLabel;

    QString titleLabel;
};

class PlotTable : public QTabWidget
{
    Q_OBJECT

        friend class EvaluateGeometricsFigure;

    public:
        enum MultiLineMode { FirstRow, FirstCol, MultiRows, MultiCols };
        enum State { stateIdle, statePanner, stateZoomer, statePicker };

        PlotTable(QMenu *contextMenu, QWidget * parent = 0);
        ~PlotTable();

        ito::RetVal init();

        bool m_showContextMenu;
        void refreshPlot(const ito::DataObject* dataObj);

        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);
        void setPannerEnable(const bool checked);

    protected:
        /*
        void keyPressEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

        void setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel);
        */
        void updateScaleValues(bool recalculateBoundaries = false);

        void updateLabels();

    private:

        QMenu *m_contextMenu;

        bool m_xDirect;
        bool m_yDirect;
        bool m_cmplxState;

        QWidget *m_pParent;

        QTableWidget * m_geometrics;
        QTableWidget * m_relations;

		QMenu *m_pCmplxMenu;

        State m_state;

        ito::PrimitiveContainer m_data;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        //void replot();

};


#endif // PLOTTABLE_H
