/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut f�r Technische Optik (ITO), 
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

#ifndef PLOTTREEWIDGET_H
#define PLOTTREEWIDGET_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresPrimitives.h"
#include "DataObject/dataobj.h"


#include <qtreewidget.h>
#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qmenu.h>
#include <qvector.h>
#include <qstringlist.h>

#include <qfileinfo.h>


class EvaluateGeomatricsFigure;

static char const* primitivNames[] = {"none", "point", "line", "elipse", "circle", "retangle", "square", "err", "err", "polygon"};

struct relationsShip
{
    ito::int32 firstElementIdx;
    ito::uint32 type;
    ito::int32 secondElementIdx;
    ito::int32 firstElementRow;
    ito::int32 secondElementRow;
    QTreeWidgetItem* myWidget;
    ito::float32 extValue;
};

struct InternalInfo
{
    bool m_autoTitle;
    QString m_title;
    bool m_autoValueUnit;
    QString m_valueUnit;
    QString titleLabel;
    QStringList m_relationNames;
    QVector<relationsShip> m_relationsList;
};

class PlotTreeWidget : public QTreeWidget
{
    Q_OBJECT

        friend class EvaluateGeometricsFigure;

    public:
        enum State { stateIdle, statePanner, stateZoomer, statePicker };

        PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent = 0);
        ~PlotTreeWidget();

        ito::RetVal init();

        enum tMeasurementType
        {
            tNoType       =   0,
            tRadius       =   1,
            tAngle        =   2,
            tDistance     =   3,
            tIntersection =   4,
            tLength       =   5,
            tArea         =   6,
            tProtected    =   0x4000,
            tExtern       =   0x8000
        }; 

        bool m_showContextMenu;
        void refreshPlot(const ito::DataObject* dataObj);

        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);
        void setPannerEnable(const bool checked);

        void setRelations(QSharedPointer<ito::DataObject> relations);
        QSharedPointer<ito::DataObject> getRelations(void) const;
        void addRelation(const QVector<QPointF> relation);
        void clearRelation(const bool apply);

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

        void updateRelationShips(const bool fastUpdate);
        void setPrimitivElement(const int row, const bool update, ito::float32 *val);

        bool calculateAngle(ito::float32 *first, ito::float32 *second, ito::float32 &angle);
        bool calculateDistance(ito::float32 *first, ito::float32 *second, ito::float32 &distance);
        bool calculateRadius(ito::float32 *first, ito::float32 &radius);
        bool calculateLength(ito::float32 *first, ito::float32 &length);
        bool calculateIntersections(ito::float32 *first, ito::float32 *second, cv::Vec3f &point);

        ito::RetVal writeToCSV(const QFileInfo &QFileInfo, const bool asTable = false);
        ito::RetVal writeToXML(const QFileInfo &fileName);
        ito::RetVal writeToRAW(const QFileInfo &fileName);

        ito::RetVal m_lastRetVal;

        QMenu *m_contextMenu;

        bool m_xDirect;
        bool m_yDirect;

        QWidget *m_pParent;

        State m_state;

        InternalInfo *m_pData;

        QVector<geometricPrimitives> m_rowHash;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:

        
        //void replot();

};


#endif // PlotTreeWidget_H
