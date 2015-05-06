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
#include <qhash.h>
#include <qstringlist.h>

#include <qfileinfo.h>

class EvaluateGeomatricsFigure;

/*!
* \struct relationsShip
* \brief  This struct defines the geometric elements.
*
* \sa PlotTreeWidget, EvaluateGeometrics, PlotTreeWidget::tMeasurementType
*/

struct relationsShip
{
    relationsShip() : firstElementIdx(0), type(0), secondElementIdx(0), myWidget(NULL), extValue(0) {}
    ~relationsShip() {}

    ito::int32 firstElementIdx;     /*!< The index of the first geometric element. The relations will be a child of this element.  */
    ito::uint32 type;               /*!< The type of the relations as defiend by PlotTreeWidget::tMeasurementType.  */
    ito::int32 secondElementIdx;    /*!< The index of the second geometric element. For some measurement types, this is ignored  */
//    ito::int32 firstElementRow;   
//    ito::int32 secondElementRow;
    QTreeWidgetItem* myWidget;      /*!< Handle to the childWidget of PlotTreeWidget, which will contain the data for this relation  */
    ito::float32 extValue;          /*!< An external value or the result from the last caluclation.  */
};

/*!
* \struct InternalInfo
* \brief  This struct contains most of the shared data for PlotTreeWidget and EvaluateGeometrics.
*
* \sa PlotTreeWidget, EvaluateGeometrics, relationsShip
*/

struct InternalInfo
{
    InternalInfo()
    {
        m_autoTitle = false;
        m_title = "";
        m_valueUnit = "";
        m_titleLabel = "";

        m_numberOfDigits = 2;
        m_consider2DOnly = false;
        m_coordsAs3D = true;
        //m_info.m_rowHash.clear();
        m_relationNames.clear();
        m_relationNames.append("N.A.");
        m_relationNames.append("radius (own)");
        m_relationNames.append("angle to");
        m_relationNames.append("distance to");
        m_relationNames.append("intersection with");
        m_relationNames.append("length (own)");
        m_relationNames.append("area");
/*        m_relationNames.append(QObject::tr("N.A."));
        m_relationNames.append(QObject::tr("radius (own)"));
        m_relationNames.append(QObject::tr("angle to"));
        m_relationNames.append(QObject::tr("distance to"));
        m_relationNames.append(QObject::tr("intersection with"));
        m_relationNames.append(QObject::tr("length (own)"));
        m_relationNames.append(QObject::tr("area"));*/

        m_primitivNames.clear();
        m_primitivNames.insert(0, "none");
        m_primitivNames.insert(ito::tPoint, QObject::tr("point"));
        m_primitivNames.insert(ito::tLine, QObject::tr("line"));
        m_primitivNames.insert(ito::tEllipse, QObject::tr("ellipse"));
        m_primitivNames.insert(ito::tCircle, QObject::tr("circle"));
        m_primitivNames.insert(ito::tRectangle, QObject::tr("rectangle"));
        m_primitivNames.insert(ito::tSquare, QObject::tr("square"));
        m_primitivNames.insert(ito::tPolygon, QObject::tr("polygon"));

        static char const* primitivNames[] = {"none", "point", "line", "elipse", "circle", "rectangle", "square", "err", "err", "polygon"};

        m_relationsList.clear();
        m_rowPrintSpacing = 2;
        m_tpPrintSpacing = 2;
        m_columnPrintSpacing = 2;
    }
    ~InternalInfo() { }

    bool m_autoTitle;                               /*!< If true, the table will get a title one day */
    QString m_title;                                /*!< If m_autoTitle == false, this title will be used one day.  */
    QString m_valueUnit;                            /*!< The value unit for all caluclations */
    QString m_titleLabel;                           /*!< A title labeld which will be used on day */
    QHash<ito::uint16, QString> m_primitivNames;     /*!< A hashTable containing all possible primites type in it */
    QStringList m_relationNames;                    /*!< A list with the relation names to be plotted. First 6 are protected other can be added but should be used with external defined values */
    QVector<relationsShip> m_relationsList;         /*!< A list with all relations to be evaluated for the figure*/
    //QHash<int, ito::GeometricPrimitive> m_rowHash;    /*!< A hashList with all geometric elements to be evaluated for the figure*/
    ito::uint8 m_numberOfDigits;                    /*!< Number of digits to be plotted */
    bool m_consider2DOnly;                          /*!< Toggle wether only x and y or all coordinates of primitivs should be considered for evaluation of relations */
    bool m_coordsAs3D;                              /*!< Toggle wether only x and y or all coordinates of primitivs should be shown */
    ito::int32 m_rowPrintSpacing;                    /*!< Number of pixels for each row pixmap export*/
    ito::int32 m_tpPrintSpacing;                    /*!< Number of pixels for each row pixmap export*/
    ito::int32 m_columnPrintSpacing;                    /*!< Number of pixels for each column pixmap export */
};

/*!
* \class PlotTreeWidget
* \brief  The main functionality of the evaluate geometric widget is realized here.
* \detail The PlotTreeWidget visualisated the geometric primitives and the relations between different relativs.
*         It is inheritad from QTreeWidget.
*
* \sa EvaluateGeometricsPlugin, EvaluateGeometrics
*/

class PlotTreeWidget : public QTreeWidget
{
    Q_OBJECT

        friend class EvaluateGeometricsFigure;

    public:
        enum State { stateIdle, statePanner, stateZoomer, statePicker };    /*!< Statemaschine currently not used */

        PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent = 0); /*!< Class constructor */
        ~PlotTreeWidget();                                                            /*!< Class destructur */

        ito::RetVal init();                                                           /*!< Initialisation function for this subwidget */

        enum tMeasurementType                                                         /*!< Enumeration containing list of different relationship types. For dynamic adding of relations use a new number correspondig to new name and add tExtern to the type.*/
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

        bool m_showContextMenu;                                                       /*!< Toggle wether context menu should be accessable or not */
        void refreshPlot(const ito::DataObject* dataObj);                             /*!< Refresh plot with new primitive list defiend by a dataObject */

        //ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);
        //void setZoomerEnable(const bool checked);   
        //void setPickerEnable(const bool checked);
        //void setPannerEnable(const bool checked);

        void setRelations(QSharedPointer<ito::DataObject> relations);                 /*! set a set of relations and overwrite old relations */
        QSharedPointer<ito::DataObject> getRelations(void) const;                     /*! get all relations of this widget */
        void addRelation(const QVector<QPointF> relation);                            /*! add a single relations, multiple relation setting is possible */
        void clearRelation(const bool apply);                                         /*! clear the relations list and be happy */

    protected:
        static double quietNaN;
        /*
        void keyPressEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

        void setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel);
        */
        void updatePrimitives();                                                      /*! force a replot of all primitives and its relations */

        void autoFitCols();                                                           /*! calculate the idle width of the widget columns */
    private:

        void updateRelationShips(const bool fastUpdate);                              /*! force an update of all relation ships. */
        void setPrimitivElement(const int row, const bool update, ito::float32 *val); /*! Set a primitiv element in row number row */

        bool calculateAngle(ito::float32 *first, ito::float32 *second, const bool eval2D, ito::float32 &angle);         /*! calculate the angle between to elements */
        bool calculateDistance(ito::float32 *first, ito::float32 *second, const bool eval2D, ito::float32 &distance);   /*! calculate the distance between to elements */
        bool calculateRadius(ito::float32 *first, ito::float32 &radius);                                                /*! calculate the radius of a sigle circle or ellipse */
        bool calculateLength(ito::float32 *first, const bool eval2D, ito::float32 &length);                             /*! calculate the length of a line */
        //bool calculateIntersections(ito::float32 *first, ito::float32 *second, const bool eval2D, cv::Vec3f &point);  /*! calculate the intersection point of two lines */
        bool calculateArea(ito::float32 *first, const bool eval2D, ito::float32 &area);                                 /*! calculate the area of a geometric element */
        bool calculateCircumference(ito::float32 *first, ito::float32 &length);                                         /*! calculate the circumference of a circle, rectangle or square */

        ito::RetVal writeToCSV(const QFileInfo &QFileInfo, const bool asTable = false);                                 /*! export content to comma seperated values */
        ito::RetVal writeToXML(const QFileInfo &fileName);                                                              /*! export content to XML-style file */
        ito::RetVal writeToRAW(const QFileInfo &fileName);                                                              /*! export raw content as two tables */

        ito::RetVal m_lastRetVal;                                                                                       /*! last retval for error checking, currently not used */

        QMenu *m_contextMenu;                                                                                           /*! handle to the internal context menu */

        QWidget *m_pParent;                                                                                             /*! handle to the parent widget of this (see EvaluateGeometrics) */

        State m_state;                                                                                                  /*! current state for the state maschine */

        InternalInfo *m_pData;                                                                                          /*! handle to the shared configuration struct */

        //QVector<ito::GeometricPrimitive> m_rowHash;
        QHash<ito::int32, ito::GeometricPrimitive> m_rowHash;                                                               /*! the hash table with data for all primitives */

        ito::RetVal updateElement(const ito::int32 &idx, const ito::int32 &flags,const QVector<ito::float32> &values);  /*! update a certain element of the hash table */

    signals:
        //void spawnNewChild(QVector<QPointF>);
        //void updateChildren(QVector<QPointF>);

    public slots:

        
        //void replot();

};


#endif // PlotTreeWidget_H
