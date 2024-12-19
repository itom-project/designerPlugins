/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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
#include "common/shape.h"
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
* \struct relationShip
* \brief  This struct defines the geometric elements.
*
* \sa PlotTreeWidget, EvaluateGeometricsFigure, PlotTreeWidget::tMeasurementType
*/

struct relationShip
{
    relationShip() : firstElementIdx(0), type(0), secondElementIdx(0), myWidget(NULL), extValue(0) {}
    relationShip(ito::int32 firstIdx, ito::int32 secondIdx, ito::uint32 type)
        : firstElementIdx(firstIdx), type(type), secondElementIdx(secondIdx), myWidget(NULL), extValue(0) {}
    ~relationShip() {}

    ito::int32 firstElementIdx;     /*!< The index of the first geometric element. The relations will be a child of this element.  */
    ito::uint32 type;               /*!< The type of the relations as defined by PlotTreeWidget::tMeasurementType.  */
    ito::int32 secondElementIdx;    /*!< The index of the second geometric element. For some measurement types, this is ignored  */
    QTreeWidgetItem* myWidget;      /*!< Handle to the childWidget of PlotTreeWidget, which will contain the data for this relation  */
    ito::float32 extValue;          /*!< An external value or the result from the last calculation.  */
};

/*!
* \struct InternalInfo
* \brief  This struct contains most of the shared data for PlotTreeWidget and EvaluateGeometricsFigure.
*
* \sa PlotTreeWidget, EvaluateGeometricsFigure, relationShip
*/

struct InternalInfo
{
    InternalInfo()
    {
        m_valueUnit = "";
        m_numberOfDigits = 2;
        m_relationNames.clear();
        m_relationNames.append("N.A.");
        m_relationNames.append("radius (own)");
        m_relationNames.append("angle to");
        m_relationNames.append("distance to");
        m_relationNames.append("intersection with");
        m_relationNames.append("length (own)");
        m_relationNames.append("area");

        m_shapeTypeNames.clear();
        m_shapeTypeNames.insert(0, "none");
        m_shapeTypeNames.insert(ito::Shape::Point, QObject::tr("point"));
        m_shapeTypeNames.insert(ito::Shape::Line, QObject::tr("line"));
        m_shapeTypeNames.insert(ito::Shape::Ellipse, QObject::tr("ellipse"));
        m_shapeTypeNames.insert(ito::Shape::Circle, QObject::tr("circle"));
        m_shapeTypeNames.insert(ito::Shape::Rectangle, QObject::tr("rectangle"));
        m_shapeTypeNames.insert(ito::Shape::Square, QObject::tr("square"));
        m_shapeTypeNames.insert(ito::Shape::Polygon, QObject::tr("polygon"));

        m_relationsList.clear();
        m_rowPrintSpacing = 2;
        m_tpPrintSpacing = 2;
        m_columnPrintSpacing = 2;
    }
    ~InternalInfo() { }

    QString m_valueUnit;                            /*!< The value unit for all calculations */
    QHash<ito::uint16, QString> m_shapeTypeNames;     /*!< A hashTable containing all possible primites type in it */
    QStringList m_relationNames;                    /*!< A list with the relation names to be plotted. First 6 are protected other can be added but should be used with external defined values */
    QVector<relationShip> m_relationsList;         /*!< A list with all relations to be evaluated for the figure*/
    //QHash<int, geometricPrimitives> m_rowHash;    /*!< A hashList with all geometric elements to be evaluated for the figure*/
    ito::uint8 m_numberOfDigits;                    /*!< Number of digits to be plotted */
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
* \sa EvaluateGeometricsPlugin, EvaluateGeometricsFigure
*/

class PlotTreeWidget : public QTreeWidget
{
    Q_OBJECT

        friend class EvaluateGeometricsFigure;

    public:
        PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent = 0); /*!< Class constructor */
        ~PlotTreeWidget();                                                            /*!< Class destructur */

        ito::RetVal init();                                                           /*!< Initialisation function for this subwidget */

        enum tMeasurementType                                                         /*!< Enumeration containing list of different relationship types. For dynamic adding of relations use a new number corresponding to new name and add tExtern to the type.*/
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

        bool m_showContextMenu;                                                       /*!< Toggle whether context menu should be accessible or not */

    protected:
        static double quietNaN;
        void setShapes(const QVector<ito::Shape> &shapes);
        void updateGeometricShapes();                                                 /*! force a replot of all primitives and its relations */
        void autoFitCols();                                                           /*! calculate the idle width of the widget columns */
        void contextMenuEvent(QContextMenuEvent *event);                              //!> context, i.e. right click menu

    private:
        void updateRelationShips(const bool fastUpdate);                                /*! force an update of all relation ships. */
        void displayShape(const int row, const bool update, const ito::Shape &shape);   /*! Set a primitive element in row number row */

        bool calculateAngle(const ito::Shape &first, const ito::Shape &second, ito::float32 &angle);        /*! calculate the angle between to elements */
        bool calculateDistance(const ito::Shape &first, const ito::Shape &second, ito::float32 &distance);  /*! calculate the distance between to elements */
        bool calculateRadius(const ito::Shape &first, ito::float32 &radius);                                /*! calculate the radius of a single circle or ellipse */
        bool calculateLength(const ito::Shape &first, ito::float32 &length);                                /*! calculate the length of a line */
        //bool calculateIntersections(ito::float32 *first, ito::float32 *second, const bool eval2D, cv::Vec3f &point);  /*! calculate the intersection point of two lines */
        bool calculateArea(const ito::Shape &first, ito::float32 &area);                                 /*! calculate the area of a geometric element */

        ito::RetVal writeToCSV(const QFileInfo &QFileInfo, const bool asTable = false);                                 /*! export content to comma separated values */
        ito::RetVal writeToXML(const QFileInfo &fileName);                                                              /*! export content to XML-style file */
        ito::RetVal writeToRAW(const QFileInfo &fileName);                                                              /*! export raw content as two tables */

        ito::RetVal m_lastRetVal;                                                                                       /*! last retval for error checking, currently not used */
        QMenu *m_contextMenu;                                                                                           /*! handle to the internal context menu */
        QWidget *m_pParent;                                                                                             /*! handle to the parent widget of this (see EvaluateGeometricsFigure) */
        State m_state;                                                                                                  /*! current state for the state machine */
        InternalInfo *m_pData;                                                                                          /*! handle to the shared configuration struct */
        QHash<ito::int32, ito::Shape> m_rowHash;                                                                        /*! the hash table with data for all geometric shapes */
        ito::RetVal updateElement(const ito::int32 &idx, const ito::Shape &shape);                                      /*! update a certain element of the hash table */

    signals:

    public slots :
        void itemPressed(QTreeWidgetItem *item, int column);
};


#endif // PlotTreeWidget_H
