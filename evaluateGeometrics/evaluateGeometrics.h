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

/*!
 * \file evaluateGeometrics.h
 * \brief This file contains basic container for the evaluateGeometrics-Widget.
 */

#ifndef ITOMEVALUATEGEOMETRICS_H
#define ITOMEVALUATEGEOMETRICS_H

#if defined(ITOMSHAREDDESIGNER)
    #define EVALUATEGEOMETRICS_EXPORT Q_DECL_EXPORT
#else
    #define EVALUATEGEOMETRICS_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractFigure.h"
#include "common/shape.h"
#include "DataObject/dataobj.h"

#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qaction.h>
#include <qsharedpointer.h>
#include <qwidget.h>

/*!
* \class EvaluateGeometricsFigure
* \brief The container class for the evaluate geometric widget.
* \detail This container contains the toolbar, context menu, setter- / getter-functions for the properties and slots.
*          It manages most of the relevant communication to the itom surface. For visualisation, it contains a plotTreeWidget.
*
* \sa EvaluateGeometricsPlugin, PlotTreeWidget
*/

class PlotTreeWidget;
struct InternalInfo;

class EVALUATEGEOMETRICS_EXPORT EvaluateGeometricsFigure : public ito::AbstractFigure
{
    Q_OBJECT

    Q_PROPERTY(QString valueUnit READ getValueUnit WRITE setValueUnit RESET resetValueUnit)
    Q_PROPERTY(QSharedPointer<ito::DataObject> relations READ getRelations WRITE setRelations DESIGNABLE false)
    Q_PROPERTY(QStringList relationNames READ getRelationNames WRITE setRelationNames DESIGNABLE true)
    Q_PROPERTY(QString destinationFolder READ getDestinationFolder WRITE setDestinationFolder DESIGNABLE true)
    Q_PROPERTY(int lastAddedRelation READ getLastRelation DESIGNABLE true)
    Q_PROPERTY(int numberOfDigits READ getNumberOfDigits WRITE setNumberOfDigits DESIGNABLE true)
    Q_PROPERTY(int printRowSpacing READ getPrintRowSpacing WRITE setPrintRowSpacing DESIGNABLE true)
    Q_PROPERTY(int printTopLevbelSpacing READ getPrintTopLevelRowSpacing WRITE setPrintTopLevelRowSpacing DESIGNABLE true)
    Q_PROPERTY(int printColumnSpacing READ getPrintColumnSpacing WRITE setPrintColumnSpacing DESIGNABLE true)
    Q_PROPERTY(QVector<ito::Shape> geometricShapes READ getGeometricShapes WRITE setGeometricShapes DESIGNABLE true)


    Q_CLASSINFO("prop://valueUnit", "The value unit for the metrical calculations that is used within the plot.")
    Q_CLASSINFO("prop://relations", "Get or set geometric elements via N x 11 dataObject of type float32.")
    Q_CLASSINFO("prop://destinationFolder", "Set a default export directory.")
    Q_CLASSINFO("prop://relationNames", "A string list with the names of possible relation. The first elements [N.A., radius, angle to, distance to, intersection with, length and area] are read only and are calculated with these widget. For external calculated values you can define custom names e.g. roughness.")
    Q_CLASSINFO("prop://lastAddedRelation", "Get the index of the last added relation.")
    Q_CLASSINFO("prop://numberOfDigits", "Define the number of digits to be displayed.")
    Q_CLASSINFO("prop://printRowSpacing", "Define the number of pixels between to rows when printed.")
    Q_CLASSINFO("prop://printTopLevbelSpacing", "Define the number of pixels after a top level element when printed.")
    Q_CLASSINFO("prop://printColumnSpacing", "Define the number of pixels between to columns when printed.")
    Q_CLASSINFO("prop://geometricShape", "Set / get a list of all geometric shapes.")

    Q_CLASSINFO("slot://addRelation", "Add a set of relations via dataObject")
    Q_CLASSINFO("slot://modifyRelation", "Change a single relation")
    Q_CLASSINFO("slot://addRelationName", "Add a relation name to the internal relation list")
    Q_CLASSINFO("slot://exportData", "Export data to file")
    Q_CLASSINFO("slot://geometricShapeChanged", "Slot for c++ internal communication between a plot an this widget")
    Q_CLASSINFO("slot://clearAll", "Delete all relations and geometric elements")

    Q_CLASSINFO("signal://fitToObject", "This signal is emitted when the users requests a fit via context menu\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "currentItem : {int}\n"
    "    currently selected item in the table (index), which should be fitted")

    DESIGNER_PLUGIN_ITOM_API

    public:
        EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~EvaluateGeometricsFigure();

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree
        ito::RetVal update(void) { return ito::retOk;  };

       // QSharedPointer<ito::DataObject> getData() const {return };

        //properties
        bool getContextMenuEnabled() const;
        void setContextMenuEnabled(bool show);

        QVector<ito::Shape> getGeometricShapes() const;
        void setGeometricShapes(QVector<ito::Shape> shapes);

        QString getValueUnit() const;
        void setValueUnit(const QString &label);
        void resetValueUnit();

        QStringList getRelationNames(void) const;

        void setRelationNames(const QStringList input);

        void setRelations(QSharedPointer<ito::DataObject> relations);

        QSharedPointer<ito::DataObject> getRelations(void) const;
        QSharedPointer<ito::DataObject> getCurrentRelation(void) const;

        int getNumberOfDigits() const;
        void setNumberOfDigits(const int val);

        //QSharedPointer<ito::DataObject> readLastRelation(void) {return QSharedPointer<ito::DataObject>(new ito::DataObject());}

        enum exportFlags
        {
            exportCSVTree  = 0x00,
            exportCSVTable = 0x01,
            exportXMLTree  = 0x02,
            exportCSVList  = 0x03,
            showExportWindow = 0x10
        };

        void clearRelation(const bool apply);

        void setDestinationFolder(const QString folder) {m_lastFolder = folder;}
        QString getDestinationFolder() const {return m_lastFolder;}

        int getLastRelation(void) const {return m_lastAddedRelation;}
        int getCurrentItem(void);
        ito::RetVal delRelation(const int idx);                 //> remove a relation
        QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

        int getPrintRowSpacing(void) const;
        void setPrintRowSpacing(const int val);
        int getPrintTopLevelRowSpacing(void) const;
        void setPrintTopLevelRowSpacing(const int val);
        int getPrintColumnSpacing(void) const;
        void setPrintColumnSpacing(const int val);
        ito::Shape getShape(const int idx);                     //> return shape with specific index

    protected:
        ito::RetVal init();
        PlotTreeWidget *m_pContent;

    private:
        InternalInfo* m_pInfo;

        QAction *m_actSetting;
        QAction *m_actSave;
        QAction *m_actAddRel;
        QAction *m_actRemoveRel;
        QAction *m_actUpdate;
        QAction *m_actAutoFitCols;
        QAction *m_actFitToObject;
        QMenu   *m_mnuSaveSwitch;

        QString m_lastFolder;

        int m_lastAddedRelation;

    public slots:
        ito::RetVal addRelation(QSharedPointer<ito::DataObject> relation);
        ito::RetVal modifyRelation(const int idx, QSharedPointer<ito::DataObject> relation);

        ito::RetVal addRelationName(const QString newName);

        ito::RetVal exportData(QString fileName, int exportFlag);

        ito::RetVal geometricShapeChanged(int idx, ito::Shape shape);
        ito::RetVal clearAll(void);

        void mnuAutoFitCols();
        void mnuUpdate();

    private slots:
        void mnuSetting();
        void mnuExport(QAction* action);
        void mnuAddRelation();
        void mnuDeleteRelation();
        void mnuFitToObject();

    signals:
        void fitToObject(const int currentItem);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMEVALUATEGEOMETRICS_H
