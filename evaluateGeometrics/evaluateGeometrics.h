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

/*!
 * \file evaluateGeometrics.h
 * \brief This file contains basic container for the evaluateGeometrics-Widget.
 */

#ifndef ITOMPLOTTREE_H
#define ITOITOMPLOTTREE_HMPLOT_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOM1DPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOM1DPLOT_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjFigure.h"

//#include "plotTreeWidget.h"

#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#include <qstring.h>
#include <qstringlist.h>

#include <qaction.h>

#include <qsharedpointer.h>
#include <qwidget.h>

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif

/*!
* \class EvaluateGeometricsFigure
* \brief The container class for the evaluate geometric widget.
* \detail This container contains the toolbar, context menu, setter- / getter-functions for the properties and slots.
*          It manages most of the relevant communication to the itom surface. For visualisation, it contains a plotTreeWidget.
*
* \sa EvaluateGeometricsPlugin, PlotTreeWidget
*/

class PlotTreeWidget;

class ITOM1DPLOT_EXPORT EvaluateGeometricsFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString valueUnit READ getValueUnit WRITE setValueUnit RESET resetValueUnit)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont)
    Q_PROPERTY(QSharedPointer<ito::DataObject> relations READ getRelations WRITE setRelations DESIGNABLE false)
    Q_PROPERTY(QStringList relationNames READ getRelationNames WRITE setRelationNames DESIGNABLE true)
    Q_PROPERTY(QString destinationFolder READ getDestinationFolder WRITE setDestinationFolder DESIGNABLE true)
    Q_PROPERTY(int lastAddedRelation READ getLastRelation DESIGNABLE true)
    Q_PROPERTY(bool considerOnly2D READ getConsider2dStatus WRITE setConsider2dStatus DESIGNABLE true)
    

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://valueUnit", "The value unit for the metrical calculations that is used within the plot.")
    Q_CLASSINFO("prop://titleFont", "Font for title (toDo).")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions (toDo).")
    Q_CLASSINFO("prop://relations", "Get or set geometric elements via N x 11 dataObject of type float32.")
    Q_CLASSINFO("prop://destinationFolder", "Set a default export directory.")
    Q_CLASSINFO("prop://relationNames", "A string list with the names of possible relation. The first elements [N.A., radius, angle to, distance to, intersection with, length and area] are read only and are calculated with these widget. For external calculated values you can define custom names e.g. roughness.")
    Q_CLASSINFO("prop://lastAddedRelation", "Get the index of the last added relation.")
    Q_CLASSINFO("prop://considerOnly2D",    "If true, only the x & y coordinates are considered.")

    DESIGNER_PLUGIN_ITOM_API

    public:
        EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~EvaluateGeometricsFigure();

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree

        void setData(QSharedPointer<ito::DataObject> dataObj); 

       // QSharedPointer<ito::DataObject> getData() const {return };

        //properties
        bool getContextMenuEnabled() const;
        void setContextMenuEnabled(bool show); 

        void enableComplexGUI(const bool checked);

        QString getTitle() const;
        void setTitle(const QString &title);
        void resetTitle();

        QString getValueUnit() const;
        void setValueUnit(const QString &label);
        void resetValueUnit();

        QPointF getYAxisInterval(void) const;
        void setYAxisInterval(QPointF);

        QFont getTitleFont(void) const;
        void setTitleFont(const QFont &font);

        QFont getLabelFont(void) const;
        void setLabelFont(const QFont &font);

        QStringList getRelationNames(void) const;

        void setRelationNames(const QStringList input);

        void setRelations(QSharedPointer<ito::DataObject> relations);

        QSharedPointer<ito::DataObject> getRelations(void) const;

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

        bool getConsider2dStatus(void) const;
        void setConsider2dStatus(const bool enabled);

        void setSource(QSharedPointer<ito::DataObject> source);
    
    protected:
        ito::RetVal init();

        PlotTreeWidget *m_pContent;

    private:
        void* m_pInfo;

        QAction *m_actSetting;
        QAction *m_actSave;
        QAction *m_actAddRel;
        QAction *m_actRemoveRel;
        QAction *m_actUpdate;
        QAction *m_actAutoFitCols;
        QMenu   *m_mnuSaveSwitch;

        QString m_lastFolder;

        int m_lastAddedRelation;

    public slots:


        ito::RetVal addRelation(QSharedPointer<ito::DataObject> importedData);
        ito::RetVal modifyRelation(const int idx, QSharedPointer<ito::DataObject> relation);

        ito::RetVal addRelationName(const QString newName);

        ito::RetVal exportData(QString fileName, ito::uint8 exportFlag);

        ito::RetVal plotItemChanged(ito::int32 idx, ito::int32 flags, QVector<ito::float32> values);
        ito::RetVal clearAll(void);

    private slots:
        void mnuSetting();
        void mnuExport(QAction* action);
        void mnuAddRelation();
        void mnuDeleteRelation();
        void mnuUpdate();
        void mnuAutoFitCols();

        void mnuHome();
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOTTREE_H
