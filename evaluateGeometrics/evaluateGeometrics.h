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

#ifndef ITOMPLOT_H
#define ITOMPLOT_H

#include "plot/AbstractDObjFigure.h"

#include "plotTreeWidget.h"

#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>

#include <qaction.h>

#include <qsharedpointer.h>
#include <qwidget.h>

Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)


class EvaluateGeometricsFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString valueUnit READ getValueUnit WRITE setValueUnit RESET resetValueUnit)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont)
    Q_PROPERTY(QSharedPointer<ito::DataObject> relations READ getRelations WRITE setRelations DESIGNABLE false)
    Q_PROPERTY(QString destinationFolder READ getDestinationFolder WRITE setDestinationFolder DESIGNABLE true)
    

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://valueUnit", "The value unit for the metrical calculations that is used within the plot.")
    Q_CLASSINFO("prop://titleFont", "Font for title (toDo).")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions (toDo).")
    Q_CLASSINFO("prop://relations", "Get or set geometric elements via N x 11 dataObject of type float32.")
    Q_CLASSINFO("prop://destinationFolder", "Set a default export directory.")

    public:
        EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~EvaluateGeometricsFigure();

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree

        void setData(QSharedPointer<ito::DataObject> dataObj) 
        {
            if(m_pContent != NULL)
            {
                m_pContent->refreshPlot(dataObj.data());
            }
        };
       // QSharedPointer<ito::DataObject> getData() const {return };

        //properties
        bool getContextMenuEnabled() const;
        void setContextMenuEnabled(bool show); 

        //QVector<QPointF> getBounds(void);
        //void setBounds(QVector<QPointF> bounds);

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

        QStringList getRelationNames(void) const 
        {
            return m_info.m_relationNames;
        }

        void setRelationNames(const QStringList input)
        {
            if(input.size() < 7)
            {
                return;
            }

            if(m_info.m_relationNames.length() < input.size())
            {
                m_info.m_relationNames.reserve(input.length());
            }

            for( int i = 6; i < input.length(); i++)
            {
                if(m_info.m_relationNames.length() > i)
                {
                    m_info.m_relationNames[i] = input[i];
                }
                else
                {
                    m_info.m_relationNames.append(input[i]);
                }
            }

            while(m_info.m_relationNames.length() > input.size())
            {
                m_info.m_relationNames.removeLast();
            }
            return;
        }


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

        void clearRelation(const bool apply)
        {
            m_info.m_relationsList.clear();
        }

        void setDestinationFolder(const QString folder) {m_lastFolder = folder;}
        QString getDestinationFolder() const {return m_lastFolder;};

        void setSource(QSharedPointer<ito::DataObject> source);
    
    protected:
        ito::RetVal init() { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

        PlotTreeWidget *m_pContent;

    private:

        InternalInfo m_info;

        QAction * m_actScaleSetting;
		QAction *m_actSave;
        QMenu   *m_mnuSaveSwitch;

        QString m_lastFolder;

    public slots:
        void mnuScaleSetting();
        void mnuExport(QAction* action);
        
        ito::RetVal addRelation(QSharedPointer<ito::DataObject> importedData);
        //ito::RetVal addRelation(QVector<ito::float64> importedData);
        ito::RetVal exportData(QString fileName, ito::uint8 exportFlag);

        ito::RetVal plotItemChanged(ito::int32 idx, ito::int32 flags, QVector<ito::float32> values);

    private slots:
        
        void mnuHome();
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOT_H
