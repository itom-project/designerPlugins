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

    //Q_PROPERTY(QSharedPointer<ito::DataObject> data /*READ getData */ WRITE setData DESIGNABLE false);
    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont)
    Q_PROPERTY(QSharedPointer<ito::DataObject> appendRelation READ readLastRelation WRITE addRelation DESIGNABLE false)
    Q_PROPERTY(QSharedPointer<ito::DataObject> relations READ getRelations WRITE setRelations DESIGNABLE false)
    Q_PROPERTY(QString destinationFolder READ getDestinationFolder WRITE setDestinationFolder DESIGNABLE true)
    

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://axisLabel", "Label of the direction (x/y) axis or '<auto>' if the descriptions from the data object should be used.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis (y-axis) or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")

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

        QString getAxisLabel() const;
        void setAxisLabel(const QString &label);
        void resetAxisLabel();

        QString getValueLabel() const;
        void setValueLabel(const QString &label);
        void resetValueLabel();

        QPointF getYAxisInterval(void) const;
        void setYAxisInterval(QPointF);

        QFont getTitleFont(void) const;
        void setTitleFont(const QFont &font);

        QFont getLabelFont(void) const;
        void setLabelFont(const QFont &font);

        QFont getAxisFont(void) const;
        void setAxisFont(const QFont &font);

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

        QSharedPointer<ito::DataObject> readLastRelation(void) {return QSharedPointer<ito::DataObject>(new ito::DataObject());}
        void addRelation(QSharedPointer<ito::DataObject> importedData);

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

        QAction* m_actScaleSetting;

		QAction *m_actSave;
        QString m_lastFolder;

    public slots:
        void mnuScaleSetting();
        void mnuExport();
        
    private slots:
        void mnuHome();
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOT_H
