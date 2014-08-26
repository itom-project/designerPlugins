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

#ifndef ITOMFIGURE_H
#define ITOMFIGURE_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMGVPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOMGVPLOT_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjFigure.h"
#include "plotWidget.h"

#include <qsharedpointer.h>
#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#include <qaction.h>
#include <qwidget.h>
#include <qwidgetaction.h>
#if QT_VERSION >= 0x050000
#include <QtWidgets/qlabel.h>
#endif

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif

class ITOMGVPLOT_EXPORT GraphicViewPlot : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(int colorMode READ getColorMode WRITE setColorMode RESET resetColorMode USER true)
    Q_PROPERTY(bool colorBarVisible READ colorBarVisible WRITE setColorBarVisible DESIGNABLE true USER true)
    Q_PROPERTY(QString colorMap READ getColorMap WRITE setColorMap DESIGNABLE true USER true)

    Q_PROPERTY(bool xAxisFlipped READ getxAxisFlipped WRITE setxAxisFlipped USER true)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped USER true)

    //Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    //Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel USER true)
    //Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible USER true)
    //Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel USER true)
    //Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible USER true)
    //Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)

    //Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    //Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    //Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)

    Q_CLASSINFO("prop://colorMode", "Defines color handling.")
    Q_CLASSINFO("prop://colorBarVisible", "Defines whether the color bar should be visible.")
    Q_CLASSINFO("prop://colorMap", "Defines which color map should be used [e.g. grayMarked, hotIron].")

    Q_CLASSINFO("prop://xAxisFlipped", "Sets whether x-axis should be flipped (default: false, zero is at the left side).")
    Q_CLASSINFO("prop://yAxisFlipped", "Sets whether y-axis should be flipped (default: true, zero is at the top).")
    
    //Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    //Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis or '<auto>' if the description from the data object should be used.")
    //Q_CLASSINFO("prop://xAxisVisible", "Sets visibility of the x-axis.")
    //Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis or '<auto>' if the description from the data object should be used.")
    //Q_CLASSINFO("prop://yAxisVisible", "Sets visibility of the y-axis.")
    //Q_CLASSINFO("prop://valueLabel", "Label of the value axis or '<auto>' if the description should be used from data object.")
    //Q_CLASSINFO("prop://titleFont", "Font for title.")
    //Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    //Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")

    DESIGNER_PLUGIN_ITOM_API

    public:
        GraphicViewPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        ~GraphicViewPlot();

        //properties
        ito::RetVal displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID);

        ito::RetVal applyUpdate();  //!> does the real update work

        QSharedPointer<ito::DataObject> getSource(void) const;
        QSharedPointer<ito::DataObject> getDisplayed(void);

        virtual inline void setOutpBounds(QVector<QPointF> bounds)
        {
            double *pointArr = new double[2 * bounds.size()];
            for (int np = 0; np < bounds.size(); np++)
            {
                pointArr[np * 2] = bounds[np].x();
                pointArr[np * 2 + 1] = bounds[np].y();
            }
            m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
            delete[] pointArr;
        }

        void setCoordinates(const QVector<QPointF> pts, const bool visible);

        //properties (setter/getter)
        void setContextMenuEnabled(bool show);
        bool getContextMenuEnabled() const;

        bool colorBarVisible() const;
        void setColorBarVisible(bool value);

        QString getTitle() const;
        void setTitle(const QString &title);
        void resetTitle();

        QString getxAxisLabel() const;
        void setxAxisLabel(const QString &label);
        void resetxAxisLabel();

        QString getyAxisLabel() const;
        void setyAxisLabel(const QString &label);
        void resetyAxisLabel();

        QString getValueLabel() const;
        void setValueLabel(const QString &label);
        void resetValueLabel();

        bool getxAxisVisible() const;
        void setxAxisVisible(const bool &value);

        bool getyAxisVisible() const;
        void setyAxisVisible(const bool &value);

        bool getxAxisFlipped() const;
        void setxAxisFlipped(const bool &value);

        bool getyAxisFlipped() const;
        void setyAxisFlipped(const bool &value);

        QString getColorMap() const;
        void setColorMap(const QString &name);

        void setPlaneRange(int min, int max);
        void setCmplxSwitch(PlotWidget::ComplexType type, bool visible);

        virtual QPointF getXAxisInterval(void) const;
        virtual void setXAxisInterval(QPointF point);

        virtual QPointF getYAxisInterval(void) const;
        virtual void setYAxisInterval(QPointF point);

        virtual QPointF getZAxisInterval(void) const;
        virtual void setZAxisInterval(QPointF point);

        QFont getTitleFont(void) const;
        void setTitleFont(const QFont &font);

        QFont getLabelFont(void) const;
        void setLabelFont(const QFont &font);

        QFont getAxisFont(void) const;
        void setAxisFont(const QFont &font);

        int getColorMode(void) const {return m_data.m_colorMode;}
        void setColorMode(const int type);
        void resetColorMode(void);

        QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

        friend class PlotWidget;

    protected:
        ito::RetVal init()
        {
            if(!m_pContent) return ito::retError;
            else return m_pContent->init();
        } //called when api-pointers are transmitted, directly after construction
        PlotWidget *m_pContent;

        void enableComplexGUI(const bool checked);
        void enableZStackGUI(const bool checked);

        void setPaletteText(const QString newPalette)
        {
            m_pPaletteRep->setText(newPalette);
        }

    private:
        void createActions();

        QAction  *m_pActScaleSetting;

        QAction  *m_pActSave;
        QAction  *m_pActHome;

        QAction  *m_pActPan;
        QAction  *m_pActZoomToRect;
        QAction  *m_pActValuePicker;
        QAction  *m_pActLineCut;

        QAction  *m_pActPalette;
        QAction  *m_pActToggleColorBar;

        QAction  *m_pActAScan;
        QWidgetAction  *m_pActPlaneSelector;
        QAction  *m_pActCmplxSwitch;
        QMenu    *m_pMnuCmplxSwitch;

        QAction  *m_pActAspectSwitch;
        QMenu    *m_pMnuAspectSwitch;

        QAction  *m_pActColorSwitch;
        QMenu    *m_pMnuColorSwitch;

        QLabel   *m_pPaletteRep;
        QAction  *m_curPalette;

        QPixmap  m_pixMap;

        QLabel *m_lblCoordinates;

        QAction *m_pActProperties;

        InternalData m_data;

        ito::RetVal exportCanvas(const bool exportType, const QString &fileName, QSizeF curSize = QSizeF(0.0,0.0), const int resolution = 300);

    signals:

    private slots:
        void mnuHome();
        void mnuPanner(bool checked);
        void mnuPalette();
        void mnuValuePicker(bool checked);
        void mnuAScanPicker(bool checked);
        void mnuLinePicker(bool checked);
        void mnuScaleSetting();
        void mnuColorBar(bool checked);
        void mnuCmplxSwitch(QAction *action);
        void mnuAspectSwitch(QAction *action);
        void mnuZoomer(bool checked);
        void mnuActSave();
        void mnuSwitchColorMode(QAction *action);
        void mnuActPlaneSelector(int plane);

    public slots:
        ito::RetVal copyToClipBoard();

};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMFIGURE_H
