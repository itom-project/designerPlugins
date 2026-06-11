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

#ifndef ITOMFIGURE_H
#define ITOMFIGURE_H

#include "plot/AbstractDObjPCLFigure.h"
#include "plotIsoGLWidget.h"

#if defined(CONNEXION_FOUND) //&& !_DEBUG
    #ifdef _DEBUG
        #undef _DEBUG
        #define CONNEXION_ENABLE 1
        #include <windows.h>
        #include "spwmacro.h"
        #include "si.h"
        #include "spwmath.h"
        #include "siapp.h"
        #define _DEBUG
    #else
        #define CONNEXION_ENABLE 1
        #include <windows.h>
        #include "spwmacro.h"
        #include "si.h"
        #include "spwmath.h"
        #include "siapp.h"
    #endif
#else
    #define CONNEXION_ENABLE 0
#endif

#include <qsharedpointer.h>
#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#include <qaction.h>
#include <qwidget.h>
#include <qtimer.h>
#include <QtWidgets/qlabel.h>

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif

class ItomIsoGLWidget;
//----------------------------------------------------------------------------------------------------------------------------------
class GL3DEFilter : public QObject
{
    Q_OBJECT

    public:
        GL3DEFilter(ItomIsoGLWidget *plotObj)
            : m_plotObj(plotObj), rotating(false), moving(false) {memset(startPos, 0, sizeof(double)*6); memset(endPos, 0, sizeof(double)*6); }
        ~GL3DEFilter() {}
        virtual bool eventFilter(QObject *, QEvent *);
        virtual bool event(QEvent *);
        ItomIsoGLWidget *m_plotObj;

    private:
        double startPos[6];
        double endPos[6];

        bool rotating;
        bool moving;

    signals:

    public slots:

    private slots:

};

class ItomIsoGLWidget : public ito::AbstractDObjPclFigure
{
    Q_OBJECT

    DESIGNER_PLUGIN_ITOM_API

    public:
        friend class GL3DEFilter;

#if(CONNEXION_ENABLE)
        SiHdl m_SpwDeviceHandle;
        SiOpenData m_SpwData;
#endif
        ItomIsoGLWidget(const QString &itomSettingsFile, const ito::ParamBase::Type inpType, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        ~ItomIsoGLWidget();

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
            getOutputParam("bounds")->setVal(pointArr, 2 * bounds.size());
            delete[] pointArr;
        }

        void enableComplexGUI(const bool checked);
        void enableIlluGUI(const bool checked);
        void enableZStackGUI(const bool checked);
        void setLinePlotCoordinates(const QVector<QPointF> pts);

        GL3DEFilter *m_pEventFilter;

        ito::AutoInterval getZAxisInterval(void) const;
        void setZAxisInterval(ito::AutoInterval interval);

        QString getColorMap(void) const;
        void setColorMap(QString);

        void setContextMenuEnabled(bool show);
        bool getContextMenuEnabled() const;

    protected:
        plotGLWidget* m_pContent;

#if CONNEXION_ENABLE
        bool winEvent(MSG * message, long * result);
#endif //CONNEXION_ENABLE

    private:
        QAction* m_actScaleSetting;
/*
        QAction  *m_actPan;
        QAction  *m_actZoomToRect;
        QAction  *m_actMarker;
        QAction  *m_actLineCut;
*/
        QAction  *m_actPalette;
        QAction  *m_actToggleColorBar;
        QAction  *m_actChangeBGColor;

        QAction *m_actSave;
        QAction *m_actHome;

        QAction  *m_toggleIllumination;
        QAction  *m_toggleIlluminationRotation;
        QAction  *m_toggleInfoText;
/*
        QAction  *m_actColorDisplay;
        QAction  *m_actGrayDisplay;
        QAction  *m_actAScan;

        QAction  *m_actForward;
        QAction  *m_actBack;
*/
        QAction  *m_actCmplxSwitch;

        QMenu    *m_mnuCmplxSwitch;

        QAction  *m_actTringModeSwitch;
        QMenu    *m_mnuTringModeSwitch;

        QLabel *m_lblCoordinates;

        void riseZAmplifierer(const double value){if(m_pContent) ((plotGLWidget*)m_pContent)->riseZAmplifierer(value);}
        void reduceZAmplifierer(const double value){if(m_pContent) ((plotGLWidget*)m_pContent)->reduceZAmplifierer(value);}

        QTimer myTimer;

    signals:

    private slots:

        void mnuHome();

        /*
        void mnuPanner(bool checked);
        void mnuValuePicker(bool checked);
        void mnuAScanPicker(bool checked);
        void mnuLinePicker(bool checked);
        */

        void mnuScaleSettings();
        void mnuPalette();
        void mnuColorBar();
        void mnuToggleBPColor();
        void mnutoggleIllumination(const bool checked);
        void mnutoggleIlluminationRotation(const bool checked);
        void mnuCmplxSwitch(QAction *action);
        void mnuTringModeSwitch(QAction *action);
        //void mnuZoomer(bool checked);
        void mnuActSave();
        //void mnuSwitchColorMode(QAction *action);

    public slots:

/*
#if CONNEXION_ENABLE
        void callMeMaybe();
#endif
*/
        void triggerReplot();
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMFIGURE_H
