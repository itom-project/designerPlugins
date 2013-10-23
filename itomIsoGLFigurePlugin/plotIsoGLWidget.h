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

#ifndef plotGLWidget_H
#define plotGLWidget_H

#include "GL/glew.h"

#include <QtOpenGL/qgl.h>

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qqueue.h>
#include <qmenu.h>

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include "common/sharedStructures.h"

//#include "valuepicker2d.h"


struct axisProperties
{
    axisProperties(): label(""), unit(""), dimIdx(0), scale(1),
        autoScale(1), startScaled(0), isMetric(0), show(1), showTicks(1) 
    {
        idx[0] = 0;
        idx[1] = 0;
        phys[0] = 0;
        phys[1] = 0;
    }

    std::string label;
    std::string unit;
    unsigned int dimIdx;
    unsigned int idx[2];
    double phys[2];
    double scale;
    bool autoScale;
    bool startScaled;
    bool isMetric;
    bool show;
    bool showTicks;
};

struct protocol
{
    protocol() : show(0), m_psize(0), text("") {}

    bool show;
    double m_psize;
    std::string text;
};

struct objectInfo
{
    objectInfo() : show(1), meanVal(0), divVal(0), xLength(""), yLength(""),
        PeakText(""), MeanText(""), DevText("") {}

    bool show;
    double meanVal;
    double divVal;
    std::string xLength;
    std::string yLength;
    std::string matrix;
    std::string PeakText;
    std::string MeanText;
    std::string DevText;
};

class ItomIsoGLWidget;


class plotGLWidget :  public QGLWidget
{
    Q_OBJECT
    public:

        plotGLWidget(QMenu *contextMenu, QGLFormat &fmt, QWidget *parent = 0, const QGLWidget *shareWidget = 0);
        ~plotGLWidget();

        bool m_showContextMenu;
        void refreshPlot(ito::ParamBase *dataObj);

        friend class ItomIsoGLWidget;

        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);
        ito::RetVal setCanvasZoom(const int zoolLevel);

        inline void setStackStatus(const int value){ m_stackState = value  % 4; m_forceReplot = true;}
        inline void setCmplxMode(const int cmplxMode){m_cmplxMode = cmplxMode % 4; m_forceReplot = true;}

        inline bool getStackStatus(){return m_stackState;}
        inline bool getCmplxStatus(){return m_cmplxState;}

        inline void setBGColor(const int color){m_backgnd = color % 2; paintGL();}
        inline int getBGColor(){return m_backgnd;}

        void setCurrentVisMode(const int mode);
        inline int getCurrentVisMode(){return m_elementMode;}


        void rotateLightArrow(const double deltaA, const double deltaB, const double deltaC){lighDirAngles[0] +=deltaA; lighDirAngles[1] += deltaC;}
        void rotateView(const double deltaA, const double deltaB, const double deltaC){m_RotA +=deltaA; m_RotB += deltaB; m_RotC += deltaC;}
        void moveView(const double deltaX, const double deltaY, const double deltaZ){m_TransX +=deltaX; m_TransY += deltaY; m_TransZ += deltaZ;}

        ito::RetVal setColor(const int col);

        void enableInit() { if (!(m_isInit & -1)) m_isInit |= 1; }
        void disableInit() { m_isInit &= ~1; }

        bool lightArrowEnabled();

        void paintGL();
        void paintEvent(QPaintEvent *pevent);
        void resizeEvent(QResizeEvent *pevent);

    protected:


    private:

        ito::RetVal GLSetTriangles(int &mode);
        template<typename _Type> inline ito::RetVal NormalizeObj(cv::Mat &scaledTopo, ito::float64 &normedInvalid);

        inline void generateObjectInfoText();

        QTimer m_timer;
        ito::uint32 m_lineplotUID;
        QMenu *m_contextMenu;

        QSharedPointer<ito::DataObject> m_pContent;               /*!< borrowed reference, do not delete here */
        QSharedPointer<ito::DataObject> m_pContentWhileRastering;

        unsigned char m_colorMode;
        ito::float64 m_invalid;

        axisProperties m_axisX;
        axisProperties m_axisY;
        axisProperties m_axisZ;

        double m_windowXScale;
        double m_windowYScale;
        double m_windowZScale;

        double m_zAmpl;
        double m_xybase;

        protocol m_protocol;
        objectInfo m_objectInfo;

        int m_paletteNum;
        //int m_linePlotID;

        int m_elementMode;

        QWidget *m_pParent;

        bool m_cmplxState;
        int m_cmplxMode;

        bool m_stackState;

        int m_isInit;
        int m_currentColor;

        double m_TransX, m_TransY, m_TransZ, m_RotA, m_RotB, m_RotC;

        double m_xAxisVector[3], m_yAxisVector[3], m_zAxisVector[3];
        double m_lightAxisVector[3];
        double lighDirAngles[2];

        int m_fontsize;
        char m_colorBarMode;

        QVector<ito::uint32> m_currentPalette;

        int m_linewidth;
        bool m_forceReplot;
        bool m_backgnd;
        bool m_forceCubicVoxel;
        bool m_drawTitle;

        bool m_drawLightDir;

        double m_ticklength;
        double m_z_tickmulti;

        GLuint m_cBarTexture;

        double m_gamma;
        int m_glVer;

        void OGLMakeFont(int fsize);
        ito::RetVal DrawProtocol(const int y, const char *buffer, const int charsperline);

        void paintLightArrow();

        void paintAxisTicksOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1, const double v0, const double v1, const double VorzX, const double VorzY, const double VorzZ, const std::string &symbol, const std::string &unit, const bool write);
        void paintAxisOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1);
        void paintAxisLabelOGL(const void *vd, const double x, const double y, const double v);
        void threeDAxis(void);
        void threeDRotationMatrix(void);
        void rescaleTriangles(const double xscaleing, const double yscaleing, const double zscaleing);
        void DrawTitle(const std::string &myTitle, const int texty, int &yused);
        void DrawColorBar(const char xPos, const char yPos, const GLfloat dX, const GLfloat dY, const GLfloat zMin, const GLfloat zMax);
        void DrawObjectInfo(void);
        void ProtocolSize(void);

        int OGLTextOut(const char *buffer, const double xpos, const double ypos);
        ito::RetVal ResetColors();

        std::string m_errorDisplMsg;

        enum elementModeEnum
        {
            PAINT_TRIANG = 0x00, //* Do not show ColorBar */
            PAINT_POINTS = 0x01
        };

        enum colorBarOptions
        {
            COLORBAR_NO = 0x00, //* Do not show ColorBar */
            COLORBAR_LEFT = 0x01, //* Do not show ColorBar */
            COLORBAR_RIGHT = 0x02,
            COLORBAR_UPPER_RIGHT = 0x04

        };

        enum initStatus
        {
            NO_INIT = 0x00,
            IS_INIT = 0x01,
            HAS_TRIANG = 0x02,
            IS_RENDERING = 0x10,
            IS_CALCTRIANG = 0x20
        };

        GLuint m_myCharBitmapBuffer;

        GLfloat       m_NumElements;
        GLfloat       *m_pTriangles;
        GLubyte       *m_pColTriangles;
        GLfloat       *m_pNormales;
        GLfloat       *m_pPoints;
        unsigned char *m_pColIndices;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
//        void setSize(int sizex, int sizey);
//        void setPos(int xpos, int ypos);

        void setZAmplifierer(double value);
        void reduceZAmplifierer(double value);
        void riseZAmplifierer(const double value);

        void togglePaletteMode();
        void homeView();

        void toggleIllumination(const bool checked);
        void toggleIlluminationRotation(const bool checked);

        void paintTimeout();
        void stopTimer()
        {
            m_timer.stop();
        }

        inline void toogleObjectInfoText(const bool enabled);

#if linux
        void setColorMap(QString colormap = QString());
#else
        void setColorMap(QString colormap = QString::QString(""));
#endif


};


#endif
