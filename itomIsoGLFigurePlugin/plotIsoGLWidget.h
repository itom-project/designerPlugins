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

#ifndef plotGLWidget_H
#define plotGLWidget_H

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
#include "common/numeric.h"
#include "common/sharedStructures.h"
#ifdef USEPCL
    #include "PointCloud/pclStructures.h"
#endif

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

struct AxisLabel
{
    AxisLabel() : dx(0), dy(0), write(0), unitydigit(0), lastdigit(0), unity(0), maxlen(0), rightAligned(false), topAligned(false){}
    double dx, dy, write;
    int unitydigit, lastdigit;
    double unity;
    long maxlen;
    bool rightAligned;
    bool topAligned;
};

struct AxisProperties
{
    AxisProperties(): label(""), unit(""), dimIdx(0), scale(1),
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

struct ObjectInfo
{
    ObjectInfo() : show(1), meanVal(0), divVal(0), xLength(""), yLength(""),
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


class plotGLWidget : public QGLWidget
{
    Q_OBJECT
    public:
        friend class ItomIsoGLWidget;

        plotGLWidget(QMenu *contextMenu, QGLFormat &fmt, QWidget *parent = 0, const QGLWidget *shareWidget = 0);
        ~plotGLWidget();

        void refreshPlot(ito::ParamBase *dataObj);
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);
        ito::RetVal setCanvasZoom(const int zoolLevel);
        inline void setStackStatus(const int value) { m_stackState = value  % 4; m_forceReplot = true; }
        inline void setCmplxMode(const int cmplxMode) {m_cmplxMode = cmplxMode % 4; m_forceReplot = true; }
        inline bool getStackStatus() { return m_stackState; }
        inline bool getCmplxStatus() { return m_cmplxState; }
        inline void setBGColor(const int color) { m_backgnd = color % 2; update(); }
        inline int getBGColor() { return m_backgnd; }
        inline int getCurrentVisMode(){ return m_elementMode; }
        void setCurrentVisMode(const int mode);

        void rotateLightArrow(const double deltaA, const double deltaB, const double deltaC);
        void rotateView(const double deltaA, const double deltaB, const double deltaC);
        void moveView(const double deltaX, const double deltaY, const double deltaZ);

        void setView(const double transX, const double transY, const double transZ, const double rotA, const double rotB, const double rotC);

        ito::RetVal setColor(const int col);
        void enableInit() { if (!(m_isInit & -1)) m_isInit |= 1; }
        void disableInit() { m_isInit &= ~1; }
        bool lightArrowEnabled();
        void paintGL();
        void resizeGL(int width, int height);
//        void paintEvent(QPaintEvent *pevent);
//        void resizeEvent(QResizeEvent *pevent);
        void initializeGL();

        bool m_showContextMenu;

    protected:
        int m_nthreads;

    private:
        QTimer m_timer;
        ito::uint32 m_lineplotUID;
        QMenu *m_contextMenu;
        QSharedPointer<ito::DataObject> m_pContentDObj;               //!< borrowed reference, do not delete here
#ifdef USEPCL
        QSharedPointer<ito::PCLPointCloud> m_pContentPC;              //!< borrowed reference, do not delete here
        QSharedPointer<ito::PCLPolygonMesh> m_pContentPM;             //!< borrowed reference, do not delete here
#endif
        QSharedPointer<ito::DataObject> m_pContentWhileRastering;
        unsigned char m_colorMode;
        ito::float64 m_invalid;
        AxisProperties m_axisX;
        AxisProperties m_axisY;
        AxisProperties m_axisZ;
        double m_windowXScale;
        double m_windowYScale;
        double m_windowZScale;
        double m_zAmpl;
        double m_xybase;
        ObjectInfo m_objectInfo;
        int m_paletteNum;
        int m_elementMode;
        QWidget *m_pParent;
        bool m_cmplxState;
        int m_cmplxMode;
        bool m_stackState;
        int m_isInit;
        int m_currentColor;
        double m_TransX, m_TransY, m_TransZ, m_RotA, m_RotB, m_RotC;
        double m_xAxisVector[3], m_yAxisVector[3], m_zAxisVector[3];
        double m_baseVector[3];
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
        std::string m_errorDisplMsg;
        GLuint m_myCharBitmapBuffer;
        int           m_NumElements;
        GLfloat       *m_pTriangles;
        GLubyte       *m_pColTriangles;
        GLfloat       *m_pNormales;
        GLfloat       *m_pPoints;
        unsigned char *m_pColIndices;

//        int initOGL2(const int width, const int height);
        ito::RetVal GLSetTriangles(void);
        ito::RetVal GLSetPointsPCL(void);
        void generateObjectInfoText();
        template<typename _Type> inline ito::RetVal NormalizeObj(cv::Mat &scaledTopo, ito::float64 &normedInvalid);
        void OGLMakeFont(int fsize);
        void paintLightArrow();
        void paintAxisTicksOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1, const double v0, const double v1, const double VorzX, const double VorzY, const double VorzZ, const std::string &symbol, const std::string &unit, const bool write);
        void paintAxisOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1);
        void paintAxisLabelOGL(const struct AxisLabel &vd, const double x, const double y, const double v);
        void threeDAxis(void);
        void threeDRotationMatrix(void);
        void rescaleTriangles(const double xscaleing, const double yscaleing, const double zscaleing);
        void DrawTitle(const std::string &myTitle, const int texty, int &yused);
        void DrawColorBar(const char xPos, const char yPos, const GLfloat dX, const GLfloat dY, const GLfloat zMin, const GLfloat zMax);
        void DrawObjectInfo(void);
        int OGLTextOut(const char *buffer, double xpos, double ypos, const bool rightAligned, const bool topAligned);
        ito::RetVal ResetColors();
#ifdef USEPCL
        template<typename _Tp> void pclFindMinMax(pcl::PointCloud<_Tp> *pcl, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax)
        {
            xmin = std::numeric_limits<ito::float64>::max();
            xmax = std::numeric_limits<ito::float64>::min();
            ymin = std::numeric_limits<ito::float64>::max();
            ymax = std::numeric_limits<ito::float64>::min();
            zmin = std::numeric_limits<ito::float64>::max();
            zmax = std::numeric_limits<ito::float64>::min();
            _Tp pt;

            for (int np = 0; np < m_pContentPC->height() * m_pContentPC->width(); np++)
            {
                pt = pcl->at(np);
                if (pt.x < xmin)
                    xmin = pt.x;
                if (pt.x > xmax)
                    xmax = pt.x;
                if (pt.y < ymin)
                    ymin = pt.y;
                if (pt.y > ymax)
                    ymax = pt.y;
                if (pt.z < zmin)
                    zmin = pt.z;
                if (pt.z > zmax)
                    zmax = pt.z;
            }
        }

        template<typename _Tp> void pclFillPtBuf(pcl::PointCloud<_Tp> *pcl, double xscale, double xshift, double yscale, double yshift, double zscale, double zshift)
        {
            m_NumElements = 0;
            #if (USEOMP)
            #pragma omp parallel num_threads(m_nthreads)
            {
            #endif

            _Tp pt;
            int count = 0;

            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int npx = 0; npx < m_pContentPC->width() * m_pContentPC->height(); npx++)
            {
                pt = pcl->at(npx);
//                if ((fabs(color - pt.z) < threshold) && ito::isFinite<ito::float64>(pt.z))
                if (ito::isFinite<ito::float64>(pt.z))
                {
                    #if (USEOMP)
                    #pragma omp critical
                    {
                    #endif
                    count = m_NumElements++;
                    #if (USEOMP)
                    }
                    #endif
                    m_pPoints[count * 3] = pt.x * xscale * m_windowXScale - xshift;
                    m_pPoints[count * 3 + 1] = pt.y * yscale * m_windowYScale - yshift;
                    m_pPoints[count * 3 + 2] = pt.z * zscale - zshift;

                    m_pColIndices[count] = cv::saturate_cast<unsigned char>(pt.z * 255.0);
                }
            }

            #if (USEOMP)
            }
            #endif
        }
#endif

        enum elementModeEnum
        {
            PAINT_TRIANG = 0x00, // Do not show ColorBar
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

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
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

        inline void toggleObjectInfoText(const bool enabled);

#ifndef WIN32
        void setColorMap(QString colormap = QString());
#else
        void setColorMap(QString colormap = QString::QString(""));
#endif

};

#endif
