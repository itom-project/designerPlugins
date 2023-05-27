/* ********************************************************************
#    twipOGLFigure-Plugin for itom
#    URL: http://www.twip-os.com
#    Copyright (C) 2014, twip optical solutions GmbH,
#    Stuttgart, Germany
#
#    This files is part of the designer-Plugin twipOGLFigure for the
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General
#    Public Licence and must behandled accordingly.
#
#    twipOGLFigure is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    Information under www.twip-os.com or mail to info@twip-os.com.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, University Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef TWIPOGLWIDGET_H
#define TWIPOGLWIDGET_H

//#include <QtOpenGL/qgl.h>
#include <qmenu.h>
#include <qhash.h>
#include <qslider.h>
#if QT_VERSION >= 0x050000
#if QT_VERSION >= 0x050400
#include <QOpenGLWidget>
#else
#include <QGLWidget>
#endif
#include <qopenglfunctions.h>
#include <qopenglvertexarrayobject.h>
#else
#include <QGLWidget>
#include <QGLFormat>
#endif

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include "framework.h"
#include "common/interval.h"
#ifdef USEPCL
    #include "PointCloud/pclStructures.h"
    #include <pcl/common/common.h>
#endif

#define USEWIDGETEVENTS 1

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

#define ROLL0        0.0f
#define PITCH0       2.35f
#define YAW0        -0.785f
#define GL_PI         3.14159265358979323846f
#define GL_RAD_GRAD  57.29577951308232087684f

/** \struct InternalData
*   \brief structure holding settings of the plot
*/
struct InternalData
{
    InternalData()
    {
        m_drawTitle = false;

        m_keepVoxel = true;
        m_drawLightDir = false;
        m_forceCubicVoxel = true;
        m_plotAngles = false;
        m_paletteNum = 0;
        m_state = 0;
        m_colorBarMode = 0;
        m_cmplxMode = 0;
        m_cmplxState = false;
        m_stackState = false;
        m_elementMode = 0;
        m_enableOverlay = false;
        m_alpha = 0.0f;
        m_autoTitle = true;

//        m_overlayInterval = QPointF(0.0f, 255.0f);
        m_overlayInterval = ito::AutoInterval(0, 255.0, true);
//        m_curvatureInterval = QPointF(0.0f, 0.0f);#
        m_curvatureInterval = ito::AutoInterval(0.0f, 0.0f, true);

        m_lightDirPitch = 0.0f;
        m_lightDirYaw = 0.0f;
        m_rollAng = ROLL0;
        m_pitchAng = PITCH0;
        m_yawAng = YAW0;
        m_zAmpl = 1.0f;

        m_axisColor = 0x00000000;
        m_textColor = 0x00000000;
        m_backgnd = 0x00FFFFFF;

        m_xAxisVisible = true;
        m_yAxisVisible = true;
        m_vAxisVisible = true;
        m_infoVisible = false;
        m_showCurvature = false;
        m_showInvalids = true;
        m_invColor = 0x000F0F0F;
    }

    ~InternalData() {}

    ito::tDataType m_dataType;      //!> stores type of current object (dataObject, pointCloud, ...)

    bool m_showInvalids;
    bool m_xAxisVisible;
    bool m_yAxisVisible;
    bool m_vAxisVisible;
    bool m_infoVisible;             //!> flag for enabling / disabling plot info text

    bool m_showCurvature;           //!> use curvature for color coding

    int m_state;                    //!> plot state, e.g. initialize or drawing
    bool m_plotAngles;
    float m_zAmpl;                  //!> amplication of z-values
    int m_paletteNum;               //!> number of selected palette from paletteManager
    int m_elementMode;              //!> information about the currently displayed data (points or triangles, illumination)
    bool m_cmplxState;
    int m_cmplxMode;                //!> flag to choose which part / representation for complex numbers is used
    bool m_stackState;
    bool m_enableOverlay;           //!> flag for enabling / disabling intensity overlay
    char m_colorBarMode;            //!> flag indicating if and where colorbar is displayed

    bool m_forceCubicVoxel;         //!> flag to set isometric voxel displaying
    bool m_drawTitle;               //!> flag inidcating if title is shown
    bool m_drawLightDir;            //!> flag inidicating if light direction arrow is shown
    bool m_keepVoxel;
    float m_alpha;                  //!> aplha value for intensity overlay
    ito::AutoInterval m_overlayInterval;
    ito::AutoInterval m_curvatureInterval;
    QString m_title;                //!> plot title string
    bool m_autoTitle;               //!> flag setting automatic / manual title generation

    ito::int32 m_backgnd;           //!> plot background color
    ito::int32 m_axisColor;         //!> color of axis
    ito::int32 m_textColor;         //!> text color
    ito::int32 m_invColor;          //!> color for invalid marked triangles or points

    float m_lightDirPitch;          //!> pitch angle of light direction
    float m_lightDirYaw;            //!> yaw angle of light direction
    float m_pitchAng;               //!> pitch angle of plot (3D data)
    float m_rollAng;                //!> roll angle of plot (3D data)
    float m_yawAng;                 //!> yaw angle of plot (3D data)
};

#ifdef USEPCL
    //!> helper function to check if pointCloud has curvature field
    template<typename _Tp> inline bool hasPointToCurvature(void){return false;}
    template<> inline bool hasPointToCurvature<pcl::PointNormal>(void){return true;}
    template<> inline bool hasPointToCurvature<pcl::PointXYZINormal>(void){return true;}
    template<> inline bool hasPointToCurvature<pcl::PointXYZRGBNormal>(void){return true;}

    //!> helper function to check if pointCloud has intensity field
    template<typename _Tp> inline bool hasPointToIntensity(void){return false;}
    template<> inline bool hasPointToIntensity<pcl::PointXYZI>(void){return true;}
    template<> inline bool hasPointToIntensity<pcl::PointXYZRGBA>(void){return true;}
    template<> inline bool hasPointToIntensity<pcl::PointXYZINormal>(void){return true;}
    template<> inline bool hasPointToIntensity<pcl::PointXYZRGBNormal>(void){return true;}

    //!> helper function to check if pointCloud has normal
    template<typename _Tp> inline bool hasPointToNormal(void){return false;}
    template<> inline bool hasPointToNormal<pcl::PointNormal>(void){return true;}
    template<> inline bool hasPointToNormal<pcl::PointXYZINormal>(void){return true;}
    template<> inline bool hasPointToNormal<pcl::PointXYZRGBNormal>(void){return true;}

    template<typename _Tp> inline void pointToIntensity(_Tp &point, GLfloat &r, GLfloat &g, GLfloat &b, const ito::float32 &norm)
    {
        r = 1.0;
        g = 1.0;
        b = 1.0;
    }

    template<> inline void pointToIntensity<pcl::PointXYZI>(pcl::PointXYZI &point, GLfloat &r, GLfloat &g, GLfloat &b, const ito::float32 &norm)
    {
        r = point.intensity /* * norm*/;
        g = point.intensity /* * norm*/;
        b = point.intensity /* * norm*/;
        return;
    }

    template<> inline void pointToIntensity<pcl::PointXYZRGBA>(pcl::PointXYZRGBA &point, GLfloat &r, GLfloat &g, GLfloat &b, const ito::float32 & /*norm*/)
    {
        r = point.r / 255.0;
        g = point.g / 255.0;
        b = point.b / 255.0;
        return;
    }

    template<> inline void pointToIntensity<pcl::PointXYZINormal>(pcl::PointXYZINormal &point, GLfloat &r, GLfloat &g, GLfloat &b, const ito::float32 &norm)
    {
        r = point.intensity * norm;
        g = point.intensity * norm;
        b = point.intensity * norm;
        return;
    }

    template<> inline void pointToIntensity<pcl::PointXYZRGBNormal>(pcl::PointXYZRGBNormal &point, GLfloat &r, GLfloat &g, GLfloat &b, const ito::float32 & /*norm*/)
    {
        r = point.r / 255.0;
        g = point.g / 255.0;
        b = point.b / 255.0;
        return;
    }

    template<typename _Tp> inline void pointToCurvature(_Tp &point, GLfloat &diff)
    {
        diff = 0.0f;
    }

    template<> inline void pointToCurvature<pcl::PointNormal>(pcl::PointNormal &point, GLfloat &diff)
    {
        diff = point.curvature;
    }

    template<> inline void pointToCurvature<pcl::PointXYZINormal>(pcl::PointXYZINormal &point, GLfloat &diff)
    {
        diff = point.curvature;
    }

    template<> inline void pointToCurvature<pcl::PointXYZRGBNormal>(pcl::PointXYZRGBNormal &point, GLfloat &diff)
    {
        diff = point.curvature;
    }

    template<typename _Tp> inline void pointToNormal(_Tp &point, GLfloat *normalVec)
    {
        normalVec[0] = 0.0;
        normalVec[1] = 0.0;
        normalVec[2] = 0.0;
        return;
    }

    template<> inline void pointToNormal<pcl::PointXYZINormal>(pcl::PointXYZINormal &point, GLfloat *normalVec)
    {
        normalVec[0] = point.normal_x;
        normalVec[1] = point.normal_y;
        normalVec[2] = point.normal_z;
        return;
    }

    template<> inline void pointToNormal<pcl::PointXYZRGBNormal>(pcl::PointXYZRGBNormal &point, GLfloat *normalVec)
    {
        normalVec[0] = point.normal_x;
        normalVec[1] = point.normal_y;
        normalVec[2] = point.normal_z;
        return;
    }

#endif

struct ViewZoomer
{
    ViewZoomer() : shiftx(0.0f), shifty(0.0f), shiftz(0.0f), scalexy(1.0f), enabled(false) {}
    ~ViewZoomer() {}

    float shiftx;
    float shifty;
    float shiftz;
    float scalexy;
    bool enabled;
};

/** \struct AxisLabel
*   \brief holding all properties of an axis label
*/
struct AxisLabel
{
    AxisLabel() : dx(0), dy(0), write(0), unitydigit(0), lastdigit(0), unity(0), maxlen(0), rightAligned(false), topAligned(false) {}
    double dx;
    double dy;
    double write;
    int unitydigit;
    int lastdigit;
    double unity;
    long maxlen;
    bool rightAligned;
    bool topAligned;
};

/** \struct ObjectInfo
*   \brief Object information as it is shown with the object info text
*/
struct ObjectInfo
{
    ObjectInfo() : show(1), meanVal(0), divVal(0), xLength(""), yLength(""),
        PeakText(""), MeanText(""), DevText("") {}

    bool show;
    double meanVal;         //!> mean value of data values
    double divVal;
    std::string xLength;    //!> object x size in pixel
    std::string yLength;    //!> object y size in pixel
    std::string matrix;
    std::string PeakText;
    std::string MeanText;
    std::string DevText;
};

/** \struct VBO
*   \brief structure for storing vertex buffer objects
*/
struct VBO {
    VBO() : m_vbufId(0), m_ebufId(0), m_VAO(0) {}
    ~VBO() {}

    GLuint m_vbufId;
    GLuint m_ebufId;
#if QT_VERSION < 0x050000
    GLuint m_VAO;
#else
    QOpenGLVertexArrayObject *m_VAO;
#endif
};

/** \struct FChar
*   \brief holding all information and texture id of a single font character
*/
struct FChar {
    FChar() : m_width(0), m_texID(0), m_vbo(NULL) {}
    ~FChar() {}

    ito::uint8 m_width; //!> character width in pixel
    GLuint m_texID;     //!> texture identifier on gpu
    VBO *m_vbo;         //!> vertex buffer object on gpu
};

/** \struct SFont
*   \brief holding all properties of a font
*/
struct SFont {
    SFont() : m_name(""), m_height(0), m_weight(0) {}
    ~SFont();

    QString m_name;
    int m_height;
    char m_weight;
    QVector<FChar> m_chars;
    QHash <int, VBO *> m_vboBufs;
};

#if QT_VERSION >= 0x050400
class TwipOGLWidget : public QOpenGLWidget
#else
class TwipOGLWidget : public QGLWidget
#endif
{
    Q_OBJECT
    public:
        friend class TwipOGLFigure;
#if QT_VERSION >= 0x050400
		TwipOGLWidget(QMenu *contextMenu, void* configData, QSurfaceFormat &fmt, QWidget *parent = 0, const QOpenGLWidget *shareWidget = 0);
#else
        TwipOGLWidget(QMenu *contextMenu, void* configData, QGLFormat &fmt, QWidget *parent = 0, const QGLWidget *shareWidget = 0);
#endif
		~TwipOGLWidget();
        void doCleanUp(void);
        void refreshPlot(ito::ParamBase *data, const int id = 0);
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setCanvasZoomView(const double factor);
        inline void zoomInByOne()
        {
            setCanvasZoomView( m_zoomer.scalexy + 0.1);
        }
        void zoomOutByOne()
        {
            setCanvasZoomView( m_zoomer.scalexy - 0.1);
        }

        inline void setStackStatus(const int value)
        {
            m_pConfigData->m_stackState = value  % 4; m_forceReplot = true;
        }

        inline void setCmplxMode(const int cmplxMode)
        {
            m_pConfigData->m_cmplxMode = cmplxMode % 4; m_forceReplot = true;
        }

        void updateVisMode();
        void updateColorsAndVisibility(const bool forceUpdate = true);

        void moveView(const double dX, const double dY, const double dZ);
        void setViewTranslation(const double transX, const double transY, const double transZ);

        ito::RetVal setColor(const int col);
        void enableInit() { if (!(m_isInit & -1)) m_isInit |= 1; }
        void disableInit() { m_isInit &= ~1; }
        void paintGL();
        void resizeGL(int width, int height);
        void initializeGL();
        void updateAlpha();
        void setPlaneAlpha(int idx, int alpha);
        void setPlaneVisState(int idx, bool state);
        void updateCurvature();
        ito::RetVal setOverlayImage(QSharedPointer< ito::DataObject > overlayImage, const int objectID);
        ito::RetVal setInvalidImage(QSharedPointer< ito::DataObject > invalidImage, const int objectID);

        void getFrameBuffer(QImage &img, const int oversampling);

        enum State
        {
            tIdle = 0,
            tMoving = 0x01,
            tRotating = 0x02,
            tZoomed = 0x04
        };

        enum ComplexType
        {
            tAbs = 0,
            tImag = 1,
            tReal = 2,
            tPhase = 3
        }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value

        enum ElementModeEnum
        {
            PAINT_TRIANG = 0x01,            //!> Display triangles
            PAINT_POINTS = 0x02,            //!> Display points only
            ENABLE_ILLUMINATION = 0x04,     //!> Toggle between illumination on and of
            HAS_CURVATURE = 0x08            //!> allow curvature plot
        };

        enum ColorBarOptions
        {
            COLORBAR_NO = 0x00,             //!> Do not show colorbar
            COLORBAR_LEFT = 0x01,           //!> colorbar on the left centered
            COLORBAR_RIGHT = 0x02,          //!> colorbar on the right centered
            COLORBAR_UPPER_RIGHT = 0x03     //!> colorbar in the upper right
        };

        enum InitStatus
        {
            NO_INIT = 0x00,
            IS_INIT = 0x01,                 //!> opengl has been initialized
            HAS_TRIANG = 0x02,              //!> triangle plotting mode
            IS_RENDERING = 0x10,            //!> currently an opengl paint is executed
            IS_CALCTRIANG = 0x20,           //!> updating triangles
            CACHE_STARTUP = 0x40            //!> opengl is being initialized, data has been pushed and is being processed
        };

        static void normalizeAngle(float &ioAngle)
        {
            ioAngle = ioAngle < - GL_PI ? GL_PI + fmod(ioAngle, GL_PI) : (ioAngle > GL_PI ? (- GL_PI) + fmod(ioAngle, GL_PI) : ioAngle);
        }
        InternalData *m_pConfigData;
    protected:

#if USEWIDGETEVENTS
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void wheelEvent ( QWheelEvent * event );
#endif

    private:
        // itom variables
        ito::uint32 m_lineplotUID;
        QHash<int, QSharedPointer<ito::DataObject> > m_invalidMap;
        QHash<int, QSharedPointer<ito::DataObject> > m_overlayImage;
        QHash<int, QSharedPointer<ito::DataObject> > m_pContentDObj;  //!< borrowed reference, do not delete here
#ifdef USEPCL
        QHash<int, QSharedPointer<ito::PCLPointCloud> > m_pContentPC; //!< borrowed reference, do not delete here
        QHash<int, QSharedPointer<ito::PCLPolygonMesh> > m_pContentPM;//!< borrowed reference, do not delete here
#endif
#if QT_VERSION >= 0x050000
        QOpenGLFunctions *m_glf;
//        char *m_glf;
#else
        // just a dummy pointer so we don't need to adapt the rest of the code
        char *m_glf;
#endif

        // event variables
        int m_activeModifiers;
        QPoint m_mouseStartPos;     //!> starting position of mouse when handeling mouse events

        // config variables
        int m_isInit;               //!> flag indicating plot status (initialized)
        bool m_forceReplot;         //!> flag to force replotting
        bool m_showContextMenu;     //!> status of context menu
        ViewZoomer m_zoomer;        //!> zoomer
        ObjectInfo m_objectInfo;    //!> object information
        FontStyle m_titleFont;      //!> title font
        Axes m_axes;                //!> container class with all axes

        // Qt variables
        QMenu *m_contextMenu;
        QWidget *m_pParent;

        double m_transX, m_transY, m_transZ;
        double m_scaleX, m_scaleY, m_scaleZ;

        QVector<ito::uint32> m_currentPalette;

        std::string m_errorDisplMsg;
        QHash<int, int> m_numVert;
        QHash<QString, SFont> m_fonts;
        QHash<int, int *> m_pUseTextPix;

        QHash<int, ito::float64> m_pclMinX, m_pclMinY, m_pclMinZ, m_pclMinDev;
        QHash<int, ito::float64> m_pclMaxX, m_pclMaxY, m_pclMaxZ, m_pclMaxDev;

        QHash<int, ito::uint8> m_transperency;
        QHash<int, bool> m_enabledHash;

#if QT_VERSION < 0x050000
        QHash<int, GLuint> m_VAO3D;
        GLuint m_VAO3DPri;
        GLuint m_cBarVAO;
#else
        QHash<int, QOpenGLVertexArrayObject*> m_VAO3D;
        QOpenGLVertexArrayObject *m_VAO3DPri;
        QOpenGLVertexArrayObject *m_cBarVAO;
#endif
        GLint m_prog3D;
        GLint m_prog3DPri;
        GLint m_prog2DPx;
        QHash<int, GLuint> m_vertBuf3D;
        //GLuint m_textBuf3D;
        QHash<int, GLuint> m_textBuf3D;
        GLuint m_vertBuf3DPri;
        GLint m_attribVert;
        GLint m_attribVertColor;
        GLint m_attribVert3DPri;
        GLint m_attribDiff;
        GLint m_attribTxtVert;
        GLint m_attribTxtUV;
        GLint m_attribNorm;
        GLint m_attribTexCol;
        GLint m_unifMVP;
        GLint m_unifVCT;
        GLint m_unifMVPPri;
        GLint m_unifVCTPri;
        GLint m_unifPalette;
        GLint m_unifGlColorInv;
        GLint m_unifGlColor2D;
        GLint m_unifGlColor3D;
        GLint m_unifGlColor3DPri;
        GLint m_unifUseTex;
        GLint m_unifUsePalette;
        GLint m_unifTextColor;
        GLint m_unifScaleX;
        GLint m_unifScaleY;
        GLint m_unifPosX;
        GLint m_unifPosY;
        GLint m_unifLighting;
        GLint m_unifLColor;
        GLint m_unifAmbient;
        GLint m_unifDiffuse;
        GLint m_unifDiffuseDir;
        GLint m_unifText;
        GLint m_unifDiffMode;
        GLint m_unifDiffNorm;
        GLint m_unifDiffMin;
        GLint m_uniCurAlpha;
        GLuint m_cBarVBuf;
        GLuint m_cBarTex;

        template<typename _Tp, typename _TpMat> ito::RetVal GLSetTriangles(const int id = 0, const bool isComplex = false);
#ifdef USEPCL
        template<typename _Tp> ito::RetVal GLSetPointsPCL(pcl::PointCloud<_Tp> *pcl, const int id = 0);
#endif
        template<typename _Tp> cv::Mat* newMatFromComplex(const cv::Mat* inData);
        void generateObjectInfoText(const int fromID = 0);
        void DrawObjectInfo(void);
        void DrawTitle();
        void paintLightArrow();
        int OGLTextOut(QString &text, double xpos, double ypos, const bool rightAligned, const bool topAligned, const QString ffamily, const int fsize, const ito::uint32 fcolor);
        ito::RetVal ResetColors();
        void DrawAxesOGL(void);
        void DrawColorBar(const GLfloat posX, const GLfloat posY, const GLfloat dX, const GLfloat dY, const bool centerRight);
        ito::RetVal prepareFont(const QFont &font, struct SFont &glfont);
        int makeShaderProg(const QString &progStr, GLint &progName);

        template<typename _Tp> ito::RetVal updateOverlayImage(const int objectID);

#ifdef USEPCL
        template<typename _Tp> void pclFindMinMax(pcl::PointCloud<_Tp> *pcl, const int id);
#endif

        inline uint32_t nearestPOT(const uint32_t num)
        {
            uint32_t n = num > 0 ? num - 1 : 0;

            n |= n >> 1;
            n |= n >> 2;
            n |= n >> 4;
            n |= n >> 8;
            n |= n >> 16;
            n++;

            return n;
        }

        inline cv::Mat makePerspective(const float field_of_view,
                const float nearPt, const float farPt, const float aspect_ratio)
        {
            float size = nearPt * tanf(field_of_view / 180.0 * CV_PI / 2.0f);

            return makeFrustum(-size, size, -size / aspect_ratio,
                     size / aspect_ratio, nearPt, farPt);
        }

        inline cv::Mat makeFrustum(const float left, const float right,
                const float bottom, const float top, const float nearPt,
                const float farPt)
        {
            cv::Mat fMat = cv::Mat::zeros(4, 4, CV_32F);
            fMat.at<float>(0, 0) = 2.0f * nearPt / (right - left);
            fMat.at<float>(0, 1) = 0.0f;
            fMat.at<float>(0, 2) = 0.0f;
            fMat.at<float>(0, 3) = 0.0f;

            fMat.at<float>(1, 0) = 0.0f;
            fMat.at<float>(1, 1) = 2.0f * nearPt / (top - bottom);
            fMat.at<float>(1, 2) = 0.0f;
            fMat.at<float>(1, 3) =  0.0f;

            fMat.at<float>(2, 0) = (right + left) / (right - left);
            fMat.at<float>(2, 1) = (top + bottom) / (top - bottom);
            fMat.at<float>(2, 2) = - (farPt + nearPt) / (farPt - nearPt);
            fMat.at<float>(2, 3) = -1.0f;

            fMat.at<float>(3, 0) = 0.0f;
            fMat.at<float>(3, 1) = 0.0f;
            fMat.at<float>(3, 2) = -2.0f * farPt * nearPt / (farPt - nearPt);
            fMat.at<float>(3, 3) = 0.0f;

            return fMat;
        }

        inline cv::Mat getWorldMatrix(const bool noScale = 0)
        {
            cv::Mat fMat = cv::Mat::zeros(4, 4, CV_32F);

            fMat.at<float>(0, 1) = 0.0f;
            fMat.at<float>(0, 2) = 0.0f;
            fMat.at<float>(0, 3) = 0.0f;

            fMat.at<float>(1, 0) = 0.0f;
            fMat.at<float>(1, 2) = 0.0f;
            fMat.at<float>(1, 3) = 0.0f;

            if (!m_pConfigData->m_keepVoxel || noScale)
            {
                fMat.at<float>(0, 0) = 1.0f;
                fMat.at<float>(1, 1) = m_axes.m_axisY.m_isflipped ? 1.0f : -1.0f;
                fMat.at<float>(2, 2) = 1.0f;
            }
            else
            {
                if(m_axes.m_axisX.getLength() < m_axes.m_axisY.getLength())
                {
                    fMat.at<float>(0, 0) = 1.0f * m_axes.m_axisX.getLength() / m_axes.m_axisY.getLength();
                    fMat.at<float>(1, 1) = m_axes.m_axisY.m_isflipped ? 1.0f : -1.0f;
                    fMat.at<float>(2, 2) = 1.0f * m_axes.m_axisZ.getLength() / m_axes.m_axisY.getLength();
                }
                else
                {
                    fMat.at<float>(0, 0) = 1.0f;
                    fMat.at<float>(1, 1) = m_axes.m_axisY.m_isflipped ? 1.0f * m_axes.m_axisY.getLength() / m_axes.m_axisX.getLength(): -1.0f * m_axes.m_axisY.getLength() / m_axes.m_axisX.getLength();
                    fMat.at<float>(2, 2) = 1.0f * m_axes.m_axisZ.getLength() / m_axes.m_axisX.getLength();
                }
            }

            if (m_pConfigData && !noScale)
                fMat.at<float>(2, 2) *= m_pConfigData->m_zAmpl;

            fMat.at<float>(2, 0) = 0.0f;
            fMat.at<float>(2, 1) = 0.0f;
//            fMat.at<float>(2, 2) = 1.0f;
            fMat.at<float>(2, 3) = 0.0f;

            fMat.at<float>(3, 0) = 0.0f;
            fMat.at<float>(3, 1) = 0.0f;
            fMat.at<float>(3, 2) = -0.5f;
            fMat.at<float>(3, 3) = 1.0f;

            //return  makeTransMat(0.0, 0.0, 0.0) * makeRotMatFromEuler(m_pitchAng, m_rollAng, m_yawAng) * makeScaleMat(0.7071067811 , 0.7071067811, 0.7071067811 * m_zAmpl);
            if (width() < height())
            {
//                return fMat * makeRotMatFromEuler(m_pConfigData->m_rollAng, m_pConfigData->m_pitchAng, m_pConfigData->m_yawAng) * makeScaleMat(0.5, 0.5 * width() / height(), 0.5 * m_pConfigData->m_zAmpl);
                return fMat * makeRotMatFromEuler(m_pConfigData->m_rollAng, m_pConfigData->m_pitchAng, m_pConfigData->m_yawAng) * makeScaleMat(0.5, 0.5 * width() / height(), 0.5);
            }
            else
            {
//                return fMat * makeRotMatFromEuler(m_pConfigData->m_rollAng, m_pConfigData->m_pitchAng, m_pConfigData->m_yawAng) * makeScaleMat(0.5 * height() / width(), 0.5, 0.5 * m_pConfigData->m_zAmpl);
                return fMat * makeRotMatFromEuler(m_pConfigData->m_rollAng, m_pConfigData->m_pitchAng, m_pConfigData->m_yawAng) * makeScaleMat(0.5 * height() / width(), 0.5, 0.5);
            }
        }

        inline cv::Mat makeTransMat(const float tx, const float ty, const float tz)
        {
            cv::Mat tMat = cv::Mat::zeros(4, 4, CV_32F);
            tMat.at<float>(0, 0) = 1.0f;
            tMat.at<float>(1, 1) = 1.0f;
            tMat.at<float>(2, 2) = 1.0f;

            tMat.at<float>(3, 0) = tx;
            tMat.at<float>(3, 1) = ty;
            tMat.at<float>(3, 2) = tz;
            tMat.at<float>(3, 3) = 1.0f;

            return tMat;
        }

        inline cv::Mat makeScaleMat(const float sx, const float sy, const float sz)
        {
            cv::Mat sMat = cv::Mat::zeros(4, 4, CV_32F);
            sMat.at<float>(0, 0) = sx;
            sMat.at<float>(1, 1) = sy;
            sMat.at<float>(2, 2) = sz;
            sMat.at<float>(3, 3) = 1.0;

            return sMat;
        }

        inline cv::Mat makeRotMatFromEuler(const float pitch,
                const float roll, const float yaw)
        {
            float cp = cosf(pitch);
            float sp = sinf(pitch);
            float cy = cosf(yaw);
            float sy = sinf(yaw);
            float cr = cosf(roll);
            float sr = sinf(roll);

            cv::Mat rMat = cv::Mat::zeros(4, 4, CV_32F);
            rMat.at<float>(0, 0) = cp * cy;
            rMat.at<float>(0, 1) = - cr * sy + sr * sp * cy;
            rMat.at<float>(0, 2) = sr * sy + cr * sp * sy;

            rMat.at<float>(1, 0) = cp * sy;
            rMat.at<float>(1, 1) = cr * cy + sr * sp * sy;
            rMat.at<float>(1, 2) = -sr * cy + cr * sp * sy;

            rMat.at<float>(2, 0) = -sp;
            rMat.at<float>(2, 1) = sr * cp;
            rMat.at<float>(2, 2) = cr * cp;

            rMat.at<float>(3, 3) = 1.0f;

            return rMat;
        }

        QHash<QString, const char*> m_shaderProgs;
        cv::Mat setMVP(const GLint uniformLoc, const float fov, const float near, const float far, const float ratio, const bool noScale = 0);
        cv::Mat setVCT(const GLint uniformLoc, const int state);

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        //void setZAmplification(double value);
        //void reduceZAmplification(double value);
        //void riseZAmplification(const double value);
        void validateZAmplification();
        void togglePaletteMode();
        void homeView();
        void toggleObjectInfoText(const bool enabled, const int fromID = 0);
        void oglAboutToDestroy();

#if linux
        void setColorMap(QString colormap = QString());
#elif __APPLE__
        void setColorMap(QString colormap = QString());
#else
        void setColorMap(QString colormap = QString::QString(""));
#endif

};

#endif
