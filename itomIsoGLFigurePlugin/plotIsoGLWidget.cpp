/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
   Universitaet Stuttgart, Germany

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
//#include "GL/glew.h"

#ifndef WIN32
    #include <unistd.h>
#endif

#include "itomIsoGLFigure.h"
#include "plotIsoGLWidget.h"
#include "common/sharedStructuresGraphics.h"
#include "common/numeric.h"

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>

#if (defined __APPLE__)
    #include <OpenGL/gl.h>
    #include <OpenGL/glu.h>
#else // __APPLE__
    #include <GL/gl.h>
    #include <GL/glu.h>
#endif // __APPLE__

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"

using namespace ito;

const char *dont_scale_units[] = {"frame", "frames", "frm", "frms", "digit", "digits", "Bild", "Bilder", "Wert", "-"};

#define GL_PI         3.14159265358979323846
#define GL_TWO_PI     6.28318530717958647692
#define GL_HALF_PI    1.57079632679489661923
#define GL_RAD_GRAD  57.29577951308232087684

#define RotA0       -1.4
#define RotB0       -1.5
#define RotC0       0.0

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize openGL (below version two - i.e. using static pipelines)
*    @param [in]    width    window width
*    @param [in] height    window height
*    @return        zero for no error, openGL error code otherwise
*/
/*
int plotGLWidget::initOGL2(const int width, const int height)
{
    int ret = 0;

    //somewhere, only once:
    QOpenGLFunctions *m_oglFunctions=new QOpenGLFunctions(QOpenGLContext::currentContext());
    m_oglFunctions->initializeOpenGLFunctions();

    //everytime before doing anything OpenGL:
    m_oglFunctions->glUseProgram(0);

    glShadeModel(GL_SMOOTH);                            //Smooth Shading
    ret = glGetError();
    glClearDepth(1.0f);                                    //Tiefenpuffer setzen
    glEnable(GL_DEPTH_TEST);                            //Tiefenpuffertest aktivieren
    glDepthFunc(GL_LEQUAL);                                //welcher Test
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);    //Perspektivenkorrektur an
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);                //Linien Antialiasing
    glClearColor(255.0f, 255.0f, 255.0f, 0.0f);                //weisser Hintergrund

    glEnable(GL_TEXTURE_2D);
    ret = glGetError();

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glMatrixMode(GL_PROJECTION);                    //Projektionsmatrix waehlen
    glLoadIdentity();
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);                //Window resizen
    gluOrtho2D(-1.1, 1.1, -1.1, 1.1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if ((ret = glGetError()))
    {
        std::cerr << "error enabeling texutres gl-window init\n";
    }

    return ret;
}
*/

//----------------------------------------------------------------------------------------------------------------------------------
plotGLWidget::plotGLWidget(QMenu *contextMenu, QGLFormat &fmt, QWidget *parent, const QGLWidget *shareWidget):
    QGLWidget(fmt, parent, shareWidget, Qt::Widget),
    m_contextMenu(contextMenu),
    m_pParent(parent),
    m_paletteNum(0),
    m_lineplotUID(0),
    m_cBarTexture(0),
    m_drawTitle(false),
    m_backgnd(true),
    m_drawLightDir(false),
    m_forceCubicVoxel(false),
    m_forceReplot(true),
    m_isInit(0),
    m_gamma (1),
    m_TransX(0.0),
    m_TransY(0.0),
    m_TransZ(-2.0),
    m_colorBarMode(COLORBAR_NO),
    m_currentColor(1),
    m_fontsize(10),
    m_linewidth(1),
    m_ticklength(1.0),
    m_z_tickmulti(1.0),
    m_cmplxMode(0.0),
    m_cmplxState(false),
    m_stackState(false),
    m_zAmpl(1.0),
    m_elementMode(PAINT_TRIANG),

    // GL Plot vars
    m_NumElements(0),
    m_pTriangles(NULL),
    m_pColTriangles(NULL),
    m_pColIndices(NULL),
    m_pNormales(NULL),
    m_pPoints(NULL),

    // DataObject Vars
    m_colorMode(1),
    m_pContentDObj(NULL),
#ifdef USEPCL
    m_pContentPC(NULL),
    m_pContentPM(NULL),
#endif
    m_pContentWhileRastering(NULL),
    m_invalid(1.6e308),
    m_nthreads(2)
{
    this->setMouseTracking(false); //(mouse tracking is controled by action in WinMatplotlib)

    m_timer.setSingleShot(true);
    QObject::connect(&m_timer, SIGNAL(timeout()), this, SLOT(paintTimeout()));

    m_myCharBitmapBuffer = 0;
    int ret = 0;

    lighDirAngles[0] = 0.0;
    lighDirAngles[1] = 0.0;

    m_xAxisVector[0] = 0.0;
    m_xAxisVector[1] = 0.0;
    m_xAxisVector[2] = 0.0;
    m_yAxisVector[0] = 0.0;
    m_yAxisVector[1] = 0.0;
    m_yAxisVector[2] = 0.0;
    m_zAxisVector[0] = 0.0;
    m_zAxisVector[1] = 0.0;
    m_zAxisVector[2] = 0.0;
    m_baseVector[0] = 0.0;
    m_baseVector[1] = 0.0;
    m_baseVector[2] = 0.0;

    m_axisX.label = "";
    m_axisX.unit = "";
    m_axisX.dimIdx = 1;
    m_axisX.idx[0] = 0;
    m_axisX.idx[1] = 0;
    m_axisX.phys[0] = 0.0;
    m_axisX.phys[1] = 0.0;
    m_axisX.scale = 1.0;
    m_axisX.autoScale = true;
    m_axisX.startScaled = false;
    m_axisX.isMetric = false;
    m_axisX.show = true;
    m_axisX.showTicks = true;

    m_axisY.label = "";
    m_axisY.unit = "";
    m_axisY.dimIdx = 1;
    m_axisY.idx[0] = 0;
    m_axisY.idx[1] = 0;
    m_axisY.phys[0] = 0.0;
    m_axisY.phys[1] = 0.0;
    m_axisY.scale = 1.0;
    m_axisY.autoScale = true;
    m_axisY.startScaled = false;
    m_axisY.isMetric = false;
    m_axisY.show = true;
    m_axisY.showTicks = true;

    m_axisZ.label = "";
    m_axisZ.unit = "";
    m_axisZ.dimIdx = 1;
    m_axisZ.idx[0] = 0;
    m_axisZ.idx[1] = 0;
    m_axisZ.phys[0] = 0.0;
    m_axisZ.phys[1] = 0.0;
    m_axisZ.scale = 1.0;
    m_axisZ.autoScale = true;
    m_axisZ.startScaled = false;
    m_axisZ.isMetric = false;
    m_axisZ.show = true;
    m_axisZ.showTicks = true;

    m_windowXScale = 1.0;
    m_windowYScale = 1.0;
    m_windowZScale = 1.0;

    m_objectInfo.show = false;
    m_objectInfo.divVal = 0;
    m_objectInfo.meanVal = 0;

    m_currentPalette.clear();
    m_errorDisplMsg.clear();
    m_errorDisplMsg.append("No Data");

    /* Basic view coordinates */
    //RotA0 = -1.4;
    //RotB0 = -1.5;
    //RotC0 = 0.0;

    m_RotA = 0.855 + RotA0;
    m_RotB = 1.571 + RotB0;
    m_RotC = 1.025 + RotC0;

//    m_pContentDObj = QSharedPointer<ito::DataObject>(new ito::DataObject());

//    refreshPlot(NULL);
    m_isInit = 1;

    m_nthreads  = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
plotGLWidget::~plotGLWidget()
{
    hide();
    m_isInit |= ~IS_INIT;
    Sleep(100);

    if (m_myCharBitmapBuffer)
        glDeleteLists(m_myCharBitmapBuffer, 256);

    glDeleteTextures(1, &m_cBarTexture);                    // Create The Texture

    if(m_myCharBitmapBuffer != 0) glDeleteLists(m_myCharBitmapBuffer, 256);

    if (m_pColTriangles != NULL)
        free(m_pColTriangles);
    if (m_pTriangles != NULL)
        free(m_pTriangles);
    if (m_pColIndices != NULL)
        free(m_pColIndices);
    if (m_pNormales != NULL)
        free(m_pNormales);
    if (m_pPoints != NULL)
        free(m_pPoints);

    if(m_pContentWhileRastering)
    {
        m_pContentWhileRastering.clear();
    }

    if(m_pContentDObj)
    {
        m_pContentDObj.clear();
    }
#ifdef USEPCL
    if(m_pContentPC)
    {
        m_pContentPC.clear();
    }
    if(m_pContentPM)
    {
        m_pContentPM.clear();
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::paintTimeout()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::initializeGL()
{
    int ret = 0;

    if(ITOM_API_FUNCS_GRAPH != NULL && *ITOM_API_FUNCS_GRAPH != NULL)
    {
        int numColorMaps = 0;
        ito::ItomPalette newPalette;

        apiPaletteGetNumberOfColorBars(numColorMaps);

        if (numColorMaps > 0)
        {
            apiPaletteGetColorBarIdx((m_currentColor + 1) % numColorMaps, newPalette);
            m_currentPalette = newPalette.colorVector256;
        }
        else
        {
            //should never happen, since there should always be more than 0 colorbars
            m_currentPalette = QVector<ito::uint32>(256);

            for (int i = 0; i < 256; i++)
            {
                m_currentPalette[i] = i + (i << 8) + (i << 16);
            }
        }
    }
    else
    {
        m_currentPalette = QVector<ito::uint32>(256);

        for(int i = 0; i < 256; i++)
        {
            m_currentPalette[i] = i + (i << 8) + (i << 16);
        }
    }

    glShadeModel(GL_SMOOTH);                            //Smooth Shading
    glClearDepth(1.0f);                                 //Tiefenpuffer setzen
    glEnable(GL_DEPTH_TEST);                            //Tiefenpuffertest aktivieren
    glDepthFunc(GL_LEQUAL);                             //welcher Test
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  //Perspektivenkorrektur an
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);             //Linien Antialiasing
    glClearColor(255.0f, 255.0f, 255.0f, 0.0f);         //weisser Hintergrund

    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glMatrixMode(GL_PROJECTION);                    //Projektionsmatrix waehlen
    glLoadIdentity();
    glViewport(0, 0, (GLsizei)width(), (GLsizei)height());                //Window resizen
    gluOrtho2D(-1.1, 1.1, -1.1, 1.1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &m_cBarTexture);

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);

    GLfloat *par, *pag, *pab;

    par = (GLfloat*)calloc(255, sizeof(GLfloat));
    pag = (GLfloat*)calloc(255, sizeof(GLfloat));
    pab = (GLfloat*)calloc(255, sizeof(GLfloat));

    for (int i=0; i<255; i++)
    {
        par[i] = (GLfloat)((m_currentPalette[i] & 0xFF0000L)>>16)/255.0;
        pag[i] = (GLfloat)((m_currentPalette[i] & 0xFF00L)>>8)/255.0;
        pab[i] = (GLfloat)(m_currentPalette[i] & 0xFFL)/255.0;
    }

    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 256, pag);
//    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 256, par);
//    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 256, pab);
//    ret = glGetError();

    free(par);
    free(pag);
    free(pab);

    glPixelTransferi(GL_RED_SCALE, 1);
    glPixelTransferi(GL_RED_BIAS, 0);
    glPixelTransferi(GL_GREEN_SCALE, 1);
    glPixelTransferi(GL_GREEN_BIAS, 0);
    glPixelTransferi(GL_BLUE_SCALE, 1);
    glPixelTransferi(GL_BLUE_BIAS, 0);
    glPixelTransferf(GL_ALPHA_SCALE, 0.0);
    glPixelTransferf(GL_ALPHA_BIAS,  1.0);

    glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //Screen und Tiefenpuffer leeren
    glPixelTransferi(GL_MAP_COLOR, GL_FALSE);

    int paletteSize = m_currentPalette.size();
    unsigned char *src = NULL;

    if(paletteSize == 0)
    {
        src = new unsigned char[8];
        memset(src, 255, 8);
    }
    else
    {
        src = new unsigned char[paletteSize * 4 * 2];
        unsigned char* ptrPal =  (unsigned char*)m_currentPalette.data();

        #if (USEOMP)
        #pragma omp parallel num_threads(m_nthreads)
        {
        #pragma omp for schedule(guided)
        #endif
        for(int i = 0; i < paletteSize; i++)
        {
            src[8 * i] = ptrPal[4*paletteSize - 4 * i - 4];
            src[8 * i + 1] = ptrPal[4*paletteSize - 4 * i - 3];
            src[8 * i + 2] = ptrPal[4*paletteSize - 4 * i - 2];
            src[8 * i + 3] = 255;
            src[8 * i + 4] = ptrPal[4*paletteSize - 4 * i - 4];
            src[8 * i + 5] = ptrPal[4*paletteSize - 4 * i - 3];
            src[8 * i + 6] = ptrPal[4*paletteSize - 4 * i - 2];
            src[8 * i + 7] = 255;
        }
        #if (USEOMP)
        }
        #endif
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, paletteSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, src);
    delete[] src;

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);
    OGLMakeFont(m_fontsize);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //Screen und Tiefenpuffer leeren

    if (ret = glGetError())
    {
        std::cerr << "error setting up openGLWindow window: " << ret << "\n";
    }
    else
    {
        m_isInit = IS_INIT;
        refreshPlot(NULL);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::paintGL()
{
    double winSizeX = (double)width();
    double winSizeY = (double)height();
    ito::RetVal retval = ito::retOk;
    ito::float64 Sqrt2Div2 = sqrt(2.0) / 2.0;
    ito::float64 xs = 1.0, ys = 1.0, zs = 1.0, maxl = 1.0;
    ito::float64 nDims = 0;

    if (m_isInit != 3)
        return;

    m_isInit |= IS_RENDERING;
    m_ticklength = (int)(sqrt(winSizeX * winSizeX + winSizeY * winSizeY) * 10.0 / 1000.0);

    glMatrixMode(GL_MODELVIEW);                             // select projection matrix
    glLoadIdentity();

    if (!m_backgnd)
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);               // black background
    else
        glClearColor(255.0f, 255.0f, 255.0f, 0.0f);         // white background

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);     // clear screen and depth buffer

    if ( ( ((m_pTriangles == NULL) && (m_elementMode == PAINT_TRIANG)) && ((m_pPoints == NULL) && (m_elementMode == PAINT_POINTS)) ) || (m_pColTriangles == NULL) || (m_NumElements == 0))
    {
        m_isInit &= ~IS_RENDERING;
        return;
    }

    glTranslated(m_TransX, m_TransY, m_TransZ);
    glRotated(m_RotA * GL_RAD_GRAD, 1.0f, 0.0f, 0.0f);
    glRotated(m_RotB * GL_RAD_GRAD, 0.0f, 1.0f, 0.0f);
    glRotated(m_RotC * GL_RAD_GRAD, 0.0f, 0.0f, 1.0f);


    threeDRotationMatrix();

    if (m_colorMode == 0 && m_elementMode == PAINT_TRIANG)
    {
        GLfloat ambientLS[4] = {0.2f, 0.2f, 0.2f, 1.0f};
        GLfloat diffuseLS[4] = {0.3f, 0.3f, 0.3f, 1.0f};
        GLfloat specularLS[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        GLfloat positionLS[4] = {1.0f, 1.0f, 1.0f, 0.0f};
        GLfloat directionLS[3] = {0.0f, 0.0f, 1.0f};

        GLfloat diffuseObj[4]={0.8f, 0.8f, 0.8f, 1.0f};
        GLfloat specularObj[4]={0.03f, 0.03f, 0.03f, 1.0f};
        GLfloat emissionObj[4]={0.0f, 0.0f, 0.0f, 1.0f};
        double norm;

        positionLS[0] = cos(lighDirAngles[0]);
        positionLS[1] = sin(lighDirAngles[0]);
        positionLS[2] = tan(lighDirAngles[1]);

        norm = sqrt((positionLS[0] * positionLS[0]) + (positionLS[1] * positionLS[1]) + (positionLS[2] * positionLS[2]));
        positionLS[0] /= norm * 1;
        positionLS[1] /= norm * 1;
        positionLS[2] /= norm * -20;
//        position[2] *= 10;
        directionLS[0] = 1.0f * positionLS[0];
        directionLS[1] = 1.0f * positionLS[1];
        directionLS[2] = 1.0f * positionLS[2];

        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_NORMALIZE);

        glLightfv(GL_LIGHT0, GL_POSITION, positionLS);
        glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, directionLS);
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLS);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLS);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specularLS);

        //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissionObj);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseObj);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specularObj);

        glEnableClientState(GL_NORMAL_ARRAY);
    }
    else
    {
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisable(GL_LIGHT0);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
        glDisable(GL_NORMALIZE);
    }

    if(m_elementMode == PAINT_POINTS)
    {
        glPointSize(1.0f);
        glVertexPointer(3, GL_FLOAT, 0, m_pPoints);

        glColorPointer(4, GL_UNSIGNED_BYTE, 0, m_pColTriangles);
        glDrawArrays(GL_POINTS, 0, m_NumElements);
    }
    else
    {
        glVertexPointer(3, GL_FLOAT, 0, m_pTriangles);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, m_pColTriangles);

        if (m_colorMode == 0)
            glNormalPointer (GL_FLOAT, 0, m_pNormales);
        glDrawArrays(GL_TRIANGLES, 0, m_NumElements * 3);
    }

    if (m_colorMode == 0 && m_elementMode == PAINT_TRIANG)
    {
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisable(GL_LIGHT0);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
        glDisable(GL_NORMALIZE);
    }

    threeDAxis();

    if (m_drawLightDir)
    {
        paintLightArrow();
    }

/*
    if (m_drawTitle)
    {    // noobjinfo
        glLoadIdentity();
        gluOrtho2D(-1.1, 1.1, -1.1, 1.1);
//        setcolor(win,dd->backgnd?win->bcolor:win->fcolor);
        //int yused;
//        int texty = 1.0;
        //DrawTitle(m_title, texty, yused);
    }
*/
    if(m_objectInfo.show)
    {
        DrawObjectInfo();
    }

//    int ret = glGetError();

//    glFlush();
    //glFinish();
//    if (context()->format().doubleBuffer())
//        swapBuffers();

    m_isInit &= ~IS_RENDERING;

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Type> inline ito::RetVal plotGLWidget::NormalizeObj(cv::Mat &scaledTopo, ito::float64 &normedInvalid)
{
    ito::RetVal retVal(retOk);
    cv::Mat* topoMat = (cv::Mat*)(m_pContentWhileRastering->get_mdata()[0]);
    _Type *ptrTopo = (_Type*)topoMat->ptr(0);
    ito::float64* ptrScaledTopo = (ito::float64*)scaledTopo.ptr(0);
    ito::float64 norm = m_axisZ.phys[1] - m_axisZ.phys[0];

    if (!ito::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

    #if (USEOMP)
    #pragma omp parallel num_threads(m_nthreads)
    {
    #endif
    ito::float64 pixval = 0.0;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif
    for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
    {
        ptrTopo = (_Type*)topoMat->ptr(cntY);
        ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

        for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
        {
            pixval = (ito::float64)ptrTopo[cntX];
            pixval -= m_axisZ.phys[0];
            pixval /=  norm;
            ptrScaledTopo[cntX] = pixval;
        }
    }
    #if (USEOMP)
    }
    #endif

    // rescale the invalud value
    normedInvalid = (m_invalid - m_axisZ.phys[0]) / norm;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> inline ito::RetVal plotGLWidget::NormalizeObj<ito::complex64>(cv::Mat &scaledTopo, ito::float64 &normedInvalid)
{
    ito::RetVal retVal(retOk);
    cv::Mat* topoMat = (cv::Mat*)(m_pContentWhileRastering->get_mdata()[0]);
    ito::complex64 *ptrTopo = (ito::complex64*)topoMat->ptr(0);
    ito::float64* ptrScaledTopo = (ito::float64*)scaledTopo.ptr(0);
    ito::float64 norm = m_axisZ.phys[1] - m_axisZ.phys[0];

    if (!ito::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

    #if (USEOMP)
    #pragma omp parallel num_threads(m_nthreads)
    {
    #endif

    ito::float64 pixval = 0.0;

    switch(m_cmplxMode)
    {
        default:
        case 0:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex64*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = abs(ptrTopo[cntX]);
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 1:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex64*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = ptrTopo[cntX].imag();
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 2:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex64*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = ptrTopo[cntX].real();
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 3:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex64*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = arg(ptrTopo[cntX]);
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
    }

    #if (USEOMP)
    }
    #endif

    // rescale the invalid value
    normedInvalid = (m_invalid - m_axisZ.phys[0]) / norm;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> inline ito::RetVal plotGLWidget::NormalizeObj<ito::complex128>(cv::Mat &scaledTopo, ito::float64 &normedInvalid)
{
    ito::RetVal retVal(retOk);
    cv::Mat* topoMat = (cv::Mat*)(m_pContentWhileRastering->get_mdata()[0]);
    ito::complex128 *ptrTopo = (ito::complex128*)topoMat->ptr(0);
    ito::float64* ptrScaledTopo = (ito::float64*)scaledTopo.ptr(0);;
    ito::float64 norm = m_axisZ.phys[1] - m_axisZ.phys[0];

    if (!ito::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

    #if (USEOMP)
    #pragma omp parallel num_threads(m_nthreads)
    {
    #endif

    ito::float64 pixval = 0.0;
    switch(m_cmplxMode)
    {
        default:
        case 0:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex128*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = abs(ptrTopo[cntX]);
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 1:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex128*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = ptrTopo[cntX].imag();
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 2:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex128*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = ptrTopo[cntX].real();
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
        case 3:
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < scaledTopo.rows; cntY++)
            {
                ptrTopo = (ito::complex128*)topoMat->ptr(cntY);
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);

                for (int cntX = 0; cntX < scaledTopo.cols; cntX++)
                {
                    pixval = arg(ptrTopo[cntX]);
                    pixval -= m_axisZ.phys[0];
                    pixval /=  norm;
                    ptrScaledTopo[cntX] = pixval;
                }
            }
        break;
    }

    #if (USEOMP)
    }
    #endif

    // rescale the invalid value
    normedInvalid = (m_invalid - m_axisZ.phys[0]) / norm;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::GLSetPointsPCL(void)
{
    ito::RetVal retVal(retOk);

    if (m_pColTriangles != NULL)
    {
        free(m_pColTriangles);
        m_pColTriangles = NULL;
    }
    if (m_pTriangles != NULL)
    {
        free(m_pTriangles);
        m_pTriangles = NULL;
    }
    if (m_pColIndices != NULL)
    {
        free(m_pColIndices);
        m_pColIndices = NULL;
    }
    if (m_pNormales != NULL)
    {
        free(m_pNormales);
        m_pNormales = NULL;
    }
    if (m_pPoints != NULL)
    {
        free(m_pPoints);
        m_pPoints = NULL;
    }

    m_NumElements = 0;
#ifdef USEPCL
    ito::float64 xscale = 1.0 / (m_axisX.phys[1] - m_axisX.phys[0]);
    ito::float64 xshift = m_windowXScale * xscale * (m_axisX.phys[0] + m_axisX.phys[1]) / 2.0;

    ito::float64 yscale = 1.0 / (m_axisY.phys[1] - m_axisY.phys[0]);
    ito::float64 yshift = m_windowYScale * yscale * (m_axisY.phys[0] + m_axisY.phys[1]) / 2.0;

    ito::float64 zscale = 1.0 / (m_axisZ.phys[1] - m_axisZ.phys[0]);
    ito::float64 zshift = zscale * (m_axisZ.phys[0] + m_axisZ.phys[1]) / 2.0;

    bool isFinite = false;

    ito::float64 threshold = 1.0;
//    pcl::PointCloud<pcl::PointXYZ> *pcl = m_pContentPC->toPointXYZ().get();
    int width = m_pContentPC->width();
    int height = m_pContentPC->height();

    if (!retVal.containsError())
    {
        // If m_elementMode == PAINT_POINTS try to paint points for faster, less memory consuming visualisation
        if (m_elementMode == PAINT_POINTS)
        {
            m_pColTriangles = static_cast<GLubyte *>(calloc(width * height * 4, sizeof(GLubyte)));
            m_pColIndices = static_cast<unsigned char *>(calloc(width * height, sizeof(unsigned char)));
            m_pPoints = static_cast<GLfloat *>(calloc(width * height * 3, sizeof(GLfloat)));

            if (m_pColTriangles == NULL || m_pPoints == NULL || m_pColIndices == NULL)
            {
                m_isInit &= ~HAS_TRIANG;
                m_NumElements = 0;
                retVal += ito::RetVal(ito::retError, 0, tr("Error allocating memory").toLatin1().data());
            }
        }
    }


    if (!retVal.containsError())
    {
        if (m_elementMode == PAINT_POINTS)
        {
            pcl::PointCloud<pcl::PointXYZ> *ppclXYZ;
            pcl::PointCloud<pcl::PointXYZI> *ppclXYZI;
            pcl::PointCloud<pcl::PointXYZINormal> *ppclXYZINormal;
            pcl::PointCloud<pcl::PointNormal> *ppclXYZNormal;
            pcl::PointCloud<pcl::PointXYZRGBA> *ppclXYZRGBA;
            pcl::PointCloud<pcl::PointXYZRGBNormal> *ppclXYZRGBNormal;

            switch (m_pContentPC->getType())
            {
                case ito::pclXYZ:
                    ppclXYZ = m_pContentPC->toPointXYZ().get();
                    pclFillPtBuf<pcl::PointXYZ>(ppclXYZ, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                case ito::pclXYZI:
                    ppclXYZI = m_pContentPC->toPointXYZI().get();
                    pclFillPtBuf<pcl::PointXYZI>(ppclXYZI, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                case ito::pclXYZINormal:
                    ppclXYZINormal = m_pContentPC->toPointXYZINormal().get();
                    pclFillPtBuf<pcl::PointXYZINormal>(ppclXYZINormal, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                case ito::pclXYZNormal:
                    ppclXYZNormal = m_pContentPC->toPointXYZNormal().get();
                    pclFillPtBuf<pcl::PointNormal>(ppclXYZNormal, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                case ito::pclXYZRGBA:
                    ppclXYZRGBA = m_pContentPC->toPointXYZRGBA().get();
                    pclFillPtBuf<pcl::PointXYZRGBA>(ppclXYZRGBA, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                case ito::pclXYZRGBNormal:
                    ppclXYZRGBNormal = m_pContentPC->toPointXYZRGBNormal().get();
                    pclFillPtBuf<pcl::PointXYZRGBNormal>(ppclXYZRGBNormal, xscale, xshift, yscale, yshift, zscale, zshift);
                break;

                default:
                case ito::pclInvalid:
                    retVal += ito::RetVal(ito::retError, 0, tr("invalid point cloud").toLatin1().data());
                break;
            }
        }
    }

    if (m_NumElements != 0 || !retVal.containsError())
    {
        if(m_elementMode == PAINT_POINTS)
        {
            m_pColIndices = (unsigned char*)realloc(m_pColIndices, m_NumElements * sizeof(unsigned char));
            m_pPoints = (GLfloat*)realloc(m_pPoints, m_NumElements * 3 * sizeof(GLfloat));
            m_pColTriangles = (unsigned char*)realloc(m_pColTriangles, m_NumElements * 4 * sizeof(GLubyte));
        }
        else
        {
            m_pColIndices = (unsigned char*)realloc(m_pColIndices, m_NumElements * 2 * 3 * sizeof(unsigned char));
            m_pTriangles = (GLfloat*)realloc(m_pTriangles, m_NumElements * 2 * 9 * sizeof(GLfloat));
            m_pColTriangles = (unsigned char*)realloc(m_pColTriangles, m_NumElements * 2 * 12 * sizeof(GLubyte));
            m_pNormales = (GLfloat*)realloc(m_pNormales, m_NumElements * 2 * 9 * sizeof(GLfloat));
        }

        ResetColors();
        m_errorDisplMsg.clear();
        m_isInit &= ~IS_CALCTRIANG;
    }

    if (m_NumElements == 0 || retVal.containsError())
    {
        m_isInit &= ~HAS_TRIANG;
        if (m_pColTriangles != NULL)
        {
            free(m_pColTriangles);
            m_pColTriangles = NULL;
        }
        if (m_pTriangles != NULL)
        {
            free(m_pTriangles);
            m_pTriangles = NULL;
        }
        if (m_pColIndices != NULL)
        {
            free(m_pColIndices);
            m_pColIndices = NULL;
        }
        if (m_pNormales != NULL)
        {
            free(m_pNormales);
            m_pNormales = NULL;
        }
        if (m_pPoints != NULL)
        {
            free(m_pPoints);
            m_pPoints = NULL;
        }
        m_NumElements = 0;
        retVal += ito::RetVal(ito::retError, 0, tr("Error calculating points / triangles").toLatin1().data());
        m_isInit &= ~IS_CALCTRIANG;
    }
#endif
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::GLSetTriangles(void)
{
    ito::RetVal retVal(retOk);

    int xsizeObj = m_axisX.idx[1] - m_axisX.idx[0];
    int ysizeObj = m_axisY.idx[1] - m_axisY.idx[0];

    if (m_pColTriangles != NULL)
    {
        free(m_pColTriangles);
        m_pColTriangles = NULL;
    }
    if (m_pTriangles != NULL)
    {
        free(m_pTriangles);
        m_pTriangles = NULL;
    }
    if (m_pColIndices != NULL)
    {
        free(m_pColIndices);
        m_pColIndices = NULL;
    }
    if (m_pNormales != NULL)
    {
        free(m_pNormales);
        m_pNormales = NULL;
    }
    if (m_pPoints != NULL)
    {
        free(m_pPoints);
        m_pPoints = NULL;
    }

    if(m_pContentDObj && xsizeObj && ysizeObj)
    {
        m_pContentWhileRastering = m_pContentDObj;
    }
#ifdef USEPCL
    else if (m_pContentPC == NULL && m_pContentPM == NULL)
#else
    else
#endif
    {
        m_isInit &= ~HAS_TRIANG;
        return ito::RetVal(ito::retError, 0, tr("DataObject empty, calc triangles failed").toLatin1().data());
    }

    cv::Mat_<float64> scaledTopo(ysizeObj, xsizeObj);
    ito::float64 invalidValue = m_invalid;

    switch(m_pContentWhileRastering->getType())
    {
        case ito::tUInt8:
        {
            retVal = NormalizeObj<ito::uint8>(scaledTopo, invalidValue);
        }
        break;
        case ito::tInt8:
        {
            retVal = NormalizeObj<ito::int8>(scaledTopo, invalidValue);
        }
        break;
        case ito::tUInt16:
        {
            retVal = NormalizeObj<ito::uint16>(scaledTopo, invalidValue);
        }
        break;
        case ito::tInt16:
        {
            retVal = NormalizeObj<ito::int16>(scaledTopo, invalidValue);
        }
        break;
        case ito::tUInt32:
        {
            retVal = NormalizeObj<ito::uint32>(scaledTopo, invalidValue);
        }
        break;
        case ito::tInt32:
        {
            retVal = NormalizeObj<ito::int32>(scaledTopo, invalidValue);

        }
        break;
        case ito::tFloat32:
        {
            retVal = NormalizeObj<ito::float32>(scaledTopo, invalidValue);
        }
        break;
        case ito::tFloat64:
        {
            retVal = NormalizeObj<ito::float64>(scaledTopo, invalidValue);
        }
        break;

        case ito::tComplex64:
        {
            retVal = NormalizeObj<ito::complex64>(scaledTopo, invalidValue);
        }
        break;
        case ito::tComplex128:
        {
            retVal = NormalizeObj<ito::complex128>(scaledTopo, invalidValue);
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown DataObject-Type, calc triangles failed").toLatin1().data());
    }

    m_NumElements = 0;
    int totCount = 0;

    ito::float64 xshift = m_windowXScale * xsizeObj / 2.0;
    ito::float64 yshift = m_windowYScale * ysizeObj / 2.0;
    ito::float64 zshift = 0.5;

    int cntY;
    int cntX;
    bool isFinite = false;

    ito::float64 threshold = 1.0;
    ito::float64 zscale = 1.0;

    if(!retVal.containsError())
    {
        // If m_elementMode == PAINT_POINTS try to paint points for faster, less memory consuming visualisation
        if(m_elementMode == PAINT_POINTS)
        {
            m_pColTriangles = static_cast<GLubyte *>(calloc(xsizeObj * ysizeObj * 4, sizeof(GLubyte)));
            m_pColIndices = static_cast<unsigned char *>(calloc(xsizeObj * ysizeObj, sizeof(unsigned char)));
            m_pPoints = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 3, sizeof(GLfloat)));

            if(m_pColTriangles == NULL || m_pPoints == NULL || m_pColIndices == NULL)
            {
                m_isInit &= ~HAS_TRIANG;
                m_NumElements = 0;
                retVal += ito::RetVal(ito::retError, 0, tr("Error allocating memory").toLatin1().data());
            }
        }
        else // PAINT_TRIANG
        {
            m_pColTriangles = static_cast<GLubyte *>(calloc(xsizeObj * ysizeObj * 2 * 12, sizeof(GLubyte)));
            m_pColIndices = static_cast<unsigned char *>(calloc(xsizeObj * ysizeObj * 2 * 3, sizeof(unsigned char)));

            m_pTriangles = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 2 * 9, sizeof(GLfloat)));
            m_pNormales = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 2 * 9, sizeof(GLfloat)));

            if(m_pColTriangles == NULL || m_pTriangles == NULL || m_pColIndices == NULL || m_pNormales == NULL)
            {
                m_isInit &= ~HAS_TRIANG;
                m_NumElements = 0;
                retVal += ito::RetVal(ito::retError, 0, tr("Error allocating memory").toLatin1().data());
            }
        }
    }

    #if (USEOMP)
    #pragma omp parallel num_threads(m_nthreads)
    {
    #endif
    ito::float64 color = 0.0;
    ito::float64 zsum = 0.0;
    ito::float64 dpixel1;
    ito::float64 dpixel2;
    ito::float64 dpixel3;
    int count = 0;

    GLfloat Vec1[3];
    GLfloat Vec2[3];

    ito::float64 *ptrScaledTopo = NULL;
    ito::float64 *ptrScaledTopoNext = NULL;

    if(!retVal.containsError())
    {
        if(m_elementMode == PAINT_POINTS)
        {
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (cntY = 0; cntY < ysizeObj -1; cntY++)
            {
                ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(cntY);

                for (cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    dpixel1 = ptrScaledTopoNext[cntX];
                    if ((fabs(color - dpixel1) < threshold) && ito::isFinite<ito::float64>(dpixel1) && (dpixel1 != invalidValue))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        {
                        #endif
                        count = totCount++;
                        m_NumElements++;
                        #if (USEOMP)
                        }
                        #endif
                        m_pPoints[count * 3] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pPoints[count * 3 + 1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pPoints[count * 3 + 2] = zscale * dpixel1 - zshift;

                        m_pColIndices[count] = cv::saturate_cast<unsigned char>(dpixel1 * 255.0);
                    }
                }
            }
        }
        else
        {
            //ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(0);

            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (cntY = 0; cntY < ysizeObj -1; cntY++)
            {
                ptrScaledTopo = (ito::float64*)scaledTopo.ptr(cntY);
                ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(cntY+1);

                for (cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    dpixel1 = ptrScaledTopoNext[cntX];
                    dpixel2 = ptrScaledTopo[cntX + 1];
                    dpixel3 = ptrScaledTopo[cntX];

                    if(ito::isFinite<ito::float64>(dpixel1) && ito::isFinite<ito::float64>(dpixel2) && ito::isFinite<ito::float64>(dpixel2))
                    {
                        zsum = dpixel1 + dpixel2 + dpixel3;
                        color = zsum / 3.0;
                        isFinite = true;
                    }
                    else
                    {
                        color = 0.0;
                        isFinite = false;
                    }

                    if ((fabs(color - dpixel1) < threshold) && (fabs(color - dpixel2) < threshold) && (fabs(color - dpixel3) < threshold)
                        && /*(fabs(dpixel1) < 1.6e308)*/ isFinite && (dpixel1 != invalidValue) && (dpixel2 != invalidValue) && (dpixel3 != invalidValue))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        {
                        #endif
                        count = totCount++;
                        m_NumElements++;
                        #if (USEOMP)
                        }
                        #endif

                        m_pTriangles[count * 9] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 2] = zscale * dpixel1 - zshift;

                        m_pTriangles[count * 9 + 3] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 4] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 5] = zscale * dpixel2 - zshift;

                        m_pTriangles[count * 9 + 6] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 7] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 8] = zscale * dpixel3 - zshift;

                        for (int n = 0; n < 3; n++)
                        {
                            Vec1[n] = m_pTriangles[count * 9 + 3 + n] - m_pTriangles[count * 9 + n];
                            Vec2[n] = m_pTriangles[count * 9 + 6 + n] - m_pTriangles[count * 9 + n];
                        }

                        m_pNormales[count * 9 + 6] = m_pNormales[count * 9 + 3] = m_pNormales[count * 9] = (Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1]);
                        m_pNormales[count * 9 + 7] = m_pNormales[count * 9 + 4] = m_pNormales[count * 9 + 1] = (Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2]);
                        m_pNormales[count * 9 + 8] = m_pNormales[count * 9 + 5] = m_pNormales[count * 9 + 2] = (Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0]);

                        m_pColIndices[count * 3] = cv::saturate_cast<unsigned char>(dpixel1 * 255.0);
                        m_pColIndices[count * 3 + 1] = cv::saturate_cast<unsigned char>(dpixel2 * 255.0);
                        m_pColIndices[count * 3 + 2] = cv::saturate_cast<unsigned char>(dpixel3 * 255.0);
                    }

                    dpixel1 = ptrScaledTopoNext[cntX];
                    dpixel2 = ptrScaledTopoNext[cntX + 1];
                    dpixel3 = ptrScaledTopo[cntX + 1];

                    if(ito::isFinite<ito::float64>(dpixel1) && ito::isFinite<ito::float64>(dpixel2) && ito::isFinite<ito::float64>(dpixel3))
                    {
                        zsum = dpixel1 + dpixel2 + dpixel3;
                        color = zsum / 3.0;
                        isFinite = true;
                    }
                    else
                    {
                        color = 0.0;
                        isFinite = false;
                    }

                    if ((fabs(color - dpixel1) < threshold) && (fabs(color - dpixel2) < threshold) && (fabs(color - dpixel3) < threshold)
                        && isFinite /*(fabs(dpixel1) < 1.6e308)*/ && (dpixel1 != invalidValue) && (dpixel2 != invalidValue) && (dpixel3 != invalidValue))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        {
                        #endif
                        count = totCount++;
                        m_NumElements++;
                        #if (USEOMP)
                        }
                        #endif

                        m_pTriangles[count * 9] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 2] = zscale * dpixel1 - zshift;

                        m_pTriangles[count * 9 + 3] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 4] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 5] = zscale * dpixel2 - zshift;

                        m_pTriangles[count * 9 + 6] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count * 9 + 7] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count * 9 + 8] = zscale * dpixel3 - zshift;

                        for (int n = 0; n < 3; n++)
                        {
                            Vec1[n] = m_pTriangles[count * 9 + 3 + n] - m_pTriangles[count * 9 + n];
                            Vec2[n] = m_pTriangles[count * 9 + 6 + n] - m_pTriangles[count * 9 + n];
                        }
                        m_pNormales[count * 9 + 6] = m_pNormales[count * 9 + 3] = m_pNormales[count * 9] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
                        m_pNormales[count * 9 + 7] = m_pNormales[count * 9 + 4] = m_pNormales[count * 9 + 1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
                        m_pNormales[count * 9 + 8] = m_pNormales[count * 9 + 5] = m_pNormales[count * 9 + 2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];

                        m_pColIndices[count * 3] = cv::saturate_cast<unsigned char>(dpixel1 * 255.0);
                        m_pColIndices[count * 3 + 1] = cv::saturate_cast<unsigned char>(dpixel2 * 255.0);
                        m_pColIndices[count * 3 + 2] = cv::saturate_cast<unsigned char>(dpixel3 * 255.0);
                    }
                }
            }
        }
    }
    #if (USEOMP)
    }
    #endif

    if (m_NumElements != 0 && !retVal.containsError())
    {
        if(m_elementMode == PAINT_POINTS)
        {
            m_pColIndices = (unsigned char*)realloc(m_pColIndices, m_NumElements * sizeof(unsigned char));
            m_pPoints = (GLfloat*)realloc(m_pPoints, m_NumElements * 3 * sizeof(GLfloat));
            m_pColTriangles = (unsigned char*)realloc(m_pColTriangles, m_NumElements * 4 * sizeof(GLubyte));
        }
        else
        {
            m_pColIndices = (unsigned char*)realloc(m_pColIndices, m_NumElements * 2 * 3 * sizeof(unsigned char));
            m_pTriangles = (GLfloat*)realloc(m_pTriangles, m_NumElements * 2 * 9 * sizeof(GLfloat));
            m_pColTriangles = (unsigned char*)realloc(m_pColTriangles, m_NumElements * 2 * 12 * sizeof(GLubyte));
            m_pNormales = (GLfloat*)realloc(m_pNormales, m_NumElements * 2 * 9 * sizeof(GLfloat));
        }

        ResetColors();
        m_errorDisplMsg.clear();
        m_isInit &= ~IS_CALCTRIANG;
    }

    m_pContentWhileRastering.clear();

CLEAREXIT:
    if (m_NumElements == 0 || retVal.containsError())
    {
        m_isInit &= ~HAS_TRIANG;
        if (m_pColTriangles != NULL)
        {
            free(m_pColTriangles);
            m_pColTriangles = NULL;
        }
        if (m_pTriangles != NULL)
        {
            free(m_pTriangles);
            m_pTriangles = NULL;
        }
        if (m_pColIndices != NULL)
        {
            free(m_pColIndices);
            m_pColIndices = NULL;
        }
        if (m_pNormales != NULL)
        {
            free(m_pNormales);
            m_pNormales = NULL;
        }
        if (m_pPoints != NULL)
        {
            free(m_pPoints);
            m_pPoints = NULL;
        }
        m_NumElements = 0;
        retVal += ito::RetVal(ito::retError, 0, tr("Error calculating points / triangles").toLatin1().data());
        m_isInit &= ~IS_CALCTRIANG;
    }

    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::rescaleTriangles(const double xscaleing, const double yscaleing, const double zscaleing)
{
    if(m_elementMode == PAINT_POINTS)
    {
        if(!m_pPoints)
            return;

        #if (USEOMP)
        #pragma omp parallel num_threads(m_nthreads)
        {
        #endif

        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (ito::int32 n = 0; n < m_NumElements; n++)
        {
            m_pPoints[n * 3] *= xscaleing;
            m_pPoints[n * 3 + 1] *= yscaleing;
            m_pPoints[n * 3 + 2] *= zscaleing;
        }
        #if (USEOMP)
        }
        #endif
    }
    else
    {
        if(!m_pTriangles || !m_pNormales)
            return;

        #if (USEOMP)
        #pragma omp parallel num_threads(m_nthreads)
        {
        #endif

        double Vec1[3], Vec2[3];

        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (ito::int32 n = 0; n < (m_NumElements * 3); n++)
        {
            m_pTriangles[n * 3] *= xscaleing;
            m_pTriangles[n * 3 + 1] *= yscaleing;
            m_pTriangles[n * 3 + 2] *= zscaleing;
        }

        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (ito::int32 n = 0; n < m_NumElements; n++)
        {
            for (ito::int32 m = 0; m < 3; m++)
            {
                Vec1[m] = m_pTriangles[n * 9 + 3 + m] - m_pTriangles[n * 9 + m];
                Vec2[m] = m_pTriangles[n * 9 + 6 + m] - m_pTriangles[n * 9 + m];
            }
            m_pNormales[n * 9 + 6] = m_pNormales[n * 9 + 3] = m_pNormales[n * 9] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
            m_pNormales[n * 9 + 7] = m_pNormales[n * 9 + 4] = m_pNormales[n * 9 + 1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
            m_pNormales[n * 9 + 8] = m_pNormales[n * 9 + 5] = m_pNormales[n * 9 + 2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
        }
        #if (USEOMP)
        }
        #endif
    }

    ResetColors();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::refreshPlot(ito::ParamBase *param)
{
    ito::RetVal retval = ito::retOk;
    ito::float64 Sqrt2Div2 = sqrt(2.0) / 2.0;
    ito::float64 xs = 1.0, ys = 1.0, zs = 1.0, maxl = 1.0; //, tempVal;
    int dims = 0;

    if (!(m_isInit & IS_INIT))
    {
        m_errorDisplMsg.clear();
        m_errorDisplMsg.append("GLWidget not init");
        return;
    }

    if (m_isInit & IS_RENDERING)
    {
        m_errorDisplMsg.clear();
        m_errorDisplMsg.append("Currently rendering");
        return;
    }

    if ((param != NULL) && (param->getType() == (ito::Param::DObjPtr & ito::paramTypeMask)))
    {
        //check dataObj
        ito::DataObject *dataObj = (ito::DataObject*)param->getVal<char*>();
        dims = dataObj->getDims();
        if( dims > 1)
        {
            m_pContentDObj = QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj));

            if(dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
            {
                if(!m_cmplxState) ((ItomIsoGLWidget*)m_pParent)->enableComplexGUI(true);
                m_cmplxState = true;
            }
            else
            {
                if(!m_cmplxState) ((ItomIsoGLWidget*)m_pParent)->enableComplexGUI(false);
                m_cmplxState = false;
            }

            int x1 = m_pContentDObj->getSize(dims - 1) - 1;
            int y1 = m_pContentDObj->getSize(dims - 2) - 1;
            bool test;

            if((m_axisX.autoScale) || ((int)m_axisX.idx[1] > x1) || ((int)m_axisX.dimIdx != dims - 1))
            {
                m_axisX.idx[0] = 0;
                m_axisX.idx[1] = x1;
                m_axisX.dimIdx = dims - 1;
            }

            if((m_axisY.autoScale) || ((int)m_axisY.idx[1] > y1) || ((int)m_axisY.dimIdx != dims - 2))
            {
                m_axisY.idx[0] = 0;
                m_axisY.idx[1] = y1;
                m_axisY.dimIdx = dims - 2;
            }

            m_axisX.phys[0] = m_pContentDObj->getPixToPhys(m_axisX.dimIdx, (double)m_axisX.idx[0], test);
            m_axisX.phys[1] = m_pContentDObj->getPixToPhys(m_axisX.dimIdx, (double)m_axisX.idx[1], test);

            m_axisY.phys[0] = m_pContentDObj->getPixToPhys(m_axisY.dimIdx, (double)m_axisY.idx[0], test);
            m_axisY.phys[1] = m_pContentDObj->getPixToPhys(m_axisY.dimIdx, (double)m_axisY.idx[1], test);

            //m_title = internalObj.getTag("title", test).getVal_ToString();

            double tempVal;

            if(m_axisX.phys[1] < m_axisX.phys[0])
            {
                tempVal = m_axisX.phys[0];
                m_axisX.phys[0] = m_axisX.phys[1];
                m_axisX.phys[1] = tempVal;
            }
            if(m_axisY.phys[1] < m_axisY.phys[0])
            {
                tempVal = m_axisY.phys[0];
                m_axisY.phys[0] = m_axisY.phys[1];
                m_axisY.phys[1] = tempVal;
            }

            m_axisX.label = m_pContentDObj->getAxisDescription(dims - 1, test);
            m_axisX.unit  = m_pContentDObj->getAxisUnit(dims - 1, test);
            m_axisY.label = m_pContentDObj->getAxisDescription(dims - 2, test);
            m_axisY.unit  = m_pContentDObj->getAxisUnit(dims - 2, test);
            m_axisZ.label = m_pContentDObj->getValueDescription();
            m_axisZ.unit  = m_pContentDObj->getValueUnit();

            if(!m_axisZ.unit.compare("mm") || !m_axisZ.unit.compare("m"))
                m_axisZ.isMetric = true;

            if(!m_axisX.unit.compare("mm") || !m_axisX.unit.compare("m"))
                m_axisX.isMetric = true;

            if(!m_axisY.unit.compare("mm") || !m_axisY.unit.compare("m"))
                m_axisY.isMetric = true;

            if(m_objectInfo.show)
            {
                ito::dObjHelper::devValue(m_pContentDObj.data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
                generateObjectInfoText();
            }

            m_forceReplot = true;
        }
    }
    if ((param != NULL) && (param->getType() == (ito::Param::PointCloudPtr & ito::paramTypeMask)))
    {
#ifdef USEPCL
        //check pointCloud
        ito::PCLPointCloud *pc = (ito::PCLPointCloud*)param->getVal<char*>();
        m_pContentPC = QSharedPointer<ito::PCLPointCloud>(new ito::PCLPointCloud(*pc));
//        m_pContentPC->lockRead();

        ito::float64 xmin, xmax, ymin, ymax, zmin, zmax;

        pcl::PointCloud<pcl::PointXYZ> *ppclXYZ;
        pcl::PointCloud<pcl::PointXYZI> *ppclXYZI;
        pcl::PointCloud<pcl::PointXYZINormal> *ppclXYZINormal;
        pcl::PointCloud<pcl::PointNormal> *ppclXYZNormal;
        pcl::PointCloud<pcl::PointXYZRGBA> *ppclXYZRGBA;
        pcl::PointCloud<pcl::PointXYZRGBNormal> *ppclXYZRGBNormal;
        switch (m_pContentPC->getType())
        {
            case ito::pclXYZ:
                ppclXYZ = m_pContentPC->toPointXYZ().get();
                pclFindMinMax<pcl::PointXYZ>(ppclXYZ, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            case ito::pclXYZI:
                ppclXYZI = m_pContentPC->toPointXYZI().get();
                pclFindMinMax<pcl::PointXYZI>(ppclXYZI, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            case ito::pclXYZINormal:
                ppclXYZINormal = m_pContentPC->toPointXYZINormal().get();
                pclFindMinMax<pcl::PointXYZINormal>(ppclXYZINormal, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            case ito::pclXYZNormal:
                ppclXYZNormal = m_pContentPC->toPointXYZNormal().get();
                pclFindMinMax<pcl::PointNormal>(ppclXYZNormal, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            case ito::pclXYZRGBA:
                ppclXYZRGBA = m_pContentPC->toPointXYZRGBA().get();
                pclFindMinMax<pcl::PointXYZRGBA>(ppclXYZRGBA, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            case ito::pclXYZRGBNormal:
                ppclXYZRGBNormal = m_pContentPC->toPointXYZRGBNormal().get();
                pclFindMinMax<pcl::PointXYZRGBNormal>(ppclXYZRGBNormal, xmin, xmax, ymin, ymax, zmin, zmax);
            break;

            default:
            case ito::pclInvalid:
                retval += ito::RetVal(ito::retError, 0, tr("invalid point cloud").toLatin1().data());
            break;
        }

        if (!retval.containsError())
        {

            m_axisX.phys[0] = xmin;
            m_axisX.phys[1] = xmax;
            m_axisY.phys[0] = ymin;
            m_axisY.phys[1] = ymax;
            m_axisZ.phys[0] = zmin;
            m_axisZ.phys[1] = zmax;

            if (m_zAmpl < 0.000000001) // make sure mu m can be displayed
                m_zAmpl = 0.000000001; // make sure mu m can be displayed

            zs = m_axisZ.phys[1] - m_axisZ.phys[0];
            xs = m_axisX.phys[1] - m_axisX.phys[0];
            ys = m_axisY.phys[1] - m_axisY.phys[0];

            // To get cubic voxel in case of metric data
            maxl = xs;
            if ((ys > maxl) && (ys != 1))
                maxl = ys;
            if ((zs > maxl) && (zs != 1))
                maxl = zs;
        }
#else
        retval += ito::RetVal(ito::retError, 0, tr("compiled without pointCloud support").toLatin1().data());
#endif // #ifdef USEPCL
    }

    if (m_pContentDObj != NULL)
    {
        double windowXScaleOld = m_windowXScale;
        double windowYScaleOld = m_windowYScale;
        double windowZScaleOld = m_windowZScale;

        ito::uint32 firstMin[3];
        ito::uint32 firstMax[3];

        if(m_axisZ.autoScale)
        {
            switch(m_pContentDObj->getType())
            {
                case ito::tUInt8:
                case ito::tInt8:
                case ito::tUInt16:
                case ito::tInt16:
                case ito::tUInt32:
                case ito::tInt32:
                case ito::tFloat32:
                case ito::tFloat64:
                    ito::dObjHelper::minMaxValue(m_pContentDObj.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tComplex64:
                case ito::tComplex128:
                    ito::dObjHelper::minMaxValue(m_pContentDObj.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true, m_cmplxMode);
                break;
                default:
                    retval == ito::retError;
                    m_errorDisplMsg.append("Object has invalid type");
            }
        }

        if(!retval.containsError())
        {
            if (m_zAmpl < 0.000000001) // make sure mu m can be displayed
                m_zAmpl = 0.000000001; // make sure mu m can be displayed

            if(m_axisZ.isMetric) zs = m_axisZ.phys[1] - m_axisZ.phys[0];

            if(m_axisX.isMetric) xs = m_axisX.phys[1] - m_axisX.phys[0];

            if(m_axisY.isMetric) ys = m_axisY.phys[1] - m_axisY.phys[0];

            //if(m_title.length() == 0)
            //{
            //    m_drawTitle = false;
            //}

            // To get cubic voxel in case of metric data
            maxl = xs;
            if ((ys > maxl) && (ys != 1))
                maxl = ys;
            if ((zs > maxl) && (zs != 1))
                maxl = zs;

            m_windowXScale = 1.0;
            m_windowYScale = 1.0;
            m_windowZScale = 1.0;

            if ((xs != 1) && (ys != 1))
            {
                if (width() > height())
                {
                    m_windowXScale = (double)height() / (double)width();
                    m_windowYScale = 1;
                }
                else
                {
                    m_windowYScale = (double)width() / (double)height();
                    m_windowXScale= 1;
                }
            }

            m_windowXScale *= xs / maxl;
            m_windowYScale *= ys / maxl;
            if (zs!=1 && m_forceCubicVoxel)
                m_windowZScale *= zs / maxl;

            m_windowXScale /= 1.2; // * fabs((double)(this->height()) / (double)this->height());
            m_windowYScale /= 1.2; // * fabs((double)(this->height()) / (double)this->height());

            if(ito::isNotZero<double>(m_axisZ.phys[1] - m_axisZ.phys[0]))
            {
                m_windowZScale /= (double)(m_axisZ.phys[1] -  m_axisZ.phys[0]) * m_zAmpl;
                //m_windowZScale /= (double)(m_maxZValue - m_minZValue);
            }
            else
            {
                m_windowZScale = 1.0;
            }

            m_windowXScale /= fabs(m_axisX.idx[1] - m_axisX.idx[0] + 1.0) * Sqrt2Div2;
            m_windowYScale /= fabs(m_axisY.idx[1] - m_axisY.idx[0] + 1.0) * Sqrt2Div2;

            if( m_NumElements == 0 || m_forceReplot == true)
            {
                GLSetTriangles();
            }
            else if ((m_windowXScale != windowXScaleOld) || (m_windowYScale != windowYScaleOld) || (m_windowZScale != windowZScaleOld))
                rescaleTriangles(m_windowXScale / windowXScaleOld, m_windowYScale / windowYScaleOld, m_windowZScale / windowZScaleOld);
        }
    }
#ifdef USEPCL
    else if (m_pContentPC != NULL)
    {

        double windowXScaleOld = m_windowXScale;
        double windowYScaleOld = m_windowYScale;
        double windowZScaleOld = m_windowZScale;

        m_windowXScale = 1.0;
        m_windowYScale = 1.0;
        m_windowZScale = 1.0;

        if ((xs != 1) && (ys != 1))
        {
            if (width() > height())
            {
                m_windowXScale = (double)height() / (double)width();
                m_windowYScale = 1;
            }
            else
            {
                m_windowYScale = (double)width() / (double)height();
                m_windowXScale= 1;
            }
        }

        m_windowXScale *= xs / maxl;
        m_windowYScale *= ys / maxl;
        if (zs!=1 && m_forceCubicVoxel)
            m_windowZScale *= zs / maxl;

        m_windowXScale /= 1.2; // * fabs((double)(this->width()) / (double)this->width());
        m_windowYScale /= 1.2; // * fabs((double)(this->height()) / (double)this->height());

        if(ito::isNotZero<double>(m_axisZ.phys[1] - m_axisZ.phys[0]))
        {
            m_windowZScale /= (double)(m_axisZ.phys[1] -  m_axisZ.phys[0]) * m_zAmpl;
            //m_windowZScale /= (double)(m_maxZValue - m_minZValue);
        }
        else
        {
            m_windowZScale = 1.0;
        }

        m_forceReplot = true;
        m_elementMode = PAINT_POINTS;

        if( m_NumElements == 0 || m_forceReplot == true)
        {
            GLSetPointsPCL();
        }
        else if ((m_windowXScale != windowXScaleOld) || (m_windowYScale != windowYScaleOld) || (m_windowZScale != windowZScaleOld))
            rescaleTriangles(m_windowXScale / windowXScaleOld, m_windowYScale / windowYScaleOld, m_windowZScale / windowZScaleOld);
    }
#else
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("DataObject-Container empty and compiled without pointCloud support").toLatin1().data());
    }
#endif // #ifdef USEPCL


    if (m_pContentDObj == NULL)
#ifdef USEPCL
    if (m_pContentDObj == NULL
        && m_pContentPC == NULL
        && m_pContentPM == NULL)
#endif
    {
        m_errorDisplMsg.clear();
        m_errorDisplMsg.append("Object empty");
        retval == ito::retError;
    }

    if (retval == ito::retOk)
    {
        m_isInit |= HAS_TRIANG;
        update();
    }
    else
    {
        m_isInit &= ~HAS_TRIANG;
    }

    m_forceReplot = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);    // resize window
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);        // select projection matrix
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width / (GLfloat) height, 0.1f, 100.0f); // set perspective

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::setColor(const int col)
{
    ito::RetVal retval = ito::retOk;
    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::ResetColors()
{
    long count;

    if (m_colorMode > 1)
        return ito::retWarning;

    if(m_elementMode == PAINT_POINTS)
    {
        if (m_pColIndices == NULL || m_pPoints == NULL || m_pColTriangles==NULL)
            return ito::retError;
    }
    else
    {
        if (m_pColIndices == NULL || m_pTriangles == NULL || m_pColTriangles==NULL)
            return ito::retError;
    }

    if(m_elementMode == PAINT_POINTS)
    {

        if(m_currentPalette.size() >= 255)
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColTriangles[4 * count] =   ((m_currentPalette[m_pColIndices[count]] & 0xFF0000L) >> 16);
                m_pColTriangles[4 * count + 1] = ((m_currentPalette[m_pColIndices[count]] & 0x00FF00L) >> 8);
                m_pColTriangles[4 * count + 2] = ((m_currentPalette[m_pColIndices[count]] & 0x0000FFL));
                m_pColTriangles[4 * count + 3] = 255;
            }
        }
        else
        {
/*
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColTriangles[4 * count]    = m_pColIndices[count];
                m_pColTriangles[4 * count + 1] = m_pColIndices[count];
                m_pColTriangles[4 * count + 2] = m_pColIndices[count];

                m_pColTriangles[count * 4 + 3] = 255;
            }
*/
        }

    }
    else
    {
        if(m_currentPalette.size() > 255)
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColTriangles[count * 12] = ((m_currentPalette[m_pColIndices[count * 3]] & 0xFF0000L) >> 16);
                m_pColTriangles[count * 12 + 4] = ((m_currentPalette[m_pColIndices[count * 3 + 1]] & 0xFF0000L) >> 16);
                m_pColTriangles[count * 12 + 8] = ((m_currentPalette[m_pColIndices[count * 3 + 2]] & 0xFF0000L) >> 16);
                m_pColTriangles[count * 12 + 1] = ((m_currentPalette[m_pColIndices[count * 3]] & 0x00FF00L) >> 8);
                m_pColTriangles[count * 12 + 5] = ((m_currentPalette[m_pColIndices[count * 3 + 1]] & 0x00FF00L) >> 8);
                m_pColTriangles[count * 12 + 9] = ((m_currentPalette[m_pColIndices[count * 3 + 2]] & 0x00FF00L) >> 8);
                m_pColTriangles[count * 12 + 2] = ((m_currentPalette[m_pColIndices[count * 3]] & 0x0000FFL));
                m_pColTriangles[count * 12 + 6] = ((m_currentPalette[m_pColIndices[count * 3 + 1]] & 0x0000FFL));
                m_pColTriangles[count * 12 + 10] = ((m_currentPalette[m_pColIndices[count * 3 + 2]] & 0x0000FFL));
                m_pColTriangles[count * 12 + 11] = m_pColTriangles[count * 12 + 7] = m_pColTriangles[count * 12 + 3] = 255;
            }
        }
        else
        {
/*
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColIndices[count * 12] = m_pColIndices[count * 3];
                m_pColIndices[count * 12 + 4] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count * 12 + 8] = m_pColIndices[count * 3 + 2];
                m_pColIndices[count * 12 + 1] = m_pColIndices[count * 3];
                m_pColIndices[count * 12 + 5] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count * 12 + 9] = m_pColIndices[count * 3 + 2];
                m_pColIndices[count * 12 + 2] = m_pColIndices[count * 3];
                m_pColIndices[count * 12 + 6] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count * 12 + 10] = m_pColIndices[count * 3 + 2];

                m_pColIndices[count * 12 + 11] = m_pColIndices[count * 12 + 7] = m_pColIndices[count * 12 + 3] = 255;
            }
*/
        }
    }

    return ito::retOk;
}

//-----------------------------------------------------------------------------------------------
void plotGLWidget::threeDRotationMatrix()
{
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);
    static double a = 0;
    static double b = 0;
    static double c = 0;
    gluProject(a, b, c, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &m_baseVector[0], &m_baseVector[1], &m_baseVector[2]);
    gluProject(1, 0, 0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &m_xAxisVector[0], &m_xAxisVector[1], &m_xAxisVector[2]);
    gluProject(0, 1, 0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &m_yAxisVector[0], &m_yAxisVector[1], &m_yAxisVector[2]);
    gluProject(0, 0, 1, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &m_zAxisVector[0], &m_zAxisVector[1], &m_zAxisVector[2]);
}

//-----------------------------------------------------------------------------------------------
void plotGLWidget::threeDAxis(void)
{
    ito::float64 Sqrt2div2 = sqrt(2.0) / 2.0;
    ito::float64 xb, yb;
    //ito::float64 dt;
    ito::float64 xmin, xmax, ymin, ymax, zmin, zmax, xsizep, ysizep;
    ito::float64 ax, ay, az, xe[6], ye[6], ze[6];
    ito::float64 signedY = 1.0, signedX = 1.0;

    ito::float64 signedXAx, signedXAy, signedYAx, signedYAy, signedZAx, signedZAy, signedZA;
    ito::float64 dxx, dxy, dyx, dyy, dzx, dzy, VRX, VRY;
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];

    ax = atan((m_xAxisVector[1] - m_baseVector[1]) / (m_xAxisVector[0] - m_baseVector[0]));
    ay = atan((m_yAxisVector[1] - m_baseVector[1]) / (m_yAxisVector[0] - m_baseVector[0]));
    az = atan((m_zAxisVector[1] - m_baseVector[1]) / (m_zAxisVector[0] - m_baseVector[0]));

    if (cos(m_RotC) < 0)
    {
        signedX *= -1.0;
    }
    if (sin(m_RotB) < 0)
    {
        signedX *= -1.0;
    }
    if (cos(m_RotA) < 0)
    {
        signedX *= -1.0;
    }
    if (sin(m_RotC) < 0)
    {
        signedY *= -1.0;
    }
    if (sin(m_RotB) < 0)
    {
        signedY *= -1.0;
    }
    if (cos(m_RotA) < 0)
    {
        signedY *= -1.0;
    }
    xsizep = (m_axisX.idx[1] - m_axisX.idx[0] + 1.0);
    ysizep = (m_axisY.idx[1] - m_axisY.idx[0] + 1.0);

    xmin = -m_windowXScale * xsizep / 2.0;
    xmax = m_windowXScale * xsizep / 2.0;
    ymin = m_windowYScale * ysizep / 2.0;
    ymax = -m_windowYScale * ysizep / 2.0;
    zmin = -m_windowZScale * (m_axisZ.phys[1] - m_axisZ.phys[0]) / 2.0;
    zmax = m_windowZScale * (m_axisZ.phys[1] - m_axisZ.phys[0]) / 2.0;

    if ((((ay  < az) - 0.5) * signedY) > 0)
    {
        xb = xmin;
    }
    else
    {
        xb = xmax;
    }
    if ((((ax > az) - 0.5) * signedX) > 0)
    {
        yb = ymin;
    }
    else
    {
        yb = ymax;
    }

    signedZA = 0;

    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);

//    gluProject(m_axisX.phys[0], m_axisY.phys[0], m_axisZ.phys[0], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[0], &ye[0], &ze[0]);
//    gluProject(m_axisX.phys[0], m_axisY.phys[0], m_axisZ.phys[0], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[1], &ye[1], &ze[1]);
//    gluProject(m_axisX.phys[1], m_axisY.phys[1], m_axisZ.phys[1], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[2], &ye[2], &ze[2]);
//    gluProject(m_axisX.phys[1], m_axisY.phys[1], m_axisZ.phys[1], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[3], &ye[3], &ze[3]);
    gluProject(xmin, ymin, zmin, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[0], &ye[0], &ze[0]);
    gluProject(xmin, ymax, zmin, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[1], &ye[1], &ze[1]);
    gluProject(xmax, ymin, zmin, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[2], &ye[2], &ze[2]);
    gluProject(xmax, ymax, zmin, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[3], &ye[3], &ze[3]);
    gluProject(xb, -yb, zmin, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[4], &ye[4], &ze[4]);
    gluProject(xb, -yb, zmax, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[5], &ye[5], &ze[5]);

    if (xb == xmin)
    {
        dyx = xe[1] - xe[0];
        dyy = ye[1] - ye[0];
    }
    else
    {
        dyx = xe[2] - xe[3];
        dyy = ye[2] - ye[3];
    }

    if (yb == ymin)
    {
        dxx = xe[3] - xe[0];
        dxy = ye[3] - ye[0];
    }
    else
    {
        dxx = xe[2] - xe[1];
        dxy = ye[2] - ye[1];
    }

    dzx = xe[5] - xe[4];
    dzy = ye[5] - ye[4];

    VRX = cos(m_RotA) / fabs(cos(m_RotA));
    VRY = cos(m_RotB) / fabs(cos(m_RotB));
    /*
    std::cout << "\n";
    std::cout << "m_RotA: " << m_RotA << "; m_RotB: " << m_RotB <<"m_RotC: " << m_RotC << "\n";
    std::cout << "VRX: " << VRX << "; VRY: " << VRY << "\n";
    std::cout << "signedX: " << signedX << "; signedY: " << signedY << "\n";
    std::cout << "xb: " << xb << "; yb: " << yb << "\n";
    */
    //Y-Axis X signed.
    if (xb == xmin)
    {
        if (dyy < 0)
        {
            signedYAx = -1 * VRX * VRY;
            //signedYAx = -1;
        }
        else
        {
            signedYAx = 1 * VRX * VRY;
            //signedYAx = 1;
        }

        if (dyx > 0)
        {
            signedYAy = -1 * VRX * VRY;
            //signedYAy = -1;
        }
        else
        {
            signedYAy = 1 * VRX * VRY;
            //signedYAy = 1;
        }
    }
    else
    {
        if (dyy < 0)
        {
            signedYAx = 1 * VRX * VRY;
            //signedYAx = 1;
        }
        else
        {
            signedYAx = -1 * VRX * VRY;
            //signedYAx = -1;
        }
        if (dyx > 0)
        {
            signedYAy = 1 * VRX * VRY;
            //signedYAy = 1;
        }
        else
        {
            signedYAy = -1 * VRX * VRY;
            //signedYAy = -1;
        }
    }

    if (yb == ymin)
    {
        if (dxx < 0)
        {
            signedXAy = -1 * VRX * VRY;
            //signedXAy = -1;
        }
        else
        {
            signedXAy = 1 * VRX * VRY;
            //signedXAy = 1;
        }

        if (dxy > 0)
        {
            signedXAx = -1 * VRX * VRY;
            //signedXAx = -1;
        }
        else
        {
            signedXAx = 1 * VRX * VRY;
            //signedXAx = 1;
        }
    }
    else
    {
        if (dxx < 0)
        {
            signedXAy = 1 * VRX * VRY;
            //signedXAy = 1;
        }
        else
        {
            signedXAy = -1 * VRX * VRY;
            //signedXAy = -1;
        }
        if (dxy > 0)
        {
            signedXAx = 1 * VRX * VRY;
            //signedXAx = 1;
        }
        else
        {
            signedXAx = -1 * VRX * VRY;
            //signedXAx = -1;
        }
    }

    if (((xb == xmin) && (yb==ymin)) || ((xb != xmin) && (yb!=ymin)))
    {
        if (dzy < 0)
        {
            signedZAx = -1 * VRX * VRY;
            //signedZAx = -1;
        }
        else
        {
            signedZAx = 1 * VRX * VRY;
            //signedZAx = 1;
        }


        if (dzx > 0)
        {
            signedZAy = -1 * VRX * VRY;
            //signedZAy = -1;
        }
        else
        {
            signedZAy = 1 * VRX * VRY;
            //signedZAy = 1;
        }

    }
    else
    {
        if (dzy < 0)
        {
            signedZAx = 1 * VRX * VRY;
            //signedZAx = 1;
        }
        else
        {
            signedZAx = -1 * VRX * VRY;
            //signedZAx = -1;
        }

        if (dzx > 0)
        {
            signedZAy = 1 * VRX * VRY;
            //signedZAy = 1;
        }
        else
        {
            signedZAy = -1 * VRX * VRY;
            //signedZAy = -1;
        }

    }
/*
    std::cout << "signedXAx: " << signedXAy << "; signedXAy: " << signedXAy <<
                 "; signedYAx: " << signedYAy << "; signedYAy: " << signedXAy <<
                 "; signedZAx: " << signedZAy << "; signedZAy: " << signedZAy << "\n";

    std::cout << "dyy: " << (dyy > 0 ? "true" : "false") << "; dyx: " << (dyx > 0 ? "true" : "false") <<
                 "; dxy: " << (dxy > 0 ? "true" : "false") << "; dxx: " << (dyx > 0 ? "true" : "false") <<
                 "; dzy: " << (dzy > 0 ? "true" : "false") << "; dzx: " << (dzx > 0 ? "true" : "false") << "\n";

    */
    if(m_axisX.show && m_axisY.show)
    {
        paintAxisOGL(xmin, -yb, zmin, xmax, -yb, zmin);
        paintAxisOGL(-xb, ymin, zmin, -xb, ymax, zmin);

        paintAxisOGL(xmin, yb, zmin, xmax, yb, zmin);
        paintAxisOGL(xb, ymin, zmin, xb, ymax, zmin);

        paintAxisTicksOGL(xmin, yb, zmin, xmax, yb, zmin,  m_axisX.phys[0], m_axisX.phys[1], signedXAx, signedXAy, signedZA, m_axisX.label, m_axisX.unit, 0);
        paintAxisTicksOGL(xb, ymin, zmin, xb, ymax, zmin, m_axisY.phys[0], m_axisY.phys[1], signedYAx, signedYAy, signedZA, m_axisY.label, m_axisY.unit, 0);
    }
    // make z-ticks adjustable
    m_ticklength /= m_z_tickmulti;

    if(m_axisZ.show)
    {
        paintAxisOGL(xb, -yb, zmin, xb, -yb, zmax);
        paintAxisTicksOGL(xb, -yb, zmin, xb, -yb, zmax, m_axisZ.phys[0], m_axisZ.phys[1], signedZAx, signedZAy, signedZA, m_axisZ.label, m_axisZ.unit, 0);
    }
    // make z-ticks adjustable
    m_ticklength *= 10.0*m_z_tickmulti;

    if(m_axisX.show && m_axisY.show)
    {
        //static double a = -1.0;
        //static double b = -1.0;
        paintAxisTicksOGL(xmin, yb, zmin, xmax, yb, zmin, m_axisX.phys[0], m_axisX.phys[1], signedXAx, signedXAy, signedZA, m_axisX.label, m_axisX.unit, 1 & m_axisX.showTicks);
        paintAxisTicksOGL(xb, ymin, zmin, xb, ymax, zmin, m_axisY.phys[0], m_axisY.phys[1], signedYAx, signedYAy, signedZA, m_axisY.label, m_axisY.unit, 1 & m_axisY.showTicks);
    }

    // make z-ticks adjustable
    m_ticklength /= m_z_tickmulti;

    if(m_axisZ.show)
    {
        paintAxisTicksOGL(xb, -yb, zmin, xb, -yb, zmax, m_axisZ.phys[0], m_axisZ.phys[1], signedZAx, signedZAy, signedZA, m_axisZ.label, m_axisZ.unit, 1 & m_axisZ.showTicks);
    }

    if(m_colorBarMode != COLORBAR_NO)
    {
        switch(m_colorBarMode)
        {
            case COLORBAR_LEFT:
                DrawColorBar(-1, 0, 0.02f, 0.6f, m_axisZ.phys[0], m_axisZ.phys[1]);
                break;
            case COLORBAR_RIGHT:
                DrawColorBar(1, 0, 0.02f, 0.6f, m_axisZ.phys[0], m_axisZ.phys[1]);
                break;
            case COLORBAR_UPPER_RIGHT:
                DrawColorBar(1, 1, 0.02f, 0.6f, m_axisZ.phys[0], m_axisZ.phys[1]);
                break;
        }
    }

    m_ticklength *= (m_z_tickmulti / 10.0);

    return;
}

//-----------------------------------------------------------------------------------------------
/**
*\fn int dreidogl_achse(FilterObject *fo, double x0, double y0, double z0, double x1, double y1, double z1)
*\brief Zeichnet die Achsenstriche
*\return error
*/
void plotGLWidget::paintAxisOGL(double x0, double y0, double z0, double x1, double y1, double z1)
{
    if (m_linewidth == 0)
        return;

    glLineWidth(m_linewidth);

    if (m_backgnd)
        glColor3f(0, 0, 0);
    else
        glColor3f(1, 1, 1);

    //Paint the axis itsself
    glBegin(GL_LINES);
        glVertex3f(x0, y0, z0);
        glVertex3f(x1, y1, z1);
    glEnd();

    return;
}

//-----------------------------------------------------------------------------------------------
/**
*\fn static void dreidogl_lichtpfeil(class FilterObject *fo)
*\brief Zeichnet den Lichtpfeil fuer die Beleuchtungsdarstellung
*\param[in] *fo Zeiger auf das Goofi-Filterobjekt
*\return error
*\ingroup 3DOGLFuncsGroup
*/
void plotGLWidget::paintLightArrow()
{
    GLfloat position1[3];
    double norm1;

    glPushMatrix();
    glLoadIdentity();

    glLineWidth(2);

    if (m_backgnd)
        glColor3f(0, 0, 0);
    else
        glColor3f(1, 1, 1);

    position1[0] = cos(lighDirAngles[0]);
    position1[1] = sin(lighDirAngles[0]);
    position1[2] = tan(lighDirAngles[1]);
    norm1 = sqrt((position1[0] * position1[0]) + (position1[1]*position1[1]) + (position1[2]*position1[2]));
    position1[0] /= norm1;
    position1[1] /= norm1;
    position1[2] /= norm1;
    position1[2] *= 0.8f;

    //Paint the axis itsself
    glBegin(GL_LINES);
        glVertex3f(position1[0]-0.03, position1[1], position1[2]);
        glVertex3f(position1[0], position1[1], position1[2]+0.1);

        glVertex3f(position1[0]+0.03, position1[1], position1[2]);
        glVertex3f(position1[0], position1[1], position1[2]+0.1);

        glVertex3f(position1[0], position1[1]-0.03, position1[2]);
        glVertex3f(position1[0], position1[1], position1[2]+0.1);

        glVertex3f(position1[0], position1[1]+0.03, position1[2]);
        glVertex3f(position1[0], position1[1], position1[2]+0.1);

        glVertex3f(position1[0]+0.03, position1[1], position1[2]);
        glVertex3f(position1[0], position1[1]+0.03, position1[2]);

        glVertex3f(position1[0], position1[1]+0.03, position1[2]);
        glVertex3f(position1[0]-0.03, position1[1], position1[2]);

        glVertex3f(position1[0]-0.03, position1[1], position1[2]);
        glVertex3f(position1[0], position1[1]-0.03, position1[2]);

        glVertex3f(position1[0], position1[1]-0.03, position1[2]);
        glVertex3f(position1[0]+0.03, position1[1], position1[2]);
    glEnd();
    glPopMatrix();
}

//-----------------------------------------------------------------------------------------------
/**
*\brief Zeichnet die Achsenstriche
*\param[in] *fo Zeiger auf das Goofi-Filterobjekt
*\param[in] write Flag fuer das Schreiben von Tick-Lables
*\return error
*\ingroup 3DOGLFuncsGroup
*/
void plotGLWidget::paintAxisTicksOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1, const double v0, const double v1, const double VorzX, const double VorzY, const double VorzZ, const std::string &symbol, const std::string &unit, const bool write)
{
    double s0, s1, d0, d1, p = 0, v, l, ldv, a;
    long e, b = 0;

    double sl1 = m_ticklength;
    double xstart, xend, ystart, yend, phi;
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];
    GLdouble glx0, gly0, glz0, glx1, gly1, glz1, xpos, ypos, zpos;
    struct AxisLabel al;
    double ticklength = 0.01;
    double VorzXAs, VorzYAs, VorzZAs;
    int firstdigit, i;
    std::string label(" ");

    static double corrX = 0;
    static double corrY = 0;
    al.rightAligned = VorzX > 0;
    al.topAligned = VorzY > 0;

    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);

    gluProject(x0, y0, z0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &glx0, &gly0, &glz0);
    gluProject(x1, y1, z1, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &glx1, &gly1, &glz1);

    d0 = s0 = v0;
    d1 = s1 = v1;
    if (m_linewidth == 0)
        return;

    glLineWidth(m_linewidth);

    if (m_backgnd)
        glColor3f(0, 0, 0);
    else
        glColor3f(1, 1, 1);

    l = sqrt((double)(glx1 - glx0) * (glx1 - glx0) + (double)(gly1 - gly0) * (gly1 - gly0));

    if(sl1 >= l || !sl1)
        return;

    if(!ito::isFinite<double>(s0) || !ito::isFinite<double>(s1) ||s0 >= s1)
        return;

    ldv = log10((s1 - s0) * sl1 / l);

    e = floor(ldv + .167);
    ldv -= e;
    b = 1;
    if(ldv > .167)
    {
        b = 2;
        if( ldv > .5)
            b=5;
    }
    p = pow(10.0, e > 0 ? e : -e);

    al.lastdigit = e;
    if ((x1 == x0) && (z1 == z0))
    {
        VorzYAs = 0;
    }
    else
    {
        VorzYAs = fabs(y0) / y0;
    }

    if ((y1 == y0) && (z1 == z0))
    {
        VorzXAs = 0;
    }
    else
    {
        VorzXAs = fabs(x0) / x0;
    }

    if ((x1 == x0) && (y1 == y0))
    {
        VorzZAs = 0;
    }
    else
    {
        VorzZAs = 1;
    }

    if (write)
    {
        al.write = write;
        label.reserve(100);
        bool alreadyScaled = false;
        al.maxlen = 0;
        firstdigit = floor( log10(fabs(v0) > fabs(v1) ? fabs(v0) : fabs(v1) ) + 10 * DBL_EPSILON );

        if(firstdigit >= -15 && firstdigit < 21)
        {
            al.unitydigit = firstdigit > 0 ? firstdigit / 3 * 3 : -(2 - firstdigit) / 3 * 3;
        }
        else
        {
            al.unitydigit = firstdigit;
        }

        if(unit.length() == 0 || !unit.compare(" "))
        {
            al.unitydigit = std::numeric_limits<int>::min();
        }
        else if(!unit.compare("%"))
        {
            al.unitydigit = -2;
        }
        else
        {
            for(i = 0; dont_scale_units[i] != NULL; i++)
            {
                if(0 == unit.compare(dont_scale_units[i]))
                {
                    al.unitydigit = std::numeric_limits<int>::min();
                    break;
                }
            }
        }

        if( unit.compare("m") || unit.compare("mm") ? 0 : m_xybase)
        {
            al.unitydigit = floor(log10(m_xybase) + 10 * DBL_EPSILON);
        }
        else if(!unit.compare("mm"))
        {
            alreadyScaled = false;
            al.unitydigit += 3;
        }

        if( al.unitydigit< al.lastdigit && al.unitydigit != std::numeric_limits<int>::min() && al.lastdigit != std::numeric_limits<int>::max())
        {
            al.lastdigit = al.unitydigit;
        }

        if( al.unitydigit >= al.lastdigit )
        {
            al.unity = al.unitydigit > 0 ? pow(10.0, al.unitydigit) : 1 / pow(10.0,-al.unitydigit);
            if(!al.unitydigit || al.unitydigit == -2)
            {
                label.append(symbol);
                if(unit.length())
                {
                    label.append(" in ");
                    label.append(unit);
                }

                //sprintf(label,"%s [%s]", symbol, unit);
            }
            else if(al.unitydigit >= -15 && al.unitydigit < 21)
            {
                label.append(symbol);
                if(unit.length())
                {
                    label.append(" in ");

                    char sign[2] = {("afpn\u00B5m-kMGTPE"[al.unitydigit / 3 + 6]), 0}; // Ermittlung der Masseinheit
                    label.append(sign);

                    if(alreadyScaled)
                    {
                        label.append(&(unit.data()[1]));
                    }
                    else
                    {
                        label.append(unit);
                    }
                }
                //sprintf( label,"%s [%c%s]", symbol, , unit);
            }
            else
            {
                label.append(symbol);
                char temp[50] = {0};
                if(unit.length())
                {
                    sprintf(temp," in %.0e%s", al.unity, unit.data());
                    label.append(temp);
                }
                else
                {
                    sprintf(temp,"[%.0e]", al.unity);
                    label.append(temp);
                }
            }
        }
        else
        {
            label.append(symbol);
            if(unit.length())
            {
                label.append(" in ");
                label.append(unit);
            }
        }

        glPushMatrix();
        GLViewport[0] = -1;
        GLViewport[1] = -1;
        GLViewport[2] = 2;
        GLViewport[3] = 2;
        gluProject(x0, y0, z0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xstart, &ystart, &zpos);
        gluProject(x1, y1, z1, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xend, &yend, &zpos);
        phi = atan2((yend - ystart), (xend - xstart));

        al.dx = VorzX * 0.05 * fabs(sin(phi));
        al.dy = VorzY * 0.1 * fabs(cos(phi));

        al.maxlen = 0;
    }

    v = s0;
    v = ceil(e > 0 ? v / p / b : v * p / b);
    v = e > 0 ? v * b * p : v * b / p;
    while(v <= s1)
    {
        a = (v - d0) / (d1 - d0);
        glBegin(GL_LINES);
            glVertex3f(x0 + a * (x1 - x0), y0 + a * (y1 - y0), z0 + a * (z1 - z0));
            if (write)
                glVertex3f(x0 + a * (x1 - x0) + VorzXAs * ticklength * 1.7,
                    y0 + a * (y1 - y0) + VorzYAs * ticklength * 1.7,
                    z0 + a * (z1 - z0) - VorzZAs * ticklength * 1.7);
            else
                glVertex3f(x0 + a * (x1 - x0) + VorzXAs * ticklength,
                    y0 + a * (y1 - y0) + VorzYAs * ticklength,
                    z0 + a * (z1 - z0) - VorzZAs * ticklength);
        glEnd();

        if (write)
        {
            gluProject(x0 + a * (x1 - x0) + VorzXAs * ticklength * 3, y0 + a * (y1 - y0) + VorzYAs * ticklength * 3,
                z0 + a * (z1 - z0) - VorzZAs * ticklength, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xpos, &ypos, &zpos);

            //paintAxisLabelOGL((void*)&al, xpos*(1 + 0.07 * m_windowXScale * fabs(m_axisX.idx[1] - m_axisX.idx[0] + 1.0)),
            //    ypos * (1 + 0.03 * m_windowYScale * fabs(m_axisY.idx[1] - m_axisY.idx[0] + 1.0)), v);

            paintAxisLabelOGL(al, xpos, ypos, v);
        }

        v = v * (v > 0 ? 1 + 4 * DBL_EPSILON : 1 - 4 * DBL_EPSILON);
        v = ceil((e > 0 ? v / p / b : v * p / b) + .5);
        v = e > 0 ? v * b * p : v * b / p;
    }

    if (write)
    {
        gluProject(x0 + (x1 - x0) / 2.0, y0 + (y1 - y0) / 2.0,
            z0 + (z1 - z0) / 2.0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xpos, &ypos, &zpos);

        //al.dx = VorzX * (0.15 * m_windowXScale * fabs(m_axisX.idx[1] - m_axisX.idx[0] + 1.0)) * fabs(sin(phi));
        //al.dy = VorzY * (0.25 * m_windowYScale * fabs(m_axisY.idx[1] - m_axisY.idx[0] + 1.0)) * fabs(cos(phi));
        //
        //if (al.dx < 0)
        //    xpos += al.dx - label.length() * 0.015;
        //else
        //    xpos += al.dx;

        //ypos -= fabs(al.dy);
        OGLTextOut(label.data(), xpos, ypos, al.rightAligned, al.topAligned);
    }

    return;
}
//-----------------------------------------------------------------------------------------------
/**
*\fn static void dreidogl_AxisLabel(class FilterObject *fo, void *vd, double x, double y,double v)
*\brief Schreibt die Achsenlabel
*\param[in] *fo Zeiger auf das Goofi-Filterobjekt
*\return error
*\ingroup 3DOGLFuncsGroup
*/
void plotGLWidget::paintAxisLabelOGL(const struct AxisLabel &axisLabel, const double x, const double y, const double v)
{
    char buffer[300];
    //long l;

    if(v == 0)
    {
        _snprintf(buffer,sizeof(buffer),"0");
    }
    else if(axisLabel.unitydigit >= axisLabel.lastdigit)
    {
        _snprintf(buffer,sizeof(buffer),"%.*f", axisLabel.unitydigit- axisLabel.lastdigit, v / axisLabel.unity);
    }
    else
    {
        int firstdigit=floor(log10(fabs(v))+10*DBL_EPSILON);
        if(axisLabel.unitydigit == INT_MIN && axisLabel.lastdigit != INT_MAX)
        {
            if (axisLabel.lastdigit >= 0)
            {
                // CK 09.08.2006 fix wrecked number displaying:
                // in case the number is larger than 8 digits use exponential notation
                if (fabs(v) >= 1.0E8)
                    _snprintf(buffer, sizeof(buffer), "%g", v);
                else
                    _snprintf(buffer, sizeof(buffer), "%.0f", v);
            }
            else if(firstdigit>=-3&&firstdigit<3)
                _snprintf(buffer,sizeof(buffer),"%.*f",(firstdigit>0?firstdigit:0)- axisLabel.lastdigit , v);
            else
            {double v1, mant;
             int expo;

                v1 = fabs(v);
                mant = pow(10.0, log10(v1)-floor(log10(v1)));
                expo = floor(log10(v1));
                _snprintf(buffer,sizeof(buffer),"%s%.*fE%d", v<0?"-":"", firstdigit - axisLabel.lastdigit, mant, expo);
            }
        }
        else if(firstdigit>=0&&firstdigit<3)
            _snprintf(buffer,sizeof(buffer),"%.0f",v);
        else if(firstdigit >= 21 || firstdigit < -18 || axisLabel.unitydigit == INT_MIN)
        {
            double v1 = fabs(v);
            _snprintf(buffer,sizeof(buffer),"%s%.0fE%d", v<0?"-":"", pow(10.0, log10(v1)-floor(log10(v1))),
            (int)floor(log10(v1)));
        }
        else
        {
            int rest=firstdigit>0?firstdigit%3:2-(2-firstdigit)%3;
            _snprintf(buffer, sizeof(buffer), "%.0f%c", pow(10.0, rest), "afpn\u00B5m-kMGTPE"[(firstdigit - rest) / 3 + 6]); // Ermittlung der Masseinheit
        }
    }

    //l = (long)strlen(buffer);
    //if(l > axisLabel.maxlen)
    //{
    //    axisLabel.maxlen = l;
    //}

    if(! axisLabel.write)
    {
        return;
    }

    double xpos = x;
    double ypos = y;
    //if (axisLabel->dx < 0)
    //    xpos = x + axisLabel->dx - strlen(buffer)*0.015;
    //else
    //    xpos = x + axisLabel->dx;

    //ypos = y - fabs(axisLabel->dy);
    OGLTextOut(buffer, x, y, axisLabel.rightAligned, axisLabel.topAligned);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
int plotGLWidget::OGLTextOut(const char *buffer, double xpos, double ypos, const bool rightAligned, const bool topAligned)
{
    glPushAttrib(GL_LIST_BIT);                // Pushes The Display List Bits
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if(rightAligned) xpos -= (1.7 * strlen(buffer) * m_fontsize) / width();
    if(topAligned) ypos -= (3.0 * m_fontsize) / height();

    glRasterPos2f(xpos, ypos);
    glListBase(m_myCharBitmapBuffer);                    // Sets The Base Character to 0
    glCallLists((GLsizei)strlen(buffer), GL_UNSIGNED_BYTE, buffer);    // Draws The Display List Text

    glPopAttrib();                        // Pops The Display List Bits

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    return glGetError();
}

//----------------------------------------------------------------------------------------------------------------------------------

void plotGLWidget::OGLMakeFont(int size)
{
    QFont oldFont = font();
#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
    QFont myFont("Arial", -size, QFont::Light & QFont::PreferBitmap);
#else
    QFont myFont("Arial", -size, QFont::Light & QFont::OpenGLCompatible & QFont::PreferBitmap);
#endif    
    this->setFont(myFont);

    if(m_myCharBitmapBuffer != 0)
        glDeleteLists(m_myCharBitmapBuffer, 256);
    m_myCharBitmapBuffer = glGenLists(256);            // Storage For 256 Characters
#ifdef WIN32
    HWND hwnd = (HWND)winId();
    wglUseFontBitmaps(GetDC(hwnd), 0, 255, m_myCharBitmapBuffer);            // Builds 96 Characters Starting At Character 32
#endif

    this->setFont(oldFont);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::DrawObjectInfo(void)
{
    double x0 = -1.0 + (double)m_fontsize / width();
    double y0 = -1.0 + (3 * m_fontsize * 3.0 ) / height();

    if(m_objectInfo.xLength.length()) OGLTextOut((char*)m_objectInfo.xLength.data(), x0, y0, false, false);

    y0 -= 3.0 * m_fontsize/ height();
    if(m_objectInfo.yLength.length()) OGLTextOut((char*)m_objectInfo.yLength.data(), x0, y0, false, false);

    y0 -= 3.0 * m_fontsize/ height();
    if(m_objectInfo.matrix.length()) OGLTextOut((char*)m_objectInfo.matrix.data(), x0, y0, false, false);

    size_t len = m_objectInfo.PeakText.length();

    if(len < m_objectInfo.MeanText.length())
        len = m_objectInfo.MeanText.length();

    if(len < m_objectInfo.DevText.length())
        len = m_objectInfo.DevText.length();

    x0 = 1.0 - (double)(1.7 * len * m_fontsize) / width();
    y0 = -1.0 + (3.0 *m_fontsize * 3.0 ) / height();
    if(m_objectInfo.PeakText.length()) OGLTextOut((char*)m_objectInfo.PeakText.data(), x0, y0, false, false);

    y0 -= 3.0 * m_fontsize / height();
    if(m_objectInfo.MeanText.length()) OGLTextOut((char*)m_objectInfo.MeanText.data(), x0, y0, false, false);

    y0 -= 3.0 * m_fontsize / height();
    if(m_objectInfo.DevText.length()) OGLTextOut((char*)m_objectInfo.DevText.data(), x0, y0, false, false);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::DrawColorBar(const char xPos, const char yPos, const GLfloat dX, const GLfloat dY, const GLfloat zMin, const GLfloat zMax)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    GLfloat x0 = -0.98f;
    GLfloat y0 = -1.0f * dY / 2.0f;

    if(xPos > 0)
    {
        x0 = 1.0f - dX - 7.0f / (double)width() - 12.0f * m_fontsize / (double)width();
    }
    if(yPos != 0)
    {
        if(yPos > 0)
        {
            y0 = 1.0f - 0.02f - m_fontsize / (double)height() - dY;
        }
        else
        {
            y0 = -1.0f + 0.02f + m_fontsize / (double)height();
        }
    }

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);                // Select Our Texture

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glColor3f(255.0, 255.0, 255.0);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0, 1.0); glVertex3f(x0, y0, -1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f(x0 + dX, y0, -1.0);
        glTexCoord2f(1.0, 0.0); glVertex3f(x0 + dX, y0 + dY, -1.0);
        glTexCoord2f(0.0, 0.0); glVertex3f(x0, y0 + dY, -1.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);

    glLineWidth(m_linewidth);

    if (m_backgnd)
        glColor3f(0, 0, 0);
    else
        glColor3f(1, 1, 1);

    //Paint the axis itsself
    glBegin(GL_LINES);
        glVertex3f(x0, y0, -1.0);
        glVertex3f(x0, y0 + dY, -1.0);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(x0 + dX, y0, -1.0);
        glVertex3f(x0 + dX, y0 + dY, -1.0);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(x0, y0 + dY, -1.0);
        glVertex3f(x0 + dX + 4.0 / (double)width(), y0 + dY, -1.0);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(x0, y0, -1.0);
        glVertex3f(x0 + dX + 4.0 / (double)width(), y0, -1.0);
    glEnd();

    char buf[50] = {0};

    sprintf(buf, "%g", zMin);
    OGLTextOut(buf, x0 + dX + 7.0 / (double)width(), y0 - m_fontsize / (double)height() / 2.0, false, false);

    sprintf(buf, "%g", zMax);
    OGLTextOut(buf, x0 + dX + 7.0 / (double)width(), y0 + dY - m_fontsize / (double)height() / 2.0, false, false);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    return;
}

//-----------------------------------------------------------------------------------------------
void plotGLWidget::DrawTitle(const std::string &myTitle, const int texty, int &yused)
{
    int i = 0;
    double x0 = -1 * myTitle.length() * 1.67 * m_fontsize * 0.6 / width();
    double y0 = 0.98 - (double)(++i * 2.0 * 1.67 * m_fontsize) / height();

    /* Titel Object etc. */
    OGLMakeFont(1.67 * m_fontsize);
    if(myTitle.length())
        OGLTextOut((char*)myTitle.data(), x0, y0, false, false);
/*
    OGLMakeFont(1.5*dd->fontsize, dd);
    tags->ReadTagDef(TAG_COMMENT1,(void *)&txt,"");
    if(strlen(txt))
        OGLTextOut(dd, (char*)txt, (double)x0/width()-0.9, (double)(++i*-2.0*1.5*m_fontsize) / height()+0.9);

    tags->ReadTagDef(TAG_COMMENT2,(void *)&txt,"");
    if(strlen(txt))
        OGLTextOut(dd, (char*)txt, (double)x0/this->width()-0.9, (double)(++i*-2.0*1.5*m_fontsize)/this->height()+0.9);

    tags->ReadTagDef(TAG_COMMENT3,(void *)&txt,"");
    if(strlen(txt))
        OGLTextOut(dd, (char*)txt, (double)x0/this->width()-0.9, (double)(++i*-2.0*1.5*m_fontsize)/this->height()+0.9);
*/
    yused=(int)(-i*2.0*texty);

    OGLMakeFont(m_fontsize);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::setZAmplifierer(double value)
{
    if(value < 5.0 && value > 0.001)
        m_zAmpl = value;
    refreshPlot(NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::reduceZAmplifierer(double value)
{
    if(m_zAmpl > 0.001)
        m_zAmpl *= value;

    if(m_zAmpl < 0.001)
        m_zAmpl = 0.001;

    refreshPlot(NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::riseZAmplifierer(const double value)
{
    if(m_zAmpl < 5.0)
        m_zAmpl *= value;

    if(m_zAmpl > 5.0)
        m_zAmpl = 5.0;
    refreshPlot(NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::togglePaletteMode()
{
    switch(m_colorBarMode)
    {
        default:
        case COLORBAR_NO:
            m_colorBarMode = COLORBAR_LEFT;
            break;
        case COLORBAR_LEFT:
            m_colorBarMode = COLORBAR_RIGHT;
            break;
        case COLORBAR_RIGHT:
            m_colorBarMode = COLORBAR_UPPER_RIGHT;
            break;
        case COLORBAR_UPPER_RIGHT:
            m_colorBarMode = COLORBAR_NO;
            break;
    }
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::homeView()
{
    lighDirAngles[0] = 0.0;
    lighDirAngles[1] = 0.0;

    m_zAmpl = 1.0;
    /* Basic view coordinates */

    m_RotA = 0.855 + RotA0;
    m_RotB = 1.571 + RotB0;
    m_RotC = 1.025 + RotC0;
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::toggleIllumination(const bool checked)
{
    if(checked)
    {
        m_colorMode = 0;
    }
    else
    {
      m_colorMode = 1;
      m_drawLightDir = false;
    }
    update();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::toggleIlluminationRotation(const bool checked)
{
    if(!checked)
    {
        m_drawLightDir = false;
    }
    else
    {
        if(m_drawLightDir == false && m_colorMode == 0)
        {
            m_drawLightDir = true;
        }
    }
    update();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool plotGLWidget::lightArrowEnabled()
{
    return m_drawLightDir;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::setColorMap(QString palette)
{
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;


    if(ITOM_API_FUNCS_GRAPH == NULL)
    {
        return;
    }

    if (palette.isEmpty())
    {
        retval = apiPaletteGetNumberOfColorBars(numPalettes);
        m_paletteNum %= numPalettes;
        //if (m_paletteNum == 0)
        //    return;

        retval += apiPaletteGetColorBarIdx(m_paletteNum, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(palette, newPalette);
        retval += apiPaletteGetColorBarIdxFromName(palette, m_paletteNum);
    }

    if(newPalette.colorVector256.size() < 255)
    {
        return;
    }

    m_currentPalette = newPalette.colorVector256;
    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);
    GLfloat *par, *pag, *pab;

    par = (GLfloat*)calloc(255, sizeof(GLfloat));
    pag = (GLfloat*)calloc(255, sizeof(GLfloat));
    pab = (GLfloat*)calloc(255, sizeof(GLfloat));

    for (int i=0; i<255; i++)
    {
        par[i] = (GLfloat)((m_currentPalette[i] & 0xFF0000L) >> 16) / 255.0;
        pag[i] = (GLfloat)((m_currentPalette[i] & 0xFF00L) >> 8) / 255.0;
        pab[i] = (GLfloat)(m_currentPalette[i] & 0xFFL) / 255.0;
    }

    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 256, pag);
//    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 256, par);
//    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 256, pab);
//    ret = glGetError();

    free(par);
    free(pag);
    free(pab);

    glPixelTransferi(GL_RED_SCALE, 1);
    glPixelTransferi(GL_RED_BIAS, 0);
    glPixelTransferi(GL_GREEN_SCALE, 1);
    glPixelTransferi(GL_GREEN_BIAS, 0);
    glPixelTransferi(GL_BLUE_SCALE, 1);
    glPixelTransferi(GL_BLUE_BIAS, 0);
    glPixelTransferf(GL_ALPHA_SCALE, 0.0);
    glPixelTransferf(GL_ALPHA_BIAS,  1.0);

    glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //Screen und Tiefenpuffer leeren
    glPixelTransferi(GL_MAP_COLOR, GL_FALSE);

    unsigned char * src = new unsigned char[m_currentPalette.size() * 4 *2];
    for(int i = 0; i < m_currentPalette.size(); i++)
    {
        src[8 * i]   = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 2];
        src[8 * i + 1] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 1];
        src[8 * i + 2] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i];
        src[8 * i + 3] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 3];
        src[8 * i + 4] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 2];
        src[8 * i + 5] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 1];
        src[8 * i + 6] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i];
        src[8 * i + 7] = ((unsigned char*)m_currentPalette.data())[4 * m_currentPalette.size() - 4 * i + 3];
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, src);

    delete[] src;
    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);

    m_isInit |= IS_INIT;
    ResetColors();
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
/*
    if(m_pContent)
    {
        m_pContent->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);
        update();
        return ito::retOk;
    }
    else
    {
        switch(axis)
        {
            case Qt::ZAxis:
                m_axisZ.startScaled = true;
                m_axisZ.phys[0] = minValue;
                m_axisZ.phys[1] = maxValue;
            break;
            case Qt::YAxis:
                m_axisY.startScaled = true;
                m_axisY.phys[0] = minValue;
                m_axisY.phys[1] = maxValue;
            break;
            case Qt::XAxis:
                m_axisX.startScaled = true;
                m_axisX.phys[0] = minValue;
                m_axisX.phys[1] = maxValue;
            break;
        }
    }
 */
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::setCurrentVisMode(const int mode)
{
    m_isInit &= ~HAS_TRIANG;

    if (m_pColTriangles != NULL)
    {
        free(m_pColTriangles);
        m_pColTriangles = NULL;
    }
    if (m_pTriangles != NULL)
    {
        free(m_pTriangles);
        m_pTriangles = NULL;
    }
    if (m_pColIndices != NULL)
    {
        free(m_pColIndices);
        m_pColIndices = NULL;
    }
    if (m_pNormales != NULL)
    {
        free(m_pNormales);
        m_pNormales = NULL;
    }
    if (m_pPoints != NULL)
    {
        free(m_pPoints);
        m_pPoints = NULL;
    }

    switch(mode)
    {
        default:
        case PAINT_TRIANG:
            m_elementMode = PAINT_TRIANG;
            m_axisZ.show = true;
            if(!m_cmplxState) ((ItomIsoGLWidget*)m_pParent)->enableIlluGUI(true);
        break;
        case PAINT_POINTS:
            toggleObjectInfoText(true);
            m_colorMode = 1;
            m_backgnd = false;
            m_axisZ.show = false;
            m_elementMode = PAINT_POINTS;
            if(!m_cmplxState) ((ItomIsoGLWidget*)m_pParent)->enableIlluGUI(false);
        break;
    }

    m_forceReplot = true;
    refreshPlot(NULL);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
inline void plotGLWidget::toggleObjectInfoText(const bool enabled)
{
    if (enabled)
    {
        if (m_pContentDObj)
        {
            ito::dObjHelper::devValue(m_pContentDObj.data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
            generateObjectInfoText();
            m_objectInfo.show = 1;
        }
    }
    else
    {
        m_objectInfo.show = 0;
    }
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
inline void plotGLWidget::generateObjectInfoText()
{
    char buf[40] = {0};

    if(m_axisY.unit.length() && m_axisY.unit.length() < 5) sprintf(buf, "Heigth: %.4g %s", m_axisY.phys[1] - m_axisY.phys[0], m_axisY.unit.data());
    else sprintf(buf, "Heigth: %.4g", m_axisY.phys[1] - m_axisY.phys[0]);
    m_objectInfo.xLength = buf;

    if(m_axisX.unit.length() && m_axisX.unit.length() < 5) sprintf(buf, "Width:  %.4g %s", m_axisX.phys[1] - m_axisX.phys[0], m_axisX.unit.data());
    else sprintf(buf, "Width:  %.4g", m_axisX.phys[1] - m_axisX.phys[0]);
    m_objectInfo.yLength = buf;

    if(m_pContentDObj)
    {
        switch(m_pContentDObj->getType())
        {
            case ito::tInt8:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "int8");
            break;
            case ito::tInt16:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "int16");
            break;
            case ito::tInt32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "int32");
            break;
            case ito::tUInt8:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "uint8");
            break;
            case ito::tUInt16:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "uint16");
            break;
            case ito::tUInt32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "uint32");
            break;
            case ito::tFloat32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "float32");
            break;
            case ito::tFloat64:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "float64");
            break;
            case ito::tComplex64:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "complex64");
            break;
            case ito::tComplex128:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "complex128");
            break;
            default:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axisX.idx[1] - m_axisX.idx[0] + 1, m_axisY.idx[1] - m_axisY.idx[0] + 1, "?");
        }
    }

    m_objectInfo.matrix = buf;

    if(m_axisZ.unit.length() && m_axisZ.unit.length() < 5)
    {
        sprintf(buf, "PV:   %.4g %s", m_axisZ.phys[1] - m_axisZ.phys[0], m_axisZ.unit.data());
        m_objectInfo.PeakText = buf;
        sprintf(buf, "Mean: %.4g %s", m_objectInfo.meanVal, m_axisZ.unit.data() );
        m_objectInfo.MeanText = buf;
        sprintf(buf, "Dev:  %.4g %s", m_objectInfo.divVal, m_axisZ.unit.data() );
        m_objectInfo.DevText = buf;
    }
    else
    {
        sprintf(buf, "PV:   %.4g", m_axisZ.phys[1] - m_axisZ.phys[0]);
        m_objectInfo.PeakText = buf;
        sprintf(buf, "Mean: %.4g", m_objectInfo.meanVal);
        m_objectInfo.MeanText = buf;
        sprintf(buf, "Dev:  %.4g", m_objectInfo.divVal);
        m_objectInfo.DevText = buf;
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::rotateLightArrow(const double deltaA, const double deltaB, const double deltaC)
{
    lighDirAngles[0] += deltaA;
    lighDirAngles[1] += deltaC;
    lighDirAngles[0] = lighDirAngles[0] < - GL_PI ? GL_PI + fmod(lighDirAngles[0], GL_PI) : (lighDirAngles[0] > GL_PI ? (- GL_PI) + fmod(lighDirAngles[0], GL_PI) : lighDirAngles[0]);
    lighDirAngles[1] = lighDirAngles[1] < - GL_PI ? GL_PI + fmod(lighDirAngles[1], GL_PI) : (lighDirAngles[1] > GL_PI ? (- GL_PI) + fmod(lighDirAngles[1], GL_PI) : lighDirAngles[1]);
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::rotateView(const double deltaA, const double deltaB, const double deltaC)
{
    m_RotA += deltaA;
    m_RotB += deltaB;
    m_RotC += deltaC;
    m_RotA = m_RotA < - GL_PI ? GL_PI + fmod(m_RotA, GL_PI) : (m_RotA > GL_PI ? (- GL_PI) + fmod(m_RotA, GL_PI) : m_RotA);
    m_RotB = m_RotB < - GL_PI ? GL_PI + fmod(m_RotB, GL_PI) : (m_RotB > GL_PI ? (- GL_PI) + fmod(m_RotB, GL_PI) : m_RotB);
    m_RotC = m_RotC < - GL_PI ? GL_PI + fmod(m_RotC, GL_PI) : (m_RotC > GL_PI ? (- GL_PI) + fmod(m_RotC, GL_PI) : m_RotC);
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::moveView(const double deltaX, const double deltaY, const double deltaZ)
{
    m_TransX += deltaX;
    m_TransY += deltaY;
    m_TransZ += deltaZ;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::setView(const double transX, const double transY, const double transZ, const double rotA, const double rotB, const double rotC)
{
    m_TransX = transX;
    m_TransY = transY;
    m_TransZ = transZ;
    m_RotA = rotA < - GL_PI ? GL_PI + fmod(rotA, GL_PI) : (rotA > GL_PI ? (- GL_PI) + fmod(rotA, GL_PI) : rotA);
    m_RotB = rotB < - GL_PI ? GL_PI + fmod(rotB, GL_PI) : (rotB > GL_PI ? (- GL_PI) + fmod(rotB, GL_PI) : rotB);
    m_RotC = rotC < - GL_PI ? GL_PI + fmod(rotC, GL_PI) : (rotC > GL_PI ? (- GL_PI) + fmod(rotC, GL_PI) : rotC);
}
//----------------------------------------------------------------------------------------------------------------------------------
