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

#if linux
    #include <unistd.h>
#endif
#include "itomIsoGLFigure.h"
#include "plotIsoGLWidget.h"

#include "common/sharedStructuresGraphics.h"

#include "DataObject/dataObjectFuncs.h"


#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

using namespace ito;

const char *dont_scale_units[] = {"frame", "frames", "frm", "frms", "digit", "digits", "Bild", "Bilder", "Wert", "-"};

#define PI        3.14159265358979323846

#define RotA0       -1.4
#define RotB0       -1.5
#define RotC0       0.0

struct axislabel
{
    axislabel() : dx(0), dy(0), write(0), unitydigit(0), lastdigit(0), unity(0), maxlen(0) {}
    double dx, dy, write;
    int unitydigit, lastdigit;
    double unity;
    long maxlen;
};

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize openGL (below version two - i.e. using static pipelines)
*	@param [in]	width	window width
*	@param [in] height	window height
*	@return		zero for no error, openGL error code otherwise
*/
int initOGL2(const int width, const int height)
{
    int ret = 0;

    glShadeModel(GL_SMOOTH);							//Smooth Shading
    glClearDepth(1.0f);									//Tiefenpuffer setzen
    glEnable(GL_DEPTH_TEST);							//Tiefenpuffertest aktivieren
    glDepthFunc(GL_LEQUAL);								//welcher Test
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	//Perspektivenkorrektur an
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);				//Linien Antialiasing
    glClearColor(255.0f, 255.0f, 255.0f, 0.0f);				//weisser Hintergrund

    glEnable(GL_TEXTURE_2D);
    ret = glGetError();

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glMatrixMode(GL_PROJECTION);					//Projektionsmatrix wählen
    glLoadIdentity();
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);				//Window resizen
    gluOrtho2D(-1.1, 1.1, -1.1, 1.1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if ((ret = glGetError()))
    {
        std::cerr << "error enabeling texutres gl-window init\n";
    }

    return ret;
}


//----------------------------------------------------------------------------------------------------------------------------------
plotGLWidget::plotGLWidget(QMenu *contextMenu, QGLFormat &fmt, QWidget *parent, const QGLWidget *shareWidget):
    QGLWidget(fmt, parent, shareWidget, Qt::Widget),
    m_contextMenu(contextMenu),
    m_pParent(parent),
    m_paletteNum(0),
    m_lineplotUID(0),
    m_cBarTexture(NULL),
    m_drawTitle(false),
    m_backgnd(true),
    m_drawLightDir(false),
    m_forceCubicVoxel(false),
    m_forceReplot(true),
    m_isInit(0),
    m_gamma (1),
    m_TransX(0.0),
    m_TransY(0.0),
    m_TransZ(0.0),
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
    m_pContent(NULL),
    m_pContentWhileRastering(NULL),
    m_invalid(1.6e308)
{
    this->setMouseTracking(false); //(mouse tracking is controled by action in WinMatplotlib)

    m_timer.setSingleShot(true);
    QObject::connect(&m_timer, SIGNAL(timeout()), this, SLOT(paintTimeout()));

    fmt.setOverlay(0);
    if (fmt.swapInterval() != -1)
        fmt.setSwapInterval(0);
    fmt.setProfile(QGLFormat::CoreProfile);

    int glVer = QGLFormat::openGLVersionFlags();

    if (glVer >= 32)
    {
        fmt.setVersion(2,0);
    }

    move(0, 0);
    resize(100, 100);

    fmt.setDepth(0);

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
    m_lightAxisVector[0] = 0.0;
    m_lightAxisVector[1] = 0.0;
    m_lightAxisVector[2] = 0.0;

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

    m_protocol.show = false;
    m_protocol.m_psize = 0;
    m_protocol.text = "";

    m_objectInfo.show = false;
    m_objectInfo.divVal = 0;
    m_objectInfo.meanVal = 0;

    //m_currentPalette.resize(256);

    m_currentPalette.clear();

    if(ITOM_API_FUNCS_GRAPH != NULL && *ITOM_API_FUNCS_GRAPH != NULL)
    {
        int numColBars = 0;
        ito::ItomPalette newPalette;

        apiPaletteGetNumberOfColorBars(numColBars);
        apiPaletteGetColorBarIdx((m_currentColor + 1) % numColBars, newPalette);

        m_currentPalette = newPalette.colorVector256;

    }
    else
    {
        // Only false Colors
        /*
        ito::ItomPalette newPalette("falseColorIR", ito::ItomPalette::FCPalette | ito::ItomPalette::LinearPalette, QColor::fromRgb(165, 30, 165), Qt::white);
        newPalette.insertColorStop(0.15, Qt::blue);
        newPalette.insertColorStop(0.35, Qt::cyan);
        newPalette.insertColorStop(0.55, Qt::green);
        newPalette.insertColorStop(0.75, Qt::yellow);
        newPalette.insertColorStop(0.97, Qt::red);
        */
        m_currentPalette = QVector<ito::uint32>(256);

        for(int i = 0; i < 256; i++)
        {
            m_currentPalette[i] = i + (i << 8) + (i << 16);
        }

    }



    makeCurrent();

    m_errorDisplMsg.clear();
    m_errorDisplMsg.append("No Data");


    /* Basic view coordinates */

    //RotA0 = -1.4;
    //RotB0 = -1.5;
    //RotC0 = 0.0;

    m_RotA = 0.855 + RotA0;
    m_RotB = 1.571 + RotB0;
    m_RotC = 1.025 + RotC0;


    ret = initOGL2(width(), height());

    if(!ret)
        m_isInit = IS_INIT;

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &m_cBarTexture);
    ret = glGetError();

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);
    ret = glGetError();

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

    //glGetIntegerv(GL_MAX_PIXEL_MAP_TABLE, &glval);
    //ret = glGetError();

    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 256, pag);
    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 256, par);
    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 256, pab);
    ret = glGetError();

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

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//Screen und Tiefenpuffer leeren

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
        src = new unsigned char[paletteSize * 4 *2];
        unsigned char* ptrPal =  (unsigned char*)m_currentPalette.data();
        for(int i = 0; i < paletteSize; i++)
        {
            src[8*i] = ptrPal[4*paletteSize - 4 * i - 4];
            src[8*i + 1] = ptrPal[4*paletteSize - 4 * i - 3];
            src[8*i + 2] = ptrPal[4*paletteSize - 4 * i - 2];
            src[8*i + 3] = 255;
            src[8*i + 4] = ptrPal[4*paletteSize - 4 * i - 4];
            src[8*i + 5] = ptrPal[4*paletteSize - 4 * i - 3];
            src[8*i + 6] = ptrPal[4*paletteSize - 4 * i - 2];
            src[8*i + 7] = 255;
/*
            src[8*i] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i];
            src[8*i+1] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 1];
            src[8*i+2] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
            src[8*i+3] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 3];
            src[8*i+4] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i];
            src[8*i+5] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 1];
            src[8*i+6] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
            src[8*i+7] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 3];

            src[8*i]   = (m_currentPalette[paletteSize - i - 1])  & 0x000000FF;
            src[8*i+1] = ((m_currentPalette[paletteSize - i - 1]) & 0x0000FF00) >> 8;
            src[8*i+2] = ((m_currentPalette[paletteSize - i - 1]) & 0x00FF0000) >> 16;
            src[8*i+3] = 0xFF000000;
            src[8*i+4] = (m_currentPalette[paletteSize - i - 1])  & 0x000000FF;
            src[8*i+5] = ((m_currentPalette[paletteSize - i - 1]) & 0x0000FF00) >> 8;
            src[8*i+6] = ((m_currentPalette[paletteSize - i - 1]) & 0x00FF0000) >> 16;
            src[8*i+7] = 0xFF000000;
*/

        }
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, paletteSize, 0, GL_BGRA, GL_UNSIGNED_BYTE, src);

    delete[] src;

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);

    OGLMakeFont(m_fontsize);


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//Screen und Tiefenpuffer leeren

    if (ret = glGetError())
    {
        std::cerr << "error setting up openGLWindow window: " << ret << "\n";
    }


    doneCurrent();

    m_pContent = QSharedPointer<ito::DataObject>(new ito::DataObject());
    m_pContent->ones(3,3,ito::tFloat32);

    refreshPlot(NULL);

}

//----------------------------------------------------------------------------------------------------------------------------------
plotGLWidget::~plotGLWidget()
{
    hide();
    m_isInit |= ~IS_INIT;
    Sleep(100);

    if (m_myCharBitmapBuffer)
        glDeleteLists(m_myCharBitmapBuffer, 256);

    glDeleteTextures(1, &m_cBarTexture);					// Create The Texture

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

    if(m_pContent)
    {
        m_pContent.clear();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::paintTimeout()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::paintGL()
{
    static int drawScene = 0;
    //bool test;
    double winSizeX = (double)this->width();
    double winSizeY = (double)this->height();
    ito::RetVal retval = ito::retOk;
    ito::float64 Sqrt2Div2 = sqrt(2.0) / 2.0;
    ito::float64 xs = 1.0, ys = 1.0, zs = 1.0, maxl = 1.0; //, tempVal;
    ito::float64 nDims = 0;

    if (m_isInit != 3)
        return;

    m_isInit |= IS_RENDERING;

    makeCurrent();

    m_ticklength = (int)(sqrt(winSizeX * winSizeX + winSizeY * winSizeY) * 10/1000); //tut
//	dd->ticklength = sqrt(2.0)/100.0;

    glMatrixMode(GL_MODELVIEW);					//Projektionsmatrix wählen
    glLoadIdentity();

    if (!m_backgnd)
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);				//schwarzer Hintergrund
    else
        glClearColor(255.0f, 255.0f, 255.0f, 0.0f);				//weisser Hintergrund

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//Screen und Tiefenpuffer leeren

    if ( ( ((m_pTriangles == NULL) && (m_elementMode == PAINT_TRIANG)) && ((m_pPoints == NULL) && (m_elementMode == PAINT_POINTS)) ) || (m_pColTriangles == NULL) || (m_NumElements == 0))
    {
        doneCurrent();
        m_isInit &= ~IS_RENDERING;
        return;
    }


    //	glTranslatef(0, 0, -2);
//	glTranslated(TransX, TransY+(double)psize/dd->win->ysize, TransZ);


    glTranslated(m_TransX, m_TransY, m_TransZ);
    glRotated(m_RotA/PI*180.0, 1.0f, 0.0f, 0.0f);
    glRotated(m_RotB/PI*180.0, 0.0f, 1.0f, 0.0f);
    glRotated(m_RotC/PI*180.0, 0.0f, 0.0f, 1.0f);

    threeDRotationMatrix();

    if (m_colorMode == 0 && m_elementMode == PAINT_TRIANG)
    {
        GLfloat ambient[4]={0.0, 0.0, 0.0, 1.0};
        GLfloat diffuse[4]={1.0, 1.0, 1.0, 1.0};
        GLfloat specular[4]={1.0, 1.0, 1.0, 1.0};
        GLfloat emission[4]={0.0, 0.0, 0.0, 1.0};
        GLfloat position[4]={1.0, 1.0, 1.0, 0.0};
        double norm;

        position[0] = cos(lighDirAngles[0]);
        position[1] = sin(lighDirAngles[0]);
        position[2] = tan(lighDirAngles[1]);

        norm = sqrt((position[0]*position[0]) + (position[1]*position[1]) + (position[2]*position[2]));
        position[0] /= norm * 1;
        position[1] /= norm * 1;
        position[2] /= norm * -20;
//		position[2] *= 10;

        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_NORMALIZE);

        glLightfv(GL_LIGHT0, GL_POSITION, position);
//		glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, position);
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

//		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);

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

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, (GLsizei)this->width(), (GLsizei)this->height());				//Window resizen
    gluOrtho2D(-1.1, 1.1, -1.1, 1.1);

    if(m_protocol.show) glTranslatef(0.0, 0.7 * (double) m_protocol.m_psize / winSizeY, 0.0);
    else glTranslatef(0.0, 0.0, 0.0);

    glMatrixMode(GL_MODELVIEW);



//	glEnableClientState(GL_COLOR_ARRAY);

    //glColorPointer(4, GL_FLOAT, 0, ColTriangles);


    if(m_elementMode == PAINT_POINTS)
    {
        glPointSize(1.0f);
        glVertexPointer(3, GL_FLOAT, 0, m_pPoints);

        glColorPointer(4, GL_UNSIGNED_BYTE, 0, m_pColTriangles);
        glDrawArrays(GL_POINTS, 0, m_NumElements);

       // GLfloat test[6] = {0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f};
       // GLubyte col[8] = {255, 0, 255, 255, 255, 0, 0, 255};
        //
        //glVertexPointer(3, GL_FLOAT, 0, &test);
        //glColorPointer(4, GL_UNSIGNED_BYTE, 0, &col);
        //
        //glDrawArrays(GL_POINTS, 0, 2);
    }
    else
    {
        glVertexPointer(3, GL_FLOAT, 0, m_pTriangles);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, m_pColTriangles);

        if (m_colorMode == 0) glNormalPointer (GL_FLOAT, 0, m_pNormales);
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

    if (m_drawLightDir)
        paintLightArrow();

    threeDAxis();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//	glViewport(0, 0, this->width(), this->height());				//Window resizen
    gluOrtho2D(-1.1, 1.1, -1.1, 1.1);
    glMatrixMode(GL_MODELVIEW);

    if (m_drawTitle)
    {	/* noobjinfo */
//		setcolor(win,dd->backgnd?win->bcolor:win->fcolor);
        //int yused;
        int texty = 1.0;
        //DrawTitle(m_title, texty, yused);
    }

    if(m_protocol.show) /* protocol */
    {
//		setcolor(win,dd->backgnd?win->bcolor:win->fcolor);
        //DrawProtocol(dd, dd->gy1, fo->args[27].s, (int)fo->args[28].d);
    }

    if(m_objectInfo.show)
    {
        DrawObjectInfo();
    }

    int ret = glGetError();

    glFlush();
    //glFinish();
    this->swapBuffers();

    doneCurrent();

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

    ito::float64 pixval = 0.0;

    ito::float64 norm = m_axisZ.phys[1] - m_axisZ.phys[0];

    if (!ito::dObjHelper::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

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

    ito::float64 pixval = 0.0;

    if (!ito::dObjHelper::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

    switch(m_cmplxMode)
    {
        default:
        case 0:
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

    // rescale the invalud value
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
    ito::float64 pixval = 0.0;

    if (!ito::dObjHelper::isNotZero<ito::float64>(norm)) norm = 1.0;    // if is zero set to 1.0

    switch(m_cmplxMode)
    {
        default:
        case 0:
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

    // rescale the invalud value
    normedInvalid = (m_invalid - m_axisZ.phys[0]) / norm;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::GLSetTriangles(int &mode)
{
    ito::RetVal retVal(retOk);

    int xsizeObj = m_axisX.idx[1] - m_axisX.idx[0];
    int ysizeObj = m_axisY.idx[1] - m_axisY.idx[0];

    if(m_pContent && xsizeObj && xsizeObj)
    {
        m_pContentWhileRastering = m_pContent;
        m_pContentWhileRastering->lockRead();
    }
    else
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
        return ito::RetVal(ito::retError, 0, "DataObject empty, calc triangles failed");
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
            m_pContentWhileRastering->unlock();
            return ito::RetVal(ito::retError, 0, "Unknown DataObject-Type, calc triangles failed");
    }

    m_NumElements = 0;

    int count = 0;

    ito::float64 xshift = m_windowXScale * xsizeObj / 2.0;
    ito::float64 yshift = m_windowYScale * ysizeObj / 2.0;
    ito::float64 zshift = 0.5;

    int cntY;
    int cntX;
    bool isFinite = false;

    GLfloat Vec1[3];
    GLfloat Vec2[3];

    ito::float64 color = 0.0;
    ito::float64 zsum = 0.0;
    ito::float64 dpixel1;
    ito::float64 dpixel2;
    ito::float64 dpixel3;
    ito::float64 Schwelle = 1.0;
    ito::float64 ZScale = 1.0;

    ito::float64 *ptrScaledTopo = NULL;
    ito::float64 *ptrScaledTopoNext = NULL;

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
                if (m_pColTriangles != NULL)
                {
                    free(m_pColTriangles);
                    m_pColTriangles = NULL;
                }
                if (m_pPoints != NULL)
                {
                    free(m_pPoints);
                    m_pTriangles = NULL;
                }
                if (m_pColIndices != NULL)
                {
                    free(m_pColIndices);
                    m_pColIndices = NULL;
                }
                m_NumElements = 0;
                retVal += ito::RetVal(ito::retError, 0, "No data compueted");
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
                m_NumElements = 0;
                retVal += ito::RetVal(ito::retError, 0, "No data compueted");
            }
        }
    }
    if(!retVal.containsError())
    {
        if(m_elementMode == PAINT_POINTS)
        {
            for (cntY = 0; cntY < ysizeObj -1; cntY++)
            {
                ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(cntY);

                for (cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    dpixel1 = ptrScaledTopoNext[cntX];


                    if ((fabs(color - dpixel1) < Schwelle) && ito::dObjHelper::isFinite<ito::float64>(dpixel1) && (dpixel1 != invalidValue))
                    {
                        m_pPoints[count*3] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pPoints[count*3+1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pPoints[count*3+2] = ZScale * dpixel1 - zshift;

                        m_pColIndices[count] = cv::saturate_cast<unsigned char>(dpixel1*255.0);
                        m_NumElements++;
                        count++;
                    }
                }
            }
        }
        else
        {
            ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(0);

            for (cntY = 0; cntY < ysizeObj -1; cntY++)
            {
                ptrScaledTopo = ptrScaledTopoNext;
                ptrScaledTopoNext = (ito::float64*)scaledTopo.ptr(cntY+1);

                for (cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    dpixel1 = ptrScaledTopoNext[cntX];

                    dpixel2 = ptrScaledTopo[cntX + 1];


                    dpixel3 = ptrScaledTopo[cntX];

                    if(ito::dObjHelper::isFinite<ito::float64>(dpixel1) && ito::dObjHelper::isFinite<ito::float64>(dpixel2) && ito::dObjHelper::isFinite<ito::float64>(dpixel2))
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

                    if ((fabs(color - dpixel1) < Schwelle) && (fabs(color - dpixel2) < Schwelle) && (fabs(color - dpixel3) < Schwelle)
                        && /*(fabs(dpixel1) < 1.6e308)*/ isFinite && (dpixel1 != invalidValue) && (dpixel2 != invalidValue) && (dpixel3 != invalidValue))
                    {
                        m_pTriangles[count*9] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count*9+1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count*9+2] = ZScale * dpixel1 - zshift;

                        m_pTriangles[count*9+3] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count*9+4] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count*9+5] = ZScale * dpixel2 - zshift;

                        m_pTriangles[count*9+6] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count*9+7] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count*9+8] = ZScale * dpixel3 - zshift;

                        for (int n=0;n<3;n++)
                        {
                            Vec1[n] = m_pTriangles[count*9+3+n] - m_pTriangles[count*9+n];
                            Vec2[n] = m_pTriangles[count*9+6+n] - m_pTriangles[count*9+n];
                        }

                        m_pNormales[count*9+6] = m_pNormales[count*9+3] = m_pNormales[count*9] = (Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1]);
                        m_pNormales[count*9+7] = m_pNormales[count*9+4] = m_pNormales[count*9+1] = (Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2]);
                        m_pNormales[count*9+8] = m_pNormales[count*9+5] = m_pNormales[count*9+2] = (Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0]);

                        m_pColIndices[count * 3] = cv::saturate_cast<unsigned char>(dpixel1 * 255.0);
                        m_pColIndices[count * 3 + 1] = cv::saturate_cast<unsigned char>(dpixel2 * 255.0);
                        m_pColIndices[count * 3 + 2] = cv::saturate_cast<unsigned char>(dpixel3 * 255.0);

                        m_NumElements++;
                        count++;
                    }


                    dpixel1 = ptrScaledTopoNext[cntX];
                    dpixel2 = ptrScaledTopoNext[cntX + 1];
                    dpixel3 = ptrScaledTopo[cntX+1];

                    if(ito::dObjHelper::isFinite<ito::float64>(dpixel1) && ito::dObjHelper::isFinite<ito::float64>(dpixel2) && ito::dObjHelper::isFinite<ito::float64>(dpixel3))
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

                    if ((fabs(color - dpixel1) < Schwelle) && (fabs(color - dpixel2) < Schwelle) && (fabs(color - dpixel3) < Schwelle)
                        && isFinite /*(fabs(dpixel1) < 1.6e308)*/ && (dpixel1 != invalidValue) && (dpixel2 != invalidValue) && (dpixel3 != invalidValue))
                    {

                        m_pTriangles[count*9] = ((double)(cntX) * m_windowXScale - xshift);
                        m_pTriangles[count*9+1] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count*9+2] = ZScale * dpixel1 - zshift;

                        m_pTriangles[count*9+3] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count*9+4] = ((double)(cntY + 1) * m_windowYScale - yshift);
                        m_pTriangles[count*9+5] = ZScale * dpixel2 - zshift;

                        m_pTriangles[count*9+6] = ((double)(cntX + 1) * m_windowXScale - xshift);
                        m_pTriangles[count*9+7] = ((double)(cntY) * m_windowYScale - yshift);
                        m_pTriangles[count*9+8] = ZScale * dpixel3 - zshift;

                        for (int n=0;n<3;n++)
                        {
                            Vec1[n] = m_pTriangles[count*9+3+n] - m_pTriangles[count*9+n];
                            Vec2[n] = m_pTriangles[count*9+6+n] - m_pTriangles[count*9+n];
                        }
                        m_pNormales[count*9+6] = m_pNormales[count*9+3] = m_pNormales[count*9] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
                        m_pNormales[count*9+7] = m_pNormales[count*9+4] = m_pNormales[count*9+1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
                        m_pNormales[count*9+8] = m_pNormales[count*9+5] = m_pNormales[count*9+2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];

                        m_pColIndices[count * 3] = cv::saturate_cast<unsigned char>(dpixel1 * 255.0);
                        m_pColIndices[count * 3 + 1] = cv::saturate_cast<unsigned char>(dpixel2 * 255.0);
                        m_pColIndices[count * 3 + 2] = cv::saturate_cast<unsigned char>(dpixel3 * 255.0);

                        m_NumElements++;
                        count++;
                    }
                }
            }
        }

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
        retVal += ito::RetVal(ito::retError, 0, "No data compueted");
        m_isInit &= ~IS_CALCTRIANG;
    }
    else
    {

        if(m_elementMode == PAINT_POINTS)
        {
            m_pColIndices = static_cast<unsigned char *>(realloc(m_pColIndices, m_NumElements*sizeof(unsigned char)));
            m_pPoints = static_cast<GLfloat *>(realloc(m_pPoints, m_NumElements*3*sizeof(GLfloat)));
            m_pColTriangles = static_cast<GLubyte *>(realloc(m_pColTriangles, m_NumElements*4*sizeof(GLubyte)));
        }
        else
        {
            m_pColIndices = static_cast<unsigned char *>(realloc(m_pColIndices, m_NumElements * 3 * sizeof(unsigned char)));
            m_pTriangles = static_cast<GLfloat *>(realloc(m_pTriangles, m_NumElements*9*sizeof(GLfloat)));
            m_pColTriangles = static_cast<GLubyte *>(realloc(m_pColTriangles, m_NumElements*12*sizeof(GLubyte)));
            m_pNormales = static_cast<GLfloat *>(realloc(m_pNormales, m_NumElements*9*sizeof(GLfloat)));
        }

        ResetColors();

        m_errorDisplMsg.clear();

        m_isInit &= ~IS_CALCTRIANG;
    }

    m_pContentWhileRastering->unlock();
    m_pContentWhileRastering.clear();

    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::rescaleTriangles(const double xscaleing, const double yscaleing, const double zscaleing)
{
    long n, m;
    double Vec1[3], Vec2[3];


    if(m_elementMode == PAINT_POINTS)
    {
        if(!m_pPoints)
            return;

        for (n = 0; n < m_NumElements; n++)
        {
            m_pPoints[n*3] *= xscaleing;
            m_pPoints[n*3+1] *= yscaleing;
            m_pPoints[n*3+2] *= zscaleing;
        }

    }
    else
    {
        if(!m_pTriangles || !m_pNormales)
            return;

        for (n = 0; n < m_NumElements*3; n++)
        {
            m_pTriangles[n*3] *= xscaleing;
            m_pTriangles[n*3+1] *= yscaleing;
            m_pTriangles[n*3+2] *= zscaleing;
        }

        for ( n = 0; n < m_NumElements ; n++)
        {
            for (m=0;m<3;m++)
            {
                Vec1[m] = m_pTriangles[n*9+3+m] - m_pTriangles[n*9+m];
                Vec2[m] = m_pTriangles[n*9+6+m] - m_pTriangles[n*9+m];
            }
            m_pNormales[n*9+6] = m_pNormales[n*9+3] = m_pNormales[n*9] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
            m_pNormales[n*9+7] = m_pNormales[n*9+4] = m_pNormales[n*9+1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
            m_pNormales[n*9+8] = m_pNormales[n*9+5] = m_pNormales[n*9+2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
        }
    }


    ResetColors();

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::paintEvent(QPaintEvent * /*pevent*/)
{
    paintGL();

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::refreshPlot(ito::ParamBase *param)
{

    int ret = 0;
    int width = this->width();
    int height = this->height();
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

    if (m_isInit & IS_RENDERING || m_isInit & IS_RENDERING )
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
            m_pContent = QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj));
            m_pContent->lockRead();

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

            int x1 = m_pContent->getSize(dims - 1) - 1;
            int y1 = m_pContent->getSize(dims - 2) - 1;
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

            m_axisX.phys[0] = m_pContent->getPixToPhys(m_axisX.dimIdx, (double)m_axisX.idx[0], test);
            m_axisX.phys[1] = m_pContent->getPixToPhys(m_axisX.dimIdx, (double)m_axisX.idx[1], test);

            m_axisY.phys[0] = m_pContent->getPixToPhys(m_axisY.dimIdx, (double)m_axisY.idx[0], test);
            m_axisY.phys[1] = m_pContent->getPixToPhys(m_axisY.dimIdx, (double)m_axisY.idx[1], test);

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


            m_protocol.text = m_pContent->getTag("protocol", test).getVal_ToString();

            m_axisX.label = m_pContent->getAxisDescription(dims - 1, test);
            m_axisX.unit  = m_pContent->getAxisUnit(dims - 1, test);
            m_axisY.label = m_pContent->getAxisDescription(dims - 2, test);
            m_axisY.unit  = m_pContent->getAxisUnit(dims - 2, test);
            m_axisZ.label = m_pContent->getValueDescription();
            m_axisZ.unit  = m_pContent->getValueUnit();

            if(!m_axisZ.unit.compare("mm") || !m_axisZ.unit.compare("m"))
                m_axisZ.isMetric = true;

            if(!m_axisX.unit.compare("mm") || !m_axisX.unit.compare("m"))
                m_axisX.isMetric = true;

            if(!m_axisY.unit.compare("mm") || !m_axisY.unit.compare("m"))
                m_axisY.isMetric = true;

            if(m_objectInfo.show)
            {
                ito::dObjHelper::devValue(m_pContent.data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
                generateObjectInfoText();
            }

            m_pContent->unlock();
            m_forceReplot = true;
        }
    }

    if(m_pContent != NULL)
    {
        m_pContent->lockRead();

        //double minXValueold = m_axisX.phys[0];
        //double maxXValueold = m_axisX.phys[1];
        //double minZValueold = m_axisZ.phys[0];
        //double maxZValueold = m_axisZ.phys[1];
        //double minYValueold = m_axisY.phys[0];
        //double maxYValueold = m_axisY.phys[1];

        double windowXScaleOld = m_windowXScale;
        double windowYScaleOld = m_windowYScale;
        double windowZScaleOld = m_windowZScale;


        ito::uint32 firstMin[3];
        ito::uint32 firstMax[3];

        if(m_axisZ.autoScale)
        {
            switch(m_pContent->getType())
            {
                case ito::tUInt8:
                    ito::dObjHelper::minMaxValueFunc<ito::uint8>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tInt8:
                    ito::dObjHelper::minMaxValueFunc<ito::int8>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tUInt16:
                    ito::dObjHelper::minMaxValueFunc<ito::uint16>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tInt16:
                    ito::dObjHelper::minMaxValueFunc<ito::int16>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tUInt32:
                    ito::dObjHelper::minMaxValueFunc<ito::uint32>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tInt32:
                    ito::dObjHelper::minMaxValueFunc<ito::int32>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tFloat32:
                    ito::dObjHelper::minMaxValueFunc<ito::float32>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tFloat64:
                    ito::dObjHelper::minMaxValueFunc<ito::float64>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true);
                break;
                case ito::tComplex64:
                    ito::dObjHelper::minMaxValueFunc<ito::complex64>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true, m_cmplxMode);
                break;
                case ito::tComplex128:
                    ito::dObjHelper::minMaxValueFunc<ito::complex128>(m_pContent.data(), m_axisZ.phys[0], firstMin, m_axisZ.phys[1], firstMax, true, m_cmplxMode);
                break;
                default:
                    retval == ito::retError;
                    m_errorDisplMsg.append("Object has invalid type");

            }
        }

        if(!retval.containsError())
        {
            /*
                if(m_maxXValue < m_minXValue)
                {
                    tempVal = m_minXValue;
                    m_minXValue = m_maxXValue;
                    m_maxXValue = tempVal;
                }
                if(m_maxYValue < m_minYValue)
                {
                    tempVal = m_minYValue;
                    m_minYValue = m_maxYValue;
                    m_maxYValue = tempVal;
                }
                if(m_maxZValue < m_minZValue)
                {
                    tempVal = m_minZValue;
                    m_minZValue = m_maxZValue;
                    m_maxZValue = tempVal;
                }
            */

            if (m_zAmpl < 0.000000001) // Damit auch µm angezeigt werden können
                m_zAmpl = 0.000000001; // Damit auch µm angezeigt werden können

            if(m_axisZ.isMetric) zs = m_axisZ.phys[1] - m_axisZ.phys[0];

            if(m_axisX.isMetric) xs = m_axisX.phys[1] - m_axisX.phys[0];

            if(m_axisY.isMetric) ys = m_axisY.phys[1] - m_axisY.phys[0];

            //if(m_title.length() == 0)
            //{
            //    m_drawTitle = false;
            //}

            ProtocolSize();

            if(m_protocol.show)
            {

            }

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
                if (this->width() > this->height())
                {
                    m_windowXScale = (double)this->height() / (double)this->width();
                    m_windowYScale = 1;
                }
                else
                {
                    m_windowYScale = (double)this->width() / (double)this->height();
                    m_windowXScale= 1;
                }
            }

            m_windowXScale *= xs / maxl;
            m_windowYScale *= ys / maxl;
            if (zs!=1 && m_forceCubicVoxel)
                m_windowZScale *= zs / maxl;

            if (m_protocol.m_psize != 0 && m_protocol.show)
            {
                m_windowXScale /= 1.2*fabs((double)(this->height() - m_protocol.m_psize) / (double)this->height());
                m_windowYScale /= 1.2*fabs((double)(this->height() - m_protocol.m_psize) / (double)this->height());
            }
            else
            {
                m_windowXScale /= 1.2*fabs((double)(this->height()) / (double)this->height());
                m_windowYScale /= 1.2*fabs((double)(this->height()) / (double)this->height());
            }


            if(ito::dObjHelper::isNotZero<double>(m_axisZ.phys[1] - m_axisZ.phys[0]))
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

            m_pContent->unlock();
            int mode = 0;

            if( m_NumElements == 0 || m_forceReplot == true)
            {
                GLSetTriangles(mode);
            }
            else if ((m_windowXScale != windowXScaleOld) || (m_windowYScale != windowYScaleOld) || (m_windowZScale != windowZScaleOld))
                rescaleTriangles(m_windowXScale / windowXScaleOld, m_windowYScale / windowYScaleOld, m_windowZScale / windowZScaleOld);

        }
    }
    else
    {
        m_errorDisplMsg.clear();
        m_errorDisplMsg.append("Object empty");
        retval == ito::retError;
    }

    if (retval == ito::retOk)
    {
        m_isInit |= HAS_TRIANG;
        paintGL();
    }
    else
    {
        m_isInit &= ~HAS_TRIANG;
    }

    m_forceReplot = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::resizeEvent(QResizeEvent *pevent)
{
    QSize newSize = pevent->size();
    resize(newSize.width(), newSize.height());

    glViewport(0, 0, newSize.width(), newSize.height());			//Window resizen

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);					//Projektionsmatrix wählen
    glLoadIdentity();

    gluPerspective(45.0f, (GLfloat)newSize.width()/(GLfloat) newSize.height(), 0.1f, 100.0f);//Perspektive einstellen
//	glOrtho(0,width,0,height,-1,1);
//	gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::setColor(const int col)
{
    ito::RetVal retval = ito::retOk;
    int ret = 0;


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
            return -ito::retError;
    }
    else
    {
        if (m_pColIndices == NULL || m_pTriangles == NULL || m_pColTriangles==NULL)
            return -ito::retError;
    }

    if(m_elementMode == PAINT_POINTS)
    {
        if(m_currentPalette.size() > 255)
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColTriangles[4*count] =   ((m_currentPalette[m_pColIndices[count]]&0xFF0000L)>>16);
                m_pColTriangles[4*count+1] = ((m_currentPalette[m_pColIndices[count]]&0x00FF00L)>>8);
                m_pColTriangles[4*count+2] = ((m_currentPalette[m_pColIndices[count]]&0x0000FFL));
                m_pColTriangles[4*count+3] = 255;
            }
        }
        else
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColIndices[4*count]    = m_pColIndices[count];
                m_pColIndices[4*count +1] = m_pColIndices[count];
                m_pColIndices[4*count +2] = m_pColIndices[count];

                m_pColIndices[count +3] = 255;
            }
        }
    }
    else
    {
        if(m_currentPalette.size() > 255)
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColTriangles[count*12+8] = ((m_currentPalette[m_pColIndices[count * 3]]&0xFF0000L)>>16);
                m_pColTriangles[count*12+4] = ((m_currentPalette[m_pColIndices[count * 3 + 1]]&0xFF0000L)>>16);
                m_pColTriangles[count*12] = ((m_currentPalette[m_pColIndices[count * 3 + 2]]&0xFF0000L)>>16);
                m_pColTriangles[count*12+9] = ((m_currentPalette[m_pColIndices[count * 3]]&0x00FF00L)>>8);
                m_pColTriangles[count*12+5] = ((m_currentPalette[m_pColIndices[count * 3 + 1]]&0x00FF00L)>>8);
                m_pColTriangles[count*12+1] = ((m_currentPalette[m_pColIndices[count * 3 + 2]]&0x00FF00L)>>8);
                m_pColTriangles[count*12+10] = ((m_currentPalette[m_pColIndices[count * 3]]&0x0000FFL));
                m_pColTriangles[count*12+6] = ((m_currentPalette[m_pColIndices[count * 3 + 1]]&0x0000FFL));
                m_pColTriangles[count*12+2] = ((m_currentPalette[m_pColIndices[count * 3 + 2]]&0x0000FFL));
                m_pColTriangles[count*12+11] = m_pColTriangles[count*12+7] = m_pColTriangles[count*12+3] = 255;
            }
        }
        else
        {
            for (count = 0; count < m_NumElements; count++)
            {
                m_pColIndices[count*12+8] = m_pColIndices[count * 3];
                m_pColIndices[count*12+4] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count*12] = m_pColIndices[count * 3 + 2];
                m_pColIndices[count*12+9] = m_pColIndices[count * 3];
                m_pColIndices[count*12+5] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count*12+1] = m_pColIndices[count * 3 + 2];
                m_pColIndices[count*12+10] = m_pColIndices[count * 3];
                m_pColIndices[count*12+6] = m_pColIndices[count * 3 + 1];
                m_pColIndices[count*12+2] = m_pColIndices[count * 3 + 2];

                m_pColIndices[count*12+11] = m_pColIndices[count*12+7] = m_pColIndices[count*12+3] = 255;
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::ProtocolSize()
{
    int res = 1;
    int length = m_protocol.text.length()-1;
    for(int i = 0; i < length; i++)
    {
        if(m_protocol.text[i]=='\n')
            res++;
    }
    m_protocol.m_psize = 3*m_fontsize/2.0*res;
    return;
}

//-----------------------------------------------------------------------------------------------
void plotGLWidget::threeDRotationMatrix()
{
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];
    double a,b,c;

    makeCurrent();
    a=m_RotA; b=m_RotB; c=m_RotC;

    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);

    gluProject(0, 0, 0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &m_lightAxisVector[0], &m_lightAxisVector[1], &m_lightAxisVector[2]);
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
    ito::float64 signedY=1, signedX=1, signedXAx, signedXAy, signedYAx, signedYAy, signedZAx, signedZAy, signedZA;
    ito::float64 dxx, dxy, dyx, dyy, dzx, dzy, VRX, VRY;
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];

    ax = atan((m_xAxisVector[1]-m_lightAxisVector[1]) / (m_xAxisVector[0]-m_lightAxisVector[0]));
    ay = atan((m_yAxisVector[1]-m_lightAxisVector[1]) / (m_yAxisVector[0]-m_lightAxisVector[0]));
    az = atan((m_zAxisVector[1]-m_lightAxisVector[1]) / (m_zAxisVector[0]-m_lightAxisVector[0]));

    if (cos(fmod(m_RotC, 2.0*PI))<0)
        signedX = signedX * -1.0;
    if (sin(fmod(m_RotB, 2.0*PI))<0)
        signedX = signedX * -1.0;
    if (cos(fmod(m_RotA, 2.0*PI))<0)
        signedX = signedX * -1.0;

    if (sin(fmod(m_RotC, 2.0*PI))<0)
        signedY = signedY * -1.0;
    if (sin(fmod(m_RotB, 2.0*PI))<0)
        signedY = signedY * -1.0;
    if (cos(fmod(m_RotA, 2.0*PI))<0)
        signedY = signedY * -1.0;

    xsizep = (m_axisX.idx[1] - m_axisX.idx[0] + 1.0);
    ysizep = (m_axisY.idx[1] - m_axisY.idx[0] + 1.0);

    xmin = -m_windowXScale * xsizep / 2.0;
    xmax = m_windowXScale * xsizep / 2.0;
    ymin = m_windowYScale * ysizep / 2.0;
    ymax = -m_windowYScale * ysizep / 2.0;
    zmin = -m_windowZScale * (m_axisZ.phys[1] - m_axisZ.phys[0]) / 2.0;
    zmax = m_windowZScale * (m_axisZ.phys[1] - m_axisZ.phys[0]) / 2.0;

    if ((((ay < az)-0.5)*signedY)>0)
        xb = xmin;
    else
        xb = xmax;

    if ((((ax > az)-0.5)*signedX)>0)
        yb = ymin;
    else
        yb = ymax;

    signedZA = 0;

    makeCurrent();
    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);

    gluProject(m_axisX.phys[0], m_axisY.phys[0], m_axisZ.phys[0], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[0], &ye[0], &ze[0]);
    gluProject(m_axisX.phys[0], m_axisY.phys[0], m_axisZ.phys[0], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[1], &ye[1], &ze[1]);
    gluProject(m_axisX.phys[1], m_axisY.phys[1], m_axisZ.phys[1], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[2], &ye[2], &ze[2]);
    gluProject(m_axisX.phys[1], m_axisY.phys[1], m_axisZ.phys[1], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[3], &ye[3], &ze[3]);
    gluProject(xb, -yb, m_axisZ.phys[0], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[4], &ye[4], &ze[4]);
    gluProject(xb, -yb, m_axisZ.phys[1], GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xe[5], &ye[5], &ze[5]);

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
    //Y-Axis X signed.
    if (xb == xmin)
    {
/*
        if (-dyy > 0)
            signedYAx = -1 * VRX * VRY;
        else
            signedYAx = 1 * VRX * VRY;

        if (dyx > 0)
            signedYAy = -1 * VRX * VRY;
        else
            signedYAy = 1 * VRX * VRY;
*/

        if (dyy > 0)
            signedYAx = -1 * VRX * VRY;
        else
            signedYAx = 1 * VRX * VRY;

        if (dyx > 0)
            signedYAy = 1 * VRX * VRY;
        else
            signedYAy = -1 * VRX * VRY;

    }
    else
    {
        /*
        if (-dyy > 0)
            signedYAx = 1 * VRX * VRY;
        else
            signedYAx = -1 * VRX * VRY;


        if (dyx > 0)
            signedYAy = 1 * VRX * VRY;
        else
            signedYAy = -1 * VRX * VRY;
        */

        if (dyy > 0)
            signedYAx = 1 * VRX * VRY;
        else
            signedYAx = -1 * VRX * VRY;

        if (dyx > 0)
            signedYAy = -1 * VRX * VRY;
        else
            signedYAy = 1 * VRX * VRY;
    }

    if (yb == ymin)
    {
        if (-dxx > 0)
            signedXAy = -1 * VRX * VRY;
        else
            signedXAy = 1 * VRX * VRY;
        if (dxy > 0)
            signedXAx = -1 * VRX * VRY;
        else
            signedXAx = 1 * VRX * VRY;
    }
    else
    {
        if (-dxx > 0)
            signedXAy = 1 * VRX * VRY;
        else
            signedXAy = -1 * VRX * VRY;
        if (dxy > 0)
            signedXAx = 1 * VRX * VRY;
        else
            signedXAx = -1 * VRX * VRY;
    }

    if (((xb == xmin) && (yb==ymin)) || ((xb != xmin) && (yb!=ymin)))
    {
        if (-dzy > 0)
            signedZAx = -1 * VRX * VRY;
        else
            signedZAx = 1 * VRX * VRY;
        if (dzx > 0)
            signedZAy = -1 * VRX * VRY;
        else
            signedZAy = 1 * VRX * VRY;
    }
    else
    {
        if (-dzy > 0)
            signedZAx = 1 * VRX * VRY;
        else
            signedZAx = -1 * VRX * VRY;
        if (dzx > 0)
            signedZAy = 1 * VRX * VRY;
        else
            signedZAy = -1 * VRX * VRY;
    }




    if(m_axisX.show && m_axisY.show)
    {
        paintAxisOGL(xmin, -yb, zmin, xmax, -yb, zmin);
        paintAxisOGL(-xb, ymin, zmin, -xb, ymax, zmin);

        paintAxisOGL(xmin, yb, zmin, xmax, yb, zmin);
        paintAxisOGL(xb, ymin, zmin, xb, ymax, zmin);

        paintAxisTicksOGL(xmin, yb, zmin, xmax, yb, zmin,  m_axisX.phys[0], m_axisX.phys[1], signedXAx, signedXAy, signedZA, m_axisX.label, m_axisX.unit, 0);
        paintAxisTicksOGL(xb, ymin, zmin, xb, ymax, zmin, m_axisY.phys[0], m_axisY.phys[1], signedYAx, signedYAy, signedZA, m_axisY.label, m_axisY.unit, 0);
    }
    //Damit die Z-Tickes einstellbar werden
    m_ticklength /= m_z_tickmulti;

    if(m_axisZ.show)
    {
        paintAxisOGL(xb, -yb, zmin, xb, -yb, zmax);
        paintAxisTicksOGL(xb, -yb, zmin, xb, -yb, zmax, m_axisZ.phys[0], m_axisZ.phys[1], signedZAx, signedZAy, signedZA, m_axisZ.label, m_axisZ.unit, 0);
    }
    //Damit die Z-Tickes einstellbar werden
    m_ticklength *= 10.0*m_z_tickmulti;

    if(m_axisX.show && m_axisY.show)
    {
        paintAxisTicksOGL(xmin, yb, zmin, xmax, yb, zmin, m_axisX.phys[0], m_axisX.phys[1], signedXAx, signedXAy, signedZA, m_axisX.label, m_axisX.unit, 1 & m_axisX.showTicks);
        paintAxisTicksOGL(xb, ymin, zmin, xb, ymax, zmin, m_axisY.phys[0], m_axisY.phys[1], signedYAx, signedYAy, signedZA, m_axisY.label, m_axisY.unit, 1 & m_axisY.showTicks);
    }

    //Damit die Z-Tickes einstellbar werden
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

    // Und wieder zurück
    //dd->ticklength /= 10.0;
    m_ticklength *= (m_z_tickmulti/10.0);

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
    makeCurrent();

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
*\brief Zeichnet den Lichtpfeil für die Beleuchtungsdarstellung
*\param[in] *fo Zeiger auf das Goofi-Filterobjekt
*\return error
*\ingroup 3DOGLFuncsGroup
*/
void plotGLWidget::paintLightArrow()
{
    GLfloat position1[3];
    double norm1;

    makeCurrent();

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
*\param[in] write Flag für das Schreiben von Tick-Lables
*\return error
*\ingroup 3DOGLFuncsGroup
*/
void plotGLWidget::paintAxisTicksOGL(const double x0, const double y0, const double z0, const double x1, const double y1, const double z1, const double v0, const double v1, const double VorzX, const double VorzY, const double VorzZ, const std::string &symbol, const std::string &unit, const bool write)
{
    double s0,s1,d0,d1,p=0,q=0,v,l,ldv,a;
    long e,b=0;

    double sl1= m_ticklength;
    double xstart, xend, ystart, yend, phi;
    GLdouble GLModelViewMatrix[16], GLProjectionMatrix[16];
    GLint GLViewport[4];
    GLdouble glx0, gly0, glz0, glx1, gly1, glz1, xpos, ypos, zpos;
    struct axislabel al;
    double ticklength=0.01;
    double VorzXAs, VorzYAs;
    int firstdigit, i;
    std::string label(" ");

    makeCurrent();

    glGetDoublev(GL_MODELVIEW_MATRIX, GLModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, GLProjectionMatrix);
    glGetIntegerv(GL_VIEWPORT, GLViewport);

    gluProject(x0, y0, z0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &glx0, &gly0, &glz0);
    gluProject(x1, y1, z1, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &glx1, &gly1, &glz1);

    d0=s0=v0;
    d1=s1=v1;
    if (m_linewidth == 0)
        return;

    glLineWidth(m_linewidth);

    if (m_backgnd)
        glColor3f(0, 0, 0);
    else
        glColor3f(1, 1, 1);

    l = sqrt((double)(glx1-glx0)*(glx1-glx0)+(double)(gly1-gly0)*(gly1-gly0));

    if(sl1 >= l || !sl1)
        return;

    if(!ito::dObjHelper::isFinite<double>(s0) || !ito::dObjHelper::isFinite<double>(s1) ||s0 >= s1)
        return;

    ldv=log10((s1-s0)*sl1/l);

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
        VorzYAs = 0;
    else
        VorzYAs = fabs(y0) / y0;
    if ((y1 == y0) && (z1 == z0))
        VorzXAs = 0;
    else
        VorzXAs = fabs(x0) / x0;

    if (write)
    {
        al.write = write;

        label.reserve(100);

        bool alreadyScaled = false;

        al.maxlen=0;

        firstdigit = floor( log10(fabs(v0) > fabs(v1) ? fabs(v0) : fabs(v1) ) + 10 * DBL_EPSILON );

        if(firstdigit >= -15 && firstdigit < 21)
        {
            al.unitydigit = firstdigit > 0 ? firstdigit / 3 * 3 : -(2-firstdigit) / 3 * 3;
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
            for(i=0; dont_scale_units[i] != NULL; i++)
                if(0 == unit.compare(dont_scale_units[i]))
                {
                    al.unitydigit = std::numeric_limits<int>::min();
                    break;
                }
        }

        if( unit.compare("m") || unit.compare("mm") ? 0 : m_xybase)
        {
            al.unitydigit = floor(log10(m_xybase)+10*DBL_EPSILON);
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

                    char sign[2] ={("afpnµm-kMGTPE"[al.unitydigit/3+6]), 0};
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
        phi=atan2((yend-ystart),(xend-xstart));

        al.dx = VorzX * 0.05 * fabs(sin(phi));
        al.dy = VorzY * 0.1 * fabs(cos(phi));

        al.maxlen=0;
    }

    v=s0;
    v=ceil(e>0?v/p/b:v*p/b);
    v=e>0?v*b*p:v*b/p;
    while(v<=s1)
    {
        a=(v-d0)/(d1-d0);
        glBegin(GL_LINES);
            glVertex3f(x0+a*(x1-x0), y0+a*(y1-y0), z0+a*(z1-z0));
            if (write)
                glVertex3f(x0+a*(x1-x0)+VorzXAs*ticklength*1.7, y0+a*(y1-y0)+VorzYAs*ticklength*1.7, z0+a*(z1-z0)-ticklength*1.7);
            else
                glVertex3f(x0+a*(x1-x0)+VorzXAs*ticklength, y0+a*(y1-y0)+VorzYAs*ticklength, z0+a*(z1-z0)-ticklength);
        glEnd();

        if (write)
        {
            gluProject(x0+a*(x1-x0), y0+a*(y1-y0), z0+a*(z1-z0), GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xpos, &ypos, &zpos);
//			dreidogl_AxisLabel(fo, (void*)&al, xpos*1.1, ypos*1.1, v);
            //paintAxisLabelOGL((void*)&al, xpos*(1+0.07 * m_windowXScale * internalObj.getSize(internalObj.getDims()-1,false)), ypos*(1+0.03 * m_windowYScale * internalObj.getSize(internalObj.getDims()-2,false)), v);
            paintAxisLabelOGL((void*)&al, xpos*(1+0.07 * m_windowXScale * fabs(m_axisX.idx[1] - m_axisX.idx[0] + 1.0)), ypos*(1+0.03 * m_windowYScale * fabs(m_axisY.idx[1] - m_axisY.idx[0] + 1.0)), v);
        }

        v=v*(v>0?1+4*DBL_EPSILON:1-4*DBL_EPSILON);
        v=ceil((e>0?v/p/b:v*p/b)+.5);
        v=e>0?v*b*p:v*b/p;
    }

    if (write)
    {
        gluProject(x0+(x1-x0)/2.0, y0+(y1-y0)/2.0, z0+(z1-z0)/2.0, GLModelViewMatrix, GLProjectionMatrix, GLViewport, &xpos, &ypos, &zpos);

        al.dx = VorzX * (0.15*m_windowXScale * fabs(m_axisX.idx[1] - m_axisX.idx[0] + 1.0)) * fabs(sin(phi));
        al.dy = VorzY * (0.25*m_windowYScale * fabs(m_axisY.idx[1] - m_axisY.idx[0] + 1.0)) * fabs(cos(phi));

// Tut  al.dx = VorzX * (0.15*m_windowXScale*internalObj.getSize(internalObj.getDims()-1,false)) * fabs(sin(phi));
// TUT  al.dy = VorzY * (0.25*m_windowYScale*internalObj.getSize(internalObj.getDims()-2,false)) * fabs(cos(phi));

        if (al.dx < 0)
            xpos += al.dx - label.length() * 0.015;
        else
            xpos += al.dx;

        if (al.dy < 0)
            ypos += al.dy;
        else
            ypos += al.dy;

        OGLTextOut(label.data(), xpos, ypos);
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
void plotGLWidget::paintAxisLabelOGL(const void *vd, const double x, const double y, const double v)
{
 char buffer[300];
 //char decimal;
 //char *p;
 struct axislabel *al = (struct axislabel *)vd;
 long l;
 int ret;

  if(v==0)
    _snprintf(buffer,sizeof(buffer),"0");
  else if(al->unitydigit>=al->lastdigit)
  {
    _snprintf(buffer,sizeof(buffer),"%.*f",al->unitydigit-al->lastdigit,v/al->unity);
  }
  else
  {
    int firstdigit=floor(log10(fabs(v))+10*DBL_EPSILON);
    if(al->unitydigit==INT_MIN&&al->lastdigit!=INT_MAX)
    {
        if (al->lastdigit>=0)
        {
            //CK 09.08.2006 anti-Killer-Objekt Hack:
            //ist darzustellende Zahl länger als 8 Ziffern, dann machs in Exp-Darstellung
            if (fabs(v) >= 1.0E8)
                _snprintf(buffer, sizeof(buffer), "%g", v);
            else
                _snprintf(buffer, sizeof(buffer), "%.0f", v);
        }
        else if(firstdigit>=-3&&firstdigit<3)
            _snprintf(buffer,sizeof(buffer),"%.*f",(firstdigit>0?firstdigit:0)-al->lastdigit,v);
        else
        {double v1, mant;
         int expo;

            v1 = fabs(v);
            mant = pow(10.0, log10(v1)-floor(log10(v1)));
            expo = floor(log10(v1));
            _snprintf(buffer,sizeof(buffer),"%s%.*fE%d", v<0?"-":"", firstdigit-al->lastdigit, mant, expo);
        }
    }
    else if(firstdigit>=0&&firstdigit<3)
        _snprintf(buffer,sizeof(buffer),"%.0f",v);
    else if(firstdigit>=21||firstdigit<-18||al->unitydigit==INT_MIN)
    {
        double v1 = fabs(v);
        _snprintf(buffer,sizeof(buffer),"%s%.0fE%d", v<0?"-":"", pow(10.0, log10(v1)-floor(log10(v1))),
        (int)floor(log10(v1)));
    }
    else
    {
        int rest=firstdigit>0?firstdigit%3:2-(2-firstdigit)%3;
        _snprintf(buffer,sizeof(buffer),"%.0f%c",pow(10.0,rest),"afpnµm-kMGTPE"[(firstdigit-rest)/3+6]);
    }
  }

  l=strlen(buffer);
  if(l>al->maxlen)
    al->maxlen=l;
  if(!al->write)
    return;
/*
  (void)GetDecimalChar(&decimal);
  if (decimal != '.')
        while (NULL!=(p=strchr(buffer, '.')))
            *p = decimal;
*/
    double xpos, ypos;

    if (al->dx < 0)
        xpos = x + al->dx - strlen(buffer)*0.015;
    else
        xpos = x + al->dx;

    if (al->dy < 0)
        ypos = y + al->dy;
    else
        ypos = y + al->dy;

    ret = OGLTextOut(buffer, xpos, ypos);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
int plotGLWidget::OGLTextOut(const char *buffer, const double xpos, const double ypos)
{
    makeCurrent();

    glPushAttrib(GL_LIST_BIT);				// Pushes The Display List Bits
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glRasterPos2f(xpos, ypos);
    glListBase(m_myCharBitmapBuffer);					// Sets The Base Character to 0
    glCallLists(strlen(buffer), GL_UNSIGNED_BYTE, buffer);	// Draws The Display List Text

    glPopAttrib();						// Pops The Display List Bits

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
//	gluOrtho2D(-1.1, 1.1, -1.1, 1.1);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    int ret = glGetError();

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------

void plotGLWidget::OGLMakeFont(int size)
{

    QFont oldFont = this->font();

    QFont myFont("Arial", -size, QFont::Light & QFont::OpenGLCompatible & QFont::PreferBitmap);
    this->setFont(myFont);

    if(m_myCharBitmapBuffer != 0) glDeleteLists(m_myCharBitmapBuffer, 256);
    m_myCharBitmapBuffer = glGenLists(256);			// Storage For 256 Characters
#if (defined linux)

#elif (defined Q_OS_WIN32 || defined(Q_OS_WIN64))
    wglUseFontBitmaps(this->getDC(), 0, 255, m_myCharBitmapBuffer);			// Builds 96 Characters Starting At Character 32
#endif

    this->setFont(oldFont);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::DrawObjectInfo(void)
{
    double x0 = -1.0 + (double)m_fontsize / width();
    double y0 = -1.0 + (3 * m_fontsize * 3.0 + (m_protocol.show ? m_protocol.m_psize : 0.0)) / height();

    if(m_objectInfo.xLength.length()) OGLTextOut((char*)m_objectInfo.xLength.data(), x0, y0);

    y0 -= 3.0 * m_fontsize/ height();
    if(m_objectInfo.yLength.length()) OGLTextOut((char*)m_objectInfo.yLength.data(), x0, y0);

    y0 -= 3.0 * m_fontsize/ height();
    if(m_objectInfo.matrix.length()) OGLTextOut((char*)m_objectInfo.matrix.data(), x0, y0);

    size_t len = m_objectInfo.PeakText.length();

    if(len < m_objectInfo.MeanText.length())
        len = m_objectInfo.MeanText.length();

    if(len < m_objectInfo.DevText.length())
        len = m_objectInfo.DevText.length();

    x0 = 1.0 - (double)(1.7 * len * m_fontsize) / width();
    y0 = -1.0 + (3.0 *m_fontsize * 3.0 + (m_protocol.show ? m_protocol.m_psize : 0.0)) / height();
    if(m_objectInfo.PeakText.length()) OGLTextOut((char*)m_objectInfo.PeakText.data(), x0, y0);

    y0 -= 3.0 * m_fontsize / height();
    if(m_objectInfo.MeanText.length()) OGLTextOut((char*)m_objectInfo.MeanText.data(), x0, y0);

    y0 -= 3.0 * m_fontsize / height();
    if(m_objectInfo.DevText.length()) OGLTextOut((char*)m_objectInfo.DevText.data(), x0, y0);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::DrawColorBar(const char xPos, const char yPos, const GLfloat dX, const GLfloat dY, const GLfloat zMin, const GLfloat zMax)
{
    makeCurrent();

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

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);				// Select Our Texture

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    //glPixelTransferi(GL_MAP_COLOR, GL_TRUE);

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

    OGLTextOut(buf, x0 + dX + 7.0 / (double)width(), y0 - m_fontsize / (double)height() / 2.0);

    sprintf(buf, "%g", zMax);
    OGLTextOut(buf, x0 + dX + 7.0 / (double)width(), y0 + dY - m_fontsize / (double)height() / 2.0);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
//	gluOrtho2D(-1.1, 1.1, -1.1, 1.1);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    int ret = glGetError();

    return;
}

//-----------------------------------------------------------------------------------------------
//int DrawTitle(struct graf *win, TagSpace *tags, int texty, int *yused)
void plotGLWidget::DrawTitle(const std::string &myTitle, const int texty, int &yused)
{
    int i = 0;
    double x0 = -1 * myTitle.length() * 1.67 * m_fontsize * 0.6 / this->width();
    double y0 = 0.98 - (double)(++i * 2.0 * 1.67 * m_fontsize) / this->height();

    /* Titel Objekt etc. */

    OGLMakeFont(1.67*m_fontsize);
    if(myTitle.length())
        OGLTextOut((char*)myTitle.data(), x0, y0);
/*
    OGLMakeFont(1.5*dd->fontsize, dd);
    tags->ReadTagDef(TAG_COMMENT1,(void *)&txt,"");
    if(strlen(txt))
        OGLTextOut(dd, (char*)txt, (double)x0/this->width()-0.9, (double)(++i*-2.0*1.5*m_fontsize)/this->height()+0.9);

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
    refreshPlot(NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::riseZAmplifierer(const double value)
{
    if(m_zAmpl < 5.0)
        m_zAmpl *= value;
    refreshPlot(NULL);
}
//----------------------------------------------------------------------------------------------------------------------------------
void plotGLWidget::togglePaletteMode()
{
    //m_colorBarMode = (m_colorBarMode + 1) % 4;

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
    paintGL();
};
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
    this->paintGL();
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
    paintGL();
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
    paintGL();
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

    makeCurrent();

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);
    int ret = glGetError();

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

    //glGetIntegerv(GL_MAX_PIXEL_MAP_TABLE, &glval);
    //ret = glGetError();

    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 256, pag);
    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 256, par);
    ret = glGetError();
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 256, pab);
    ret = glGetError();

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

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//Screen und Tiefenpuffer leeren

    glPixelTransferi(GL_MAP_COLOR, GL_FALSE);

    unsigned char * src = new unsigned char[m_currentPalette.size() * 4 *2];

    for(int i = 0; i < m_currentPalette.size(); i++)
    {
        src[8*i]   = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i];
        src[8*i+1] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 1];
        src[8*i+2] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
        src[8*i+3] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
        src[8*i+4] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i];
        src[8*i+5] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 1];
        src[8*i+6] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
        src[8*i+7] = ((unsigned char*)m_currentPalette.data())[4*m_currentPalette.size() - 4 * i + 2];
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, src);

    delete src;

    glBindTexture(GL_TEXTURE_2D, m_cBarTexture);

    doneCurrent();

    m_isInit |= IS_INIT;
    ResetColors();
    paintGL();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal plotGLWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
    /*
    if(m_pContent)
    {
        m_pContent->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);
        repaint();
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
                toogleObjectInfoText(true);
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
inline void plotGLWidget::toogleObjectInfoText(const bool enabled)
{
    if(enabled)
    {
        ito::dObjHelper::devValue(m_pContent.data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
        generateObjectInfoText();
    }
    m_objectInfo.show = enabled;
    paintGL();
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

    if(m_pContent)
    {
        switch(m_pContent->getType())
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
