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
#    itom is free software by ITO, Universität Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#if linux
    #include <unistd.h>
#endif
#include <qglobal.h>
#if (QT_VERSION < 0x050000)
    #include "GL/glew.h"
#endif

#include "twipOGLFigure.h"
#include "twipOGLWidget.h"
#include "twipOGLLegend.h"

#if QT_VERSION >= 0x050000
#define GLFPTR(func) m_glf->func
#else
#define GLFPTR(func) func
#endif

#if (defined WIN32) && (QT_VERSION >= 0x050000)
    #define NOMINMAX
    #include <Windows.h>
    #include <gl/GL.h>
    #include <gl/GLU.h>
#endif

#include "common/sharedStructuresGraphics.h"
#include "common/numeric.h"
#include "shaderEngines.h"

#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QMessageBox>
#include <QCoreApplication>
#include <QPainter>
#if (QT_VERSION >= 0x050000)
    #include <QKeyEvent>
    #include <QOpenGLFramebufferObject>
#endif

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"

using namespace ito;

const char *dont_scale_units[] = {"frame", "frames", "frm", "frms", "digit", "digits", "Bild", "Bilder", "Wert", "-"};

struct textVertData
{
    GLfloat x;
    GLfloat y;
    GLfloat z;
    GLfloat u;
    GLfloat v;
};

extern int NTHREADS;

float tickDist = 20;

//----------------------------------------------------------------------------------------------------------------------------------
/**
*   \class TwipOGLWidget
*   \brief OpenGL Widget for displaying perspective plots of dataObjects and pointClouds
*
*   Inner widget which actually does all the drawing job. The widget can render perspective
*   plots of dataObjects and different types of pointClouds. The 3D data can be overlaid
*   with an intensity image and some basic illumination is available. For pointClouds it is
*   also possible to use the values stored in the curvature for color coding. This can e.g.
*   used to color code the difference to a model.
*/
#if QT_VERSION >= 0x050400
TwipOGLWidget::TwipOGLWidget(QMenu *contextMenu, void* configData, QSurfaceFormat &fmt, QWidget *parent, const QOpenGLWidget *shareWidget) :
	QOpenGLWidget(parent, Qt::Widget),
#else
TwipOGLWidget::TwipOGLWidget(QMenu *contextMenu, void* configData, QGLFormat &fmt, QWidget *parent, const QGLWidget *shareWidget) :
	QGLWidget(fmt, parent, shareWidget, Qt::Widget),
#endif
	m_contextMenu(contextMenu),
    m_pParent(parent),
    m_activeModifiers(Qt::NoModifier),
    m_lineplotUID(0),
    m_forceReplot(false),
    m_isInit(0),
    m_glf(NULL),
    m_transX(0.0),
    m_transY(0.0),
    m_transZ(0.0),
    m_scaleX(1.0),
    m_scaleY(1.0),
    m_scaleZ(1.0),

    // GL Plot vars
    m_VAO3DPri(0),
    m_prog3D(-1),
    m_prog3DPri(-1),
    m_prog2DPx(-1),
    m_attribVert(-1),
    m_attribDiff(-1),
    m_attribVertColor(-1),
    m_attribVert3DPri(-1),
    m_attribTxtVert(-1),
    m_attribTxtUV(-1),
    m_attribNorm(-1),
    m_attribTexCol(-1),
    m_unifMVP(-1),
    m_unifVCT(-1),
    m_unifMVPPri(-1),
    m_unifVCTPri(-1),
    m_unifPalette(-1),
    m_unifGlColor2D(-1),
    m_unifGlColor3D(-1),
    m_unifGlColorInv(-1),
    m_unifGlColor3DPri(-1),
    m_unifUseTex(-1),
    m_unifUsePalette(-1),
    m_unifTextColor(-1),
    m_unifScaleX(-1),
    m_unifScaleY(-1),
    m_unifPosX(-1),
    m_unifPosY(-1),
    m_unifLighting(-1),
    m_unifLColor(-1),
    m_unifAmbient(-1),
    m_unifDiffuse(-1),
    m_unifDiffuseDir(-1),
    m_unifText(-1),
    m_unifDiffMode(-1),
    m_unifDiffNorm(-1),
    m_unifDiffMin(-1),
    m_uniCurAlpha(-1),
    m_cBarVBuf(0),
    m_cBarVAO(0),
    m_cBarTex(0),

    // DataObject Vars
//    m_pContentDObj(NULL),
//    m_pUseTextPix(NULL),
#ifdef USEPCL
//    m_pContentPC(NULL),
//    m_pContentPM(NULL),
#endif
    m_mouseStartPos(0,0)
{
//	setFormat(fmt);
//    m_isInit = 0;

    m_pConfigData = (InternalData *)configData;
    updateColorsAndVisibility(false);

    //(mouse tracking is controlled by action in WinMatplotlib)
    this->setMouseTracking(false);
    int ret = 0;

    m_transperency.clear();
    m_enabledHash.clear();

    m_currentPalette.clear();
    m_errorDisplMsg.clear();
    m_errorDisplMsg.append("No Data");

    // Basic view coordinates
    m_shaderProgs.insert("Vert_3D", VERT_3D);
    m_shaderProgs.insert("Frag_3D", FRAG_3D);
    m_shaderProgs.insert("Vert_3D_130", VERT_3D_130);
    m_shaderProgs.insert("Frag_3D_130", FRAG_3D_130);

    m_shaderProgs.insert("Vert_3DPri", VERT_3DPRI);
    m_shaderProgs.insert("Frag_3DPri", FRAG_3DPRI);
    m_shaderProgs.insert("Vert_3DPri_130", VERT_3DPRI_130);
    m_shaderProgs.insert("Frag_3DPri_130", FRAG_3DPRI_130);

    m_shaderProgs.insert("Vert_2DPX", VERT_2DPX);
    m_shaderProgs.insert("Frag_2DPX", FRAG_2DPX);
    m_shaderProgs.insert("Vert_2DPX_130", VERT_2DPX_130);
    m_shaderProgs.insert("Frag_2DPX_130", FRAG_2DPX_130);

    m_isInit = CACHE_STARTUP;
    setFocusPolicy(Qt::StrongFocus); // Enable key events
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::doCleanUp(void)
{
    makeCurrent();
    if (m_prog2DPx >= 0)
    {
        GLFPTR(glUseProgram(m_prog2DPx));
        GLFPTR(glBindBuffer(GL_ARRAY_BUFFER, 0));
        GLFPTR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
        GLFPTR(glBindTexture)(GL_TEXTURE_2D, 0);
#if QT_VERSION < 0x050000
        glBindVertexArray(0);
#endif
        //!> clean up font textures and buffers
        for (QHash<QString, SFont>::iterator tf = m_fonts.begin(); tf != m_fonts.end(); tf++)
        {
            for (int n = 0; n < tf.value().m_chars.size(); n++)
            {
                GLFPTR(glDeleteTextures)(1, &tf.value().m_chars[n].m_texID);
            }
            for (int n = 0; n < tf.value().m_vboBufs.size(); n++)
            {
                if (tf.value().m_vboBufs[n])
                {
                    GLFPTR(glDeleteBuffers)(1, &tf.value().m_vboBufs[n]->m_vbufId);
                    GLFPTR(glDeleteBuffers)(1, &tf.value().m_vboBufs[n]->m_ebufId);
#if QT_VERSION < 0x050000
                    glDeleteVertexArrays(1, &tf.value().m_vboBufs[n]->m_VAO);
#else
                    tf.value().m_vboBufs[n]->m_VAO->release();
                    tf.value().m_vboBufs[n]->m_VAO->destroy();
                    delete tf.value().m_vboBufs[n]->m_VAO;
#endif
                    delete(tf.value().m_vboBufs[n]);
                }
            }
            tf.value().m_vboBufs.clear();
            tf.value().m_chars.clear();
        }
        m_fonts.clear();
        GLFPTR(glUseProgram)(0);
        GLFPTR(glDeleteProgram)(m_prog2DPx);
    }

    //!> clean up 3D buffers, i.e. vertex and intensity arrays
    if (m_prog3D >= 0)
    {
        GLFPTR(glUseProgram)(m_prog3D);
        GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, 0);
        GLFPTR(glBindBuffer)(GL_ELEMENT_ARRAY_BUFFER, 0);
        GLFPTR(glBindTexture)(GL_TEXTURE_2D, 0);
#if QT_VERSION < 0x050000
        glBindVertexArray(0);
#endif

        QHash<int, GLuint>::ConstIterator it = m_vertBuf3D.begin();
        for (; it != m_vertBuf3D.end(); it++)
        {
            GLuint tmpVal = it.value();
            GLFPTR(glDeleteBuffers)(1, &tmpVal);
        }

#if QT_VERSION < 0x050000
        QHash<int, GLuint>::ConstIterator it2 = m_VAO3D.begin();
        for (; it2 != m_VAO3D.end(); it2++)
        {
            GLuint tmpVal = it2.value();
            glDeleteVertexArrays(1, &tmpVal);
        }
#else
        QHash<int, QOpenGLVertexArrayObject*>::ConstIterator it2 = m_VAO3D.begin();
        for (; it2 != m_VAO3D.end(); it2++)
        {
            QOpenGLVertexArrayObject* tmpVal = it2.value();
            tmpVal->release();
            tmpVal->destroy();
            delete tmpVal;
        }
#endif

        it = m_textBuf3D.begin();
        for (; it != m_textBuf3D.end(); it++)
        {
            GLuint tmpVal = it.value();
            GLFPTR(glDeleteBuffers)(1, &tmpVal);
        }
        //glDeleteBuffers(1, &m_textBuf3D);
        GLFPTR(glUseProgram)(0);
        GLFPTR(glDeleteProgram)(m_prog3D);
    }

    //!> clean up other simple painting stuff, i.e. axis
    if (m_prog3DPri >= 0)
    {
        GLFPTR(glDeleteProgram)(m_prog3DPri);
        GLFPTR(glDeleteBuffers)(1, &m_vertBuf3DPri);
#if QT_VERSION < 0x050000
        glDeleteVertexArrays(1, &m_VAO3DPri);
#else
        m_VAO3DPri->release();
        m_VAO3DPri->destroy();
        delete m_VAO3DPri;
#endif
    }
    doneCurrent();

    if (m_glf)
        delete m_glf;
}

//----------------------------------------------------------------------------------------------------------------------------------
TwipOGLWidget::~TwipOGLWidget()
{
    delete m_pConfigData;
    hide();
    m_isInit |= ~IS_INIT;
    Sleep(100);

    doCleanUp();

    for (int ndo = 0; ndo < m_pContentDObj.size(); ndo++)
    {
        if (m_pContentDObj[ndo])
            m_pContentDObj[ndo].clear();
    }

    for (int nut = 0; nut < m_pUseTextPix.size(); nut++)
    {
        if (m_pUseTextPix[nut])
        {
            free(m_pUseTextPix[nut]);
            m_pUseTextPix[nut] = NULL;
        }
    }

#ifdef USEPCL
    for (int npc = 0; npc < m_pContentPC.size(); npc++)
    {
        if (m_pContentPC[npc])
            m_pContentPC[npc].clear();
    }

    for (int npm = 0; npm < m_pContentPM.size(); npm++)
    {
        if (m_pContentPM[npm])
            m_pContentPM[npm].clear();
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Compile and link shader programs
*   @param [in] progStr         String identifier of program to compile. The code is stored in the m_shaderProgs member
*   @param [in / out] progName  Internal id for referencing the shader program on the gpu.
*   @return 0 if everything went well, otherwise the result of glGetError()
*/
int TwipOGLWidget::makeShaderProg(const QString &progStr, GLint &progName)
{
    char buf[1024];
    int len = 0;
    int ret = 0;

    //!> create fragment and vertex shader
    GLuint Vert = GLFPTR(glCreateShader)(GL_VERTEX_SHADER);
    GLuint Frag = GLFPTR(glCreateShader)(GL_FRAGMENT_SHADER);

    //!> load source code for fragment and vertex shader and change version number of vertex and
    //!> fragment shader code to match the set version of the opengl context, in order to avoid
    //!> backward compatible code generation (slow)
    char *VertFinal = NULL;
    char *FragFinal = NULL;

    ret = glGetError();
#if QT_VERSION >= 0x050400
	QSurfaceFormat fmt = format();
	QPair<int, int> glVer = fmt.version();

	if (glVer.first < 3)
	{
		VertFinal = _strdup(m_shaderProgs[QString("Vert_") + progStr]);
		FragFinal = _strdup(m_shaderProgs[QString("Frag_") + progStr]);
	}
	else
	{
		VertFinal = _strdup(m_shaderProgs[QString("Vert_") + progStr + QString("_130")]);
		FragFinal = _strdup(m_shaderProgs[QString("Frag_") + progStr + QString("_130")]);
	}
#else
    int glVer = QGLFormat::openGLVersionFlags();

	if (glVer < 4096)
    {
        VertFinal = _strdup(m_shaderProgs[QString("Vert_") + progStr]);
        FragFinal = _strdup(m_shaderProgs[QString("Frag_") + progStr]);
    }
    else
    {
        VertFinal = _strdup(m_shaderProgs[QString("Vert_") + progStr + QString("_130")]);
        FragFinal = _strdup(m_shaderProgs[QString("Frag_") + progStr + QString("_130")]);
    }
#endif

    if (VertFinal == NULL || FragFinal == NULL)
    {
        std::cerr << "vertex or fragment shader code not defined\n";
        return -1;
    }

    char *vertVerPos = strstr(VertFinal, "#version");
    char *fragVerPos = strstr(FragFinal, "#version");

#if QT_VERSION >= 0x050400
	if (glVer.first >= 3 && glVer.second >= 2)
	{
		vertVerPos[9] = '1';
		vertVerPos[10] = '5';
		vertVerPos[11] = '0';
		fragVerPos[9] = '1';
		fragVerPos[10] = '5';
		fragVerPos[11] = '0';
	}
	else if (glVer.first >= 3 && glVer.second >= 1)
	{
		vertVerPos[9] = '1';
		vertVerPos[10] = '4';
		vertVerPos[11] = '0';
		fragVerPos[9] = '1';
		fragVerPos[10] = '4';
		fragVerPos[11] = '0';
	}
	else if (glVer.first >= 3 && glVer.second >= 0)
	{
		vertVerPos[9] = '1';
		vertVerPos[10] = '3';
		vertVerPos[11] = '0';
		fragVerPos[9] = '1';
		fragVerPos[10] = '3';
		fragVerPos[11] = '0';
	}
	else if (glVer.first >= 2 && glVer.second >= 1)
	{
		vertVerPos[9] = '1';
		vertVerPos[10] = '2';
		vertVerPos[11] = '0';
		fragVerPos[9] = '1';
		fragVerPos[10] = '2';
		fragVerPos[11] = '0';
	}
	else if (glVer.first >= 2 && glVer.second >= 0)
	{
		vertVerPos[9] = '1';
		vertVerPos[10] = '1';
		vertVerPos[11] = '0';
		fragVerPos[9] = '1';
		fragVerPos[10] = '1';
		fragVerPos[11] = '0';
	}

#else
    if (glVer >= 32768)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '5';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '5';
        fragVerPos[11] = '0';
    }
    else if (glVer >= 16384)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '5';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '5';
        fragVerPos[11] = '0';
    }
    else if (glVer >= 8192)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '4';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '4';
        fragVerPos[11] = '0';
    }
    else if (glVer >= 4096)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '3';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '3';
        fragVerPos[11] = '0';
    }
    else if (glVer >= 64)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '2';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '2';
        fragVerPos[11] = '0';
    }
    else if (glVer >= 32)
    {
        vertVerPos[9] = '1';
        vertVerPos[10] = '1';
        vertVerPos[11] = '0';
        fragVerPos[9] = '1';
        fragVerPos[10] = '1';
        fragVerPos[11] = '0';
    }
#endif

    GLFPTR(glShaderSource)(Vert, 1, (const GLchar**)&VertFinal, NULL);
    GLFPTR(glShaderSource)(Frag, 1, (const GLchar**)&FragFinal, NULL);
    free(VertFinal);
    free(FragFinal);

    //!> compile vertex shader
    GLFPTR(glCompileShader)(Vert);
    GLFPTR(glGetShaderiv)(Vert, GL_COMPILE_STATUS, &ret);
    if (ret != GL_TRUE)
    {
        memset(buf, 0, 1024);
        GLFPTR(glGetShaderInfoLog)(Vert, 1024, &len, buf);
        std::cerr << "error compiling vertex shader\n" << buf << "\n";
        return -1;
    }

    //!> compile fragment shader
    GLFPTR(glCompileShader)(Frag);
    GLFPTR(glGetShaderiv)(Frag, GL_COMPILE_STATUS, &ret);
    if (ret != GL_TRUE)
    {
        memset(buf, 0, 1024);
        GLFPTR(glGetShaderInfoLog)(Frag, 1024, &len, buf);
        std::cerr << "error compiling fragment shader\n" << buf << "\n";
        return -1;
    }

    //!> create program and attach compiled vertex and fragment shader to it
    progName = GLFPTR(glCreateProgram)();
    GLFPTR(glAttachShader)(progName, Vert);
    GLFPTR(glAttachShader)(progName, Frag);
    if (ret = glGetError())
    {
        std::cerr << "error attaching shaders\n";
        return -1;
    }

    //!> link shader program
    GLFPTR(glLinkProgram)(progName);
    GLFPTR(glGetProgramiv)(progName, GL_LINK_STATUS, &ret);
    if (ret != GL_TRUE)
    {
        memset(buf, 0, 1024);
        GLFPTR(glGetProgramInfoLog)(progName, 1024, &len, buf);
        std::cerr << "error linking shader program\n" << buf << "\n";
        return -1;
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize openGL context, buffers, shader programs, ...
*
*   This method initializes all stuff necessary for openGL output. Including the compilation of the shader programs
*/
void TwipOGLWidget::initializeGL()
{
    int ret = 0;

    if (m_isInit & IS_INIT)
    {
        m_vertBuf3D.clear();
        m_VAO3D.clear();
        m_textBuf3D.clear();
        //        return;
    }

#if QT_VERSION >= 0x050400
    connect(context(), SIGNAL(aboutToBeDestroyed()), this, SLOT(oglAboutToDestroy()));
#endif
    if(ITOM_API_FUNCS_GRAPH != NULL && *ITOM_API_FUNCS_GRAPH != NULL)
    {
        int numColBars = 0;
        ito::ItomPalette newPalette;

        apiPaletteGetNumberOfColorBars(numColBars);
        apiPaletteGetColorBarIdx((m_pConfigData->m_paletteNum + 1) % numColBars, newPalette);

        m_currentPalette = newPalette.colorVector256;
    }
    else
    {
        // Only false Colors
        m_currentPalette = QVector<ito::uint32>(256);

        for(int i = 0; i < 256; i++)
        {
            m_currentPalette[i] = i + (i << 8) + (i << 16);
        }
    }

    QFont nf;
    SFont sf;

#if QT_VERSION < QT_VERSION_CHECK(5,0,0)
    if ((ret = glewInit()) != GLEW_OK)
    {
        m_isInit = 0;
        return;
    }
    GLuint tmpArr;
    glGenVertexArrays(1, &tmpArr);
    m_VAO3D.insert(0, tmpArr);
    m_transperency.insert(0, 255);
    m_enabledHash.insert(0, true);
#else
    // Create VAO for first object to render
    // see http://stackoverflow.com/questions/17578266/where-are-glgenvertexarrays-glbindvertexarrays-in-qt-5-1
    QOpenGLVertexArrayObject *tmpArr = new QOpenGLVertexArrayObject(this);
    tmpArr->create();
    m_VAO3D.insert(0, tmpArr);
    m_transperency.insert(0, 255);
    m_enabledHash.insert(0, true);

    #if QT_VERSION >= 0x050400
    m_glf = new QOpenGLFunctions(context());
    #else
    m_glf = new QOpenGLFunctions(context()->contextHandle());
    #endif

    if (!m_glf)
    {
        m_isInit = 0;
        return;
    }
    m_glf->initializeOpenGLFunctions();
#endif

     //!> initialization of the 3D output stuff
    glShadeModel(GL_SMOOTH);                            //Smooth Shading
    glClearDepth(2.0f);                                 //Tiefenpuffer setzen
    GLFPTR(glEnable)(GL_DEPTH_TEST);                            //Tiefenpuffertest aktivieren
    GLFPTR(glDepthFunc)(GL_LEQUAL);                             //welcher Test
//    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  //Perspektivenkorrektur an
    GLFPTR(glHint)(GL_LINE_SMOOTH_HINT, GL_NICEST);             //Linien Antialiasing
    GLFPTR(glHint)(GL_POINT_SMOOTH_HINT, GL_NICEST);

    GLFPTR(glClearColor)((float)((m_pConfigData->m_backgnd >> 16) & 0xFF),
                 (float)((m_pConfigData->m_backgnd >> 8 ) & 0xFF),
                 (float)((m_pConfigData->m_backgnd      ) & 0xFF),
                 0.0f);

    if (makeShaderProg("3D", m_prog3D) != 0)
    {
        m_isInit = 0;
        return;
    }
    GLFPTR(glUseProgram)(m_prog3D);

    //!> retrieve location of uniform variables MVP and Diffuse from shader program
    m_unifMVP = GLFPTR(glGetUniformLocation)(m_prog3D, "MVP");
    m_unifVCT = GLFPTR(glGetUniformLocation)(m_prog3D, "VCT");
    m_unifPalette = GLFPTR(glGetUniformLocation)(m_prog3D, "palette");
    m_unifUsePalette = GLFPTR(glGetUniformLocation)(m_prog3D, "usePalette");
    m_unifGlColor3D = GLFPTR(glGetUniformLocation)(m_prog3D, "glColor");
    m_unifLighting = GLFPTR(glGetUniformLocation)(m_prog3D, "useLigthing");
    m_unifLColor = GLFPTR(glGetUniformLocation)(m_prog3D, "lColor");
    m_unifAmbient = GLFPTR(glGetUniformLocation)(m_prog3D, "ambient");
    m_unifDiffuse = GLFPTR(glGetUniformLocation)(m_prog3D, "diffuse");
    m_unifDiffuseDir = GLFPTR(glGetUniformLocation)(m_prog3D, "diffuseDir");
    m_unifText = GLFPTR(glGetUniformLocation)(m_prog3D, "useTexture");
    m_unifDiffMode = GLFPTR(glGetUniformLocation)(m_prog3D, "diffMode");
    m_unifDiffNorm = GLFPTR(glGetUniformLocation)(m_prog3D, "diffNorm");
    m_unifDiffMin = GLFPTR(glGetUniformLocation)(m_prog3D, "diffMin");
    m_uniCurAlpha = GLFPTR(glGetUniformLocation)(m_prog3D, "curAlpha");
    m_attribVert = GLFPTR(glGetAttribLocation)(m_prog3D, "vertex");
    m_attribNorm = GLFPTR(glGetAttribLocation)(m_prog3D, "normal");
    m_attribVertColor = GLFPTR(glGetAttribLocation)(m_prog3D, "vertColor");
    m_attribTexCol = GLFPTR(glGetAttribLocation)(m_prog3D, "texture");
    m_attribDiff = GLFPTR(glGetAttribLocation)(m_prog3D, "diff");

    m_unifGlColorInv = GLFPTR(glGetUniformLocation)(m_prog3D, "invColor");

    GLfloat umat [16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    GLint tmpmat = GLFPTR(glGetUniformLocation)(m_prog3D, "gl_ProjectionMatrix");
    if (tmpmat != -1)
        GLFPTR(glUniformMatrix4fv)(tmpmat, 1, GL_FALSE, umat);
    tmpmat = GLFPTR(glGetUniformLocation)(m_prog3D, "gl_ModelViewMatrix");
    if (tmpmat != -1)
        GLFPTR(glUniformMatrix4fv)(tmpmat, 1, GL_FALSE, umat);
    tmpmat = GLFPTR(glGetUniformLocation)(m_prog3D, "gl_ModelViewProjectionMatrix");
    if (tmpmat != -1)
        GLFPTR(glUniformMatrix4fv)(tmpmat, 1, GL_FALSE, umat);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(0.0, 0.0, 0.0, 0.0);
    glScalef(1.0, 1.0, 1.0);

    GLFPTR(glClear)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //Screen und Tiefenpuffer leeren
    setMVP(m_unifMVP, 45.0f, 0.01f, 100.0f, (float)width() / (float)height());
    setVCT(m_unifVCT, 0);
    GLFPTR(glUniform1f)(m_unifText, 0.0f);
    GLFPTR(glUniform1i)(m_unifLighting, 0);

    GLFPTR(glUniform4f)(m_unifGlColorInv,
        ((m_pConfigData->m_invColor & 0x00FF0000) >> 16) / 255.0f,
        ((m_pConfigData->m_invColor & 0x0000FF00) >> 8 ) / 255.0f,
        ((m_pConfigData->m_invColor & 0x000000FF)      ) / 255.0f,
        ((m_pConfigData->m_invColor & 0xFF000000)>> 24 ) / 255.0f);

    //!> create vertex buffer on device
    if (!m_vertBuf3D.contains(0))
    {
        GLuint tmpVal;
        GLFPTR(glGenBuffers)(1, &tmpVal);
        m_vertBuf3D.insert(0, tmpVal);
    }
    if (!m_textBuf3D.contains(0))
    {
        GLuint tmpVal;
        GLFPTR(glGenBuffers)(1, &tmpVal);
        m_textBuf3D.insert(0, tmpVal);
    }

    //!> Initialization of the stuff used for plotting axis etc.
#if QT_VERSION < 0x050000
    glGenVertexArrays(1, &m_VAO3DPri);
#else
    m_VAO3DPri = new QOpenGLVertexArrayObject(this);
    m_VAO3DPri->create();
#endif
    if (makeShaderProg("3DPri", m_prog3DPri) != 0)
    {
        m_isInit = 0;
        return;
    }
    GLFPTR(glUseProgram)(m_prog3DPri);
    m_unifMVPPri = GLFPTR(glGetUniformLocation)(m_prog3DPri, "MVP");
    m_unifVCTPri = GLFPTR(glGetUniformLocation)(m_prog3DPri, "VCT");
    m_attribVert3DPri = GLFPTR(glGetAttribLocation)(m_prog3DPri, "vertex");
    m_unifGlColor3DPri = GLFPTR(glGetUniformLocation)(m_prog3DPri, "glColor");
    setMVP(m_unifMVPPri, 45.0f, 0.01f, 100.0f, (float)width() / (float)height());
    setVCT(m_unifVCTPri, 0);
    GLFPTR(glGenBuffers)(1, &m_vertBuf3DPri);
    GLFPTR(glUseProgram)(0);

    //!> initialization of the 2D output stuff, i.e. used for plotting text
    if (makeShaderProg("2DPX", m_prog2DPx) != 0)
    {
        m_isInit = 0;
        return;
    }
    GLFPTR(glUseProgram)(m_prog2DPx);
    m_unifTextColor = GLFPTR(glGetUniformLocation)(m_prog2DPx, "textColor");
    m_unifScaleX = GLFPTR(glGetUniformLocation)(m_prog2DPx, "scaleX");
    m_unifScaleY = GLFPTR(glGetUniformLocation)(m_prog2DPx, "scaleY");
    m_unifPosX = GLFPTR(glGetUniformLocation)(m_prog2DPx, "posX");
    m_unifPosY = GLFPTR(glGetUniformLocation)(m_prog2DPx, "posY");
    m_unifGlColor2D = GLFPTR(glGetUniformLocation)(m_prog2DPx, "glColor");
    m_unifUseTex = GLFPTR(glGetUniformLocation)(m_prog2DPx, "useTex");
    m_attribTxtVert = GLFPTR(glGetAttribLocation)(m_prog2DPx, "inVert");
    m_attribTxtUV = GLFPTR(glGetAttribLocation)(m_prog2DPx, "inUV");
    GLFPTR(glUniform3f)(m_unifTextColor, 1.0, 0.0, 1.0);

    ret = glGetError();

    nf = QFont(QString::fromStdString(m_axes.m_fontName));
    nf.setPixelSize(m_axes.m_fontSize);
    prepareFont(nf, sf);

    // prepare vao, vertex buffer and texture for colorbar
#if QT_VERSION < 0x050000
    glGenVertexArrays(1, &m_cBarVAO);
    glBindVertexArray(m_cBarVAO);
#else
    m_cBarVAO = new QOpenGLVertexArrayObject(this);
    m_cBarVAO->create();
    m_cBarVAO->bind();
#endif
    GLFPTR(glGenBuffers)(1, &m_cBarVBuf);
    GLFPTR(glGenTextures)(1, &m_cBarTex);
#if QT_VERSION < 0x050000
    glBindVertexArray(0);
#else
    m_cBarVAO->release();
#endif
    GLFPTR(glUseProgram)(0);

    resizeGL(width(), height());

    if (ret = glGetError())
    {
        std::cerr << "error setting up openGLWindow window: " << ret << "\n";
    }
    else
    {
        m_isInit = IS_INIT;
        m_forceReplot = 1; // as initializeGL is called for QOpenGLWidget when the window is docked in
                           // we must do a real replot in that case
        refreshPlot(NULL);
    }
}

//-----------------------------------------------------------------------------------------------
SFont::~SFont()
{
}

//-----------------------------------------------------------------------------------------------
/** calculate model projection matrix
*   @param [in] uniformLoc  shader engine variable for mvp matrix
*   @param [in] fov         field of view
*   @param [in] near        near point for projection matrix
*   @param [in] far         far point for projection matrix
*   @param [in] ratio       ratio of x/y size
*   @return     returns the calculated matrix as cv:Mat
*
*   This method calculates a projection matrix used generally for 3D output and stores it into the shader program variable passed.
*   Except the variables passed the according member variables for zoom, and shift are used for the matrix calculation.
*/
cv::Mat TwipOGLWidget::setMVP(const GLint uniformLoc, const float fov, const float near, const float far, const float ratio, const bool noScale)
{
//    cv::Mat cvMVP = makePerspective(fov, near, far, ratio) * getWorldMatrix();
    cv::Mat cvMVP = getWorldMatrix(noScale);

    //if(m_state & tZoomed) cvMVP = cvMVP * makeTransMat(s_a, s_b, s_c);
    if(m_zoomer.enabled)
    {
        cvMVP = cvMVP * makeTransMat(m_zoomer.shiftx, m_zoomer.shifty, m_zoomer.shiftz) * makeScaleMat(m_zoomer.scalexy, m_zoomer.scalexy, 1.0f);
    }

    //!> Set the value of coordinate transform (MVP) uniform.
    GLFPTR(glUniformMatrix4fv)(uniformLoc, 1, GL_FALSE, (GLfloat*)cvMVP.ptr(0));

    return cvMVP;
}

//-----------------------------------------------------------------------------------------------
/** calculate scaling and shifting matrix
*   @param [in] uniformLoc  shader variable to store matrix in
*   @param [in] state       control if identity matrix or shifting / scaling matrix is used
*   @return     returns the calculated matrix as cv:Mat
*
*   This method calculates a shifting / scaling matrix used for 3D projections. The state variable controls
*   whether an identity matrix is uploaded or a matrix calculated from the according member variables.
*/
cv::Mat TwipOGLWidget::setVCT(const GLint uniformLoc, const int state)
{
    cv::Mat mVCT;
    if (state == 1)
    {
//        mVCT = makeTransMat(m_transX, m_transY, m_transZ) * makeScaleMat(m_scaleX, m_scaleY, m_scaleZ * m_pConfigData->m_zAmpl);
        mVCT = makeTransMat(m_transX, m_transY, m_transZ) * makeScaleMat(m_scaleX, m_scaleY, m_scaleZ);
    }
    else
    {
        mVCT = cv::Mat::zeros(4, 4, CV_32F);
        mVCT.at<float>(0, 0) = 1.0;
        mVCT.at<float>(1, 1) = 1.0;
        mVCT.at<float>(2, 2) = 1.0;
        mVCT.at<float>(3, 3) = 1.0;
    }

    //!> Set the value of coordinate transform (MVP) uniform.
    GLFPTR(glUniformMatrix4fv)(uniformLoc, 1, GL_FALSE, (GLfloat*)mVCT.ptr(0));

    return mVCT;
}

//-----------------------------------------------------------------------------------------------
/** Prepare font for use in openGL widget
*   @param [in] font            font to be prepared
*   @param [in | out] glfont    created opengl font
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   This methods prepares a font for use in the opengl widget. For each font family, style and size
*   a separate font has to be created as in opengl only bitmap fonts can be used. Before
*   actually creating a font it is checked if the requested font already had been created. The
*   preparation process creates billboards for all character sizes and textures for all characters.
*   The created font is stored on the graphics board and in a hash list for later use.
*/
ito::RetVal TwipOGLWidget::prepareFont(const QFont &font, struct SFont &glfont)
{
    glfont.m_name = font.family();
    glfont.m_weight = font.weight();

    // so first we grab the font metric of the font being used
    QFontMetricsF metric(font);

    // this allows us to get the height which should be the same for all
    // fonts of the same class as this is the total glyph height
    float fontHeight = glfont.m_height = metric.height();

    // loop for all basic keyboard chars we will use space to ~
    // should really change this to unicode at some stage
    const static char startChar = ' ';
    const static char endChar = '~';

    // Most OpenGL cards need textures to be in powers of 2 (128x512 1024X1024 etc etc) so
    // to be safe we will conform to this and calculate the nearest power of 2 for the glyph height
    // we will do the same for each width of the font below
    int heightPow2 = nearestPOT(fontHeight);

    int glfontSize = (int)endChar - (int)startChar + 1;
    glfont.m_chars.resize(glfontSize);

    // we are now going to create a texture / billboard for each font
    // they will be the same height but will possibly have different widths
    for (unsigned char c = startChar; c <= endChar; ++c)
    {
        QChar ch(c);
        FChar fc;

        // get the width of the font and calculate the ^2 size
        float charWidth;

#if (QT_VERSION >= QT_VERSION_CHECK(5, 11, 0))
        charWidth = metric.horizontalAdvance(ch);
#else
        charWidth = metric.width(ch);
#endif

        int widthPow2 = nearestPOT(charWidth);

        // now we set the texture co-ords for our quad it is a simple
        // triangle billboard with tex-cords as shown
        //  s0/t0  ---- s1,t0
        //         |\ |
        //         | \|
        //  s0,t1  ---- s1,t1
        // each quad will have the same s0 and the range s0-s1 == 0.0 -> 1.0
        float s0 = 0.0;

        // we now need to scale the tex cord to it ranges from 0-1 based on the coverage
        // of the glyph and not the power of 2 texture size. This will ensure that kerns
        // / ligatures match
        float s1 = (float)charWidth / (float)nearestPOT(charWidth);

        // t0 will always be the same
#if QT_VERSION < 0x050400
        float t0 = 0.0;
#else
        float t0 = (float)fontHeight / (float)heightPow2;
#endif

        // this will scale the height so we only get coverage of the glyph as above
#if QT_VERSION < 0x050400
        float t1 = (float)fontHeight / (float)heightPow2;
#else
        float t1 = 0.0;
#endif

        // we need to store the font width for later drawing
#if (QT_VERSION >= QT_VERSION_CHECK(5, 11, 0))
        fc.m_width = metric.horizontalAdvance(ch);
#else
        fc.m_width = metric.width(ch);
#endif

        // now we will create a QImage to store the texture, basically we are going to draw
        // into the qimage then save this in OpenGL format and load as a texture.
        // This is relatively quick but should be done as early as possible for max performance when drawing
        QImage finalImage(widthPow2, heightPow2, QImage::Format_ARGB32);

        // set the background for transparent so we can avoid any areas which don't have text in them
        finalImage.fill(Qt::transparent);

        // we now use the QPainter class to draw into the image and create our billboards
        QPainter painter;
        painter.begin(&finalImage);

        // try and use high quality text rendering (works well on the mac not as good on linux)
        painter.setRenderHints(
            QPainter::Antialiasing |
            QPainter::SmoothPixmapTransform |
            QPainter::TextAntialiasing |
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
            QPainter::Antialiasing
#else
            QPainter::HighQualityAntialiasing |
            QPainter::NonCosmeticDefaultPen
#endif
        );

        // set the font to draw with
        painter.setFont(font);

        // we set the glyph to be drawn in black the shader will override the actual colour later
        // see TextShader.h in src/shaders/
        painter.setPen(Qt::black);

        // finally we draw the text to the Image
        painter.drawText(0, metric.ascent(), QString(ch));
        painter.end();

        // now we create the OpenGL texture ID and bind to make it active
        GLFPTR(glGenTextures)(1, &fc.m_texID);
        GLFPTR(glBindTexture)(GL_TEXTURE_2D, fc.m_texID);
        GLFPTR(glTexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        GLFPTR(glTexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        GLFPTR(glPixelStorei)(GL_UNPACK_ALIGNMENT, 1);

        // QImage has a method to convert itself to a format suitable for OpenGL
        // we call this and then load to OpenGL
#if QT_VERSION >= 0x050400
//        finalImage = QOpenGLWidget::convertToGLFormat(finalImage);
#else
        finalImage = QGLWidget::convertToGLFormat(finalImage);
#endif

        int w = finalImage.width();
        int h = finalImage.height();
        // the image in in RGBA format and unsigned byte load it ready for later
        GLFPTR(glTexImage2D)(GL_TEXTURE_2D, 0, GL_RGBA, finalImage.width(), finalImage.height(),
          0, GL_RGBA, GL_UNSIGNED_BYTE, finalImage.bits());
        GLFPTR(glBindTexture)(GL_TEXTURE_2D, 0);

        // this structure is used by the VAO to store the data to be uploaded
        // for drawing the quad
/*
        struct textVertData
        {
            GLfloat x;
            GLfloat y;
            GLfloat z;
            GLfloat u;
            GLfloat v;
        };
*/

        // we are creating a billboard with two triangles so we only need the
        // 6 verts, (could use index and save some space but shouldn't be too much of an
        // issue
        textVertData d[4];

        // load values for triangle 1
        d[0].x = 0;
        d[0].y = 0;
        d[0].z = 0;
        d[0].u = s0;
        d[0].v = t1;

        d[1].x = 0;
        d[1].y = fontHeight;
        d[1].z = 0;
        d[1].u = s0;
        d[1].v = t0;

        d[2].x = charWidth;
        d[2].y = 0;
        d[2].z = 0;
        d[2].u = s1;
        d[2].v = t1;

        d[3].x = charWidth;
        d[3].y = fontHeight;
        d[3].z = 0;
        d[3].u = s1;
        d[3].v = t0;

        // see if we have a Billboard of this width already
        if (!glfont.m_vboBufs.contains(charWidth))
        {
            VBO *cvbo = new VBO();
#if QT_VERSION < 0x050000
            glGenVertexArrays(1, &cvbo->m_VAO);
            glBindVertexArray(cvbo->m_VAO);
#else
            cvbo->m_VAO = new QOpenGLVertexArrayObject(this);
            cvbo->m_VAO->create();
            cvbo->m_VAO->bind();
#endif
            GLFPTR(glEnableVertexAttribArray)(m_attribTxtVert);
            GLFPTR(glEnableVertexAttribArray)(m_attribTxtUV);

            GLFPTR(glGenBuffers)(1, &cvbo->m_vbufId);
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, cvbo->m_vbufId);

            //!> copy vertex coordinates
            GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 4 * sizeof(textVertData), d, GL_STATIC_DRAW);

            // now we set the attribute pointer to be 0 (as this matches vertIn in our shader)
            GLFPTR(glVertexAttribPointer)(m_attribTxtVert, 3, GL_FLOAT, 0, sizeof(textVertData), 0);

            // We can now create another set of data (which will be added to the VAO)
            // in this case the UV co-ords
            // now we set this as the 2nd attribute pointer (1) to match inUV in the shader
            GLFPTR(glVertexAttribPointer)(m_attribTxtUV, 2, GL_FLOAT, 0, sizeof(textVertData), (void*)(3 * sizeof(GLfloat)));

            //!> unbind buffer
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, 0);

#if QT_VERSION < 0x050000
            glBindVertexArray(0);
#else
            cvbo->m_VAO->release();
#endif
            // store the vao pointer for later use in the draw method
            fc.m_vbo = glfont.m_vboBufs[charWidth] = cvbo;
            fc.m_vbo = cvbo;
        }
        else
        {
            fc.m_vbo = glfont.m_vboBufs[charWidth];
        }

        // finally add the element to the map, this must be the last
        // thing we do
        glfont.m_chars[c - startChar] = fc;
    }
    m_fonts.insert(font.family() + QString::number(font.pixelSize()), glfont);

    return ito::retOk;
}

//-----------------------------------------------------------------------------------------------
/** Draw 3D axis
*
*   This methods draws the axis including their ticks and labels around the plot. The z-axis
*   can be turned on / off with the member variable m_axes.m_axisZ.m_isVisible. All settings
*   for the axis are defined via the member variables in m_axes.maxisN. Drawing the axis is
*   quite simple. What makes the whole thing a little bit tricky is to determine when and where
*   we want to have the axes ticks and labels. Therefore we find out how the 3D pose is oriented.
*/
void TwipOGLWidget::DrawAxesOGL(void)
{
    if (m_axes.m_lineWidth == 0)
        return;

    GLFPTR(glUseProgram)(m_prog3DPri);
    cv::Mat mvpMat = setMVP(m_unifMVPPri, 45.0f, 0.01f, 100.0f, (float)width() / (float)height());
#if QT_VERSION < 0x050000
    glBindVertexArray(m_VAO3DPri);
#else
    m_VAO3DPri->bind();
#endif

    GLFPTR(glLineWidth)(m_axes.m_lineWidth);

    GLFPTR(glUniform3f)(m_unifGlColor3DPri,
        ((m_pConfigData->m_axisColor & 0x00FF0000) >> 16) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x0000FF00) >> 8) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x000000FF)) / 255.0f);

    GLfloat vertices[12] = {
        -1.0, -1.0,  0.0,
         1.0, -1.0,  0.0,
         1.0,  1.0,  0.0,
        -1.0,  1.0,  0.0
    };

    //Paint the axis itself
    GLFPTR(glEnableVertexAttribArray)(m_attribVert3DPri);
    GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_vertBuf3DPri);
    GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 12 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
    GLFPTR(glVertexAttribPointer)(m_attribVert3DPri, 3, GL_FLOAT, 0, 3 * sizeof(GLfloat), 0);

    GLFPTR(glDrawArrays)(GL_LINE_LOOP, 0, 4);
    GLFPTR(glDisableVertexAttribArray)(m_attribVert3DPri);

    //!> we want that our ticks and labels always appear on the front / lower and the outer most (left /right)
    //!> axis. To know the right axis we check the combination of pitch / yaw angle.
    float ticklen = 0.03f;
    float signX1 = 1.0f;
    float signY1 = 1.0f;
    if ((m_pConfigData->m_pitchAng > -GL_PI && m_pConfigData->m_pitchAng <= -GL_PI / 2.0) ||
        (m_pConfigData->m_pitchAng > 0 && m_pConfigData->m_pitchAng <= GL_PI / 2.0))
    {
        signX1 = 1.0;
        //signX1 = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
        //signY1 = -1.0;
        signY1 = m_axes.m_axisY.m_isflipped ? -1.0 : 1.0;
    }
    else
    {
        signX1 = -1.0;
        //signX1 = m_axes.m_axisY.m_isflipped ? -1.0 : 1.0;
        //signY1 = 1.0;
        signY1 = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
    }

    float signX3 = 1.0;
    if ((m_pConfigData->m_yawAng > GL_PI / 2.0 && m_pConfigData->m_yawAng <= GL_PI) ||
        (m_pConfigData->m_yawAng > -GL_PI && m_pConfigData->m_yawAng <= -GL_PI / 2.0))
    {
        //signX3 = 1.0;
        signX3 = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
    }
    else
    {
        //signX3 = -1.0;
        signX3 = m_axes.m_axisY.m_isflipped ? -1.0 : 1.0;
    }

    float signY3 = 1.0;
    if ((m_pConfigData->m_yawAng > -GL_PI && m_pConfigData->m_yawAng <= 0.0))
    {
        //signY3 = 1.0;
        signY3 = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
    }
    else
    {
        //signY3 = -1.0;
        signY3 = m_axes.m_axisY.m_isflipped ? -1.0 : 1.0;
    }

    float tickDistNorm = width() > height() ? height() : width();


//    float numticksx = sqrt(ayp.dot(ayp)) / tickDist * tickDistNorm;
//    float numticksy = sqrt(axp.dot(axp)) / tickDist * tickDistNorm;
    float numticksx = 20;
    float numticksy = 20;

    GLfloat *ticksVert = (GLfloat*)calloc((numticksx + numticksy + 4) * 6, sizeof(GLfloat));

    // x-axis ticks
    ticksVert[0] = -1.0;
    ticksVert[1] = signX1 * signX3 * 1.0;
    ticksVert[3] = -1.0;
    ticksVert[4] = signX1 * signX3 * (1.0 + ticklen);
    for (int nt = 1; nt < numticksx; nt++)
    {
        float xval = -1.0 + nt * 2.0 / numticksx;
        ticksVert[6 + (nt - 1) * 6] = xval;
        ticksVert[6 + (nt - 1) * 6 + 1] = signX1 * signX3 * 1.0;
        ticksVert[6 + (nt - 1) * 6 + 3] = xval;
        ticksVert[6 + (nt - 1) * 6 + 4] = signX1 * signX3 * (1.0 + ticklen);
    }
    ticksVert[6 + (int)numticksx * 6] = 1.0;
    ticksVert[6 + (int)numticksx * 6 + 1] = signX3 * 1.0;
    ticksVert[6 + (int)numticksx * 6 + 3] = 1.0;
    ticksVert[6 + (int)numticksx * 6 + 4] = signX1 * signX3 * (1.0 + ticklen);

    // y-axis ticks
    ticksVert[12 + (int)numticksx * 6] = signY1 * signY3 * 1.0;
    ticksVert[12 + (int)numticksx * 6 + 1] = -1.0;
    ticksVert[12 + (int)numticksx * 6 + 3] = signY1 * signY3 * (1.0 + ticklen);
    ticksVert[12 + (int)numticksx * 6 + 4] = -1.0;
    for (int nt = 1; nt < numticksy; nt++)
    {
        float yval = -1.0 + nt * 2.0 / numticksy;
        ticksVert[18 + (int)numticksx * 6 + (nt - 1) * 6] = signY1 * signY3 * 1.0;
        ticksVert[18 + (int)numticksx * 6 + (nt - 1) * 6 + 1] = yval;
        ticksVert[18 + (int)numticksx * 6 + (nt - 1) * 6 + 3] = signY1 * signY3 * (1.0 + ticklen);
        ticksVert[18 + (int)numticksx * 6 + (nt - 1) * 6 + 4] = yval;
    }
    ticksVert[18 + (int)numticksx * 6 + (int)numticksy * 6] = signY1 * signY3 * 1.0;
    ticksVert[18 + (int)numticksx * 6 + (int)numticksy * 6] = 1.0;
    ticksVert[18 + (int)numticksx * 6 + (int)numticksy * 6] = signY1 * signY3 * (1.0 + ticklen);
    ticksVert[18 + (int)numticksx * 6 + (int)numticksy * 6] = 1.0;

    GLFPTR(glBufferData)(GL_ARRAY_BUFFER, (numticksx + numticksy + 2) * 6 * sizeof(GLfloat), ticksVert, GL_STATIC_DRAW);
    GLFPTR(glEnableVertexAttribArray)(m_attribVert3DPri);

    GLFPTR(glDrawArrays)(GL_LINES, 0, (2 + numticksx + numticksy) * 2);
    GLFPTR(glDisableVertexAttribArray)(m_attribVert3DPri);

    free(ticksVert);

    cv::Mat zvec;
    cv::Mat zaxPos;
    char align, alignrt;
    if (m_axes.m_axisZ.m_isVisible)
    {
//        float numticksz = sqrt(azp.dot(azp)) / tickDist * tickDistNorm;

        float zaxX = 1.0, zaxY = 0;

        if (m_pConfigData->m_yawAng > -GL_PI && m_pConfigData->m_yawAng <= -GL_PI / 2.0)
        {
            //zaxY = 1.0;
            zaxY = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
            zaxX = 1.0;
        }
        else if (m_pConfigData->m_yawAng > -GL_PI / 2.0 && m_pConfigData->m_yawAng <= 0.0)
        {
            //zaxY = 1.0;
            zaxY = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
            zaxX = -1.0;
        }
        else if (m_pConfigData->m_yawAng > 0 && m_pConfigData->m_yawAng <= GL_PI / 2.0)
        {
            //zaxY = 1.0;
            zaxY = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
            zaxX = 1.0;
        }
        else if (m_pConfigData->m_yawAng > GL_PI / 2.0 && m_pConfigData->m_yawAng <= GL_PI)
        {
            //zaxY = 1.0;
            zaxY = m_axes.m_axisY.m_isflipped ? 1.0 : -1.0;
            zaxX = -1.0;
        }

        zvec = cv::Mat::zeros(4, 1, CV_32F);
        zvec.at<float>(0, 0) = zaxX;
        zvec.at<float>(1, 0) = zaxY;
        zvec.at<float>(3, 0) = 1.0;
        zaxPos = mvpMat.t() * zvec;
        align = zaxPos.at<float>(0, 0) > 0 ? 1 : -1;
        alignrt = zaxPos.at<float>(0, 0) > 0 ? 0 : 1;

        float numticksz = 10;
        float signZ1 = zaxX > 0 ? 1.0 : -1.0;
        float signZ3 = zaxY > 0 ? 1.0 : -1.0;

        GLfloat *ticksVert = (GLfloat*)calloc((numticksz + 3) * 6, sizeof(GLfloat));
        // the z-axis
        ticksVert[0] = zaxX;
        ticksVert[1] = zaxY;

        ticksVert[3] = zaxX;
        ticksVert[4] = zaxY;
//        ticksVert[5] = 2.0 * m_pConfigData->m_zAmpl;
        ticksVert[5] = 2.0;

        ticksVert[6] = zaxX;
        ticksVert[7] = zaxY;

        ticksVert[9] = zaxX + signZ1 * ticklen;
        ticksVert[10] = zaxY + signZ3 * ticklen;
        for (int nt = 1; nt < numticksz; nt++)
        {
//            float zval = 2.0 * m_pConfigData->m_zAmpl * nt / numticksz;
            float zval = 2.0 * nt / numticksz;
            ticksVert[12 + (nt - 1) * 6] = zaxX;
            ticksVert[12 + (nt - 1) * 6 + 1] = zaxY;
            ticksVert[12 + (nt - 1) * 6 + 2] = zval;

            ticksVert[12 + (nt - 1) * 6 + 3] = zaxX + signZ1 * ticklen;
            ticksVert[12 + (nt - 1) * 6 + 4] = zaxY + signZ3 * ticklen;
            ticksVert[12 + (nt - 1) * 6 + 5] = zval;
        }
        ticksVert[12 + (int)numticksz * 6] = zaxX;
        ticksVert[12 + (int)numticksz * 6 + 1] = zaxY;
//        ticksVert[12 + (int)numticksz * 6 + 2] = 2.0 * m_pConfigData->m_zAmpl;
        ticksVert[12 + (int)numticksz * 6 + 2] = 2.0;

        ticksVert[12 + (int)numticksz * 6 + 3] = zaxX + signZ1 * ticklen;
        ticksVert[12 + (int)numticksz * 6 + 4] = zaxY + signZ3 * ticklen;
//        ticksVert[12 + (int)numticksz * 6 + 5] = 2.0 * m_pConfigData->m_zAmpl;
        ticksVert[12 + (int)numticksz * 6 + 5] = 2.0;

        GLFPTR(glBufferData)(GL_ARRAY_BUFFER, (numticksz + 3) * 6 * sizeof(GLfloat), ticksVert, GL_STATIC_DRAW);
        GLFPTR(glEnableVertexAttribArray)(m_attribVert3DPri);
        GLFPTR(glDrawArrays)(GL_LINES, 0, (3 + numticksz) * 2);
        free(ticksVert);

        GLFPTR(glDisableVertexAttribArray)(m_attribVert3DPri);
    }
#if QT_VERSION < 0x050000
    glBindVertexArray(0);
#else
    m_VAO3DPri->release();
#endif
    GLFPTR(glUseProgram)(0);

    //!> the axes are drawn first and later on the labels. This is done, as we are using a different
    //!> shader program for text output and we want to avoid switching between programs. So from here
    //!> on is the label handling. The labels are not shown if they would superimpose each other -
    //!> at least in theory, depending on the font used this needs some fine tuning. A superimposition
    //!> is assumed when we look quite 'flat' onto the scene.
    if (m_axes.m_axisZ.m_isVisible)
    {
        if (m_pConfigData->m_pitchAng < GL_PI - 0.17 && m_pConfigData->m_pitchAng > -GL_PI + 0.17)
        {
            QString buf;
            buf.asprintf("%.3g", m_axes.m_axisZ.getMin());
            OGLTextOut(buf, ((zaxPos.at<float>(0, 0) + align * ticklen) / 2.0 + 0.5) * width(),
                ((zaxPos.at<float>(1, 0) + align * ticklen) / -2.0 + 0.5) * height(), alignrt, alignrt,
                QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
        }

//        zvec.at<float>(2, 0) = 2.0 * m_pConfigData->m_zAmpl;
        zvec.at<float>(2, 0) = 2.0;
        zaxPos = mvpMat.t() * zvec;
        if (m_pConfigData->m_pitchAng < -0.17 || m_pConfigData->m_pitchAng > 0.17)
        {
            QString buf;
            buf.asprintf("%.3g", m_axes.m_axisZ.getMax());
            OGLTextOut(buf, ((zaxPos.at<float>(0, 0) + align * ticklen) / 2.0 + 0.5) * width(),
                ((zaxPos.at<float>(1, 0) + align * ticklen) / -2.0 + 0.5) * height(), alignrt, alignrt,
                QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
        }

//        zvec.at<float>(2, 0) = 1.0 * m_pConfigData->m_zAmpl;
        zvec.at<float>(2, 0) = 1.0;
        zaxPos = mvpMat.t() * zvec;
        QString buf;
        if (m_axes.m_axisZ.m_label.length() != 0)
        {
            buf = QString::fromStdString(m_axes.m_axisZ.m_label);
        }
        else
        {
            buf = "";
        }

        if (m_axes.m_axisZ.m_unit.length() != 0)
        {
            buf.append(" ");
            buf.append(QString::fromStdString(m_axes.m_axisZ.m_unit));
        }

        OGLTextOut(buf, ((zaxPos.at<float>(0, 0) + align * 2.0 * ticklen) / 2.0 + 0.5) * width(),
            ((zaxPos.at<float>(1, 0) + align * 2.0 * ticklen) / -2.0 + 0.5) * height(), alignrt, alignrt,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
    }

    //!> as we also have to use a different text alignment depending on where we write the
    //!> the text we calculate the screen position for some relevant ticks. The starting
    //!> and end pixels of the ticks are used to determine if have have to align left / right
    //!> and top / bottom
    cv::Mat xvec = cv::Mat::zeros(4, 1, CV_32F);
    xvec.at<float>(0, 0) = -1.0 + ticklen;
    xvec.at<float>(1, 0) = signX1 * signX3 * (1.0 + ticklen);
    xvec.at<float>(2, 0) = -ticklen;
    xvec.at<float>(3, 0) = 1.0;
    cv::Mat xaxPos0 = mvpMat.t() * xvec;
    xvec.at<float>(0, 0) = 1.0 - ticklen;
    cv::Mat xaxPos1 = mvpMat.t() * xvec;

    cv::Mat xvec1 = cv::Mat::zeros(4, 1, CV_32F);
    xvec1.at<float>(0, 0) = -1.0 + ticklen;
    xvec1.at<float>(1, 0) = signX1 * signX3;
    xvec1.at<float>(2, 0) = -ticklen;
    xvec1.at<float>(3, 0) = 1.0;
    cv::Mat xaxPos2 = mvpMat.t() * xvec1;

    char alignrlx = xaxPos0.at<float>(0, 0) - xaxPos2.at<float>(0, 0) > 0 ? 0 : 1;
    char aligntbx = xaxPos0.at<float>(1, 0) - xaxPos2.at<float>(1, 0) > 0 ? 1 : 0;

    QString buf;
    buf.asprintf("%.3g", m_axes.m_axisX.getMin());
    if ((m_pConfigData->m_yawAng < -GL_PI / 2.0 -0.08 || m_pConfigData->m_yawAng > -GL_PI / 2.0 + 0.08)
        && (m_pConfigData->m_pitchAng < -GL_PI / 2.0 - 0.08 || m_pConfigData->m_pitchAng > -GL_PI / 2.0 + 0.08))
        OGLTextOut(buf, ((xaxPos0.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((xaxPos0.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrlx, aligntbx,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    buf.asprintf("%.3g", m_axes.m_axisX.getMax());
    if ((m_pConfigData->m_yawAng < -GL_PI / 2.0 -0.08 || m_pConfigData->m_yawAng > -GL_PI / 2.0 + 0.08)
        && (m_pConfigData->m_pitchAng < GL_PI / 2.0 - 0.08 || m_pConfigData->m_pitchAng > GL_PI / 2.0 + 0.08))
        OGLTextOut(buf, ((xaxPos1.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((xaxPos1.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrlx, aligntbx,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    xvec.at<float>(0, 0) = 0.0;
    xaxPos0 = mvpMat.t() * xvec;
    if (m_axes.m_axisX.m_label.length() != 0)
    {
        buf = QString::fromStdString(m_axes.m_axisX.m_label);
    }
    else
    {
        buf = "";
    }
    if (m_axes.m_axisX.m_unit.length() != 0)
    {
        buf.append(" ");
        buf.append(QString::fromStdString(m_axes.m_axisX.m_unit));
    }
    OGLTextOut(buf, ((xaxPos0.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((xaxPos0.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrlx, aligntbx,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    cv::Mat yvec = cv::Mat::zeros(4, 1, CV_32F);
    yvec.at<float>(0, 0) = signY1 * signY3 * (1.0 + ticklen);
    yvec.at<float>(1, 0) = -1.0 + ticklen;
    yvec.at<float>(2, 0) = -ticklen;
    yvec.at<float>(3, 0) = 1.0;
    cv::Mat yaxPos0 = mvpMat.t() * yvec;
    yvec.at<float>(1, 0) = 1.0 - ticklen;
    cv::Mat yaxPos1 = mvpMat.t() * yvec;

    cv::Mat yvec1 = cv::Mat::zeros(4, 1, CV_32F);
    yvec1.at<float>(0, 0) = signY1 * signY3;
    yvec1.at<float>(1, 0) = -1.0 + ticklen;
    yvec1.at<float>(2, 0) = -ticklen;
    yvec1.at<float>(3, 0) = 1.0;
    cv::Mat yaxPos2 = mvpMat.t() * yvec1;

    char alignrly = yaxPos0.at<float>(0, 0) - yaxPos2.at<float>(0, 0) > 0 ? 0 : 1;
    char aligntby = yaxPos0.at<float>(1, 0) - yaxPos2.at<float>(1, 0) > 0 ? 1 : 0;

    buf.asprintf("%.3g", m_axes.m_axisY.getMin());
    if ((m_pConfigData->m_yawAng < -GL_PI / 2.0 -0.08 || m_pConfigData->m_yawAng > -GL_PI / 2.0 + 0.08)
        && (m_pConfigData->m_pitchAng < -GL_PI / 2.0 - 0.08 || m_pConfigData->m_pitchAng > -GL_PI / 2.0 + 0.08))
        OGLTextOut(buf, ((yaxPos0.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((yaxPos0.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrly, aligntby,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    buf.asprintf("%.3g", m_axes.m_axisY.getMax());
    if ((m_pConfigData->m_yawAng < -GL_PI / 2.0 -0.08 || m_pConfigData->m_yawAng > -GL_PI / 2.0 + 0.08)
        && (m_pConfigData->m_pitchAng < GL_PI / 2.0 - 0.08 || m_pConfigData->m_pitchAng > GL_PI / 2.0 + 0.08))
        OGLTextOut(buf, ((yaxPos1.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((yaxPos1.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrly, aligntby,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    yvec.at<float>(1, 0) = 0.0;
    yaxPos0 = mvpMat.t() * yvec;
    if (m_axes.m_axisY.m_label.length() != 0)
    {
        buf = QString::fromStdString(m_axes.m_axisY.m_label);
    }
    else
    {
        buf = "";
    }

    if (m_axes.m_axisY.m_unit.length() != 0)
    {
        buf.append(" ");
        buf.append(QString::fromStdString(m_axes.m_axisY.m_unit));
    }
    OGLTextOut(buf, ((yaxPos0.at<float>(0, 0)) / 2.0 + 0.5) * width(),
            ((yaxPos0.at<float>(1, 0)) / -2.0 + 0.5) * height(), alignrly, aligntby,
            QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** OpenGL paint routine
*
*   In this method all painting is done. So we work through the different part of our gl scene.
*   These are the axis, the 3D data, the colorbar, the title / infotext and if necessary we handle the lighting.
*   A multiple call of paintGL is avoided using the m_isInit status variable.
*/
void TwipOGLWidget::paintGL()
{
    int glErr = 0;

    if (m_isInit != (IS_INIT | HAS_TRIANG))
        return;

    m_isInit |= IS_RENDERING;

    GLFPTR(glClearColor)((float)((m_pConfigData->m_backgnd >> 16) & 0xFF),
                 (float)((m_pConfigData->m_backgnd >> 8 ) & 0xFF),
                 (float)((m_pConfigData->m_backgnd      ) & 0xFF),
                 0.0f);

    GLFPTR(glClear)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);     // clear screen and depth buffer

    //!> Draw the axes
    DrawAxesOGL();

    GLFPTR(glUseProgram)(m_prog3D);
    GLFPTR(glEnable)(GL_BLEND);
    GLFPTR(glBlendFunc)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setMVP(m_unifMVP, 45.0f, 0.01f, 100.0f, width() / (float)height());
    if ((m_pConfigData->m_elementMode & PAINT_TRIANG && m_pConfigData->m_elementMode & PAINT_POINTS)) // || m_numVert == 0)
    {
        m_isInit &= ~IS_RENDERING;
        return;
    }

    //!> When illumination is enabled calculate the illumination vectors and push them to the shader program
    if (m_pConfigData->m_elementMode & ENABLE_ILLUMINATION)
    {
        GLfloat directionLS[3] = {0.0f, 0.0f, 1.0f};
        double norm;

        directionLS[0] = cos(m_pConfigData->m_lightDirPitch);
        directionLS[1] = sin(m_pConfigData->m_lightDirPitch);
        directionLS[2] = tan(m_pConfigData->m_lightDirYaw);

        norm = sqrt((directionLS[0] * directionLS[0]) + (directionLS[1] * directionLS[1]) + (directionLS[2] * directionLS[2]));
        directionLS[0] /= norm;
        directionLS[1] /= norm;
        directionLS[2] /= norm;

        GLFPTR(glUniform1i)(m_unifLighting, 1);
        GLFPTR(glUniform4f)(m_unifAmbient, 0.2f, 0.2f, 0.2f, 1.0f);
        GLFPTR(glUniform3f)(m_unifDiffuseDir, directionLS[0], directionLS[1], directionLS[2]);
        GLFPTR(glUniform4f)(m_unifDiffuse, 1.0f, 1.0f, 1.0f, 1.0f);
    }
    else
    {
        GLFPTR(glUniform4f)(m_unifDiffuse, 0.0f, 0.0f, 0.0f, 1.0f);
        GLFPTR(glUniform4f)(m_unifAmbient, 0.0f, 0.0f, 0.0f, 1.0f);
    }
    //!> only allow curvature / deviation plot when previously set
    if (m_pConfigData->m_elementMode & HAS_CURVATURE)
    {
        GLFPTR(glUniform1i)(m_unifDiffMode, m_pConfigData->m_showCurvature ? 1 : 0);
    }

    float devMin = m_axes.m_devAxis.getMin();
    if(!ito::isFinite(devMin)) devMin = 0.0;
    float devMax = m_axes.m_devAxis.getMax();
    if(!ito::isFinite(devMax)) devMax = 1.0;
    float devNorm = 1.0f;

    if(ito::isNotZero(devMax - devMin))
    {
        devNorm = 1.0f / fabs(devMax - devMin);
    }

    GLFPTR(glUniform1f)(m_unifDiffNorm, devNorm);
    GLFPTR(glUniform1f)(m_unifDiffMin, devMin);
    GLFPTR(glUniform1f)(m_uniCurAlpha, 1.0);


    //!> Draw all points / triangles
    setVCT(m_unifVCT, 1);
    setMVP(m_unifMVP, 45.0f, 0.01f, 100.0f, width() / (float)height());
    GLFPTR(glUniform1i)(m_unifUsePalette, 1);

    for (QHash<int, GLuint>::ConstIterator it = m_vertBuf3D.begin(); it != m_vertBuf3D.end(); it++)
    {
        int b = it.value();

        if (m_transperency.contains(it.key()))
        {
            GLFPTR(glUniform1f)(m_uniCurAlpha, m_transperency[it.key()] / 255.0);
        }

        if (!m_enabledHash.contains(it.key()) || !m_enabledHash[it.key()])
        {
            continue;
        }

#if QT_VERSION < 0x050000
        glBindVertexArray(m_VAO3D.value(it.key()));
        if(m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            glDrawArrays(GL_POINTS, 0, m_numVert.value(it.key()));
        }
        else
        {
            glDrawArrays(GL_TRIANGLES, 0, 3 * m_numVert.value(it.key()));
        }
        glBindVertexArray(0);
#else
        m_VAO3D.value(it.key())->bind();
        if (m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            GLFPTR(glDrawArrays)(GL_POINTS, 0, m_numVert.value(it.key()));
        }
        else
        {
            GLFPTR(glDrawArrays)(GL_TRIANGLES, 0, 3 * m_numVert.value(it.key()));
        }
        m_VAO3D.value(it.key())->release();
#endif
    }

    GLFPTR(glUniform1f)(m_uniCurAlpha, 1.0);
    GLFPTR(glUniform1i)(m_unifUsePalette, 0);
    GLFPTR(glUniform1i)(m_unifLighting, 0);
    setVCT(m_unifVCT, 0);
    GLFPTR(glDisable)(GL_BLEND);
    GLFPTR(glUseProgram)(0);

    //!> draw the arrow used to indicate / move the spot light direction when necessary
    if (m_pConfigData->m_drawLightDir)
    {
//        setMVP(m_unifMVP, 45.0f, 0.01f, 100.0f, width() / (float)height(), 1);
        paintLightArrow();
    }


    int yPos = 0;
    QString buf;
    buf.clear();
    if(m_pConfigData->m_plotAngles)
    {
        buf.asprintf("Angles:");
        OGLTextOut(buf, 10, yPos, 0, 0, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
        yPos += m_axes.m_fontSize;
        buf.asprintf("P: %.3f", m_pConfigData->m_pitchAng);
//        buf.sprintf("P: %.3f", m_pConfigData->m_lightDirPitch);
        OGLTextOut(buf, 10, yPos, 0, 0, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
        yPos += m_axes.m_fontSize;
        buf.asprintf("Y: %.3f", m_pConfigData->m_yawAng);
//        buf.sprintf("Y: %.3f", m_pConfigData->m_lightDirYaw);
        OGLTextOut(buf, 10, yPos, 0, 0, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

        yPos += m_axes.m_fontSize;
        buf.asprintf("A: %.3f", m_pConfigData->m_zAmpl);
        OGLTextOut(buf, 10, yPos, 0, 0, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
    }


    //!> draw the colorbar
    if(m_pConfigData->m_colorBarMode != COLORBAR_NO)
    {
        switch(m_pConfigData->m_colorBarMode)
        {
            case COLORBAR_LEFT:
                DrawColorBar(10.0, height() * 0.4 , 15.0, height() * 0.3, false);
            break;
            case COLORBAR_RIGHT:
                DrawColorBar(width() - 30.0, height() * 0.4 , 15.0, height() * 0.3, true);
            break;
            case COLORBAR_UPPER_RIGHT:
                DrawColorBar(width() - 30.0, m_axes.m_fontSize * 2.4, 15.0, height() * 0.3, true);
            break;
        }
    }

    //!> the object / plot title
    if (m_pConfigData->m_drawTitle)
    {    // noobjinfo
        DrawTitle();
    }

    //!> object info
    if(m_objectInfo.show)
    {
        DrawObjectInfo();
    }

    GLFPTR(glFlush)();
    //glFinish();
//    if (context()->format().doubleBuffer())
//        swapBuffers();

    m_isInit &= ~IS_RENDERING;

    return;
}

#ifdef USEPCL
//----------------------------------------------------------------------------------------------------------------------------------
/** method for loading point cloud to vertex array
*   @param [in] pcl        templated pointcloud
*   @param [in] id         internal id for storing / referencing the pointcloud
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   In this method a given point cloud is transformed into the according vertex array and
*   uploaded to the graphics board. If the point cloud has an intensity it is uploaded
*   as well. We use the curvature parameter of the point clouds to store the difference
*   to a model and use this as a scaling for the color palette. So the input point cloud is
*   analysed for this parameter. At last if normals are given we use them as well, as
*   we need them for lighting. If have normals we first try to get memory for points &
*   normals if that fails we just try points and if that fails as well we give up.
*   If an intensity overlay should be used the values must be scaled between 0 and 1
*   to work as expected.
*/
template<typename _Tp> ito::RetVal TwipOGLWidget::GLSetPointsPCL(pcl::PointCloud<_Tp> *pcl, const int id)
{
    ito::RetVal retVal(retOk);
    GLfloat *vertBuf = NULL;
    GLfloat *textBuf = NULL;
    m_numVert[id] = 0;
    int useNormals = 1;
    int ptSize = 4;

#ifdef USEPCL
    bool isFinite = false;
    ito::float64 threshold = 1.0;
    int width = m_pContentPC[id]->width();
    int height = m_pContentPC[id]->height();

    if(width * height > 0xEFFFFFFE /8)
    {
        useNormals = 0;
    }
    if(width * height > 0xEFFFFFFE /4)
    {
        m_isInit &= ~HAS_TRIANG;
        m_numVert[id] = 0;
        return ito::RetVal(ito::retError, 0, "Too many points, calc triangles/points failed");
    }

    //!> allocate temporary buffers in host memory so we can perpare the arrays for uploading
    if (!retVal.containsError())
    {
        // If m_elementMode == PAINT_POINTS try to paint points for faster, less memory consuming visualisation
        if (m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            if(hasPointToNormal<_Tp>() && useNormals)
            {
                //!> try to acquire memory for pointy & normals
                ptSize = 8;
                vertBuf = static_cast<GLfloat *>(calloc(width * height * ptSize, sizeof(GLfloat)));
                if (vertBuf == NULL)
                {
                    //!> not enough memory, so try just for points
                    ptSize = 4;
                    vertBuf = static_cast<GLfloat *>(calloc(width * height * ptSize, sizeof(GLfloat)));
                    retVal += ito::RetVal(ito::retWarning, 0, "low memory, disableing normals & lighting");
                    useNormals = 0;
                    ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_POINTS);
                }
                else
                {
                    ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_POINTS | ENABLE_ILLUMINATION);
                }
            }
            else
            {
                useNormals = 0;
                ptSize = 4;
                vertBuf = static_cast<GLfloat *>(calloc(width * height * ptSize, sizeof(GLfloat)));
                ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_POINTS);
            }

            if (m_pUseTextPix[id])
            {
                free(m_pUseTextPix[id]);
                m_pUseTextPix[id] = NULL;
            }

            if(hasPointToIntensity<_Tp>())
            {
                textBuf = (GLfloat*)calloc(width * height * 3, sizeof(GLfloat));
            }

            if (vertBuf == NULL)
            {
                //!> not enough memory so return
                m_isInit &= ~HAS_TRIANG;
                m_numVert[id] = 0;
                retVal += ito::RetVal(ito::retError, 0, "Error allocating memory");
                return retVal;
            }
        }
    }

    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    if (!retVal.containsError())
    {
        //!> opengl uses 'normalized' coordinates therefore we must transform the
        //!> input data accordingly. Normalized means in this case:
        //!> x in [-1, 1], y in [-1, 1], z in [0, -1]
        if (m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            float zmin = m_axes.m_axisZ.getMin();
            float zmax = m_axes.m_axisZ.getMax();
            float znorm = 1.0;

            if(ito::isNotZero(zmax - zmin))
            {
                znorm = 1.0f / fabs(zmax - zmin);
            }

            if (hasPointToCurvature<_Tp>() && hasPointToIntensity<_Tp>() && textBuf != NULL)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int npx = 0; npx < width * height; npx++)
                {
                    int count = 0;
                    _Tp pt = pcl->at(npx);
                    if (ito::isFinite<float>(pt.z))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }
                        vertBuf[count * ptSize] = pt.x;
                        vertBuf[count * ptSize + 1] = pt.y;
                        vertBuf[count * ptSize + 2] = pt.z;
                        vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((pt.z - zmin) * znorm * 255.0);
                        pointToCurvature<_Tp>(pt, vertBuf[count * ptSize + 4]);
                        if (useNormals)
                        {
                            pointToNormal(pt, &vertBuf[count * ptSize + 5]);
                        }
                        pointToIntensity<_Tp>(pt, textBuf[count * 3], textBuf[count * 3 + 1], textBuf[count * 3 + 2], 1.0f);
                    }
                }
                m_pConfigData->m_elementMode |= HAS_CURVATURE;
            }
            else if(hasPointToIntensity<_Tp>() && textBuf != NULL)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int npx = 0; npx < width * height; npx++)
                {
                    int count = 0;
                    _Tp pt = pcl->at(npx);
                    if (ito::isFinite<float>(pt.z))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }
                        vertBuf[count * ptSize] = pt.x;
                        vertBuf[count * ptSize + 1] = pt.y;
                        vertBuf[count * ptSize + 2] = pt.z;
                        vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((pt.z - zmin) * znorm * 255.0);
                        if (useNormals)
                        {
                            pointToNormal(pt, &vertBuf[count * ptSize + 5]);
                        }
                        pointToIntensity<_Tp>(pt, textBuf[count * 3], textBuf[count * 3 + 1], textBuf[count * 3 + 2], 1.0f);
                    }
                }
            }
            else if (hasPointToCurvature<_Tp>())
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int npx = 0; npx < width * height; npx++)
                {
                    int count = 0;
                    _Tp pt = pcl->at(npx);
                        if (ito::isFinite<float>(pt.z))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }
                        vertBuf[count * ptSize] = pt.x;
                        vertBuf[count * ptSize + 1] = pt.y;
                        vertBuf[count * ptSize + 2] = pt.z;
                        vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((pt.z - zmin) * znorm * 255.0f);
                        pointToCurvature<_Tp>(pt, vertBuf[count * ptSize + 4]);
                        if (useNormals)
                        {
                            pointToNormal(pt, &vertBuf[count * ptSize + 5]);
                        }
                    }
                }
                m_pConfigData->m_elementMode |= HAS_CURVATURE;
            }
            else
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int npx = 0; npx < width * height; npx++)
                {
                    int count = 0;
                    _Tp pt = pcl->at(npx);
                    if (ito::isFinite<float>(pt.z))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }
                        vertBuf[count * ptSize] = pt.x;
                        vertBuf[count * ptSize + 1] = pt.y;
                        vertBuf[count * ptSize + 2] = pt.z;
                        vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((pt.z - zmin) * znorm * 255.0f);
                        if (useNormals)
                        {
                            pointToNormal(pt, &vertBuf[count * ptSize + 5]);
                        }
                    }
                }
            }
        }
    }

    #if (USEOMP)
    }
    #endif

    if (m_numVert[id] != 0 || !retVal.containsError())
    {
        makeCurrent();
        if(m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            m_isInit &= ~HAS_TRIANG;
            Sleep(100);
            GLFPTR(glUseProgram)(m_prog3D);

            if (!m_vertBuf3D.contains(id))
            {
#if QT_VERSION < 0x050000
                GLuint tmpArr;
                glGenVertexArrays(1, &tmpArr);
#else
                QOpenGLVertexArrayObject *tmpArr;
                tmpArr = new QOpenGLVertexArrayObject(this);
                tmpArr->create();
#endif
                m_VAO3D.insert(id, tmpArr);
                m_transperency.insert(id, 255);
                m_enabledHash.insert(id, true);

                GLuint tmpVal;
                GLFPTR(glGenBuffers)(1, &tmpVal);
                m_vertBuf3D.insert(id, tmpVal);
            }

            if (!m_textBuf3D.contains(id))
            {
                GLuint tmpVal;
                GLFPTR(glGenBuffers)(1, &tmpVal);
                m_textBuf3D.insert(id, tmpVal);
            }

#if QT_VERSION < 0x050000
            glBindVertexArray(m_VAO3D.value(id));
#else
            m_VAO3D.value(id)->bind();
#endif
            //!> Here we activate the various buffers used in the shader, upload the buffer(s) and set their properties.
            //!> Even if we don't use some of the features, e.g. normals we must set the attributePointer to a
            //!> meaningful value, other we get problems on some graphics boards ... even if the attribute array
            //!> is disabled ...
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_vertBuf3D.value(id));
            GLFPTR(glEnableVertexAttribArray)(m_attribVert);
            GLFPTR(glEnableVertexAttribArray)(m_attribVertColor);
            GLFPTR(glEnableVertexAttribArray)(m_attribNorm);
            GLFPTR(glEnableVertexAttribArray)(m_attribDiff);

            GLFPTR(glVertexAttribPointer)(m_attribVert, 3, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), 0);
            GLFPTR(glVertexAttribPointer)(m_attribVertColor, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));

            if (hasPointToCurvature<_Tp>())
                GLFPTR(glVertexAttribPointer)(m_attribDiff, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(4 * sizeof(GLfloat)));
            else
                GLFPTR(glVertexAttribPointer)(m_attribDiff, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));

            if (useNormals)
                GLFPTR(glVertexAttribPointer)(m_attribNorm, 3, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(5 * sizeof(GLfloat)));
            else
                GLFPTR(glVertexAttribPointer)(m_attribNorm, 3, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), 0);

            GLFPTR(glBufferData)(GL_ARRAY_BUFFER, m_numVert[id] * ptSize * sizeof(GLfloat), vertBuf, GL_STATIC_DRAW);

            //!> this part is for uploading an intensity overlay image. In principle we must obey all said for the vertex buffers.
            if(textBuf)
            {
                ((TwipOGLFigure*)this->parent())->enableOverlaySlider(true);
                m_pConfigData->m_enableOverlay = true;
                GLFPTR(glEnableVertexAttribArray)(m_attribTexCol);
                GLFPTR(glUniform1f)(m_unifText, m_pConfigData->m_alpha);

                GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_textBuf3D.value(id));
                GLFPTR(glVertexAttribPointer)(m_attribTexCol, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
                GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 3 * m_numVert[id] * sizeof(GLfloat), textBuf, GL_STATIC_DRAW);
                //glBindVertexArray(0);
            }
            else
            {
                GLFPTR(glDisableVertexAttribArray)(m_attribTexCol);
            }

#if QT_VERSION < 0x050000
            glBindVertexArray(0);
#else
            m_VAO3D.value(id)->release();
#endif
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, 0);
            GLFPTR(glUseProgram)(0);

            m_isInit |= HAS_TRIANG;
        }
        else
        {
        }

        ResetColors();
        doneCurrent();

        m_errorDisplMsg.clear();
        m_isInit &= ~IS_CALCTRIANG;
    }

    if (m_numVert[id] == 0 || retVal.containsError())
    {
        m_isInit &= ~HAS_TRIANG;
        m_numVert[id] = 0;
        retVal += ito::RetVal(ito::retError, 0, "Error calculating points / triangles");
        m_isInit &= ~IS_CALCTRIANG;
    }
#endif

    if (textBuf)
        free(textBuf);
    if (vertBuf)
        free(vertBuf);

    return retVal;
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
/** method for loading dataObjects to vertex array
*   @param [in] id         internal id for storing / referencing the pointcloud
*   @param [in] isComplex  flag indicating whether the input data object is complex
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   In this method a given dataObject is transformed into the according vertex array and
*   uploaded to the graphics board. There are two methods. One painting triangles
*   and one painting points. The triangulation is quite dumb, it always uses four
*   neighbouring points to create 2 triangles including their normal vectors. This results
*   in an overall number of triangles 2 times the number of points. Painting just points
*   is far more lightweight than painting triangles.
*   For the triangles we first try to get memory for points &
*   normals if that fails we just try points and if that fails as well we give up.
*/
template<typename _Tp, typename _TpMat> ito::RetVal TwipOGLWidget::GLSetTriangles(const int id, const bool isComplex)
{
    ito::RetVal retVal(retOk);
    int xsizeObj = m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1;
    int ysizeObj = m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1;
    int useNormals = 1;
    int ptSize = 21;

    GLfloat *vertBuf = NULL;

    if(!(m_pContentDObj[id] && xsizeObj && ysizeObj))
    {
        m_isInit &= ~HAS_TRIANG;
        return ito::RetVal(ito::retError, 0, "DataObject empty, calc triangles/points failed");
    }

    m_numVert[id] = 0;

    ito::float64 threshold = 1.0;

    if(xsizeObj * ysizeObj > 0xEFFFFFFE /21 /2)
    {
        useNormals = 0;
    }
    if(xsizeObj * ysizeObj > 0xEFFFFFFE /12 /2)
    {
        m_pConfigData->m_elementMode &= ~PAINT_TRIANG;
        m_pConfigData->m_elementMode |= PAINT_POINTS;
    }
    if(xsizeObj * ysizeObj > 0xEFFFFFFE /6 /2)
    {
        m_pConfigData->m_elementMode &= ~PAINT_TRIANG;
        m_pConfigData->m_elementMode |= PAINT_POINTS;
    }
    if(xsizeObj * ysizeObj > 0xEFFFFFFE /4)
    {
        m_isInit &= ~HAS_TRIANG;
        return ito::RetVal(ito::retError, 0, "Too many points, calc triangles/points failed");
    }

    if(!retVal.containsError())
    {
        // If m_elementMode == PAINT_POINTS try to paint points for faster, less memory consuming visualisation
        if(m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            vertBuf = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 4, sizeof(GLfloat)));
            if (m_pUseTextPix[id])
            {
                free(m_pUseTextPix[id]);
                m_pUseTextPix[id] = NULL;
            }
            m_pUseTextPix[id] = (int*)calloc(xsizeObj * ysizeObj * 2, sizeof(int));

            if(vertBuf == NULL)
            {
                m_isInit &= ~HAS_TRIANG;
                m_numVert[id] = 0;
                retVal += ito::RetVal(ito::retError, 0, "Error allocating memory");
                return retVal;
            }
            ptSize = 4;
            ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_POINTS | PAINT_TRIANG);
        }
        else // PAINT_TRIANG
        {
            // we use an interleaved vertex buffer and an index buffer for our triangles
            // we need 12 or 21 float for each triangle: 3 x pts(x, y, z) + 1 x color(idx) + 3 x normal(x, y, z)
            if( useNormals == 1)
            {
                vertBuf = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 21 * 2, sizeof(GLfloat)));
            }

            if (m_pUseTextPix[id])
            {
                free(m_pUseTextPix[id]);
                m_pUseTextPix[id] = NULL;
            }
            m_pUseTextPix[id] = (int*)calloc(xsizeObj * ysizeObj * 6 * 2, sizeof(int));

            if(vertBuf == NULL)
            {
                // not enough memory for normals so we try just triangles
                vertBuf = static_cast<GLfloat *>(calloc(xsizeObj * ysizeObj * 12 * 2, sizeof(GLfloat)));
                if (vertBuf == NULL)
                {
                    m_isInit &= ~HAS_TRIANG;
                    m_numVert[id] = 0;
                    retVal += ito::RetVal(ito::retError, 0, "Error allocating memory");
                    if (vertBuf) free(vertBuf);
                    return retVal;
                }
                else
                {
                    useNormals = 0;
                    retVal += ito::RetVal(ito::retWarning, 0, "low memory, disabling normals & lighting");
                    ptSize = 12;
                    ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_TRIANG);
                }
            }
            else
            {
                ((TwipOGLFigure*)this->parent())->enableRenderModeSelector(PAINT_TRIANG | ENABLE_ILLUMINATION);
            }
        }
    }

    cv::Mat * dMat = NULL;
    cv::Mat * invalidMap = NULL;

    bool fillInvalids = m_pConfigData && ((InternalData*) m_pConfigData)->m_showInvalids;

    if(isComplex)
    {
        int ysize = m_pContentDObj[id]->getSize(m_pContentDObj[id]->getDims() - 2);
        int xsize = m_pContentDObj[id]->getSize(m_pContentDObj[id]->getDims() - 1);

        // Creates a new Matrix, must be deleted
        dMat = newMatFromComplex<_Tp>(((cv::Mat*)m_pContentDObj[id]->get_mdata()[m_pContentDObj[id]->seekMat(0)]));
    }
    else
    {
        dMat = ((cv::Mat*)m_pContentDObj[id]->get_mdata()[m_pContentDObj[id]->seekMat(0)]);
    }

    if(fillInvalids)
    {
        ito::RetVal temp = ito::dObjHelper::verify2DDataObject(m_invalidMap[id].data(), "invalidMap", dMat->rows, dMat->rows, dMat->cols, dMat->cols, 2, ito::tInt8, ito::tUInt8);
        if(temp.containsError())
        {
            m_errorDisplMsg.append(temp.errorMessage());
            fillInvalids = false;
        }
        else
        {
            invalidMap = (cv::Mat*)(m_invalidMap[id]->get_mdata()[m_invalidMap[id]->seekMat(0)]);
        }
    }

    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif
    ito::float64 dpixel1;
    ito::float64 dpixel2;
    ito::float64 dpixel3;
    ito::float64 dpixel4;
    int count = 0;

    ito::uint8 invPixel1 = 0;
    ito::uint8 invPixel2 = 0;
    ito::uint8 invPixel3 = 0;
    ito::uint8 invPixel4 = 0;

    ito::uint8* ptrInv = NULL;
    ito::uint8* ptrInvNext = NULL;

    GLfloat Vec1[3];
    GLfloat Vec2[3];

    _TpMat *ptrScaledTopo = NULL;
    _TpMat *ptrScaledTopoNext = NULL;

    if(!retVal.containsError())
    {
        if(m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            float zmin = m_axes.m_axisZ.getMin();
            float zmax = m_axes.m_axisZ.getMax();
            float znorm = 1.0f;
            if(ito::isNotZero<float>(zmax - zmin))
            {
                znorm = 255.0f / (zmax - zmin);
            }

            ito::float64 PtScaleX = (m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin()) / xsizeObj;
            ito::float64 PtScaleY = (m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin()) / ysizeObj;
            ito::float64 PtTransX = m_axes.m_axisX.getMin();
            ito::float64 PtTransY = m_axes.m_axisY.getMin();

            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < ysizeObj -1; cntY++)
            {
                ptrScaledTopo = dMat->ptr<_TpMat>(cntY);

                if(fillInvalids)
                {
                    ptrInv = invalidMap->ptr<ito::uint8>(cntY);
                }

                for (int cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    if(fillInvalids)
                    {
                        invPixel1 = ptrInv[cntX];
                    }
                    dpixel1 = ptrScaledTopo[cntX];
                    if (ito::isFinite<ito::float64>(dpixel1))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }
                        vertBuf[count * ptSize] = (GLfloat)(cntX * PtScaleX + PtTransX);
                        vertBuf[count * ptSize + 1] = (GLfloat)(cntY * PtScaleY + PtTransY);
                        vertBuf[count * ptSize + 2] = dpixel1;

                        if (fillInvalids && invPixel1)
                        {
                            vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((dpixel1 - zmin) * znorm);
                        }
                        m_pUseTextPix[id][count * 2] = cntY;
                        m_pUseTextPix[id][count * 2 + 1] = cntX;
                    }
                }
            }
        }
        else
        {
            float zmin = m_axes.m_axisZ.getMin();
            float zmax = m_axes.m_axisZ.getMax();
            float znorm = 1.0f;
            if(ito::isNotZero<float>(zmax - zmin))
            {
                znorm = 255.0f / (zmax - zmin);
            }
            ito::float64 PtScaleX = (m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin()) / xsizeObj;
            ito::float64 PtScaleY = (m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin()) / ysizeObj;
            ito::float64 PtTransX = m_axes.m_axisX.getMin();
            ito::float64 PtTransY = m_axes.m_axisY.getMin();

            Vec1[0] = 0;
            Vec1[1] = m_scaleY * PtScaleY;
            Vec2[0] = m_scaleX * PtScaleX;
            Vec2[1] = 0;

//            ptSize = useNormals ? 21 : 12;
            int pt2Inc = useNormals ? 7 : 4;
            int pt3Inc = useNormals ? 14 : 8;

            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int cntY = 0; cntY < ysizeObj - 1; cntY++)
            {
                ptrScaledTopo = dMat->ptr<_TpMat>(cntY);
                ptrScaledTopoNext = dMat->ptr<_TpMat>(cntY + 1);

                if(fillInvalids)
                {
                    ptrInv = invalidMap->ptr<ito::uint8>(cntY);
                    ptrInvNext = invalidMap->ptr<ito::uint8>(cntY + 1);
                }
                for (int cntX = 0; cntX < xsizeObj - 1; cntX++)
                {
                    if(fillInvalids)
                    {
                        invPixel1 = ptrInv[cntX];
                        invPixel2 = ptrInvNext[cntX];
                        invPixel3 = ptrInv[cntX + 1];
                        invPixel4 = ptrInvNext[cntX + 1];
                    }
                    dpixel1 = ptrScaledTopo[cntX];
                    dpixel2 = ptrScaledTopoNext[cntX];
                    dpixel3 = ptrScaledTopo[cntX + 1];
                    dpixel4 = ptrScaledTopoNext[cntX + 1];

                    if (ito::isFinite(dpixel1) && ito::isFinite(dpixel2) && ito::isFinite(dpixel3))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }

                        ito::float64 xval  = cntX * PtScaleX + PtTransX;
                        ito::float64 xval1 = (cntX + 1) * PtScaleX + PtTransX;
                        ito::float64 yval  = cntY * PtScaleY + PtTransY;
                        ito::float64 yval1 = (cntY + 1) * PtScaleY + PtTransY;

                        // Vertex & Color
                        vertBuf[count * ptSize] = xval;
                        vertBuf[count * ptSize + 1] = yval;
                        vertBuf[count * ptSize + 2] = dpixel1;


                        vertBuf[count * ptSize + pt2Inc] = xval;
                        vertBuf[count * ptSize + pt2Inc + 1] = yval1;
                        vertBuf[count * ptSize + pt2Inc + 2] = dpixel2;

                        vertBuf[count * ptSize + pt3Inc] = xval1;
                        vertBuf[count * ptSize + pt3Inc + 1] = yval;
                        vertBuf[count * ptSize + pt3Inc + 2] = dpixel3;

                        if(fillInvalids && (invPixel1 || invPixel2 || invPixel3))
                        {
                            vertBuf[count * ptSize + 3] = -1.0;
                            vertBuf[count * ptSize + pt2Inc + 3] = -1.0;
                            vertBuf[count * ptSize + pt3Inc + 3] = -1.0;
                        }
                        else
                        {
                            vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((dpixel1 - zmin) * znorm);
                            vertBuf[count * ptSize + pt2Inc + 3] = cv::saturate_cast<unsigned char>((dpixel2 - zmin) * znorm);
                            vertBuf[count * ptSize + pt3Inc + 3] = cv::saturate_cast<unsigned char>((dpixel3 - zmin) * znorm);
                        }

                        m_pUseTextPix[id][count * 6] = cntY;
                        m_pUseTextPix[id][count * 6 + 1] = cntX;
                        m_pUseTextPix[id][count * 6 + 2] = (cntY + 1);
                        m_pUseTextPix[id][count * 6 + 3] = cntX;
                        m_pUseTextPix[id][count * 6 + 4] = cntY;
                        m_pUseTextPix[id][count * 6 + 5] = cntX + 1;

                        // Normals
                        // normal calculation needs review
                        if (useNormals)
                        {
                            Vec1[2] = dpixel3 - dpixel1;
                            Vec2[2] = dpixel2 - dpixel1;
                            float norm1 = vertBuf[count * ptSize + 4] = m_scaleY * Vec1[2];
                            float norm2 = vertBuf[count * ptSize + 5] = m_scaleX * Vec2[2];
                            float norm3 = vertBuf[count * ptSize + 6] = m_scaleY * m_scaleX;
                            float norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + 4] /= norm;
                            vertBuf[count * ptSize + 5] /= norm;
                            vertBuf[count * ptSize + 6] /= -norm;

                            Vec1[2] = dpixel2 - dpixel3;
                            Vec2[2] = dpixel2 - dpixel1;
                            norm1 = vertBuf[count * ptSize + pt2Inc + 4] = m_scaleY * Vec1[2] - m_scaleY * Vec2[2];
                            norm2 = vertBuf[count * ptSize + pt2Inc + 5] = -m_scaleX * Vec2[2];
                            norm3 = vertBuf[count * ptSize + pt2Inc + 6] = m_scaleY * m_scaleX;
                            norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + pt2Inc + 4] /= -norm;
                            vertBuf[count * ptSize + pt2Inc + 5] /= -norm;
                            vertBuf[count * ptSize + pt2Inc + 6] /= -norm;

                            Vec1[2] = dpixel3 - dpixel1;
                            Vec2[2] = dpixel3 - dpixel2;
                            norm1 = vertBuf[count * ptSize + pt3Inc + 4] = m_scaleY * Vec1[2];
                            norm2 = vertBuf[count * ptSize + pt3Inc + 5] = -m_scaleX * Vec2[2] + m_scaleX * Vec1[2];
                            norm3 = vertBuf[count * ptSize + pt3Inc + 6] = -m_scaleY * m_scaleX;
                            norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + pt3Inc + 4] /= norm;
                            vertBuf[count * ptSize + pt3Inc + 5] /= norm;
                            vertBuf[count * ptSize + pt3Inc + 6] /= norm;
                        }
                    }

                    if (ito::isFinite(dpixel2) && ito::isFinite(dpixel3) && ito::isFinite(dpixel4))
                    {
                        #if (USEOMP)
                        #pragma omp critical
                        #endif
                        {
                            count = m_numVert[id]++;
                        }

                        ito::float64 xval  = cntX * PtScaleX + PtTransX;
                        ito::float64 xval1 = (cntX + 1) * PtScaleX + PtTransX;
                        ito::float64 yval  = cntY * PtScaleY + PtTransY;
                        ito::float64 yval1 = (cntY + 1) * PtScaleY + PtTransY;

                        // Vertex & Color
                        vertBuf[count * ptSize] = xval;
                        vertBuf[count * ptSize + 1] = yval1;
                        vertBuf[count * ptSize + 2] = dpixel2;


                        vertBuf[count * ptSize + pt2Inc] = xval1;
                        vertBuf[count * ptSize + pt2Inc + 1] = yval;
                        vertBuf[count * ptSize + pt2Inc + 2] = dpixel3;

                        vertBuf[count * ptSize + pt3Inc] = xval1;
                        vertBuf[count * ptSize + pt3Inc + 1] = yval1;
                        vertBuf[count * ptSize + pt3Inc + 2] = dpixel4;


                        if(fillInvalids && (invPixel2 || invPixel3 || invPixel4))
                        {
                            vertBuf[count * ptSize + 3] = -1.0;
                            vertBuf[count * ptSize + pt2Inc + 3] = -1.0;
                            vertBuf[count * ptSize + pt3Inc + 3] = -1.0;
                        }
                        else
                        {
                            vertBuf[count * ptSize + 3] = cv::saturate_cast<unsigned char>((dpixel2 - zmin) * znorm);
                            vertBuf[count * ptSize + pt2Inc + 3] = cv::saturate_cast<unsigned char>((dpixel3 - zmin) * znorm);
                            vertBuf[count * ptSize + pt3Inc + 3] = cv::saturate_cast<unsigned char>((dpixel4 - zmin) * znorm);
                        }

                        m_pUseTextPix[id][count * 6] = (cntY + 1);
                        m_pUseTextPix[id][count * 6 + 1] = cntX;
                        m_pUseTextPix[id][count * 6 + 2] = cntY;
                        m_pUseTextPix[id][count * 6 + 3] = cntX + 1;
                        m_pUseTextPix[id][count * 6 + 4] = (cntY + 1);
                        m_pUseTextPix[id][count * 6 + 5] = cntX + 1;

                        // Normals
                        // normal calculation needs review
                        if (useNormals)
                        {
                            Vec1[2] = dpixel4 - dpixel2;
                            Vec2[2] = dpixel3 - dpixel2;
                            float norm1 = vertBuf[count * ptSize + 4] = -m_scaleY * Vec1[2];
                            float norm2 = vertBuf[count * ptSize + 5] = -m_scaleX * Vec1[2] + m_scaleX * Vec2[2];
                            float norm3 = vertBuf[count * ptSize + 6] = m_scaleY * m_scaleX;
                            float norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + 4] /= -norm;
                            vertBuf[count * ptSize + 5] /= -norm;
                            vertBuf[count * ptSize + 6] /= -norm;

                            Vec1[2] = dpixel4 - dpixel3;
                            Vec2[2] = dpixel3 - dpixel2;
                            norm1 = vertBuf[count * ptSize + pt2Inc + 4] = -m_scaleY * Vec1[2] - m_scaleY * Vec2[2];
                            norm2 = vertBuf[count * ptSize + pt2Inc + 5] = -m_scaleX * Vec1[2];
                            norm3 = vertBuf[count * ptSize + pt2Inc + 6] = m_scaleY * m_scaleX;
                            norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + pt2Inc + 4] /= -norm;
                            vertBuf[count * ptSize + pt2Inc + 5] /= -norm;
                            vertBuf[count * ptSize + pt2Inc + 6] /= -norm;

                            Vec1[2] = dpixel4 - dpixel2;
                            Vec2[2] = dpixel4 - dpixel3;
                            norm1 = vertBuf[count * ptSize + pt3Inc + 4] = -m_scaleY * Vec1[2];
                            norm2 = vertBuf[count * ptSize + pt3Inc + 5] = -m_scaleX * Vec2[2];
                            norm3 = vertBuf[count * ptSize + pt3Inc + 6] = m_scaleY * m_scaleX;
                            norm = sqrt(norm1 * norm1 + norm2 * norm2 + norm3 * norm3);
                            vertBuf[count * ptSize + pt3Inc + 4] /= -norm;
                            vertBuf[count * ptSize + pt3Inc + 5] /= -norm;
                            vertBuf[count * ptSize + pt3Inc + 6] /= -norm;
                        }
                    }
                }
            }
        }
    }
    #if (USEOMP)
    }
    #endif

    int err;
    if (m_numVert[id] != 0 || !retVal.containsError())
    {
        makeCurrent();
        if(m_pConfigData->m_elementMode & PAINT_POINTS)
        {
            GLFPTR(glUseProgram)(m_prog3D);
            if (!m_vertBuf3D.contains(id))
            {
#if QT_VERSION < 0x050000
                GLuint tmpArr;
                glGenVertexArrays(1, &tmpArr);
#else
                QOpenGLVertexArrayObject *tmpArr;
                tmpArr = new QOpenGLVertexArrayObject(this);
                tmpArr->create();
#endif
                m_VAO3D.insert(id, tmpArr);
                m_transperency.insert(id, 255);
                m_enabledHash.insert(id, true);

                GLuint tmpVal;
                GLFPTR(glGenBuffers)(1, &tmpVal);
                m_vertBuf3D.insert(id, tmpVal);
            }
#if QT_VERSION < 0x050000
            glBindVertexArray(m_VAO3D.value(id));
#else
            m_VAO3D.value(id)->bind();
#endif
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_vertBuf3D.value(id));
            GLFPTR(glEnableVertexAttribArray)(m_attribVert);
            GLFPTR(glEnableVertexAttribArray)(m_attribVertColor);
            GLFPTR(glEnableVertexAttribArray)(m_attribNorm);
            GLFPTR(glEnableVertexAttribArray)(m_attribDiff);

            GLFPTR(glDisableVertexAttribArray)(m_attribTexCol);
            GLFPTR(glVertexAttribPointer)(m_attribDiff, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
            GLFPTR(glVertexAttribPointer)(m_attribVert, 3, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), 0);
            GLFPTR(glVertexAttribPointer)(m_attribVertColor, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
            GLFPTR(glVertexAttribPointer)(m_attribNorm, 3, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), 0);

            GLFPTR(glBufferData)(GL_ARRAY_BUFFER, m_numVert[id] * ptSize * sizeof(GLfloat), vertBuf, GL_STATIC_DRAW);
            if (err = glGetError())
                retVal += ito::RetVal(ito::retError, 0, "Error uploading triangles");
#if QT_VERSION < 0x050000
            GLFPTR(glBindVertexArray)(0);
#else
            m_VAO3D.value(id)->release();
#endif
            GLFPTR(glUseProgram)(0);
            err = glGetError();
        }
        else
        {
            GLFPTR(glUseProgram)(m_prog3D);
            if (!m_vertBuf3D.contains(id))
            {
#if QT_VERSION < 0x050000
                GLuint tmpArr;
                glGenVertexArrays(1, &tmpArr);
#else
                QOpenGLVertexArrayObject *tmpArr;
                tmpArr = new QOpenGLVertexArrayObject();
                tmpArr->create();
#endif
                m_VAO3D.insert(id, tmpArr);
                m_transperency.insert(id, 255);
                m_enabledHash.insert(id, true);

                GLuint tmpVal;
                GLFPTR(glGenBuffers)(1, &tmpVal);
                m_vertBuf3D.insert(id, tmpVal);
            }
#if QT_VERSION < 0x050000
            glBindVertexArray(m_VAO3D.value(id));
#else
            m_VAO3D.value(id)->bind();
#endif
            //!> Here we activate the various buffers used in the shader, upload the buffer(s) and set their properties.
            //!> Even if we don't use some of the features, e.g. normals we must set the attributePointer to a
            //!> meaningful value, other we get problems on some graphics boards ... even if the attribute array
            //!> is disabled ...
            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_vertBuf3D.value(id));
            GLFPTR(glEnableVertexAttribArray)(m_attribVert);
            GLFPTR(glEnableVertexAttribArray)(m_attribVertColor);
            GLFPTR(glEnableVertexAttribArray)(m_attribNorm);
            GLFPTR(glEnableVertexAttribArray)(m_attribDiff);

            GLFPTR(glDisableVertexAttribArray)(m_attribTexCol);

            GLFPTR(glVertexAttribPointer)(m_attribDiff, 1, GL_FLOAT, GL_FALSE, ptSize * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));

            GLFPTR(glVertexAttribPointer)(m_attribVert, 3, GL_FLOAT, GL_FALSE, (ptSize / 3) * sizeof(GLfloat), 0);
            GLFPTR(glVertexAttribPointer)(m_attribVertColor, 1, GL_FLOAT, GL_FALSE, (ptSize / 3) * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
            if (useNormals)
                GLFPTR(glVertexAttribPointer)(m_attribNorm, 3, GL_FLOAT, GL_FALSE, (ptSize / 3) * sizeof(GLfloat), (void*)(4 * sizeof(GLfloat)));
            else
                GLFPTR(glVertexAttribPointer)(m_attribNorm, 3, GL_FLOAT, GL_FALSE, (ptSize / 3) * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
            GLFPTR(glBufferData)(GL_ARRAY_BUFFER, m_numVert[id] * ptSize * sizeof(GLfloat), vertBuf, GL_STATIC_DRAW);
            if (err = glGetError())
                retVal += ito::RetVal(ito::retError, 0, "Error uploading triangles");
#if QT_VERSION < 0x050000
            glBindVertexArray(0);
#else
            m_VAO3D.value(id)->release();
#endif
            GLFPTR(glUseProgram)(0);
            err = glGetError();
        }

        ResetColors();
        doneCurrent();

        m_errorDisplMsg.clear();
        m_isInit &= ~IS_CALCTRIANG;
    }

//CLEAREXIT:
    if (m_numVert[id] == 0 || retVal.containsError())
    {
        m_isInit &= ~HAS_TRIANG;
        m_numVert[id] = 0;
        retVal += ito::RetVal(ito::retError, 0, "Error calculating points / triangles");
        m_isInit &= ~IS_CALCTRIANG;
    }

    if(isComplex)
    {
        delete dMat;
        dMat = NULL;
    }

    if (vertBuf)
        free(vertBuf);

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** method for updating plot data
*   @param [in] param      input parameter holding point cloud or data object
*   @param [in] id         internal id for storing / referencing the pointcloud
*
*   This method is used to update the plotted data. The data is encapsulated in a ito::ParamBase
*   and either dataObjects or pointClouds can be plotted. The method can be called without
*   passing a param as well. In this case only an update of the plot using the
*   previously passed object is done. Otherwise the currently displayed object
*   is replaced with the new one and an update is triggered.
*/
void TwipOGLWidget::refreshPlot(ito::ParamBase *param, const int id)
{
    ito::RetVal retval = ito::retOk;
    ito::float64 xs = 1.0, ys = 1.0, zs = 1.0, maxl = 1.0;
    int dims = 0;

    //!> new input object passed, so check it and if it  is usable substitute actual one
    if (param != NULL)
    {
        //!> disable some stuff at first
        m_pConfigData->m_elementMode &= ~ENABLE_ILLUMINATION;
        m_pConfigData->m_elementMode &= ~HAS_CURVATURE;

        //!> at first we always check for dataObject or pointCloud
        if (param->getType() == (ito::Param::DObjPtr & ito::paramTypeMask))
        {
            ito::DataObject *dataObj = (ito::DataObject*)param->getVal<char*>();
            dims = dataObj->getDims();
            if( dims > 1)
            {
                m_pContentDObj.insert(id, QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)));
                m_forceReplot = true;
                m_pConfigData->m_elementMode |= PAINT_TRIANG;
                m_pConfigData->m_elementMode &= ~PAINT_POINTS;
            }
        }
        else if (param->getType() == (ito::Param::PointCloudPtr & ito::paramTypeMask))
        {
            #ifdef USEPCL
                //check pointCloud
                ito::PCLPointCloud *pc = (ito::PCLPointCloud*)param->getVal<char*>();
                if (m_pContentPC.size() == 0)
                    m_pContentPC.insert(id, QSharedPointer<ito::PCLPointCloud>(new ito::PCLPointCloud(*pc)));
                else
                    m_pContentPC[id] = QSharedPointer<ito::PCLPointCloud>(new ito::PCLPointCloud(*pc));

                m_forceReplot = true;

                //!> disable triangle plot for pointcloud
                m_pConfigData->m_elementMode &= ~PAINT_TRIANG;
            #endif
        }
        return;
    }

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

    if (param != NULL || m_forceReplot)
    {
        //check dataObj
        if( m_pContentDObj[id] != NULL)
        {
            dims = m_pContentDObj[id]->getDims();

            //!> dataObject is principally ok. Now we have to get the details, i.e.
            //!> the physical dimensions, min max value, data type, axis names and so on
            if (id > 0)
            {
                if (m_pContentDObj[id]->getSize(dims - 1) != m_pContentDObj[0]->getSize(dims - 1)
                    || m_pContentDObj[id]->getSize(dims - 2) != m_pContentDObj[0]->getSize(dims - 2)
                    || m_pContentDObj[id]->getAxisScale(dims - 1) != m_pContentDObj[0]->getAxisScale(dims - 1)
                    || m_pContentDObj[id]->getAxisScale(dims - 2) != m_pContentDObj[0]->getAxisScale(dims - 2))
                    std::cerr << QObject::tr("Adding DataObject with different size or scaling is currently not supported\n").toLatin1().data();
            }

            bool check = false;
            ito::DataObjectTagType titleTag = m_pContentDObj[id]->getTag("title", check);
            if (check && m_pConfigData->m_autoTitle && id == 0)
            {
                m_pConfigData->m_drawTitle = true;
                m_pConfigData->m_title = QString::fromLatin1(titleTag.getVal_ToString().data());
            }
            else
            {
                if (!m_pConfigData->m_autoTitle)
                    m_pConfigData->m_drawTitle = false;
            }

            if (m_pContentDObj[id]->getType() == ito::tComplex128 || m_pContentDObj[id]->getType() == ito::tComplex64)
            {
                if(!m_pConfigData->m_cmplxState) ((TwipOGLFigure*)m_pParent)->enableComplexGUI(true);
                m_pConfigData->m_cmplxState = true;
            }
            else
            {
                if(!m_pConfigData->m_cmplxState) ((TwipOGLFigure*)m_pParent)->enableComplexGUI(false);
                m_pConfigData->m_cmplxState = false;
            }

            int x1 = m_pContentDObj[id]->getSize(dims - 1) - 1;
            int y1 = m_pContentDObj[id]->getSize(dims - 2) - 1;
            bool test;

            if (id == 0)
            {
                m_axes.m_axisX.setScaleAU(m_pContentDObj[id]->getAxisScale(dims - 1));
                m_axes.m_axisY.setScaleAU(m_pContentDObj[id]->getAxisScale(dims - 2));
            }

            if((m_axes.m_axisX.getAutoscale()) || ((int)m_axes.m_axisX.getIdx1() > x1) || ((int)m_axes.m_axisX.getIdxDim() != dims - 1))
            {
                m_axes.m_axisX.setIdx0(0);
                m_axes.m_axisX.setIdx1(x1);
                m_axes.m_axisX.setIdxDim(dims - 1);
            }

            if((m_axes.m_axisY.getAutoscale()) || ((int)m_axes.m_axisY.getIdx1() > y1) || ((int)m_axes.m_axisY.getIdxDim() != dims - 2))
            {
                m_axes.m_axisY.setIdx0(0);
                m_axes.m_axisY.setIdx1(y1);
                m_axes.m_axisY.setIdxDim(dims - 2);
            }

            m_axes.m_axisX.setMin(m_pContentDObj[id]->getPixToPhys(m_axes.m_axisX.getIdxDim(), (double)m_axes.m_axisX.getIdx0(), test));
            m_axes.m_axisX.setMax(m_pContentDObj[id]->getPixToPhys(m_axes.m_axisX.getIdxDim(), (double)m_axes.m_axisX.getIdx1(), test));

            m_axes.m_axisY.setMin(m_pContentDObj[id]->getPixToPhys(m_axes.m_axisY.getIdxDim(), (double)m_axes.m_axisY.getIdx0(), test));
            m_axes.m_axisY.setMax(m_pContentDObj[id]->getPixToPhys(m_axes.m_axisY.getIdxDim(), (double)m_axes.m_axisY.getIdx1(), test));

            //m_title = internalObj.getTag("title", test).getVal_ToString();
            if(m_axes.m_axisX.getMax() < m_axes.m_axisX.getMin())
            {
                double tempVal = m_axes.m_axisX.getMin();
                m_axes.m_axisX.setMin(m_axes.m_axisX.getMax());
                m_axes.m_axisX.setMax(tempVal);
            }
            if(m_axes.m_axisY.getMax() < m_axes.m_axisY.getMin())
            {
                double tempVal = m_axes.m_axisY.getMin();
                m_axes.m_axisY.setMin(m_axes.m_axisY.getMax());
                m_axes.m_axisY.setMax(tempVal);
            }

            if (id == 0)
            {
                m_axes.m_axisX.m_label = m_pContentDObj[id]->getAxisDescription(dims - 1, test);
                m_axes.m_axisX.m_unit = m_pContentDObj[id]->getAxisUnit(dims - 1, test);
                m_axes.m_axisY.m_label = m_pContentDObj[id]->getAxisDescription(dims - 2, test);
                m_axes.m_axisY.m_unit = m_pContentDObj[id]->getAxisUnit(dims - 2, test);
                m_axes.m_axisZ.m_label = m_pContentDObj[id]->getValueDescription();
                m_axes.m_axisZ.m_unit = m_pContentDObj[id]->getValueUnit();

                if (!m_axes.m_axisZ.m_unit.compare("mm") || !m_axes.m_axisZ.m_unit.compare("m"))
                    m_axes.m_axisZ.m_isMetric = true;
                else
                    m_pConfigData->m_keepVoxel = false;

                if (!m_axes.m_axisX.m_unit.compare("mm") || !m_axes.m_axisX.m_unit.compare("m"))
                    m_axes.m_axisX.m_isMetric = true;

                if (!m_axes.m_axisY.m_unit.compare("mm") || !m_axes.m_axisY.m_unit.compare("m"))
                    m_axes.m_axisY.m_isMetric = true;

                if (m_objectInfo.show)
                {
                    ito::dObjHelper::devValue(m_pContentDObj[id].data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
                    generateObjectInfoText(id);
                }
            }

            ((TwipOGLFigure*)parent())->updateLegend(id, TwipLegend::tDataObject, m_pContentDObj[id]->getType(), 255, true);

            m_forceReplot = true;
        }
#ifdef USEPCL
        //check pointCloud
        else if( m_pContentPC[id] != NULL)
        {
/*
            bool check = false;
            ito::DataObjectTagType titleTag =  m_pContentPC[id]->getTag("title", check);
            if (check && m_pConfigData->m_autoTitle)
            {
                m_pConfigData->m_drawTitle = true;
                m_pConfigData->m_title.fromLatin1(titleTag.getVal_ToString().data());
            }
            else
            {
                if (!m_pConfigData->m_autoTitle)
                    m_pConfigData->m_drawTitle = false;
            }
*/
            //!> pointCloud is principally ok. Now we have to get the details, i.e.
            //!> pointCloud type the physical dimensions, min max value, axis names and so on

            m_pConfigData->m_elementMode |= PAINT_POINTS;
            m_pConfigData->m_elementMode &= ~PAINT_TRIANG;
            m_pConfigData->m_zAmpl = 1.0;

            switch (m_pContentPC[id]->getType())
            {
                case ito::pclXYZ:
                {
                    pcl::PointCloud<pcl::PointXYZ> *pcl = m_pContentPC[id]->toPointXYZ().get();
                    pclFindMinMax<pcl::PointXYZ>(pcl, id);
                }
                break;

                case ito::pclXYZI:
                {
                    pcl::PointCloud<pcl::PointXYZI> *pcl = m_pContentPC[id]->toPointXYZI().get();
                    pclFindMinMax<pcl::PointXYZI>(pcl, id);
                }
                break;

                case ito::pclXYZRGBA:
                {
                    pcl::PointCloud<pcl::PointXYZRGBA> *pcl = m_pContentPC[id]->toPointXYZRGBA().get();
                    pclFindMinMax<pcl::PointXYZRGBA>(pcl, id);
                }
                break;

                case ito::pclXYZNormal:
                {
                    pcl::PointCloud<pcl::PointNormal> *pcl = m_pContentPC[id]->toPointXYZNormal().get();
                    pclFindMinMax<pcl::PointNormal>(pcl, id);
                }
                break;

                case ito::pclXYZINormal:
                {
                    pcl::PointCloud<pcl::PointXYZINormal> *pcl = m_pContentPC[id]->toPointXYZINormal().get();
                    pclFindMinMax<pcl::PointXYZINormal>(pcl, id);
                }
                break;

                case ito::pclXYZRGBNormal:
                {
                    pcl::PointCloud<pcl::PointXYZRGBNormal> *pcl = m_pContentPC[id]->toPointXYZRGBNormal().get();
                    pclFindMinMax<pcl::PointXYZRGBNormal>(pcl, id);
                }
                break;

                default:
                    retval += ito::retError;
                    m_errorDisplMsg.append("Unknown PCL type");
                break;
            }
            ito::float64 minx = std::numeric_limits<ito::float64>::max();
            ito::float64 miny = std::numeric_limits<ito::float64>::max();
            ito::float64 minz = std::numeric_limits<ito::float64>::max();
            ito::float64 minDev = std::numeric_limits<ito::float64>::max();
            ito::float64 maxx = -std::numeric_limits<ito::float64>::max();
            ito::float64 maxy = -std::numeric_limits<ito::float64>::max();
            ito::float64 maxz = -std::numeric_limits<ito::float64>::max();
            ito::float64 maxDev = -std::numeric_limits<ito::float64>::max();
            for (QHash<int, QSharedPointer<ito::PCLPointCloud> >::ConstIterator it = m_pContentPC.begin(); it != m_pContentPC.end(); it++)
            {
                if (m_pclMinX.value(it.key()) < minx)
                    minx = m_pclMinX.value(it.key());
                if (m_pclMinY.value(it.key()) < miny)
                    miny = m_pclMinY.value(it.key());
                if (m_pclMinZ.value(it.key()) < minz)
                    minz = m_pclMinZ.value(it.key());
                if (m_pclMinDev.value(it.key()) < minDev)
                    minDev = m_pclMinDev.value(it.key());
                if (m_pclMaxX.value(it.key()) > maxx)
                    maxx = m_pclMaxX.value(it.key());
                if (m_pclMaxY.value(it.key()) > maxy)
                    maxy = m_pclMaxY.value(it.key());
                if (m_pclMaxZ.value(it.key()) > maxz)
                    maxz = m_pclMaxZ.value(it.key());
                if (m_pclMaxDev.value(it.key()) > maxDev)
                    maxDev = m_pclMaxDev.value(it.key());
            }
            if (m_axes.m_axisX.getMin() == 0 || m_axes.m_axisX.getMin() > minx)
                m_axes.m_axisX.setMin(minx);
            if (m_axes.m_axisX.getMax() == 0 || m_axes.m_axisX.getMax() < maxx)
                m_axes.m_axisX.setMax(maxx);
            if (m_axes.m_axisY.getMin() == 0 || m_axes.m_axisY.getMin() > maxy)
                m_axes.m_axisY.setMin(miny);
            if (m_axes.m_axisY.getMax() == 0 || m_axes.m_axisY.getMax() < maxy)
                m_axes.m_axisY.setMax(maxy);
            if (m_axes.m_axisZ.getMin() == 0 || m_axes.m_axisZ.getMin() > minz)
                m_axes.m_axisZ.setMin(minz);
            if (m_axes.m_axisZ.getMax() == 0 || m_axes.m_axisZ.getMax() < maxz)
                m_axes.m_axisZ.setMax(maxz);

            if(m_pConfigData->m_curvatureInterval.isAuto())
            {
                m_axes.m_devAxis.setMin(minDev);
                m_axes.m_devAxis.setMax(maxDev);
                m_pConfigData->m_curvatureInterval.setMinimum(minDev);
                m_pConfigData->m_curvatureInterval.setMaximum(maxDev);
            }
            else
            {
                m_axes.m_devAxis.setMin(m_pConfigData->m_curvatureInterval.minimum());
                m_axes.m_devAxis.setMax(m_pConfigData->m_curvatureInterval.maximum());
            }

            m_axes.m_axisX.m_label = "x";
            m_axes.m_axisX.m_unit  = "mm";
            m_axes.m_axisY.m_label = "y";
            m_axes.m_axisY.m_unit  = "mm";
            m_axes.m_axisZ.m_label = "z";
            m_axes.m_axisZ.m_unit  = "mm";

            m_axes.m_devAxis.m_label = "deviation";
            m_axes.m_devAxis.m_unit  = "mm";

            if (m_pConfigData->m_zAmpl < 0.000001f) // make sure µm can be displayed
                m_pConfigData->m_zAmpl = 0.000001f; // make sure µm can be displayed
            ((TwipOGLFigure*)parent())->updateLegend(id, TwipLegend::tPointCloud, m_pContentPC[id]->getType(), 255, true);
        }

#else
        else
            retval += ito::RetVal(ito::retError, 0, tr("compiled without pointCloud support").toLatin1().data());
#endif // #ifdef USEPCL
    }

    //!> ok now we've got all we need to really push the data to the vertex array(s)
    //!> the main work do the both functions GLSetTriangles<>() and GLSetPointsPCL<>()
    if (m_pContentDObj[id] != NULL)
    {
        ito::uint32 firstMin[3];
        ito::uint32 firstMax[3];
        ito::float64 tmpMin;
        ito::float64 tmpMax;

        if(m_axes.m_axisZ.getAutoscale())
        {
            switch(m_pContentDObj[id]->getType())
            {
                case ito::tUInt8:
                case ito::tInt8:
                case ito::tUInt16:
                case ito::tInt16:
                case ito::tUInt32:
                case ito::tInt32:
                case ito::tFloat32:
                case ito::tFloat64:
                {
                    ito::dObjHelper::minMaxValue(m_pContentDObj[id].data(), tmpMin, firstMin, tmpMax, firstMax, true);
                    if (m_axes.m_axisZ.getMin() == 0 || tmpMin < m_axes.m_axisZ.getMin())
                        m_axes.m_axisZ.setMin(tmpMin);
                    if (m_axes.m_axisZ.getMax() == 0 || tmpMax > m_axes.m_axisZ.getMax())
                        m_axes.m_axisZ.setMax(tmpMax);
                }
                break;
                case ito::tComplex64:
                case ito::tComplex128:
                {
                    ito::dObjHelper::minMaxValue(m_pContentDObj[id].data(), tmpMin, firstMin, tmpMax, firstMax, true, m_pConfigData->m_cmplxMode);
                    if (m_axes.m_axisZ.getMin() == 0 || tmpMin < m_axes.m_axisZ.getMin())
                        m_axes.m_axisZ.setMin(tmpMin);
                    if (m_axes.m_axisZ.getMax() == 0 || tmpMax > m_axes.m_axisZ.getMax())
                        m_axes.m_axisZ.setMax(tmpMax);
                }
                break;
                default:
                    retval += ito::retError; //ito::RetVal(ito::retError, 0, tr("Object has invalid type").toLatin1().data());
                    m_errorDisplMsg.append("Object has invalid type");
            }
        }

        if(!retval.containsError())
        {
            if (m_pConfigData->m_zAmpl < 0.000001f) // make sure µm can be displayed
                m_pConfigData->m_zAmpl = 0.000001f; // make sure µm can be displayed

            m_scaleX = 2.0 / (m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin());
            if (cvIsNaN(m_scaleX) || cvIsInf(m_scaleX))
                m_scaleX = 1.0;

            m_scaleY = 2.0 / (m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin());
            if (cvIsNaN(m_scaleY) || cvIsInf(m_scaleY))
                m_scaleY = 1.0;

            m_scaleZ = 2.0 / (m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin());
            if (cvIsNaN(m_scaleZ) || cvIsInf(m_scaleZ))
                m_scaleZ = 1.0;

            m_transX = -(m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin()) / 2.0 - m_axes.m_axisX.getMin();
            if (cvIsNaN(m_transX) || cvIsInf(m_transX))
                m_transX = 0.0;
            m_transY = -(m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin()) / 2.0 - m_axes.m_axisY.getMin();
            if (cvIsNaN(m_transY) || cvIsInf(m_transY))
                m_transY = 0.0;
            m_transZ = -m_axes.m_axisZ.getMin();
            if (cvIsNaN(m_transZ) || cvIsInf(m_transZ))
                m_transZ = 0.0;
/*
            if (m_axes.m_axisY.m_isMetric && m_axes.m_axisX.m_isMetric)
            {
                if (m_axes.m_axisZ.m_isMetric) zs = m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin();
                if (m_axes.m_axisX.m_isMetric) xs = m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin();
                if (m_axes.m_axisY.m_isMetric) ys = m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin();

                // To get cubic voxel in case of metric data
                maxl = xs;
                if ((ys > maxl) && (ys != 1))
                    maxl = ys;
                if ((zs > maxl) && (zs != 1))
                    maxl = zs;

//                m_scaleX *= xs / maxl;
//                m_scaleY *= ys / maxl;
                if (zs != 1 && m_axes.m_axisZ.m_isMetric && m_pConfigData->m_forceCubicVoxel)
                {
                    m_scaleZ *= zs / maxl;
//                    m_transZ *= maxl / zs;
                }
            }
*/

            if( m_numVert[id] == 0 || m_forceReplot == true)
            {
                switch(m_pContentDObj[id]->getType())
                {
                    case ito::tUInt8:
                        retval += GLSetTriangles<ito::uint8, ito::uint8>(id);
                    break;
                    case ito::tInt8:
                        retval += GLSetTriangles<ito::int8, ito::int8>(id);
                    break;
                    case ito::tUInt16:
                        retval += GLSetTriangles<ito::uint16, ito::uint16>(id);
                    break;
                    case ito::tInt16:
                        retval += GLSetTriangles<ito::int16, ito::int16>(id);
                    break;
                    case ito::tUInt32:
                        retval += GLSetTriangles<ito::uint32, ito::uint32>(id);
                    break;
                    case ito::tInt32:
                        retval += GLSetTriangles<ito::int32, ito::int32>(id);
                    break;
                    case ito::tFloat32:
                        retval += GLSetTriangles<ito::float32, ito::float32>(id);
                    break;
                    case ito::tFloat64:
                        retval += GLSetTriangles<ito::float64, ito::float64>(id);
                    break;

                    case ito::tComplex64:
                        retval += GLSetTriangles<ito::complex64, ito::float32>(id);
                    break;
                    case ito::tComplex128:
                        retval += GLSetTriangles<ito::complex128, ito::float64>(id);
                    break;

                    default:
                        retval += ito::retError; //ito::RetVal(ito::retError, 0, tr("Object has invalid type").toLatin1().data());
                        m_errorDisplMsg.append("Object has invalid type");
                }
            }
        }
    }
#ifdef USEPCL
    else if (m_pContentPC[id] != NULL)
    {
        if(!retval.containsError())
        {
            if (m_pConfigData->m_zAmpl < 0.000001f) // make sure µm can be displayed
                m_pConfigData->m_zAmpl = 0.000001f; // make sure µm can be displayed

            m_scaleX = 2.0 / (m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin());
            if (cvIsNaN(m_scaleX) || cvIsInf(m_scaleX))
                m_scaleX = 1.0;

            m_scaleY = 2.0 / (m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin());
            if (cvIsNaN(m_scaleY) || cvIsInf(m_scaleY))
                m_scaleY = 1.0;

            m_scaleZ = 2.0 / (m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin());
            if (cvIsNaN(m_scaleZ) || cvIsInf(m_scaleZ))
                m_scaleZ = 1.0;

            m_transX = -(m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin()) / 2.0 - m_axes.m_axisX.getMin();
            if (cvIsNaN(m_transX) || cvIsInf(m_transX))
                m_transX = 0.0;
            m_transY = -(m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin()) / 2.0 - m_axes.m_axisY.getMin();
            if (cvIsNaN(m_transY) || cvIsInf(m_transY))
                m_transY = 0.0;
            m_transZ = -m_axes.m_axisZ.getMin();
            if (cvIsNaN(m_transZ) || cvIsInf(m_transZ))
                m_transZ = 0.0;

/*
            if (m_axes.m_axisX.m_isMetric && m_axes.m_axisY.m_isMetric)
            {
                if(m_axes.m_axisZ.m_isMetric) zs = m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin();
                if(m_axes.m_axisX.m_isMetric) xs = m_axes.m_axisX.getMax() - m_axes.m_axisX.getMin();
                if(m_axes.m_axisY.m_isMetric) ys = m_axes.m_axisY.getMax() - m_axes.m_axisY.getMin();
                // To get cubic voxel in case of metric data
                maxl = xs;
                if ((ys > maxl) && (ys != 1))
                    maxl = ys;
                if ((zs > maxl) && (zs != 1))
                    maxl = zs;

//                m_scaleX *= xs / maxl;
//                m_scaleY *= ys / maxl;
                if (zs!=1 && m_pConfigData->m_forceCubicVoxel && m_axes.m_axisZ.m_isMetric)
                {
                    m_scaleZ *= zs / maxl;
//                    m_transZ *= maxl / zs;
                }
            }
*/

            m_forceReplot = true;
            m_pConfigData->m_elementMode |= PAINT_POINTS;
            m_pConfigData->m_elementMode &= ~PAINT_TRIANG;

            if( m_numVert[id] == 0 || m_forceReplot == true)
            {
                switch (m_pContentPC[id]->getType())
                {
                    case ito::pclXYZ:
                    {
                        pcl::PointCloud<pcl::PointXYZ> *pcl = m_pContentPC[id]->toPointXYZ().get();
                        retval += GLSetPointsPCL<pcl::PointXYZ>(pcl, id);
                    }
                    break;

                    case ito::pclXYZI:
                    {
                        pcl::PointCloud<pcl::PointXYZI> *pcl = m_pContentPC[id]->toPointXYZI().get();
                        retval += GLSetPointsPCL<pcl::PointXYZI>(pcl, id);
                    }
                    break;

                    case ito::pclXYZRGBA:
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA> *pcl = m_pContentPC[id]->toPointXYZRGBA().get();
                        retval += GLSetPointsPCL<pcl::PointXYZRGBA>(pcl, id);
                    }
                    break;

                    case ito::pclXYZNormal:
                    {
                        pcl::PointCloud<pcl::PointNormal> *pcl = m_pContentPC[id]->toPointXYZNormal().get();
                        retval += GLSetPointsPCL<pcl::PointNormal>(pcl, id);
                    }
                    break;

                    case ito::pclXYZINormal:
                    {
                        pcl::PointCloud<pcl::PointXYZINormal> *pcl = m_pContentPC[id]->toPointXYZINormal().get();
                        retval += GLSetPointsPCL<pcl::PointXYZINormal>(pcl, id);
                    }
                    break;

                    case ito::pclXYZRGBNormal:
                    {
                        pcl::PointCloud<pcl::PointXYZRGBNormal> *pcl = m_pContentPC[id]->toPointXYZRGBNormal().get();
                        retval += GLSetPointsPCL<pcl::PointXYZRGBNormal>(pcl, id);
                    }
                    break;

                    default:
                        retval += ito::retError;
                        m_errorDisplMsg.append("Unknown PCL type");
                }
            }
            else
            {
            }
        }
    }
#else
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("DataObject-Container empty and compiled without pointCloud support").toLatin1().data());
    }
#endif // #ifdef USEPCL


#ifdef USEPCL
    bool hasNoPC = true;
    bool hasNoDObj = true;
    bool hasNoPM = true;

    for (int npc = 0; npc < m_pContentPC.size(); npc++)
    {
        if (m_pContentPC[npc])
        {
            hasNoPC = false;
            break;
        }
    }

    for (int npm = 0; npm < m_pContentPM.size(); npm++)
    {
        if (m_pContentPM[npm])
        {
            hasNoPM = false;
            break;
        }
    }

    for (int ndo = 0; ndo < m_pContentDObj.size(); ndo++)
    {
        if (m_pContentDObj[ndo])
        {
            hasNoDObj = false;
            break;
        }
    }

    if (hasNoDObj && hasNoPC && hasNoPM)
    {
        qDebug() << "Object or pointClouds empty";
#else
    if (m_pContentDObj.isEmpty())
    {
        qDebug() << "Object empty";
#endif

        m_errorDisplMsg.clear();
        m_errorDisplMsg.append("Object empty");
        retval += ito::retError;
    }

    if (retval == ito::retOk)
    {
        m_isInit |= HAS_TRIANG;
        update();
    }
    else
    {
        m_isInit &= ~HAS_TRIANG;
        qDebug() << retval.errorMessage() << "\n";
        std::cout << retval.errorMessage() << "\n";
        m_errorDisplMsg.clear();
        m_errorDisplMsg.append(retval.errorMessage());
    }

    m_forceReplot = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** method for adjusting the glWidget size
*   @param [in] width       width of the outer window
*   @param [in] height      height of the outer window
*
*   Adjust the openGL widget to the size of the outer window
*/
void TwipOGLWidget::resizeGL(int width, int height)
{
    if (m_isInit & IS_INIT)
    {
        GLFPTR(glViewport)(0, 0, width, height);    // resize window
        GLFPTR(glUseProgram)(m_prog2DPx);
        GLFPTR(glUniform1f)(m_unifScaleX, 2.0 / (float)width);
        GLFPTR(glUniform1f)(m_unifScaleY, 2.0 / (float)height);
        GLFPTR(glUseProgram)(0);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------
/** set plot colors to current palette
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   Set the rgb values used for plotting to the values of the currently
*   selected palette
*/
ito::RetVal TwipOGLWidget::ResetColors()
{
    if(m_prog3D < 0 || m_unifPalette < 0 || m_unifGlColorInv < 0) return ito::RetVal(ito::retError, 0, tr("Could not rebuild colors, openGl not fully initialized").toLatin1().data());
    int paletteSize = m_currentPalette.size();
    GLfloat *src = NULL;
    src = new GLfloat[paletteSize * 3];
    unsigned char* ptrPal =  (unsigned char*)m_currentPalette.data();

    //!> make a buffer with the rgb values for palette entries
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #pragma omp for schedule(guided)
    #endif
    for(int i = 0; i < paletteSize; i++)
    {
        src[3 * i]     = ptrPal[4 * i + 2] / 255.0f;
        src[3 * i + 1] = ptrPal[4 * i + 1] / 255.0f;
        src[3 * i + 2] = ptrPal[4 * i] / 255.0f;
    }
    #if (USEOMP)
    }
    #endif

    //!> upload buffer to the graphics board. So we can
    //!> use color indices to address the palette colors
    GLFPTR(glUseProgram)(m_prog3D);
    GLFPTR(glUniform3fv)(m_unifPalette, 256, src);
    GLFPTR(glUniform4f)(m_unifGlColorInv,
        ((m_pConfigData->m_invColor & 0x00FF0000) >> 16) / 255.0f,
        ((m_pConfigData->m_invColor & 0x0000FF00) >> 8 ) / 255.0f,
        ((m_pConfigData->m_invColor & 0x000000FF)      ) / 255.0f,
        ((m_pConfigData->m_invColor & 0xFF000000)>> 24 ) / 255.0f);

    GLFPTR(glUseProgram)(0);
    delete[] src;

    GLubyte *srcb = new GLubyte[m_currentPalette.size() * 4];
    for(int i = 0; i < m_currentPalette.size(); i++)
    {
        // our palette is stored as 32bit values so we multiply by 4
        srcb[4 * i]     = ((unsigned char*)m_currentPalette.data())[4 * i + 2];
        srcb[4 * i + 1] = ((unsigned char*)m_currentPalette.data())[4 * i + 1];
        srcb[4 * i + 2] = ((unsigned char*)m_currentPalette.data())[4 * i ];
        srcb[4 * i + 3] = 1.0;
    }

    //!> our color bar is a simple texture so we have to update
    //!> that texture to match the current palette
    GLFPTR(glUseProgram)(m_prog2DPx);
#if QT_VERSION < 0x050000
    glBindVertexArray(m_cBarVAO);
#else
    m_cBarVAO->bind();
#endif
    GLFPTR(glBindTexture)(GL_TEXTURE_2D, m_cBarTex);
    GLFPTR(glTexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GLFPTR(glTexParameteri)(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    GLFPTR(glPixelStorei)(GL_UNPACK_ALIGNMENT, 1);
    GLFPTR(glTexImage2D)(GL_TEXTURE_2D, 0, GL_RGBA, 1, m_currentPalette.size(), 0, GL_RGBA, GL_UNSIGNED_BYTE, srcb);
    GLFPTR(glBindTexture)(GL_TEXTURE_2D, 0);
    delete[] srcb;
#if QT_VERSION < 0x050000
    glBindVertexArray(0);
#else
    m_cBarVAO->release();
#endif
    GLFPTR(glUseProgram)(0);

    update();

    return ito::retOk;
}

//-----------------------------------------------------------------------------------------------
/** paint arrow for light direction
*
*   When lighting is used in our plot we can use the light arrow
*   to choose the current light direction. This method paints
*   the light arrow.
*/
void TwipOGLWidget::paintLightArrow()
{
/*
    GLFPTR(glUseProgram)(m_prog3DPri);
    cv::Mat mvpMat = setMVP(m_unifMVPPri, 45.0f, 0.01f, 100.0f, (float)width() / (float)height(), 1);
#if QT_VERSION < 0x050000
    glBindVertexArray(m_VAO3DPri);
#else
    m_VAO3DPri->bind();
#endif

    glLineWidth(m_axes.m_lineWidth);

    GLFPTR(glUniform3f)(m_unifGlColor3DPri,
        ((m_pConfigData->m_axisColor & 0x00FF0000) >> 16) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x0000FF00) >> 8) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x000000FF)) / 255.0f);


    int nPts = 5;
    cv::Mat arrowPts = cv::Mat_<GLfloat>::zeros(nPts, 4);
    GLfloat* arrowPtsPtr = (GLfloat*)arrowPts.ptr(0);

    // Pt 1 (0.05, 0, 0.9) 0 - 3
    arrowPtsPtr[0] = 0.05;
    arrowPtsPtr[2] = 0.9;
    arrowPtsPtr[3] = 1.0;

    // Pt 2 (-0.05, 0, 0.9) 4 - 7
    arrowPtsPtr[4] = -0.05;
    arrowPtsPtr[6] = 0.9;
    arrowPtsPtr[7] = 1.0;

    // Pt 3 (0, 0, 0.8) 8 - 11
    arrowPtsPtr[10] = 0.8;
    arrowPtsPtr[11] = 1.0;

    // Pt 4 (0, 0.05, 0.9) 12 - 15
    arrowPtsPtr[13] = 0.05;
    arrowPtsPtr[14] = 0.9;
    arrowPtsPtr[15] = 1.0;

    // Pt 6 (0, -0.05, 0.9) 16 - 19
    arrowPtsPtr[17] = -0.05;
    arrowPtsPtr[18] = 0.9;
    arrowPtsPtr[19] = 1.0;

    cv::Mat lArrRMat = makeRotMatFromEuler(m_pConfigData->m_lightDirYaw, 0.0, m_pConfigData->m_lightDirPitch + GL_PI);

    arrowPts = lArrRMat * arrowPts.t();
    arrowPts = arrowPts.t();

    //Paint the arrow
    GLFPTR(glEnableVertexAttribArray)(m_attribVert3DPri);
    GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_vertBuf3DPri);
    GLFPTR(glBufferData)(GL_ARRAY_BUFFER, nPts * 4 * sizeof(GLfloat), arrowPtsPtr, GL_STATIC_DRAW);

    GLFPTR(glVertexAttribPointer)(m_attribVert3DPri, 3, GL_FLOAT, 0, 4 * sizeof(GLfloat), 0);
    GLFPTR(glDrawArrays)(GL_LINE_LOOP, 0, 3);

    GLFPTR(glDrawArrays)(GL_LINE_LOOP, 2, 3);

    GLFPTR(glDisableVertexAttribArray)(m_attribVert3DPri);

#if QT_VERSION < 0x050000
    glBindVertexArray(0);
#else
    m_VAO3DPri->release();
#endif
    GLFPTR(glUseProgram)(0);
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Plot text to a given position
*   @param [in] text            string to plot
*   @param [in] xpos            vertical text position
*   @param [in] ypos            horizontal text position
*   @param [in] rightAligned    flag to print text right (1) or left (0) aligned or centered (2)
*   @param [in] topAligned      flag to print text top (1) or bottom (0) aligned
*   @param [in] ffamily         font family, style used for printing text
*   @param [in] fsize           font size used
*   @param [in] fcolor          font color used
*
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   The input text is transformed into a bitmap using the previously prepared font composing it from
*   the single characters and is plotted to the given x,y coordinate with the given alignment.
*/
int TwipOGLWidget::OGLTextOut(QString &text, double xpos, double ypos, const bool rightAligned, const bool topAligned, const QString ffamily, const int fsize, const ito::uint32 fcolor)
{
    // make sure we are in texture unit 0 as this is what the
    // shader expects
    GLFPTR(glUseProgram)(m_prog2DPx);
    GLFPTR(glActiveTexture)(GL_TEXTURE0);

    GLFPTR(glUniform3f)(m_unifTextColor, ((fcolor >> 16) & 0xFF) / 255.0f, ((fcolor >> 8) & 0xFF) / 255.0f, (fcolor & 0xFF) / 255.0f);

    // now enable blending and disable depth sorting so the font renders
    // correctly
    GLFPTR(glEnable)(GL_BLEND);
    GLFPTR(glDisable)(GL_DEPTH_TEST);
    GLFPTR(glBlendFunc)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    GLFPTR(glUniform1i)(m_unifUseTex, 1);

    // now loop for each of the char and draw our billboard
    int textLength = text.length();
    float _x = xpos;
    SFont uFont;
    if (!m_fonts.contains(ffamily + QString::number(fsize)))
    {
        QFont nf(ffamily);
        nf.setPixelSize(fsize);
        //nf.setPointSize(fsize);
        prepareFont(nf, uFont);
    }
    else
    {
        uFont = m_fonts[ffamily + QString::number(fsize)];
    }

    if (topAligned == 0)
    {
        GLFPTR(glUniform1f)(m_unifPosY, ypos);
    }
    else
    {
        GLFPTR(glUniform1f)(m_unifPosY, ypos - uFont.m_height);
    }

    if (rightAligned == 0) // align left
    {
        //!> left aligned text is easy, just get bitmap and plot it
        for (int i = 0; i < textLength; ++i)
        {
            // set the shader x position this will change each time
            // we render a glyph by the width of the char
            GLFPTR(glUniform1f)(m_unifPosX, _x);

            // so find the FontChar data for our current char
            if (text[i].toLatin1() - ' ' < 0 || text[i].toLatin1() - ' ' >= uFont.m_chars.size())
                continue;
            FChar f = uFont.m_chars[text[i].toLatin1() - ' '];

#if QT_VERSION < 0x050000
            glBindVertexArray(f.m_vbo->m_VAO);
#else
            f.m_vbo->m_VAO->bind();
#endif
            // bind the pre-generated texture
            GLFPTR(glBindTexture)(GL_TEXTURE_2D, f.m_texID);

            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, f.m_vbo->m_vbufId);
            GLFPTR(glDrawArrays)(GL_TRIANGLE_STRIP, 0, 4);

#if QT_VERSION < 0x050000
            glBindVertexArray(0);
#else
            f.m_vbo->m_VAO->release();
#endif

            // finally move to the next glyph x position by incrementing
            // by the width of the char just drawn
            _x += f.m_width;
        }
    }
    else // right or center aligned
    {
        //!> right and center aligned text is little bit more complicated
        //!> we first must determine the final text size from the sizes of
        //!> our characters
        int textLen = 0;
        // first calculate the length of the text we want to print
        for (int i = 0; i < textLength; ++i)
        {
            // so find the FontChar data for our current char
            if (text[i].toLatin1() - ' ' < 0 || text[i].toLatin1() - ' ' >= uFont.m_chars.size())
                continue;
            FChar f = uFont.m_chars[text[i].toLatin1() - ' '];
            textLen += f.m_width;
        }

        // align text right
        if (rightAligned == 1)
            _x -= textLen;
        else // center text
            _x -= textLen / 2;

        for (int i = 0; i < textLength; ++i)
        {
            // set the shader x position this will change each time
            // we render a glyph by the width of the char
            GLFPTR(glUniform1f)(m_unifPosX, _x);

            // so find the FontChar data for our current char
            if (text[i].toLatin1() - ' ' < 0 || text[i].toLatin1() - ' ' >= uFont.m_chars.size())
                continue;
            FChar f = uFont.m_chars[text[i].toLatin1() - ' '];

#if QT_VERSION < 0x050000
            glBindVertexArray(f.m_vbo->m_VAO);
#else
            f.m_vbo->m_VAO->bind();
#endif
            // bind the pre-generated texture
            GLFPTR(glBindTexture)(GL_TEXTURE_2D, f.m_texID);

            GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, f.m_vbo->m_vbufId);
            GLFPTR(glDrawArrays)(GL_TRIANGLE_STRIP, 0, 4);

#if QT_VERSION < 0x050000
            glBindVertexArray(0);
#else
            f.m_vbo->m_VAO->release();
#endif

            // finally move to the next glyph x position by incrementing
            // by the width of the char just drawn
            _x += f.m_width;
        }
    }

    // finally disable the blend and re-enable depth sort
    GLFPTR(glDisable)(GL_BLEND);
    GLFPTR(glEnable)(GL_DEPTH_TEST);
    GLFPTR(glUseProgram)(0);

    return glGetError();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Plot object information
*
*   plot some basic object information in the lower right edge of the window
*/
void TwipOGLWidget::DrawObjectInfo(void)
{
    double x0 = width();
    double y0 = height() - m_titleFont.m_fontSize;

    QString buf;
    if(m_objectInfo.xLength.length())
    {
        buf = QString::fromStdString(m_objectInfo.xLength);
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    y0 -= m_titleFont.m_fontSize;
    if(m_objectInfo.yLength.length())
    {
        buf = QString::fromStdString(m_objectInfo.yLength);
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    y0 -= m_titleFont.m_fontSize;
    if(m_objectInfo.matrix.length())
    {
        buf = QString::fromStdString(m_objectInfo.matrix);
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    size_t len = m_objectInfo.PeakText.length();

    if(len < m_objectInfo.MeanText.length())
        len = m_objectInfo.MeanText.length();

    if(len < m_objectInfo.DevText.length())
        len = m_objectInfo.DevText.length();

    y0 -= 2 * m_titleFont.m_fontSize;
    if(m_objectInfo.PeakText.length())
    {
        buf = QString::fromStdString(m_objectInfo.PeakText.data());
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    y0 -= m_titleFont.m_fontSize;
    if(m_objectInfo.MeanText.length())
    {
        buf = QString::fromStdString(m_objectInfo.MeanText.data());
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    y0 -= m_titleFont.m_fontSize;
    if(m_objectInfo.DevText.length())
    {
        buf = QString::fromStdString(m_objectInfo.DevText.data());
        OGLTextOut(buf, x0, y0, 1, 0, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Draw color bar
*
*   @param [in] posX        horizontal position of colorbar
*   @param [in] posY        vertical position of colorbar
*   @param [in] dX          width of colorbar
*   @param [in] dY          height of colorbar
*   @param [in] centerRight left (0) / right (1) aligned
*
*   Draws the colorbar using the texture made from the palette with the given size at the given position.
*   Except the colors the upper and lower values are written as a text.
*/
void TwipOGLWidget::DrawColorBar(const GLfloat posX, const GLfloat posY, const GLfloat dX, const GLfloat dY, const bool centerRight)
{
    GLFPTR(glUseProgram)(m_prog2DPx);
    GLFPTR(glActiveTexture)(GL_TEXTURE0);

    // now enable blending and disable depth sorting so the font renders
    // correctly
    GLFPTR(glDisable)(GL_BLEND);
    GLFPTR(glDisable)(GL_DEPTH_TEST);

    GLFPTR(glUniform1f)(m_unifPosY, posY);
    GLFPTR(glUniform1f)(m_unifPosX, posX);

    bool useDev = false;
    if(this->m_pConfigData)
    {
        useDev = this->m_pConfigData->m_showCurvature;
    }

    const GLfloat zMin = useDev ? m_axes.m_devAxis.getMin() : m_axes.m_axisZ.getMin();
    const GLfloat zMax = useDev ? m_axes.m_devAxis.getMax() : m_axes.m_axisZ.getMax();

    // draw outline
    GLFPTR(glUniform3f)(m_unifGlColor2D,
        ((m_pConfigData->m_axisColor & 0x00FF0000) >> 16) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x0000FF00) >> 8) / 255.0f,
        ((m_pConfigData->m_axisColor & 0x000000FF)) / 255.0f);

    // we are creating a billboard with two triangles so we only need the
    // 6 verts, (could use index and save some space but shouldn't be too much of an
    // issue
    textVertData d[4];

    // load values for triangle 1
    d[0].x = 0;
    d[0].y = 0;
    d[0].z = 0;
    d[0].u = 0;
    d[0].v = 1;

    d[1].x = 0;
    d[1].y = dY - 1;
    d[1].z = 0;
    d[1].u = 0;
    d[1].v = 0;

    d[2].x = dX - 1;
    d[2].y = 0;
    d[2].z = 0;
    d[2].u = 1;
    d[2].v = 1;

    d[3].x = dX - 1;
    d[3].y = dY - 1;
    d[3].z = 0;
    d[3].u = 1;
    d[3].v = 0;

    GLFPTR(glUniform1i)(m_unifUseTex, 2);
#if QT_VERSION < 0x050000
    glBindVertexArray(m_cBarVAO);
#else
    m_cBarVAO->bind();
#endif
    GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_cBarVBuf);
    GLFPTR(glBindTexture)(GL_TEXTURE_2D, m_cBarTex);
    GLFPTR(glEnableVertexAttribArray)(m_attribTxtVert);
    GLFPTR(glEnableVertexAttribArray)(m_attribTxtUV);
    GLFPTR(glVertexAttribPointer)(m_attribTxtVert, 3, GL_FLOAT, 0, sizeof(textVertData),  0);
    GLFPTR(glVertexAttribPointer)(m_attribTxtUV, 2, GL_FLOAT, 0, sizeof(textVertData), (void*)(3 * sizeof(GLfloat)));
    GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 4 * sizeof(textVertData), d, GL_STATIC_DRAW);
    GLFPTR(glDrawArrays)(GL_TRIANGLE_STRIP, 0, 4);

    GLFPTR(glUniform1i)(m_unifUseTex, 0);
    GLfloat vert[20] =
    { 0.0, 0.0, 0.0, 0.0, 0.0,
      dX,  0.0, 0.0, 0.0, 0.0,
      dX,   dY, 0.0, 0.0, 0.0,
      0.0,  dY, 0.0, 0.0, 0.0
    };
    GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 20 * sizeof(GLfloat), vert, GL_STATIC_DRAW);
    GLFPTR(glDrawArrays)(GL_LINE_LOOP, 0, 4);
    GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, 0);
    GLFPTR(glBindTexture)(GL_TEXTURE_2D, 0);
    GLFPTR(glDisableVertexAttribArray)(m_attribTxtVert);
    GLFPTR(glDisableVertexAttribArray)(m_attribTxtUV);

#if QT_VERSION < 0x050000
    glBindVertexArray(0);
#else
    m_cBarVAO->release();
#endif
    GLFPTR(glUseProgram)(0);

    QString buf;
    buf.asprintf("%g", zMin);
    OGLTextOut(buf, posX + (centerRight ? dX : 0.0) , posY + dY, centerRight, 0, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    buf.clear();
    buf.asprintf("%g", zMax);
    OGLTextOut(buf, posX + (centerRight ? dX : 0.0), posY, centerRight, 1, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());

    buf.clear();
    if(useDev)
    {
        buf = QString::fromStdString(m_axes.m_devAxis.getLabel());
        if(m_axes.m_devAxis.getUnit().length() > 0) buf.append(QString(", %1").arg(QString::fromStdString(m_axes.m_devAxis.getUnit())));

    }
    else
    {
        buf = QString::fromStdString(m_axes.m_axisZ.getLabel());
        if(m_axes.m_axisZ.getUnit().length() > 0) buf.append(QString(", %1").arg(QString::fromStdString(m_axes.m_axisZ.getUnit())));
    }


    if( !buf.isEmpty() )
    {
        OGLTextOut(buf, posX + (centerRight ? dX : 0.0), posY - m_axes.m_fontSize * 1.2, centerRight, 1, QString::fromStdString(m_axes.getFontName()), m_axes.getFontSize(), m_axes.getFontColor());
    }
//    glBindVertexArray(0);
    GLFPTR(glUseProgram)(0);

    return;
}

//-----------------------------------------------------------------------------------------------
/** Draw plot / data object title
*
*   Draws either the data object title tag (if set) or the title set as property for the plot.
*/
void TwipOGLWidget::DrawTitle()
{
    OGLTextOut(m_pConfigData->m_title, width() / 2, m_titleFont.m_fontSize, true, false, QString::fromStdString(m_titleFont.m_fontName), m_titleFont.m_fontSize, m_titleFont.m_fontColor);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Sets the z-scaling factor
*
*   @param [in] value        Amplication value
*
*   Usually the feature we want to see are quite small compared to the lateral size of the field.
*   Therefore we us an amplication factor to show them. This function sets the factor to the passed
*   value.
*/
void TwipOGLWidget::validateZAmplification()
{
    if (m_pConfigData->m_zAmpl > 500.0f)
    {
        m_pConfigData->m_zAmpl = 500.0f;
    }
    else if(m_pConfigData->m_zAmpl < 0.000001f)
    {
        m_pConfigData->m_zAmpl = 0.000001f;
    }

    //update();
}

#if 0
//----------------------------------------------------------------------------------------------------------------------------------
/** Sets the z-scaling factor
*
*   @param [in] value        Amplication value
*
*   Usually the feature we want to see are quite small compared to the lateral size of the field.
*   Therefore we us an amplication factor to show them. This function sets the factor to the passed
*   value.
*/
void TwipOGLWidget::setZAmplification(double value)
{
    if (value <= 500.0f && value >= 0.000001f)
    {
        m_pConfigData->m_zAmpl = value;
    }

    update();
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Reduces the z-scaling factor
*
*   @param [in] value        Amplication value
*
*   Usually the feature we want to see are quite small compared to the lateral size of the field.
*   Therefore we us an amplication factor to show them. This function reduces the amplication
*   factor and is e.g. used when changing the amplication with the mouse wheel
*/
void TwipOGLWidget::reduceZAmplification(double value)
{
    if(m_pConfigData->m_zAmpl * value >= 0.000001f)
    {
        m_pConfigData->m_zAmpl *= value;
    }

    if(m_pConfigData->m_zAmpl < 0.000001f)
    {
        m_pConfigData->m_zAmpl = 0.000001f;
    }

    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Increases the z-scaling factor
*
*   @param [in] value        Amplication value
*
*   Usually the feature we want to see are quite small compared to the lateral size of the field.
*   Therefore we us an amplication factor to show them. This function increases the amplication
*   factor and is e.g. used when changing the amplication with the mouse wheel
*/
void TwipOGLWidget::riseZAmplification(const double value)
{
    if(m_pConfigData->m_zAmpl * value <= 500.0)
    {
        m_pConfigData->m_zAmpl *= value;
    }

    if(m_pConfigData->m_zAmpl > 500.0)
    {
        m_pConfigData->m_zAmpl = 500.0f;
    }

    update();
}
#endif
//----------------------------------------------------------------------------------------------------------------------------------
/** Change colorbar position
*
*   The palette can be hidden or displayed in one of four predefined positions.
*   With this function the palette is moved to the position stored in the member
*   variable m_colorBarMode. The actual painting is done by the method \ref{DrawColorBar}
*/
void TwipOGLWidget::togglePaletteMode()
{
    switch(m_pConfigData->m_colorBarMode)
    {
        default:
        case COLORBAR_NO:
            m_pConfigData->m_colorBarMode = COLORBAR_LEFT;
            break;
        case COLORBAR_LEFT:
            m_pConfigData->m_colorBarMode = COLORBAR_RIGHT;
            break;
        case COLORBAR_RIGHT:
            m_pConfigData->m_colorBarMode = COLORBAR_UPPER_RIGHT;
            break;
        case COLORBAR_UPPER_RIGHT:
            m_pConfigData->m_colorBarMode = COLORBAR_NO;
            break;
    }
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Set color map
*
*   @param [in] value        New palette identifier
*
*   Changes the palette used for color encoding to the new one passed as string
*   identifier.
*/
void TwipOGLWidget::setColorMap(QString palette)
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
        m_pConfigData->m_paletteNum %= numPalettes;
        retval += apiPaletteGetColorBarIdx(m_pConfigData->m_paletteNum, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(palette, newPalette);
        retval += apiPaletteGetColorBarIdxFromName(palette, m_pConfigData->m_paletteNum);
    }

    if(newPalette.colorVector256.size() < 255)
    {
        return;
    }

    m_currentPalette = newPalette.colorVector256;

    if(m_isInit == NO_INIT) return;

    makeCurrent();
    ResetColors();
    doneCurrent();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Set interval for axis
*
*   @param [in] axis            axis to change
*   @param [in] autoCalcLimits  auto calculate limits from data
*   @param [in] minValue        minimum value to display
*   @param [in] maxValue        maximum value to display
*
*   Though obviously not working yet this function should set the limits for the displayed
*   data.
*/
ito::RetVal TwipOGLWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Update window
*
*   Update window content after an overlay image has been set.
*/
void TwipOGLWidget::updateVisMode()
{
    m_isInit &= ~HAS_TRIANG;

    m_forceReplot = true;
    refreshPlot(NULL);

    // ToDO-Add multi level update of overlay
    for (int nid = 0; nid < m_pContentDObj.size(); nid++)
    {
        if (m_pConfigData->m_enableOverlay
            && !m_pContentDObj[nid].isNull()
            && m_overlayImage[nid]->getDims() >= 2
            && m_overlayImage[nid]->getSize(0) == m_pContentDObj[nid]->getSize(0)
            && m_overlayImage[nid]->getSize(1) == m_pContentDObj[nid]->getSize(1))
        {
            switch (m_overlayImage[nid]->getType())
            {
            case ito::tUInt8:
                updateOverlayImage<ito::uint8>(nid);
                break;
            case ito::tInt8:
                updateOverlayImage<ito::int8>(nid);
                break;
            case ito::tUInt16:
                updateOverlayImage<ito::uint16>(nid);
                break;
            case ito::tInt16:
                updateOverlayImage<ito::int16>(nid);
                break;
            case ito::tUInt32:
                updateOverlayImage<ito::uint32>(nid);
                break;
            case ito::tInt32:
                updateOverlayImage<ito::int32>(nid);
                break;
            case ito::tFloat32:
                updateOverlayImage<ito::float32>(nid);
                break;
            case ito::tFloat64:
                updateOverlayImage<ito::float64>(nid);
                break;

                /*
                case ito::tComplex64:
                updateOverlayImage<ito::complex64>(nid);
                break;
                case ito::tComplex128:
                updateOverlayImage<ito::complex128>(nid);
                break;
                */
            default:
                m_errorDisplMsg.append("Object has invalid type");
            }
        }
    }

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Update plot
*
*   @param [in] forceUpdate     fore immediate repaint
*
*   Pushes a change of the configuration parameters in the m_pConfigData
*   structure to the member variables and depending on forceUpdate
*   issues an immediate repaint
*/
void TwipOGLWidget::updateColorsAndVisibility(const bool forceUpdate)
{
    m_axes.setLineColor(m_pConfigData->m_axisColor);
    m_axes.setFontColor(m_pConfigData->m_textColor);
    m_titleFont.setFontColor(m_pConfigData->m_textColor);

    m_axes.m_axisX.m_isVisible = m_pConfigData->m_xAxisVisible;
    m_axes.m_axisY.m_isVisible = m_pConfigData->m_yAxisVisible;
    m_axes.m_axisZ.m_isVisible = m_pConfigData->m_vAxisVisible;

    m_objectInfo.show = m_pConfigData->m_infoVisible;

    if(forceUpdate)
        update();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Enable or disable object info text
*
*   @param [in] enabled        State of info text
*
*   Enables or disables the displaying of the object info text. Which is actually
*   painted by the method \ref{DrawObjectInfo}
*/
void TwipOGLWidget::toggleObjectInfoText(const bool enabled, const int fromID)
{
    if (enabled)
    {
        if (m_pContentDObj[fromID])
        {
            ito::dObjHelper::devValue(m_pContentDObj[fromID].data(), 1, m_objectInfo.meanVal, m_objectInfo.divVal, true);
        }
        generateObjectInfoText(fromID);
        m_objectInfo.show = 1;
        m_pConfigData->m_infoVisible = true;
    }
    else
    {
        m_objectInfo.show = 0;
        m_pConfigData->m_infoVisible = false;
    }
    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Generates the object info text
*
*   Generates the object info text from the objects physical properties and
*   some tags. The state of the object info text is changed by the method
*   \ref{toggleObjectInfoText} and the painting is done by the method
*   \ref{DrawObjectInfo}
*/
inline void TwipOGLWidget::generateObjectInfoText(const int fromID)
{
    char buf[40] = {0};

    if (m_axes.m_axisY.m_unit.length() && m_axes.m_axisY.m_unit.length() < 5)
    {
        sprintf(buf, "Height: %.4g %s", m_axes.m_axisY.getLength(), m_axes.m_axisY.m_unit.data());
    }
    else
    {
        sprintf(buf, "Height: %.4g", m_axes.m_axisY.getLength());
    }
    m_objectInfo.yLength = buf;

    if (m_axes.m_axisX.m_unit.length() && m_axes.m_axisX.m_unit.length() < 5)
    {
        sprintf(buf, "Width:  %.4g %s", m_axes.m_axisX.getLength(), m_axes.m_axisX.m_unit.data());
    }
    else
    {
        sprintf(buf, "Width:  %.4g", m_axes.m_axisX.getLength());
    }
    m_objectInfo.xLength = buf;

    if(m_pContentDObj[fromID])
    {
        switch (m_pContentDObj[fromID]->getType())
        {
            case ito::tInt8:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "int8");
            break;
            case ito::tInt16:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "int16");
            break;
            case ito::tInt32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "int32");
            break;
            case ito::tUInt8:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "uint8");
            break;
            case ito::tUInt16:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "uint16");
            break;
            case ito::tUInt32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "uint32");
            break;
            case ito::tFloat32:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "float32");
            break;
            case ito::tFloat64:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "float64");
            break;
            case ito::tComplex64:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "complex64");
            break;
            case ito::tComplex128:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "complex128");
            break;
            default:
                sprintf(buf, "Matrix: %i x %i (%s)", m_axes.m_axisX.getIdx1() - m_axes.m_axisX.getIdx0() + 1, m_axes.m_axisY.getIdx1() - m_axes.m_axisY.getIdx0() + 1, "?");
        }
    }
#ifdef USEPCL
    else if(m_pContentPC[fromID])
    {
        switch (m_pContentPC[fromID]->getType())
        {
            default:
            case pclInvalid:
                sprintf(buf, "PointCloud Type: Invalid");
                break;
            case pclXYZ:
                sprintf(buf, "PointCloud Type: XYZ");
                break;
            case pclXYZI:
                sprintf(buf, "PointCloud Type: XYZ + I");
                break;
            case pclXYZRGBA:
                sprintf(buf, "PointCloud Type: XYZ + RGB");
                break;
            case pclXYZNormal:
                sprintf(buf, "PointCloud Type: XYZ + Normals");
                break;
            case  pclXYZINormal:
                sprintf(buf, "PointCloud Type: XYZ + I + Normals");
                break;
            case pclXYZRGBNormal:
                sprintf(buf, "PointCloud Type: XYZ + RGB + Normals");
                break;
        }
    }
#endif
    m_objectInfo.matrix = buf;

    if(m_axes.m_axisZ.m_unit.length() && m_axes.m_axisZ.m_unit.length() < 5)
    {
        sprintf(buf, "PV:   %.4g %s", m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin(), m_axes.m_axisZ.m_unit.data());
        m_objectInfo.PeakText = buf;
        if (m_pContentDObj[fromID])
        {
            sprintf(buf, "Mean: %.4g %s", m_objectInfo.meanVal, m_axes.m_axisZ.m_unit.data() );
            m_objectInfo.MeanText = buf;
            sprintf(buf, "Dev:  %.4g %s", m_objectInfo.divVal, m_axes.m_axisZ.m_unit.data() );
            m_objectInfo.DevText = buf;
        }
        else
        {
            m_objectInfo.MeanText = "";
            m_objectInfo.DevText = "";
        }
    }
    else
    {
        sprintf(buf, "PV:   %.4g", m_axes.m_axisZ.getMax() - m_axes.m_axisZ.getMin());
        m_objectInfo.PeakText = buf;
        if (m_pContentDObj[fromID])
        {
            sprintf(buf, "Mean: %.4g", m_objectInfo.meanVal);
            m_objectInfo.MeanText = buf;
            sprintf(buf, "Dev:  %.4g", m_objectInfo.divVal);
            m_objectInfo.DevText = buf;
        }
        else
        {
            m_objectInfo.MeanText = "";
            m_objectInfo.DevText = "";
        }
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Moves the view inside the window
*
*   @param [in] deltaX      Position change in x
*   @param [in] deltaY      Position change in y
*   @param [in] deltaZ      Position change in z
*
*   Moves the position of the view inside the window, which is done by moving the coordinate
*   system. This comes handy when we zoom into the object and want to see a detail that is
*   not in the central part. Uses the method \ref{setViewTranslation} to incrementally
*   move the view.
*/
void TwipOGLWidget::moveView(const double deltaX, const double deltaY, const double deltaZ)
{
    setViewTranslation(m_zoomer.shiftx + deltaX, m_zoomer.shifty + deltaY, m_zoomer.shiftz + deltaZ);
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Changes the zoom
*
*   @param [in] factor      Amplication factor
*
*   Changes the zoom for the view, e.g. when the mouse wheel is used.
*/
void TwipOGLWidget::setCanvasZoomView(const double factor)
{
    if(factor < 0.1)
    {
        m_zoomer.scalexy = 0.1f;
    }
    else if(factor > 10.0)
    {
        m_zoomer.scalexy = 10.0f;
    }
    else
    {
        m_zoomer.scalexy = factor;
    }
    if(ito::isNotZero<float>(m_zoomer.scalexy - 1.0f))
    {
        m_zoomer.enabled = true;
    }
    else if(
        fabs(m_zoomer.shiftz) < 0.001 &&
        fabs(m_zoomer.shifty) < 0.001 &&
        fabs(m_zoomer.shiftx) < 0.001)
    {
        m_zoomer.enabled = false;
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Moves the view inside the window
*
*   @param [in] transX      Position change in x
*   @param [in] transY      Position change in y
*   @param [in] transZ      Position change in z
*
*   Moves the position of the view inside the window, which is done by moving the coordinate
*   system. This comes handy when we zoom into the object and want to see a detail that is
*   not in the central part.
*/
void TwipOGLWidget::setViewTranslation(const double transX, const double transY, const double transZ)
{
    m_zoomer.shiftx = transX;
    m_zoomer.shiftx = m_zoomer.shiftx > 2.0f ? 2.0f : m_zoomer.shiftx < -2.0f ? -2.0f : fabs(m_zoomer.shiftx) > 0.001? m_zoomer.shiftx : 0.0;

    m_zoomer.shifty = transY;
    m_zoomer.shifty = m_zoomer.shifty > 2.0f ? 2.0f : m_zoomer.shifty < -2.0f ? -2.0f : fabs(m_zoomer.shifty) > 0.001? m_zoomer.shifty : 0.0;

    m_zoomer.shiftz = transZ;
    m_zoomer.shiftz = m_zoomer.shiftz > 2.0f ? 2.0f : m_zoomer.shiftz < -2.0f ? -2.0f : fabs(m_zoomer.shiftz) > 0.001? m_zoomer.shiftz : 0.0;

    if(fabs(m_zoomer.shiftz) > 0.001 || fabs(m_zoomer.shifty) > 0.001 || fabs(m_zoomer.shiftx) > 0.001) m_zoomer.enabled = true;
    else if(ito::isFinite<float>(m_zoomer.scalexy - 1.0f))
    {
        m_zoomer.enabled = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Reset view orientation, zoom, position
*
*   Reset view to default state, i.e. reset zoom, translation, rotation and shift.
*/
void TwipOGLWidget::homeView(void)
{
    m_pConfigData->m_lightDirPitch = 0.0;
    m_pConfigData->m_lightDirYaw = 0.0;

    m_pConfigData->m_zAmpl = 1.0;

    m_zoomer.enabled = false;
    m_zoomer.scalexy = 1.0f;

    m_zoomer.shiftx = 0.0f;
    m_zoomer.shifty = 0.0f;
    m_zoomer.shiftz = 0.0f;

    m_pConfigData->m_rollAng = ROLL0;
    m_pConfigData->m_pitchAng = PITCH0;
    m_pConfigData->m_yawAng = YAW0;
    update();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Change relation between height color coding and image overlay
*
*   Updates the alpha for image overlay according to the value set in m_pConfigData->m_alpha
*/
void TwipOGLWidget::updateAlpha()
{
    if (m_prog3D >= 0)
    {
        makeCurrent();
        GLFPTR(glUseProgram)(m_prog3D);
        GLFPTR(glUniform1f)(m_unifText, m_pConfigData->m_alpha);
        GLFPTR(glUseProgram)(0);
        doneCurrent();
        update();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Update the boundaries (interval) of the deviation overlay
*
*   Deviation plots, i.e. colorcode the points according to their distance to a
*   model, are possible using the curvature values of point clouds. This method
*   resets the boundaries of the color coding interval
*/
void TwipOGLWidget::updateCurvature()
{
    if(m_pConfigData->m_curvatureInterval.isAuto())
    {
        ito::float64 minDev = std::numeric_limits<ito::float64>::max();
        ito::float64 maxDev = -std::numeric_limits<ito::float64>::max();
#ifdef USEPCL
        for (QHash<int, QSharedPointer<ito::PCLPointCloud> >::ConstIterator it = m_pContentPC.begin(); it != m_pContentPC.end(); it++)
        {
            if (m_pclMinDev.value(it.key()) < minDev)
                minDev = m_pclMinDev.value(it.key());
            if (m_pclMaxDev.value(it.key()) > maxDev)
                maxDev = m_pclMaxDev.value(it.key());
        }
#endif

        m_axes.m_devAxis.setMin(minDev);
        m_axes.m_devAxis.setMax(maxDev);
        m_pConfigData->m_curvatureInterval.setMinimum(minDev);
        m_pConfigData->m_curvatureInterval.setMaximum(maxDev);
    }
    else
    {
        m_axes.m_devAxis.setMin(m_pConfigData->m_curvatureInterval.minimum());
        m_axes.m_devAxis.setMax(m_pConfigData->m_curvatureInterval.maximum());
    }

    if (m_prog3D >= 0)
    {
        makeCurrent();
        GLFPTR(glUseProgram)(m_prog3D);
        float devMin = m_axes.m_devAxis.getMin();
        if(!ito::isFinite(devMin)) devMin = 0.0;
        float devMax = m_axes.m_devAxis.getMax();
        if(!ito::isFinite(devMax)) devMax = 1.0;
        float devNorm = 1.0f;

        if(ito::isNotZero(devMax - devMin))
        {
            devNorm = 1.0f / fabs(devMax - devMin);
        }

        GLFPTR(glUniform1f)(m_unifDiffNorm, devNorm);
        GLFPTR(glUniform1f)(m_unifDiffMin, devMin);
        GLFPTR(glUseProgram)(0);
        doneCurrent();
        update();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Update the overlay image for a given object ID
*
*   @param [in] objectID    internal ID of object to update
*
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   When a new overlay image has been set this method is used to update the view
*   The intensity values are normalized and then uploaded as texture to the
*   graphics board.
*/
template<typename _Tp> ito::RetVal TwipOGLWidget::updateOverlayImage(const int objectID)
{
    if(!m_numVert.contains(objectID))
    {
        return ito::RetVal(ito::retError, 0, tr("Tried to write overlay plane to invalid object").toLatin1().data());
    }

    if(m_pUseTextPix[objectID] == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("No valid data loaded or GL-context not initialized correct").toLatin1().data());
    }

    cv::Mat *dMat = ((cv::Mat*)m_overlayImage[objectID]->get_mdata()[m_overlayImage[objectID]->seekMat(0)]);

    ito::uint32 minLoc[3], maxLoc[3];
    ito::float64 minVal, maxVal;
    ito::float64 normVal = 1.0;

    dObjHelper::minMaxValue(m_overlayImage[objectID].data(), minVal, minLoc, maxVal, maxLoc);

    if( ito::isNotZero( maxVal - minVal ) ) normVal = 1.0 / (maxVal - minVal);

    if( m_pConfigData->m_elementMode & PAINT_POINTS)
    {
        GLfloat *textBuf = (GLfloat*)calloc(m_numVert[objectID] * 3, sizeof(GLfloat));

        #if (USEOMP)
        #pragma omp parallel num_threads(NTHREADS)
        {
        #pragma omp for schedule(guided)
        #endif
        for (int np = 0; np < m_numVert[objectID]; np++)
        {
            _Tp *ptr = (_Tp*)dMat->ptr(m_pUseTextPix[objectID][np * 2]);
            textBuf[np * 3] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
            textBuf[np * 3 + 1] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
            textBuf[np * 3 + 2] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
        }

        #if (USEOMP)
        }
        #endif

        makeCurrent();
        GLFPTR(glUseProgram)(m_prog3D);

        if (!m_textBuf3D.contains(objectID))
        {
            GLuint tmpVal;
            GLFPTR(glGenBuffers)(1, &tmpVal);
            m_textBuf3D.insert(objectID, tmpVal);
        }

        GLFPTR(glUniform1f)(m_unifText, m_pConfigData->m_alpha);
#if QT_VERSION < 0x050000
        glBindVertexArray(m_VAO3D[0]);
#else
        m_VAO3D[0]->bind();
#endif
        GLFPTR(glEnableVertexAttribArray)(m_attribTexCol);
        GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_textBuf3D.value(objectID));
        GLFPTR(glVertexAttribPointer)(m_attribTexCol, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
        GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 3 * m_numVert[objectID] * sizeof(GLfloat), textBuf, GL_STATIC_DRAW);
    //    glBindBuffer(GL_ARRAY_BUFFER, 0);
#if QT_VERSION < 0x050000
        glBindVertexArray(0);
#else
        m_VAO3D[0]->release();
#endif
        if (textBuf)
            free(textBuf);

        GLFPTR(glUseProgram)(0);
        doneCurrent();
        update();
    }
    else if( m_pConfigData->m_elementMode & PAINT_TRIANG)
    {
        GLfloat *textBuf = (GLfloat*)calloc(m_numVert[objectID] * 9, sizeof(GLfloat));

        #if (USEOMP)
        #pragma omp parallel num_threads(NTHREADS)
        {
        #pragma omp for schedule(guided)
        #endif
        for (int np = 0; np < m_numVert[objectID] * 3; np++)
        {
             _Tp *ptr = (_Tp*)dMat->ptr(m_pUseTextPix[objectID][np * 2]);
             textBuf[np * 3] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
             textBuf[np * 3 + 1] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
             textBuf[np * 3 + 2] = (ptr[m_pUseTextPix[objectID][np * 2 + 1]] - minVal) * normVal;
        }

        #if (USEOMP)
        }
        #endif

        makeCurrent();
        GLFPTR(glUseProgram)(m_prog3D);

        if (!m_textBuf3D.contains(objectID))
        {
            GLuint tmpVal;
            GLFPTR(glGenBuffers)(1, &tmpVal);
            m_textBuf3D.insert(objectID, tmpVal);
        }

        GLFPTR(glUniform1f)(m_unifText, m_pConfigData->m_alpha);
#if QT_VERSION < 0x050000
        glBindVertexArray(m_VAO3D[0]);
#else
        m_VAO3D[0]->bind();
#endif
        GLFPTR(glEnableVertexAttribArray)(m_attribTexCol);
        GLFPTR(glBindBuffer)(GL_ARRAY_BUFFER, m_textBuf3D.value(objectID));
        GLFPTR(glVertexAttribPointer)(m_attribTexCol, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
        GLFPTR(glBufferData)(GL_ARRAY_BUFFER, 9 * m_numVert[objectID] * sizeof(GLfloat), textBuf, GL_STATIC_DRAW);
    //    glBindBuffer(GL_ARRAY_BUFFER, 0);
#if QT_VERSION < 0x050000
        glBindVertexArray(0);
#else
        m_VAO3D[0]->release();
#endif
        if (textBuf)
            free(textBuf);

        GLFPTR(glUseProgram)(0);
        doneCurrent();
        update();
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Element type not correct. Only PAINT_TRIANG or PAINT_POINTS allowed.").toLatin1().data());
    }

    return ito::retOk;
}


#ifdef USEPCL
//----------------------------------------------------------------------------------------------------------------------------------
/** Find the minimum and maximum positional values in a point cloud
*
*   @param [in] *pcl    reference to the point cloud object
*   @param [in] *id     index value to access the member variable and return the specific distance value
*
*/
template<typename _Tp> void TwipOGLWidget::pclFindMinMax(pcl::PointCloud<_Tp> *pcl, const int id)
{
    _Tp pt;

    ito::float64 xmin = std::numeric_limits<ito::float64>::max();
    ito::float64 ymin = std::numeric_limits<ito::float64>::max();
    ito::float64 zmin = std::numeric_limits<ito::float64>::max();
    float devmin = std::numeric_limits<float>::max();

#if linux
    ito::float64 xmax = -std::numeric_limits<ito::float64>::max();
    ito::float64 ymax = -std::numeric_limits<ito::float64>::max();
    ito::float64 zmax = -std::numeric_limits<ito::float64>::max();
    float devmax = -std::numeric_limits<float>::max();
#else
    ito::float64 xmax = std::numeric_limits<ito::float64>::lowest();
    ito::float64 ymax = std::numeric_limits<ito::float64>::lowest();
    ito::float64 zmax = std::numeric_limits<ito::float64>::lowest();
    ito::float32 devmax = std::numeric_limits<ito::float32>::lowest();
#endif

    int size = (int)pcl->points.size();
    if(hasPointToCurvature<_Tp>())
    {
        ito::float32 val;
        for (int np = 0; np < size; np++)
        {
            pt = pcl->at(np);
            if (ito::isFinite<ito::float32>(pt.z))
            {
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

                pointToCurvature<_Tp>(pt, val);
                if (ito::isFinite<float>(val))
                {
                    if (val < devmin)
                        devmin = val;
                    if (val > devmax)
                        devmax = val;
                }
            }
        }
    }
    else
    {
        for (int np = 0; np < size; np++)
        {
            pt = pcl->at(np);
            if (ito::isFinite<ito::float32>(pt.z))
            {
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
            devmax = 0.0f;
            devmin = 0.0f;
    }

    this->m_pclMinX[id] = xmin;
    this->m_pclMinY[id] = ymin;
    this->m_pclMinZ[id] = zmin;
    this->m_pclMinDev[id] = cv::saturate_cast<ito::float64>(devmin);
    this->m_pclMaxX[id] = xmax;
    this->m_pclMaxY[id] = ymax;
    this->m_pclMaxZ[id] = zmax;
    this->m_pclMaxDev[id] = cv::saturate_cast<ito::float64>(devmax);
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
/** Set new overlay image for a topography
*
*   @param [in] overlayImage    Image data
*   @param [in] objectID        internal id of topography
*
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   This method sets a new overlay image for a topography. The image is only accepted if it
*   has the same size as the addressed topography. The uploading is done by the method
*   \ref{updateOverlayImage}. If everything went well the slider for changing between
*   color coded height and overlay image is displayed.
*/
ito::RetVal TwipOGLWidget::setOverlayImage(QSharedPointer< ito::DataObject > overlayImage, const int objectID)
{
    ito::RetVal retval(ito::retOk);

    // some basic checks
    if (m_pContentDObj[objectID]
        && !overlayImage.isNull()
        && overlayImage->getDims() >= 2
        && overlayImage->getSize(0) == m_pContentDObj[objectID]->getSize(0)
        && overlayImage->getSize(1) == m_pContentDObj[objectID]->getSize(1))
    {
        //m_overlayImage = *(overlayImage.data());
        m_overlayImage.insert(objectID, QSharedPointer<ito::DataObject>(overlayImage));
        switch(m_overlayImage[objectID]->getType())
        {
            case ito::tUInt8:
                retval += updateOverlayImage<ito::uint8>(objectID);
            break;
            case ito::tInt8:
                retval += updateOverlayImage<ito::int8>(objectID);
            break;
            case ito::tUInt16:
                retval += updateOverlayImage<ito::uint16>(objectID);
            break;
            case ito::tInt16:
                retval += updateOverlayImage<ito::int16>(objectID);
            break;
            case ito::tUInt32:
                retval += updateOverlayImage<ito::uint32>(objectID);
            break;
            case ito::tInt32:
                retval += updateOverlayImage<ito::int32>(objectID);
            break;
            case ito::tFloat32:
                retval += updateOverlayImage<ito::float32>(objectID);
            break;
            case ito::tFloat64:
                retval += updateOverlayImage<ito::float64>(objectID);
            break;

            /*
            case ito::tComplex64:
                updateOverlayImage<ito::complex64>(objectID);
            break;
            case ito::tComplex128:
                updateOverlayImage<ito::complex128>(objectID);
            break;
            */
            default:
                retval = ito::RetVal(ito::retError, 0, tr("Object has invalid type").toLatin1().data());
        }
        if (!retval.containsWarningOrError())
        {
//            if(m_pValuePicker) m_pValuePicker->enableOverlay(m_pConfigData->m_alpha > 0);
            TwipOGLFigure *p = (TwipOGLFigure*)(this->parent());
            p->enableOverlaySlider(true);
            m_pConfigData->m_enableOverlay = true;
            update();
            m_errorDisplMsg.append(retval.errorMessage());
        }
    }
    else
    {
        m_pConfigData->m_enableOverlay = false;
        TwipOGLFigure *p = (TwipOGLFigure*)(this->parent());
        p->enableOverlaySlider(m_pConfigData->m_enableOverlay);
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Set new invalid map for a topography
*
*   @param [in] overlayImage    Image data
*   @param [in] objectID        internal id of topography
*
*   @return     returns ito::retOk on success otherwise ito::retError
*
*   This method sets a new invalid map for a topography. The image is only accepted if it
*   has the same size as the addressed topography.
*/
ito::RetVal TwipOGLWidget::setInvalidImage(QSharedPointer< ito::DataObject > invalidImage, const int objectID)
{
    ito::RetVal retval(ito::retOk);

    // some basic checks
    if (!m_pContentDObj[objectID])
    {
        retval = ito::RetVal(ito::retError, 0, tr("No plot object").toLatin1().data());
    }
    else if(invalidImage.isNull())
    {
//         m_invalidMap = ito::DataObject();
		 m_invalidMap.remove(objectID);
         if(m_pConfigData) ((InternalData*) m_pConfigData)->m_showInvalids = false;
    }
    else if((invalidImage->getType() == ito::tUInt8)
        && invalidImage->getDims() == 2
        && invalidImage->getSize(0) == m_pContentDObj[objectID]->getSize(0)
        && invalidImage->getSize(1) == m_pContentDObj[objectID]->getSize(1))
    {
        m_invalidMap.insert(objectID, QSharedPointer<ito::DataObject>(invalidImage.data()));
        //m_invalidMap = *(invalidImage.data());
        //m_invalidMap.zeros(m_pContentDObj->getSize(0), m_pContentDObj->getSize(1), ito::tInt8);
        if(m_pConfigData) ((InternalData*) m_pConfigData)->m_showInvalids = true;
        m_forceReplot = true;
        refreshPlot(NULL);
        m_forceReplot = false;
    }
    else
    {
        retval = ito::RetVal(ito::retError, 0, tr("Object has invalid type or size").toLatin1().data());
    }

    if (!retval.containsWarningOrError())
    {
        m_errorDisplMsg.append(retval.errorMessage());
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> cv::Mat* TwipOGLWidget::newMatFromComplex(const cv::Mat* inData)
{
    cv::error(cv::Exception(CV_StsAssert, "newMatFromComplex only defined for complex type", "", __FILE__, __LINE__));
    return new cv::Mat(inData->rows, inData->cols, inData->type() );
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> cv::Mat* TwipOGLWidget::newMatFromComplex<ito::complex64>(const cv::Mat* inData)
{
    cv::Mat *retMat = new cv::Mat(inData->rows, inData->cols, CV_32F);

    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    const ito::complex64 *ptrSrc = NULL;
    ito::float32 *ptrDst = NULL;

    switch(m_pConfigData->m_cmplxMode)
    {
        default:
        case tAbs:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex64>(cntY);
            ptrDst = retMat->ptr<ito::float32>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = abs(ptrSrc[cntY]);
            }
        }
        break;

        case tImag:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex64>(cntY);
            ptrDst = retMat->ptr<ito::float32>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = ptrSrc[cntY].imag();
            }
        }
        break;

        case tReal:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex64>(cntY);
            ptrDst = retMat->ptr<ito::float32>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = ptrSrc[cntY].real();
            }
        }
        break;

        case tPhase:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex64>(cntY);
            ptrDst = retMat->ptr<ito::float32>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = arg(ptrSrc[cntY]);
            }
        }
        break;
    }
    #if (USEOMP)
    }
    #endif

    return retMat;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> cv::Mat* TwipOGLWidget::newMatFromComplex<ito::complex128>(const cv::Mat* inData)
{
    cv::Mat *retMat = new cv::Mat(inData->rows, inData->cols, CV_32F);

    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    const ito::complex128 *ptrSrc = NULL;
    ito::float64 *ptrDst = NULL;

    switch(m_pConfigData->m_cmplxMode)
    {
        default:
        case tAbs:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex128>(cntY);
            ptrDst = retMat->ptr<ito::float64>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = abs(ptrSrc[cntY]);
            }
        }
        break;

        case tImag:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex128>(cntY);
            ptrDst = retMat->ptr<ito::float64>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = ptrSrc[cntY].imag();
            }
        }
        break;

        case tReal:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex128>(cntY);
            ptrDst = retMat->ptr<ito::float64>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = ptrSrc[cntY].real();
            }
        }
        break;

        case tPhase:
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (int cntY = 0; cntY < inData->rows; cntY++)
        {
            ptrSrc = inData->ptr<const ito::complex128>(cntY);
            ptrDst = retMat->ptr<ito::float64>(cntY);
            for (int cntX = 0; cntX < inData->cols; cntX++)
            {
                ptrDst[cntX] = arg(ptrSrc[cntY]);
            }
        }
        break;
    }
    #if (USEOMP)
    }
    #endif

    return retMat;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**Set the alpha value for a certain plane
*
*   @param [in] index    index of plane on hash table or -1 for all
*   @param [in] alpha    new alpha value
*
*
*/
void TwipOGLWidget::setPlaneAlpha(int idx, int alpha)
{
    if(idx == -1)
    {
        int index;
        foreach(index, m_transperency.keys())
        {
            m_transperency[index] = cv::saturate_cast<ito::uint8>(alpha);
        }
    }
    else if(m_transperency.contains(idx))
    {
        m_transperency[idx] = cv::saturate_cast<ito::uint8>(alpha);
    }
    update();
    repaint();
}
//----------------------------------------------------------------------------------------------------------------------------------
/**Set the alpha value for a certain plane
*
*   @param [in] index    index of plane on hash table
*   @param [in] alpha    new alpha value
*
*
*/
void TwipOGLWidget::setPlaneVisState(int idx, bool state)
{
    if(idx == -1)
    {
        int index;
        foreach(index, m_transperency.keys())
        {
            m_enabledHash[index] = state;
        }
    }
    else if(m_enabledHash.contains(idx))
    {
        m_enabledHash[idx] = state;
    }
    update();
    repaint();
}


//----------------------------------------------------------------------------------------------------------------------------------
/** Get displayed frame buffer for copying / printing
*
*   @param [in / out] img    frame buffer as QImage
*   @param [in] oversampling size for frame buffer
*
*   Grabs the currently displayed object(s) into a QImage for copy or printing purpose. An oversamling
*   factor between 1 (window resolution) and 4 (4 x window resolution) can be set for higher quality
*   images, e.g. for presentations or printing.
*
*/
void TwipOGLWidget::getFrameBuffer(QImage &img, const int oversampling)
{
    bool plotAngles = m_pConfigData->m_plotAngles;

    m_pConfigData->m_plotAngles = false;
    //!> only screen copies between 1 to 4 times the window resolution are allowed
    int resFaktor = oversampling > 2 ? 2 : oversampling < 1 ? 1 : oversampling;
//    QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);

    //!> if we have a oversampling factor larger then one the framegrabbing
    //!> is done in various steps
    if (resFaktor > 1)
    {
        int axisTextSize = m_axes.getFontSize();
        int titleTextSize = m_titleFont.getFontSize();

        m_axes.setFontSize(axisTextSize*resFaktor);

        ViewZoomer tempZoomer = m_zoomer;

#if QT_VERSION >= 0x050400
		QImage tempimg = grabFramebuffer();
#else
        QImage tempimg = grabFrameBuffer();
#endif
		QSize curSize = tempimg.size();

        double increase = resFaktor < 4 ? resFaktor : 4;
        double delta = (resFaktor - 1.0) / resFaktor;
        delta = std::min(0.75, delta);

        //this->setCanvasZoomView(increase);
        m_zoomer.scalexy *= increase;
        m_zoomer.enabled = true;

        QSize myRect(curSize.width() * increase, curSize.height() * increase);

        img = QImage(myRect, QImage::Format_ARGB32);
        QSize tempSize;

        double xTrans = 0.0;
        double yTrans = 0.0;

        bool tempDrawTitle = m_pConfigData->m_drawTitle;
        m_pConfigData->m_drawTitle = false;
        char tempBarMode = m_pConfigData->m_colorBarMode;
        m_pConfigData->m_colorBarMode = COLORBAR_NO;
        bool tempInfoShow = m_objectInfo.show;
        m_objectInfo.show = false;

        //!> apply oversampling factor in x and y direction. Therefore shift and zoom scene
        //!> and then copy the parts together
        for (int y = 0; y < increase; y++)
        {
            yTrans = y / (double)(resFaktor - 1) - delta;
            yTrans /= tempZoomer.scalexy;
            for (int x = 0; x < increase; x++)
            {
                xTrans = - x / (double)(resFaktor - 1) + delta;
                xTrans /= tempZoomer.scalexy;
                setViewTranslation(xTrans, yTrans, 0.0);

                update();
                repaint();
//                QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
#if QT_VERSION >= 0x050400
				QImage tempimg = grabFramebuffer();
#else
				QImage tempimg = grabFrameBuffer();
#endif
                tempSize = tempimg.size();
                for(int row = 0; row < tempSize.height(); row ++)
                {
                    memcpy(&(img.scanLine(row + y * curSize.height())[x * curSize.width() * tempimg.depth() / 8 ]) , tempimg.scanLine(row), curSize.width() * tempimg.depth() / 8);
                }
            }
        }

        m_zoomer = tempZoomer;


        m_pConfigData->m_drawTitle = tempDrawTitle;
        m_pConfigData->m_colorBarMode = tempBarMode;
        m_objectInfo.show = tempInfoShow;

        // okay add a new color bar
        if((m_pConfigData->m_colorBarMode != COLORBAR_NO) || m_pConfigData->m_drawTitle || m_objectInfo.show)
        {
            FontStyle curFond = m_titleFont;

            QPainter painter;
            painter.begin(&img);

            if(m_pConfigData->m_drawTitle)
            {
                painter.setPen(curFond.getFontColor()); // The font color comes from user select on a
                painter.setFont(QFont(QString::fromStdString(curFond.getFontName()), curFond.getFontSize() * resFaktor)); // The font size comes from user
                int estimatedWidth = m_pConfigData->m_title.length() * curFond.getFontSize() * resFaktor;
                QRect titleRect(std::max(0 , (myRect.width() - estimatedWidth )/2), curFond.getFontSize(), estimatedWidth, curFond.getFontSize() * resFaktor * 2);

                painter.drawText(titleRect, m_pConfigData->m_title, QTextOption(Qt::AlignBottom | Qt::AlignHCenter));
            }

            if(m_pConfigData->m_colorBarMode != COLORBAR_NO)
            {
                //QImage colorBar(15 * resFaktor, , QImage::Format_ARGB32);
                //colorBar.setColorTable(m_currentPalette);
                int x0 = 0;
                int y0 = 0;
                int rows = (int)std::max(3.0, myRect.height() * 0.3);
                int cols = (15 * resFaktor);
                Qt::AlignmentFlag align = Qt::AlignLeft;

                switch(m_pConfigData->m_colorBarMode)
                {
                    case COLORBAR_LEFT:
                    {
                        x0 = 10.0;
                        y0 = (myRect.height() - rows)/ 2;
                    }
                    break;
                    case COLORBAR_RIGHT:
                    {
                        x0 = (myRect.width() - cols - 15);
                        y0 = (myRect.height() - rows)/ 2;
                        align = Qt::AlignRight;
                    }
                    break;
                    case COLORBAR_UPPER_RIGHT:
                    {
                        x0 = (myRect.width() - cols - 15);
                        y0 = m_axes.m_fontSize * 3;
                        align = Qt::AlignRight;
                    }
                    break;
                    default: // do nothing
                    break;
                }

                painter.setPen(m_axes.getFontColor()); // The font color comes from user select on a

                //painter.drawImage(0, 0, colorBar, 0, 0, -1, -1);

                painter.drawRect(x0-1, y0-1, cols+1, rows+1);
                painter.drawRect(x0, y0, cols, rows);
                if(m_currentPalette.size() > 1)
                {

                    for(int row = 0; row < rows - 1; row++)
                    {
                        int color = (1.0 - row / (float)rows) * m_currentPalette.size();
                        color = std::max(0, color);
                        color = std::min(color, (int)m_currentPalette.size() - 1);

                        for(int col = 0; col < cols; col++)
                        {
                            img.setPixel(col + x0, y0 + row, m_currentPalette[color] | 0xFF000000);
                        }
                    }
                    for(int col = 0; col < cols; col++)
                    {
                        img.setPixel(col + x0, y0 + rows - 1, m_currentPalette[0] | 0xFF000000);
                    }

                }

                int curFontSize = m_axes.getFontSize() / 1.5;
                painter.setPen(m_axes.getFontColor()); // The font color comes from user select on a
                painter.setFont(QFont(QString::fromStdString(m_axes.getFontName()), curFontSize )); // The font size comes from user

                bool useDev = false;
                if(this->m_pConfigData)
                {
                    useDev = this->m_pConfigData->m_showCurvature;
                }
                const float zMin = useDev ? m_axes.m_devAxis.getMin() : m_axes.m_axisZ.getMin();
                const float zMax = useDev ? m_axes.m_devAxis.getMax() : m_axes.m_axisZ.getMax();

                QString upperText;
                if(useDev)
                {
                    upperText = QString("%1 %2\n%3").arg(QString::fromStdString(m_axes.m_devAxis.getLabel()),
                                                                 QString::fromStdString(m_axes.m_devAxis.getUnit()),
                                                                 QString::number(zMax, 'g', 4));

                }
                else
                {
                    upperText = QString("%1 %2\n%3").arg(QString::fromStdString(m_axes.m_axisZ.getLabel()),
                                                                 QString::fromStdString(m_axes.m_axisZ.getUnit()),
                                                                 QString::number(zMax, 'g', 4));
                }

                int estimatedWidth = upperText.length() * curFontSize;
                QRect upperRect(x0, y0 - curFontSize * 3, estimatedWidth, curFontSize * 3);
                if(align == Qt::AlignRight) upperRect.moveRight(x0 + cols);
                painter.drawText(upperRect, upperText, QTextOption(Qt::AlignBottom | align));

                QString lowerText = QString::number(zMin, 'g', 4);

                QRect lowerRect = QRect(x0, y0 + rows, estimatedWidth, curFontSize * 2);
                if(align == Qt::AlignRight) lowerRect.moveRight(x0 + cols);
                painter.drawText(lowerRect, lowerText, QTextOption(Qt::AlignTop | align));
            }

            if(m_objectInfo.show) // Paint the object info if enabled
            {
                int curFontSize = m_axes.getFontSize() / 1.5;
                int rows = 0;
                int curLen = 0;
                int maxLen = 0;
                painter.setPen(m_axes.getFontColor()); // The font color comes from user select on a
                painter.setFont(QFont(QString::fromStdString(m_axes.getFontName()), curFontSize )); // The font size comes from user

                QString buf("");


                if(curLen = m_objectInfo.MeanText.length())
                {
                    rows++;
                    maxLen = std::max(curLen, maxLen);

                    buf.append(QString::fromStdString(m_objectInfo.MeanText.data()));
                }

                if(curLen = m_objectInfo.DevText.length())
                {
                    rows++;
                    maxLen = std::max(curLen, maxLen);
                    buf.append("\n");
                    buf.append(QString::fromStdString(m_objectInfo.DevText.data()));
                }

                if(curLen = m_objectInfo.PeakText.length())
                {
                    rows++;
                    maxLen = std::max(curLen, maxLen);
                    buf.append("\n");
                    buf.append(QString::fromStdString(m_objectInfo.PeakText.data()));
                }

                if(curLen = m_objectInfo.xLength.length())
                {
                    rows += 2;
                    maxLen = std::max(curLen, maxLen);
                    buf.append("\n\n");
                    buf.append(QString::fromStdString(m_objectInfo.xLength));
                }

                if(curLen = m_objectInfo.yLength.length())
                {
                    rows++;
                    maxLen = std::max(curLen, maxLen);
                    buf.append("\n");
                    buf.append(QString::fromStdString(m_objectInfo.yLength));
                }

                if(curLen = m_objectInfo.matrix.length())
                {
                    rows++;
                    maxLen = std::max(curLen, maxLen);
                    buf.append("\n");
                    buf.append(QString::fromStdString(m_objectInfo.matrix));
                }

                if(rows > 0)
                {
                    int estimatedWidth = maxLen * curFontSize ;
                    QRect infoRect(0, 0, estimatedWidth, (rows + 2) * curFontSize * 1.5);
                    infoRect.moveBottomRight(QPoint(img.width() - 10,img.height() - 10));
                    painter.drawText(infoRect, buf, QTextOption(Qt::AlignBottom | Qt::AlignRight));
                }
            }
            painter.end();

        }

        m_axes.setFontSize(axisTextSize);
    }
    else
    {
#if QT_VERSION >= 0x050400
		img = grabFramebuffer();
#else
		img = grabFrameBuffer();
#endif
    }


    m_pConfigData->m_plotAngles = plotAngles;

    update();
    repaint();
//    QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);

    return;
}
#if USEWIDGETEVENTS

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::contextMenuEvent(QContextMenuEvent * event)
{
    /*if (m_showContextMenu)
    {
        event->accept();
        m_contextMenu->exec(event->globalPos());
    }
    else
    {
        event->ignore();
    }*/
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Handle keyboard events
*
*   @param [in] event    Keyboard event
*
*   Handle keyboard events for 3d window.
*/
void TwipOGLWidget::keyPressEvent (QKeyEvent * event)
{
    TwipOGLFigure *p = (TwipOGLFigure*)(this->parent());
    //m_activeModifiers = event->modifiers();

    if(event->matches(QKeySequence::Copy))
    {
        p->copyToClipBoard();
    }
    else
    {
        switch(event->key())
        {
            case Qt::Key_H:
            {
                homeView();
                repaint();
            }
            break;

            case Qt::Key_W:
            case Qt::Key_Up:
            {
                m_pConfigData->m_rollAng -= 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_rollAng);
                repaint();
            }
            break;

            case Qt::Key_S:
            case Qt::Key_Down:
            {
                m_pConfigData->m_rollAng += 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_rollAng);
                repaint();
            }
            break;

            case Qt::Key_D:
            case Qt::Key_Right:
            {
                m_pConfigData->m_yawAng += 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_yawAng);
                repaint();
            }
            break;

            case Qt::Key_A:
            case Qt::Key_Left:
            {
                m_pConfigData->m_yawAng -= 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_yawAng);
                repaint();
            }
            break;

            case Qt::Key_Q:
            {
                m_pConfigData->m_pitchAng -= 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_pitchAng);
                repaint();
            }
            break;

            case Qt::Key_E:
            {
                m_pConfigData->m_pitchAng += 0.05f;
                TwipOGLWidget::normalizeAngle(m_pConfigData->m_pitchAng);
                repaint();
            }
            break;

            // The following keys represent a direction, they are
            // organized on the keyboard.
            case Qt::Key_V:
            {

            }
            break;

            case Qt::Key_Control:
            {
                m_activeModifiers = Qt::ControlModifier;
            }
            break;

            default:
                event->ignore();
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::keyReleaseEvent (QKeyEvent * event)
{
    m_activeModifiers = Qt::NoModifier;
    event->ignore();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Handle mouse events
*
*   @param [in] event    Keyboard event
*
*   Handle keyboard mouse for 3d window.
*/
void TwipOGLWidget::mouseMoveEvent (QMouseEvent * event)
{
    if(m_pConfigData->m_state & tRotating)
    {
        float dyaw = (QCursor::pos().x() - m_mouseStartPos.x()) / 200.0;
        float dpitch = (QCursor::pos().y() - m_mouseStartPos.y()) / 200.0;
        float droll = 0;

        m_mouseStartPos = QCursor::pos();

        if (m_pConfigData->m_drawLightDir)
        {
            m_pConfigData->m_lightDirPitch += dpitch;
            m_pConfigData->m_lightDirYaw += dyaw;
            TwipOGLWidget::normalizeAngle(m_pConfigData->m_lightDirPitch);
            TwipOGLWidget::normalizeAngle(m_pConfigData->m_lightDirYaw);
        }
        else
        {
            m_pConfigData->m_pitchAng += dpitch;
            m_pConfigData->m_yawAng += dyaw;
            TwipOGLWidget::normalizeAngle(m_pConfigData->m_yawAng);
            TwipOGLWidget::normalizeAngle(m_pConfigData->m_pitchAng);
        }
    }
    else if(m_pConfigData->m_state & tMoving)
    {
        m_zoomer.enabled = true;
        float dx = (QCursor::pos().x() - m_mouseStartPos.x()) / 200.0f;
        float dy = - (QCursor::pos().y() - m_mouseStartPos.y()) / 200.0f;
        float dz = 0.0f;

        m_mouseStartPos = QCursor::pos();
        moveView(dx, dy, dz);

    }
    else
    {
        event->ignore();
        return;
    }

    repaint();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::mousePressEvent (QMouseEvent * event)
{
    if(m_activeModifiers == Qt::ControlModifier) m_pConfigData->m_state |= tMoving;
    else m_pConfigData->m_state |= tRotating;

    m_mouseStartPos = QCursor::pos();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::mouseReleaseEvent (QMouseEvent * event)
{
    m_pConfigData->m_state &= ~ (tRotating | tMoving);

    repaint();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::wheelEvent ( QWheelEvent * event )
{
    if (m_pConfigData->m_state != tIdle && m_activeModifiers != Qt::ControlModifier)
    {
        event->ignore();
        return;
    }

    if(m_activeModifiers == Qt::ControlModifier)
    {
        if (event->angleDelta().y() > 0)
        {
            zoomInByOne();
        }
        else
        {
            zoomOutByOne();
        }
    }
    else
    {
        if (event->angleDelta().y() > 0)
        {
            m_pConfigData->m_zAmpl *= 1.05;
        }
        else
        {
            m_pConfigData->m_zAmpl *= 0.95;
        }
        validateZAmplification();
        update();
    }
    repaint();
    return;
}

#endif
//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLWidget::oglAboutToDestroy()
{
    doCleanUp();
}

//----------------------------------------------------------------------------------------------------------------------------------
