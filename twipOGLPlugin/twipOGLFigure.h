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
#    Foundation. See <https://bitbucket.org/itom/> 
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef TWIPOGLFIGURE_H
#define TWIPOGLFIGURE_H

#include "plot/AbstractDObjPCLFigure.h"
#include "twipOGLWidget.h"
#include "twipOGLLegend.h"
#include "QPixmap"
#include "QColor"
#include "qlabel.h"

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
//#include <QtWidgets/qlabel.h>
#endif

//Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)

#if !USEWIDGETEVENTS

class TwipOGLFigure;
//----------------------------------------------------------------------------------------------------------------------------------
class OGLEventFilter : public QObject
{
    Q_OBJECT

    public:
        OGLEventFilter(TwipOGLFigure *plotObj)
            : m_plotObj(plotObj), rotating(false), moving(false) {memset(startPos, 0, sizeof(double)*6); memset(endPos, 0, sizeof(double)*6); }
        ~OGLEventFilter() {}
        virtual bool eventFilter(QObject *, QEvent *);
        virtual bool event(QEvent *);
        TwipOGLFigure *m_plotObj;

    private:
        double startPos[6];
        double endPos[6];

        bool rotating;
        bool moving;

    signals:

    public slots:

    private slots:

};
#endif

//----------------------------------------------------------------------------------------------------------------------------------
class TwipOGLFigure : public ito::AbstractDObjPclFigure
{
    Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel USER true)
    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible USER true)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel USER true)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible USER true)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped USER true)
    Q_PROPERTY(double zAxisAmplification READ getZAmplification WRITE setZAmplification RESET resetZAmplification USER true DESIGNABLE true) 
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(bool valueAxisVisible  READ getVAxisVisible WRITE setVAxisVisible USER true)
    Q_PROPERTY(bool colorBarVisible READ getColorBarVisible WRITE setColorBarVisible DESIGNABLE true USER true)
    Q_PROPERTY(QString colorMap READ getColorMap WRITE setColorMap DESIGNABLE true USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    //Q_PROPERTY(QSharedPointer< ito::DataObject > geometricElements READ getGeometricElements WRITE setGeometricElements DESIGNABLE false)
    //Q_PROPERTY(int geometricElementsCount READ getGeometricElementsCount DESIGNABLE false)
    Q_PROPERTY(bool keepAspectRatio READ getkeepAspectRatio WRITE setkeepAspectRatio USER true)
    Q_PROPERTY(bool showTriangles READ getShowTriangles WRITE setShowTriangles USER true)
    Q_PROPERTY(bool showAngleInfo READ getShowAngleInfo WRITE setShowAngleInfo USER true)

    //Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    //Q_PROPERTY(bool showCenterMarker READ getEnabledCenterMarker WRITE setEnabledCenterMarker USER true)
    //Q_PROPERTY(int selectedGeometry READ getSelectedElement WRITE setSelectedElement DESIGNABLE false)

    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > overlayImage READ getOverlayImage WRITE setOverlayImage RESET resetOverlayImage DESIGNABLE false)
    Q_PROPERTY(int overlayAlpha READ getOverlayAlpha WRITE setOverlayAlpha RESET resetOverlayAlpha USER true)
    Q_PROPERTY(ito::AutoInterval overlayInterval READ getOverlayInterval WRITE setOverlayInterval DESIGNABLE true USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > invalidMap READ getInvalidMap WRITE setInvalidMap RESET resetInvalidMap DESIGNABLE false)
    Q_PROPERTY(bool fillInvalids READ getFillInvalidsStatus WRITE setFillInvalidsStatus USER true)
    Q_PROPERTY(QColor invalidColor READ getInvalidColor WRITE setInvalidColor RESET resetInvalidColor USER true)

    Q_PROPERTY(bool enableIllumination READ getIllumunationEnabled WRITE setIllumunationEnabled DESIGNABLE true USER true)
    Q_PROPERTY(QVector<float> rotationAngle READ getRotationAngle WRITE setRotationAngle DESIGNABLE true USER true)
    Q_PROPERTY(QVector<float> lightDirection READ getRotationLightAngle WRITE setRotationLightAngle DESIGNABLE true USER true)

    Q_PROPERTY(bool curvatureToColor READ getCurvature2Color WRITE setCurvature2Color DESIGNABLE true USER true)

    Q_PROPERTY(ito::AutoInterval curvatureInterval READ getCurvatureInterval WRITE setCurvatureInterval DESIGNABLE true USER true)

    Q_PROPERTY(QVector<int> planeIndices READ getPlaneIndices)

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://xAxisVisible", "Sets visibility of the x-axis.")
    Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://yAxisVisible", "Sets visibility of the y-axis.")
    Q_CLASSINFO("prop://yAxisFlipped", "Sets whether y-axis should be flipped (default: false, zero is at the bottom).")
    Q_CLASSINFO("prop://ZAxisAmplification", "Sets an amplification factor to the z-value, to ease the visualisation of flat or high aspect objects.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://valuesAxisVisible", "Sets visibility of the value-axis / the z-axis.")
    Q_CLASSINFO("prop://colorBarVisible", "Defines whether the color bar should be visible.")
    Q_CLASSINFO("prop://colorMap", "Defines which color map should be used [e.g. grayMarked, hotIron].")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    //Q_CLASSINFO("prop://geometricElements", "Geometric elements defined by a float32[11] array for each element.")
    //Q_CLASSINFO("prop://geometricElementsCount", "Number of currently existing geometric elements.")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://showTriangles", "Toggle between single point or triangle rendering. Points should be chosen for performance reasons.")
    Q_CLASSINFO("prop://showAngleInfo", "Toggle visibility of rotation angles.")
    //Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")
    //Q_CLASSINFO("prop://showCenterMarker", "Enable a marker for the center of a data object.")
    //Q_CLASSINFO("prop://selectedGeometry", "Get or set the currently highlighted geometric element. After manipulation the last element stays selected.")

    Q_CLASSINFO("prop://backgroundColor", "Set the background / canvas color.")
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers")

    Q_CLASSINFO("prop://overlayImage", "Set an overlay which is shown as a black&white image.")
    Q_CLASSINFO("prop://overlayAlpha", "Changes the value of the overlay channel")
    Q_CLASSINFO("prop://overlayInterval", "Range of the overlayInterval to scale the values")

    Q_CLASSINFO("prop://invalidMap", "Set a map (uint8) for invalid masking with the defined invalid color.")
    Q_CLASSINFO("prop://fillInvalids", "toggle between invalid visbility mode")
    Q_CLASSINFO("prop://invalidColor", "Set the color for invalid marked pixels")

    Q_CLASSINFO("prop://enableIllumination", "Enable the directed light illumination rendering. Only for triangles or pointClouds with normal vectors")
    Q_CLASSINFO("prop://rotationAngle", "Set the rotation angle of the object in degree")
    Q_CLASSINFO("prop://lightDirection", "Set the rotation angle of the illumination in degree")

    Q_CLASSINFO("prop://curvatureToColor", "Set the color from the curvature and not from the z heigth")
    Q_CLASSINFO("prop://curvatureInterval", "Range of the curvature interval to scale the values")

    DESIGNER_PLUGIN_ITOM_API
    public:
#if !USEWIDGETEVENTS
        friend class OGLEventFilter;
#endif
        friend class TwipOGLWidget;

        /** Constructor with settings 
        *   @param [in] itomSettingsFile    General (user) settings file of itom
        *   @param [in] inpType             
        *   @param [in] windowMode          mode for opening widget (Standalone, window, widget)
        *   @param [in] parent              parent for widget
        */
        TwipOGLFigure(const QString &itomSettingsFile, const ito::ParamBase::Type inpType, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        ~TwipOGLFigure();

        //properties
        /** show linecut - not implemented
        *   @param [in] bounds   boundaries for linecut
        *   @param [in / out] uniqueID  identifier for linecut
        *   \brief actually not implemented
        */
        ito::RetVal displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID);

        /** update plot
        *   \brief trigger an immediate plot update
        */
        ito::RetVal applyUpdate();  //!> does the real update work

        //!> return plot source (dataObject only) - not implemented
        QSharedPointer<ito::DataObject> getSource(void) const;

        //!> return currently displayed dataObject - not implemented
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

        //!> enable button for switching complex mode
        void enableComplexGUI(const bool checked);

        //!> enable button for illumination settings
        void enableIlluGUI(const bool checked);

        //!> enable button for cycling through the planes of a multiplane dataObject
        void enableZStackGUI(const bool checked);

        //!> read the current z amplification factor
        double getZAmplification() const;
        
        //!> set a new z amplification factor to the plot
        void setZAmplification(const double value);

        //!> reset the z amplification factor to 1.0
        void resetZAmplification();

        void setLinePlotCoordinates(const QVector<QPointF> pts);
#if !USEWIDGETEVENTS
        OGLEventFilter *m_pEventFilter;
#endif
        //!> return interval of z-axis
        //QPointF getZAxisInterval(void) const;
        ito::AutoInterval getZAxisInterval(void) const;

        //!> set interval for z-axis - not implemented
        //void setZAxisInterval(QPointF);
        void setZAxisInterval(ito::AutoInterval);

        //!> return current colormap
        QString getColorMap(void) const;
        
        /** set color map
        *   @param [in] colormap    new colormap
        */
        void setColorMap(QString colormap);

        /** enable / disable context menu
        *   @param [in] show    bool flag to enable / disable context menu
        */
        void setContextMenuEnabled(bool show);
        
        //!> get state of context menu (enabled / disabled)
        bool getContextMenuEnabled() const;

        //!> return plot title
        QString getTitle() const;
        /** set plot title
        *   @param [in] title   new title
        */
        void setTitle(const QString &title);

        //!> empty title and set to automatic mode, i.e. retrieve from object if possible
        void resetTitle();

        //!> return label of x-axis
        QString getxAxisLabel() const 
        { 
            return QString::fromStdString(m_pContent->m_axes.m_axisX.m_label); 
        }

        /** set x-axis label
        *   @param [in] value   new x-axis label
        */
        void setxAxisLabel(const QString value, const int fromID = 0) 
        { 
            if (value != "<auto>")
                m_pContent->m_axes.m_axisX.m_label = std::string(value.toLatin1().data());
            else
            {
                if (m_pContent->m_pContentDObj[fromID] != NULL)
                {
                    bool test;
                    int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                    m_pContent->m_axes.m_axisX.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims - 1, test);
                }
                else
                {
                    m_pContent->m_axes.m_axisX.m_label = "x";
                }
            }
            return;
        }

        /** reset x-axis label
        *   set to automatic label generation i.e. from object physical properties
        */
        void resetxAxisLabel(const int fromID = 0) 
        { 
            if ( m_pContent->m_pContentDObj[fromID] != NULL )
            {
                bool test;
                int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                m_pContent->m_axes.m_axisX.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims - 1, test);
            }
            else
            {
                m_pContent->m_axes.m_axisX.m_label = "x";
            }
            return; 
        }

        bool getxAxisVisible() const;
        void setxAxisVisible(const bool value);

        //!> return label of y-axis
        QString getyAxisLabel() const 
        {
            return QString::fromStdString(m_pContent->m_axes.m_axisY.m_label);
        }

        /** set y-axis label
        *   @param [in] value   new y-axis label
        */
        void setyAxisLabel(const QString value, const int fromID = 0) 
        { 
            if (value != "<auto>")
                m_pContent->m_axes.m_axisY.m_label = std::string(value.toLatin1().data());
            else
            {
                if (m_pContent->m_pContentDObj[fromID] != NULL)
                {
                    bool test;
                    int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                    m_pContent->m_axes.m_axisY.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims - 2, test);
                }
                else
                {
                     m_pContent->m_axes.m_axisY.m_label = "y";
                }
            }
            return; 
        }

        /** reset y-axis label
        *   set to automatic label generation i.e. from object physical properties
        */
        void resetyAxisLabel(const int fromID = 0) 
        {
            if (m_pContent->m_pContentDObj[fromID] != NULL)
            {
                bool test;
                int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                m_pContent->m_axes.m_axisY.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims - 2, test);
            }
            else
            {
                    m_pContent->m_axes.m_axisY.m_label = "y";
            }
            return;
        }

        bool getyAxisVisible() const;
        void setyAxisVisible(const bool value);

        bool getyAxisFlipped() const { return m_pContent->m_axes.m_axisY.m_isflipped; }
        void setyAxisFlipped(const bool value);

        //!> get label of z-axis
        QString getValueLabel() const 
        {
            return QString::fromStdString(m_pContent->m_axes.m_axisZ.m_label);
        }

        /** set label of z-axis
        *   @param [in] value   set new label for z-axis
        */
        void setValueLabel(const QString value, const int fromID = 0) 
        {
            if (value != "<auto>")
                m_pContent->m_axes.m_axisZ.m_label = std::string(value.toLatin1().data());
            else
            {
                if (m_pContent->m_pContentDObj[fromID] != NULL)
                {
                    bool test;
                    int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                    m_pContent->m_axes.m_axisZ.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims, test);
                }
                else
                {
                        m_pContent->m_axes.m_axisZ.m_label = "z";
                }
            }
            return;
        }

        /** reset z-axis label
        *   set to automatic label generation i.e. from object physical properties
        */
        void resetValueLabel(const int fromID = 0) 
        {
            if (m_pContent->m_pContentDObj[fromID] != NULL)
            {
                bool test;
                int dims = m_pContent->m_pContentDObj[fromID]->getDims();
                m_pContent->m_axes.m_axisZ.m_label = m_pContent->m_pContentDObj[fromID]->getAxisDescription(dims, test);
            }
            else
            {
                    m_pContent->m_axes.m_axisZ.m_label = "z";
            }
            return;
        }

        bool getVAxisVisible() const;
        void setVAxisVisible(const bool value);

        //!> return if colorbar is visible
        bool getColorBarVisible() const; 

        //!> enable colorbar displaying
        void setColorBarVisible(const bool value); 

        //!> return font used for title
        QFont getTitleFont() const;

        /** set new title font
        *   @param [in] value   new font used for title
        */
        void setTitleFont(const QFont value);

        //!> return font used for labels
        QFont getLabelFont() const 
        {
            return QFont();
        }

        /** set new label font
        *   @param [in] value   new font used for labels
        */
        void setLabelFont(const QFont value) 
        {
            return;
        }

        //!> return font used for axis
        QFont getAxisFont() const;

        /** set new axis font
        *   @param [in] value   new font used for axis, i.e. numbers 
        */
        void setAxisFont(const QFont value);

        //!> return if isometric view is used, i.e. fixed ratio between x, y and z
        bool getkeepAspectRatio() const;

        //!> set isometric view, i.e. fixed ratio between x, y and z
        void setkeepAspectRatio(const bool value);

        //!> return intensity overlay image as dataObject
        QSharedPointer< ito::DataObject > getOverlayImage() const {return QSharedPointer< ito::DataObject >(NULL); }

        /** set new dataObject for intensity overlay
        *   @param [in] overlayImage    dataObject with intensity information
        */
        void setOverlayImage(QSharedPointer< ito::DataObject > overlayImage);

        //!> remove overlay image
        void resetOverlayImage(void);

        //!> return invalid map as dataObject
        QSharedPointer< ito::DataObject > getInvalidMap() const {return QSharedPointer< ito::DataObject >(NULL); }

        /** set new dataObject for invalid masking overlay
        *   @param [in] invalidMap    dataObject with invalid information as uint8-mat
        */
        void setInvalidMap(QSharedPointer< ito::DataObject > invalidMap);

        //!> setInvalidMap to zeros
        void resetInvalidMap(void);
        
        /** set invalid color
        *   @param [in] newVal  new invalid color
        */
        void setInvalidColor(const QColor newVal);

        /** reset invalid color
        */
        void resetInvalidColor();

        //!> return invalid color
        QColor getInvalidColor(void) const;


        //!> return current alpha value for intensity overlay
        int getOverlayAlpha () const;

        //!> set alpha value for intensity overlay
        void setOverlayAlpha (const int alpha);

        //!> reset alpha value for intensity overlay (overlay invisible)
        void resetOverlayAlpha(void);

        //!> return inverval of intensity image
        //QPointF getOverlayInterval() const;
        ito::AutoInterval getOverlayInterval() const;

        //void setOverlayInterval(const QPointF interval);
        void setOverlayInterval(const ito::AutoInterval interval);

        //!> return interval of curvature / deviation data
        //QPointF getCurvatureInterval() const;
        ito::AutoInterval getCurvatureInterval() const;
        //void setCurvatureInterval(const QPointF interval);
        void setCurvatureInterval(const ito::AutoInterval interval);

        /** enable slider for setting intensity overlay alpha
        *   @param [in] enabled     enable / disable slider
        */
        void enableOverlaySlider(bool enabled)
        {
            m_pActOverlaySlider->setVisible(enabled);
        }

        /** render plot to pixmap
        *   @param [in] xsize   horizontal size of pixmap
        *   @param [in] ysize   vertival size of pixmap
        *   @param [in] resolution
        */
        QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution); 

        //!> set new background color
        void setBackgroundColor(const QColor newVal);

        //!> get current background color
        QColor getBackgroundColor(void) const;

        /** set color of axis
        *   @param [in] newVal  new axis color
        */
        void setAxisColor(const QColor newVal);

        //!> return axis color
        QColor getAxisColor(void) const;

        /** set text color
        *   @param [in] newVal  new text color
        */
        void setTextColor(const QColor newVal);

        //!> return text color
        QColor getTextColor(void) const;

        //!> enable / disable illumination
        void setIllumunationEnabled(const bool newVal);

        //!> return if illumination is enabled
        bool getIllumunationEnabled(void) const;

        //!> return if currently points or triangles are rendered (for dataObjects)
        bool getShowTriangles(void) const;

        /** switch between triangle and point rendering mode (dataObject only)
        *   @param [in] newVal  true for triangle rendering, false for point rendering
        */
        void setShowTriangles(const bool newVal);

        //!> return if currently status or angle information visibility
        bool getShowAngleInfo(void) const;

        /** switch visibility of angle infomation 
        *   @param [in] newVal  true for enabled, false for invisible
        */
        void setShowAngleInfo(const bool newVal);
        

        //!> enable button to switch between triangle and point rendering
        void enableRenderModeSelector(const int newVal);

        //!> set viewport rotation angles
        void setRotationAngle(const QVector<float> newAngles);

        //!> get viewport rotation angles (3 coordiates)
        QVector<float> getRotationAngle() const;

        //!> set light vector direction towards center (2 coordinates)
        void setRotationLightAngle(const QVector<float> newAngles);

        //!> get light vector direction towards center (2 coordinates)
        QVector<float> getRotationLightAngle() const;

        //!> returns if curvature data is used for color coding
        bool getCurvature2Color(void) const;

        /**
        *   @param [in] newVal  true to use curvature for color coding else false
        */
        void setCurvature2Color(const bool newVal);

        bool getFillInvalidsStatus() const;
        void setFillInvalidsStatus(const bool enable);

        QVector<int> getPlaneIndices() const;

    protected:
        TwipOGLWidget* m_pContent;  //!> content widget
        void updateLegend(const int index, const  int type, const int subtype, const ito::uint8 alpha, const bool enabled);


    private:
        ito::uint8 m_updateLook;
        void *m_pvConfigData;

        QAction *m_pActScaleSetting;
        QAction *m_pActRiseZAmpl;
        QAction *m_pActReduceZAmpl;
        QAction *m_pActPalette;
        QAction *m_pActToggleColorBar;
        QAction *m_pActChangeBGColor;

        QAction *m_pActSave;
        QAction *m_pActHome;
        QAction *m_pActTwip;
        QAction *m_pKeepAspectRatio;

        QAction *m_pToggleIllumination;
        QAction *m_pToggleIlluminationRotation;
        QAction *m_pToggleInfoText;

        QAction *m_pActCmplxSwitch;
        QMenu   *m_pMnuCmplxSwitch;

        QAction *m_pActTringModeSwitch;
        QMenu   *m_pMnuTringModeSwitch;

        QAction *m_pActProperties;

        QLabel  *m_pLblCoordinates;
        QAction *m_pActOverlaySlider;
        QSlider *m_pOverlaySlider;

        QAction *m_pActLegend;
        QAction *m_pActLegendIcon;
        //void riseZAmplification(const double value){if(m_pContent) ((TwipOGLWidget*)m_pContent)->riseZAmplification(value);}
        //void reduceZAmplification(const double value){if(m_pContent) ((TwipOGLWidget*)m_pContent)->reduceZAmplification(value);}
        //static int m_pclID;
        int m_pclID;   //!> identifier of latest added pointCloud
        int m_dObjID;  //!> identifier of latest added dataObject

        TwipLegend* m_pLegend;
        QDockWidget* m_pLegendDock;

    signals:

    private slots:
        void mnuTwip();             //!> spawn twip info dialog
        void mnuHome();             //!> home view, i.e. reset to default orientation / zoom
        void mnuAspRatio();
        void mnuScaleSetting();     
        void mnuPalette();          
        void mnuColorBar();
        void mnuToggleBPColor();
        void mnutoggleIllumination(const bool checked);
        void mnutoggleIlluminationRotation(const bool checked);
        void mnuCmplxSwitch(QAction *action);
        void mnuTringModeSwitch(QAction *action);
        void mnuActSave();
        void mnuOverlaySliderChanged(int value);
        void mnuRiseZ();
        void mnuRedZ();

        void mnuShowLegend(bool checked) { if (m_pLegendDock) { m_pLegendDock->setVisible(checked); } }

    public slots:
        void triggerReplot();
        ito::RetVal copyToClipBoard();
#ifdef USEPCL
        int addPointCloud(ito::PCLPointCloud pointCloud);
#endif
        int addDataObject(ito::DataObject dataObject);

        ito::RetVal setPlaneAlpha(int idx, int alpha);
        ito::RetVal toggleVisibility(int idx, bool state);
        ito::RetVal addOverlayImage(QSharedPointer< ito::DataObject > overlayImage, int objID)
        {
            if(m_pContent)
                return m_pContent->setOverlayImage(overlayImage, objID);
            else 
                return ito::RetVal(ito::retError, 0, tr("Could not write overlay to uninitilized GL-Widget").toLatin1().data());            
        }

};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // TWIPOGLFIGURE_H
