#ifndef MATPLOTLIBPLOT_H
#define MATPLOTLIBPLOT_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMMATPLOTLIB_EXPORT Q_DECL_EXPORT
#else
    #define ITOMMATPLOTLIB_EXPORT Q_DECL_IMPORT
#endif

#include <QtCore/QtGlobal>
#include <qmainwindow.h>
#include <qaction.h>
#include <qtoolbar.h>
#include <qlabel.h>

#include "plot/AbstractItomDesignerPlugin.h"
#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"

class ITOMMATPLOTLIB_EXPORT MatplotlibPlot : public ito::AbstractFigure
{
    Q_OBJECT

    Q_PROPERTY(bool forceWindowResize READ getForceWindowResize WRITE setForceWindowResize USER true)

    Q_CLASSINFO("prop://forceWindowResize", "If set, the plot widget / area is resized to the desired sizes given by matplotlib. Uncheck this option, if you want to keep the canvas unchanged e.g. in an user-defined GUI")
    
    Q_CLASSINFO("slot://showSubplotConfig", "")
    Q_CLASSINFO("slot://setLabelText", "")

    Q_CLASSINFO("signal://subplotConfigSliderChanged", "")

    DESIGNER_PLUGIN_ITOM_API
public:
    MatplotlibPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~MatplotlibPlot();

    //properties
    void setContextMenuEnabled(bool show); 
    bool getContextMenuEnabled() const;

    void setForceWindowResize(bool force) { m_forceWindowResize = force; } 
    bool getForceWindowResize() const { return m_forceWindowResize; }

    void resizeCanvas(int width, int height);

    ito::RetVal applyUpdate(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }
    ito::RetVal update(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }
   
private:
    QAction *m_actHome;
    QAction *m_actForward;
    QAction *m_actBack;
    QAction *m_actPan;
    QAction *m_actZoomToRect;
    QAction *m_actSubplotConfig;
    QAction *m_actSave;
    QAction *m_actMarker;
    QAction *m_actProperties;
    QLabel *m_lblCoordinates;
    QToolBar *m_toolbar;
    QMenu *m_contextMenu;
    void *m_pContent;
    void *m_pMatplotlibSubfigConfig;
    bool m_forceWindowResize;

signals:
    void subplotConfigSliderChanged(int type, int value);

private slots:
    void mnuMarkerClick(bool /*checked*/);
    inline void mnuShowToolbar(bool /*checked*/) { setToolbarVisible(true); }

    void subplotConfigSliderLeftChanged(int value);
    void subplotConfigSliderTopChanged(int value);
    void subplotConfigSliderRightChanged(int value);
    void subplotConfigSliderBottomChanged(int value);
    void subplotConfigSliderWSpaceChanged(int value);
    void subplotConfigSliderHSpaceChanged(int value);

    void resetFixedSize() { setFixedSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX); }

public slots:
    void showSubplotConfig(int valLeft, int valTop, int valRight, int valBottom, int valWSpace, int valHSpace);
    void setLabelText(QString text) { m_lblCoordinates->setText(text); }

};

#endif // MATPLOTLIBPLOT_H
