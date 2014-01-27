#ifndef MATPLOTLIBPLOT_H
#define MATPLOTLIBPLOT_H

#include <QtCore/QtGlobal>
#include <qmainwindow.h>
#include <qaction.h>
#include <qtoolbar.h>
#include <qlabel.h>

#include "plot/AbstractItomDesignerPlugin.h"
#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"

class ITOMSHAREDDESIGNER_EXPORT MatplotlibPlot : public QMainWindow
{
    Q_OBJECT
    Q_PROPERTY(bool toolbarVisible READ getToolbarVisible WRITE setToolbarVisible DESIGNABLE true)
    Q_PROPERTY(bool contextMenuEnabled READ getContextMenuEnabled WRITE setContextMenuEnabled DESIGNABLE true)

    Q_CLASSINFO("prop://toolbarVisible", "Toggles the visibility of the toolbar of the plot.")
    Q_CLASSINFO("prop://contextMenuEnabled", "Defines whether the context menu of the plot should be enabled or not.")

public:
    MatplotlibPlot(QWidget *parent = 0);
    ~MatplotlibPlot();

    //properties
    void setToolbarVisible(bool visible);
    bool getToolbarVisible() const;
    void setContextMenuEnabled(bool show); 
    bool getContextMenuEnabled() const;

    void resizeCanvas(int width, int height);
   
private:
    QAction *m_actHome;
    QAction *m_actForward;
    QAction *m_actBack;
    QAction *m_actPan;
    QAction *m_actZoomToRect;
    QAction *m_actSubplotConfig;
    QAction *m_actSave;
    QAction *m_actMarker;
    QLabel *m_lblCoordinates;
    QToolBar *m_toolbar;
    QMenu *m_contextMenu;
//    MatplotlibWidget *m_pContent;
//    MatplotlibSubfigConfig *m_pMatplotlibSubfigConfig;
    void *m_pContent;
    void *m_pMatplotlibSubfigConfig;

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

public slots:
    void showSubplotConfig(int valLeft, int valTop, int valRight, int valBottom, int valWSpace, int valHSpace);

};

#endif // MATPLOTLIBPLOT_H
