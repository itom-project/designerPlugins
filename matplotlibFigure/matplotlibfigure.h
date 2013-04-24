#ifndef MATPLOTLIBFIGURE_H
#define MATPLOTLIBFIGURE_H

#include <qmainwindow.h>
#include <qaction.h>
#include <qtoolbar.h>
#include <qlabel.h>

#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"

class MatplotlibFigure : public QMainWindow
{
    Q_OBJECT
    Q_PROPERTY(bool toolbarVisible READ toolbarVisible WRITE setToolbarVisible DESIGNABLE true);
    Q_PROPERTY(bool showContextMenu READ showContextMenu WRITE setShowContextMenu DESIGNABLE true);

public:
    MatplotlibFigure(QWidget *parent = 0);
    ~MatplotlibFigure();

    //properties
    void setToolbarVisible(bool visible);
    bool toolbarVisible() const;
    void setShowContextMenu(bool show); 
    bool showContextMenu() const;

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
    MatplotlibWidget *m_pContent;
    MatplotlibSubfigConfig *m_pMatplotlibSubfigConfig;

signals:
    void subplotConfigSliderChanged(int type, int value);

private slots:
    void mnuMarkerClick(bool /*checked*/);
    inline void mnuShowToolbar(bool /*checked*/) { setToolbarVisible(true); }

    void subplotConfigSliderLeftChanged(int value) { emit subplotConfigSliderChanged(0, value); }
    void subplotConfigSliderTopChanged(int value) { emit subplotConfigSliderChanged(1, value); }
    void subplotConfigSliderRightChanged(int value) { emit subplotConfigSliderChanged(2, value); }
    void subplotConfigSliderBottomChanged(int value) { emit subplotConfigSliderChanged(3, value); }
    void subplotConfigSliderWSpaceChanged(int value) { emit subplotConfigSliderChanged(4, value); }
    void subplotConfigSliderHSpaceChanged(int value) { emit subplotConfigSliderChanged(5, value); }

public slots:
    void showSubplotConfig(int valLeft, int valTop, int valRight, int valBottom, int valWSpace, int valHSpace);


};

#endif // MATPLOTLIBFIGURE_H
