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

#ifndef ITOM2DQWTPLOT_H
#define ITOM2DQWTPLOT_H

#include "plot/AbstractDObjFigure.h"
#include "plot/AbstractNode.h"

#include "plotCanvas.h"

#include <qaction.h>
#include <qwidgetaction.h>
#include <qspinbox.h>

Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)

class Itom2dQwtPlot : public ito::AbstractDObjFigure
{
	Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel)
    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel)
    Q_PROPERTY(bool colorBarVisible READ colorBarVisible WRITE setColorBarVisible DESIGNABLE true)

public:
    Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
	~Itom2dQwtPlot();

    ito::RetVal displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack = false);

    ito::RetVal applyUpdate();  //!> does the real update work

	//properties (setter/getter)
	void setShowContextMenu(bool show) {}; 
	bool showContextMenu() const { return false; };

	bool colorBarVisible() const;
	void setColorBarVisible(bool value);

    QString getTitle() const;
    void setTitle(const QString &title);
    void resetTitle();

    QString getxAxisLabel() const;
    void setxAxisLabel(const QString &label);
    void resetxAxisLabel();

    QString getyAxisLabel() const;
    void setyAxisLabel(const QString &label);
    void resetyAxisLabel();

    QString getValueLabel() const;
    void setValueLabel(const QString &label);
    void resetValueLabel();

    bool getyAxisFlipped() const;
    void setyAxisFlipped(const bool &value);

    bool getxAxisVisible() const;
    void setxAxisVisible(const bool &value);
    
    bool getyAxisVisible() const;
    void setyAxisVisible(const bool &value);

    void setPlaneRange(int min, int max);
    void setCmplxSwitch(PlotCanvas::ComplexType type, bool visible);

    virtual QPointF getXAxisInterval(void);
    virtual void setXAxisInterval(QPointF point);
        
    virtual QPointF getYAxisInterval(void);
    virtual void setYAxisInterval(QPointF point);
        
    virtual QPointF getZAxisInterval(void);
    virtual void setZAxisInterval(QPointF point);

protected:
    ito::RetVal init() { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

	void createActions();

	//void setLinePlotCoordinates(const QVector<QPointF> pts);
	void closeEvent(QCloseEvent *event);

private:

	PlotCanvas *m_pContent;	
    InternalData m_data;

    QAction *m_pActSave;
    QAction *m_pActHome;
    QAction *m_pActPan;
    QAction *m_pActZoom;
    QAction *m_pActScaleSettings;
    QAction *m_pActColorPalette;
    QAction *m_pActToggleColorBar;
    QAction *m_pActValuePicker;
    QAction *m_pActLineCut;
    QAction *m_pActStackCut;
    QWidgetAction *m_pActPlaneSelector;

    QAction* m_pActCmplxSwitch;
	QMenu *m_mnuCmplxSwitch;

    QHash<QObject*,ito::uint32> m_childFigures;

private slots:
    void mnuActSave();
    void mnuActHome();
    void mnuActPan(bool checked);
    void mnuActZoom(bool checked);
    void mnuActScaleSettings();
    void mnuActColorPalette();
    void mnuActToggleColorBar(bool checked);
    void mnuActValuePicker(bool checked);
    void mnuActLineCut(bool checked);
    void mnuActStackCut(bool checked);
    void mnuActPlaneSelector(int plane);
    void mnuCmplxSwitch(QAction *action);
    void childFigureDestroyed(QObject *obj);

public slots:
    ito::RetVal pickPoints(QSharedPointer<ito::DataObject> coordsOut, int maxNrOfPoints = -1);

};

#endif // ITOM2DQWTPLOT_H
