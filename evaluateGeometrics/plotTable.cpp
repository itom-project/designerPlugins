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

#include "evaluateGeometrics.h"
#include "plotTable.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"

#include <qdebug.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
PlotTable::PlotTable(QMenu *contextMenu, QWidget * parent) :
    QTabWidget(parent),
    m_contextMenu(contextMenu),
    m_xDirect(false),
    m_yDirect(false),
    m_pParent(parent),
    m_cmplxState(false),
    m_state(stateIdle)
{
    //this is the border between the canvas and the axes and the overall mainwindow
	setContentsMargins(5,5,5,5);
	
    m_geometrics = new QTableWidget(this);
    m_relations = new QTableWidget(this);

    addTab(m_geometrics, QObject::tr("geomatric data"));
    addTab(m_relations, QObject::tr("calculations"));

}

//----------------------------------------------------------------------------------------------------------------------------------
PlotTable::~PlotTable()
{
    m_geometrics->deleteLater();
    m_relations->deleteLater();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTable::init()
{
    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();
    
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void PlotTable::setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel)
//{
//   
//}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::updateLabels()
{
  
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::refreshPlot(const ito::DataObject* dataObj)
{
   
       
}
/*
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::keyPressEvent ( QKeyEvent * event )
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mousePressEvent ( QMouseEvent * event )
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mouseMoveEvent ( QMouseEvent * event )
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mouseReleaseEvent ( QMouseEvent * event )
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::contextMenuEvent(QContextMenuEvent * event)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTable::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
 

    return ito::retOk;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setZoomerEnable(const bool checked)
{
 
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setPickerEnable(const bool checked)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setPannerEnable(const bool checked)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::updateScaleValues(bool recalculateBoundaries /*= false*/)
{
   
}

