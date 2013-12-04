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

#include "DataObject/dataObjectFuncs.h"
#include "DataObject/dataobj.h"

#include "dialogSettings.h"
#include "dialogDeleteRelation.h"
#include "dialogAddRelation.h"

#include <qmessagebox.h>
#include <qsharedpointer.h>
#include <qfiledialog.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_actSetting(NULL),
	m_actSave(NULL),
    m_actAddRel(NULL),
    m_actRemoveRel(NULL),
    m_actUpdate(NULL),
    m_actAutoFitCols(NULL),
    m_mnuSaveSwitch(NULL),
    m_lastFolder("")
{
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toAscii().data()));
    
    //int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

	//m_actSave
    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save as table, tree, xml"), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip(tr("Export current data"));

	m_mnuSaveSwitch = new QMenu("Save Switch", this);
	m_mnuSaveSwitch->addAction(tr("table"));
	m_mnuSaveSwitch->addAction(tr("tree"));
	m_mnuSaveSwitch->addAction(tr("list"));
    m_mnuSaveSwitch->addAction(tr("xml"));
	m_actSave->setMenu(m_mnuSaveSwitch);

    //m_actScaleSetting
    m_actSetting = new QAction(QIcon(":/itomDesignerPlugins/general/icons/settings.png"), tr("system settings"), this);
    m_actSetting->setObjectName("actScaleSetting");
    m_actSetting->setToolTip(tr("Set the ranges and offsets of this view"));
    
    //m_actAddRel
    m_actAddRel = new QAction(QIcon(":/evaluateGeometrics/icons/addRel.png"), tr("add relation"), this);
    m_actAddRel->setObjectName("actAddRelation");
    m_actAddRel->setToolTip(tr("Add a further relation to this table or fix a defect one."));

    //m_actAddRel
    m_actRemoveRel = new QAction(QIcon(":/evaluateGeometrics/icons/remRel.png"), tr("remove relation"), this);
    m_actRemoveRel->setObjectName("actRemoveRelation");
    m_actRemoveRel->setToolTip(tr("Remove a relation from the table."));

    //m_actUpdate
    m_actUpdate = new QAction(QIcon(":/itomDesignerPlugins/general/icons/upDate.png"), tr("update relation"), this);
    m_actUpdate->setObjectName("actUpdate");
    m_actUpdate->setToolTip(tr("Force update of this table."));

    //m_actAutoFitCols
    m_actAutoFitCols = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"), tr("auto-scale columns"), this);
    m_actAutoFitCols->setObjectName("actAutoFitCols");
    m_actAutoFitCols->setToolTip(tr("Adapts columns to idle width."));

    connect(m_mnuSaveSwitch, SIGNAL(triggered(QAction *)), this, SLOT(mnuExport(QAction *)));
    connect(m_actSetting, SIGNAL(triggered()), this, SLOT(mnuSetting()));

    connect(m_actAddRel, SIGNAL(triggered()), this, SLOT(mnuAddRelation()));
    connect(m_actRemoveRel, SIGNAL(triggered()), this, SLOT(mnuDeleteRelation()));
    connect(m_actUpdate, SIGNAL(triggered()), this, SLOT(mnuUpdate()));
    connect(m_actAutoFitCols, SIGNAL(triggered()), this, SLOT(mnuAutoFitCols()));

	QToolBar *toolbar = new QToolBar(tr("basic options"), this);
	addToolBar(toolbar, "mainToolBar");

	QMenu *contextMenu = new QMenu(QObject::tr("Calculate"), this);
    contextMenu->addAction(m_actSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(toolbar->toggleViewAction());

    // first block is zoom, scale settings, home
	toolbar->addAction(m_actSave);
    toolbar->addSeparator();
    toolbar->addAction(m_actSetting);
    toolbar->addAction(m_actAutoFitCols);
    toolbar->addAction(m_actUpdate);
	toolbar->addSeparator();
    toolbar->addAction(m_actAddRel);
    toolbar->addAction(m_actRemoveRel);



    m_pContent = new PlotTreeWidget(contextMenu, &m_info, this);
    m_pContent->setObjectName("canvasWidget");

    setFocus();
    setCentralWidget(m_pContent);

    m_pContent->setFocus();
    

}

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::~EvaluateGeometricsFigure()
{
    if(m_mnuSaveSwitch)
    {
        m_mnuSaveSwitch->deleteLater();
        m_mnuSaveSwitch = NULL;
    }

    if(m_actAddRel)
    {
        m_actAddRel->deleteLater();
        m_actAddRel = NULL;
    }

    if(m_actRemoveRel)
    {
        m_actRemoveRel->deleteLater();
        m_actRemoveRel = NULL;
    }

    if(m_actUpdate)
    {
        m_actUpdate->deleteLater();
        m_actUpdate = NULL;
    }

    if(m_actSetting)
    {
        m_actSetting->deleteLater();
        m_actSetting = NULL;
    }

    if(m_actSave)
    {
        m_actSave->deleteLater();
        m_actSave = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::applyUpdate()
{
    //QVector<QPointF> bounds = getBounds();

    if ((ito::DataObject*)m_pInput["source"]->getVal<void*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        m_pContent->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>());

    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setSource(QSharedPointer<ito::DataObject> source)
{
    AbstractDObjFigure::setSource(source);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool EvaluateGeometricsFigure::getContextMenuEnabled() const
{
    if (m_pContent) return (m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setContextMenuEnabled(bool show)
{
    if (m_pContent) (m_pContent)->m_showContextMenu = show;
}
/*
//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setBounds(QVector<QPointF> bounds) 
{ 

}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> EvaluateGeometricsFigure::getBounds(void) 
{ 
    QVector<QPointF> val(4);
    val[0] = QPointF(0.0, 0.0);
    val[1] = QPointF(0.0, 1.0);
    val[2] = QPointF(1.0, 0.0);
    val[3] = QPointF(1.0, 1.0);
    return val;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getTitle() const
{
    if (m_info.m_autoTitle)
    {
        return "<auto>";
    }
    return m_info.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        m_info.m_autoTitle = true;
    }
    else
    {
        m_info.m_autoTitle = false;
        m_info.m_title = title;
    }

    //if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetTitle()
{
    m_info.m_autoTitle = true;
    //if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getValueUnit() const
{
    return m_info.m_valueUnit;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setValueUnit(const QString &label)
{
    m_info.m_valueUnit = label;
    if (m_pContent)
    {
        m_pContent->updatePrimitives();
        m_pContent->updateRelationShips(true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetValueUnit()
{
    m_info.m_valueUnit = "";
    if (m_pContent)
    {
        m_pContent->updatePrimitives();
        m_pContent->updateRelationShips(true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont EvaluateGeometricsFigure::getTitleFont(void) const
{
    if (m_pContent)
    {

    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont EvaluateGeometricsFigure::getLabelFont(void) const
{
    if (m_pContent)
    {

    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setLabelFont(const QFont &font)
{
    if (m_pContent)
    {

    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuExport(QAction* action)
{
    QString fileName = 0;

    ito::int32 saveType = exportCSVTree;
    QString saveFilter("*.csv");

    if (action->text() == QString(tr("table")))
    {
        saveType = exportCSVTable;
    }
    else if (action->text() == QString(tr("xml")))
    {
        saveType = exportXMLTree;
        saveFilter = "*.xml";
    }
    else if (action->text() == QString(tr("list")))
    {
        saveType = exportCSVList;
        saveFilter = "*.csv";
    }

    fileName = QFileDialog::getSaveFileName(this, tr("select destination file"), m_lastFolder, saveFilter);
                
    if (m_pContent && !fileName.isEmpty())
    {
        QFileInfo exportFile = fileName; 
        m_lastFolder = exportFile.path();

        switch(saveType)
        {
            default:
            case exportCSVTree:
                m_pContent->writeToCSV(exportFile, false);
                break;
            case exportCSVTable:
                m_pContent->writeToCSV(exportFile, true);
                break;
            case exportXMLTree:
                m_pContent->writeToXML(exportFile);
                break;
            case exportCSVList:
                m_pContent->writeToRAW(exportFile);
                break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::exportData(QString fileName, ito::uint8 exportFlag)
{
    ito::RetVal retVal = ito::retOk;

    QString saveFilter("*.csv");

    if ((exportFlag& 0x0F) == exportXMLTree )
    {
        saveFilter = "*.xml";
    }
    
    if(!fileName.isEmpty())
    {
        m_lastFolder = fileName;        
    }

    if(exportXMLTree & showExportWindow)
    {
        fileName = QFileDialog::getSaveFileName(this, tr("select destination file"), m_lastFolder, saveFilter);
    }
                
    if (m_pContent && !fileName.isEmpty())
    {
        QFileInfo exportFile = fileName; 
        m_lastFolder = exportFile.path();

        switch(exportFlag & 0x0F)
        {
            default:
            case exportCSVTree:
                retVal = m_pContent->writeToCSV(exportFile, false);
                break;
            case exportCSVTable:
                retVal = m_pContent->writeToCSV(exportFile, true);
                break;
            case exportXMLTree:
                retVal = m_pContent->writeToXML(exportFile);
                break;
            case exportCSVList:
                retVal = m_pContent->writeToRAW(exportFile);
                break;
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuSetting()
{
    DialogSettings *dlg = new DialogSettings(m_info, this->m_pContent->m_rowHash.size(), this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_info);

        m_pContent->updatePrimitives();
        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}


//----------------------------------------------------------------------------------------------------------------------------------
QPointF EvaluateGeometricsFigure::getYAxisInterval(void) const
{ 
    QPointF interval(0.0, 1.0);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void EvaluateGeometricsFigure::setYAxisInterval(QPointF interval) 
{ 

    return; 
}   

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::enableComplexGUI(const bool checked)
{ 

}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuHome()
{

}

//----------------------------------------------------------------------------------------------------------------------------------

void EvaluateGeometricsFigure::setRelations(QSharedPointer<ito::DataObject> importedData)
{
    int dims = 0;

    if(importedData.isNull())
    {
        return;
    }

    if((dims = importedData->getDims()) == 0 || importedData->calcNumMats() > 1)
    {
        return;
    }

    int rows = importedData->getSize(dims-2);

    m_info.m_relationsList.clear();
    m_info.m_relationsList.reserve(rows);

    cv::Mat * myMat = (cv::Mat*)(importedData->get_mdata()[0]);
    ito::float32* ptr = NULL;

    relationsShip newRelation;
//    newRelation.secondElementRow = -1;
//    newRelation.firstElementRow = -1;
    newRelation.myWidget = NULL;


    switch(myMat->cols)
    {
        case 1:
            for(int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::uint32)ptr[0];
                newRelation.type = 0;
                newRelation.secondElementIdx = -1;
                newRelation.extValue = 0.0;
                m_info.m_relationsList.append(newRelation);
            }
        break;

        case 2:
            for(int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = -1;
                newRelation.extValue = 0.0;
                m_info.m_relationsList.append(newRelation);
            }

        break;


        case 3:
            for(int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = (ito::int32)ptr[2];
                newRelation.extValue = 0.0;
                m_info.m_relationsList.append(newRelation);
            }
        break;

        default:
        case 4:
            for(int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = (ito::int32)ptr[2];
                newRelation.extValue = (ito::float32)ptr[3];
                m_info.m_relationsList.append(newRelation);
            }
        break;
    }

    if(m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------

QSharedPointer<ito::DataObject> EvaluateGeometricsFigure::getRelations(void) const
{
    if(m_info.m_relationsList.size() == 0)
    {
        return QSharedPointer<ito::DataObject>(new ito::DataObject());
    }

    QSharedPointer<ito::DataObject> exportedData(new ito::DataObject(4, m_info.m_relationsList.size(), ito::tFloat32));

    cv::Mat * myMat = (cv::Mat*)(exportedData->get_mdata()[0]);
    ito::float32* ptr = NULL;
    for(int i = 0; i < m_info.m_relationsList.size(); i ++)
    {
        ptr = myMat->ptr<ito::float32>(i);
        ptr[0] = (ito::float32)m_info.m_relationsList[i].firstElementIdx;
        ptr[1] = (ito::float32)m_info.m_relationsList[i].type; 
        ptr[2] = (ito::float32)m_info.m_relationsList[i].secondElementIdx;
        ptr[3] = (ito::float32)m_info.m_relationsList[i].extValue;
    }

    return exportedData;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::addRelation(QSharedPointer<ito::DataObject> relation)
{
    relationsShip newRelation;
            

    if(relation.isNull() || relation->getDims() != 2 || relation->getSize(0) != 1)
    {
        return ito::RetVal(ito::retError, 0, tr("set relation failed due to invalud object dims").toAscii().data());
    }

    if((relation->getType() != ito::tFloat32 && relation->getType() != ito::tFloat64))
    {
        return ito::RetVal(ito::retError, 0, tr("set relation failed due to invalud object type").toAscii().data());
    }

//    newRelation.secondElementRow = -1;
//    newRelation.firstElementRow = -1;

    newRelation.myWidget = NULL;

    switch(relation->getSize(1))
    {
        case 4:
        default:
            if(relation->getType() == ito::tFloat32) newRelation.extValue = relation->at<ito::float32>(0,3);
            else newRelation.extValue = (ito::float32)(relation->at<ito::float64>(0,3));
        case 3:
            if(relation->getType() == ito::tFloat32) newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float32>(0,2));
            else newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float64>(0,2));
        case 2:
            if(relation->getType() == ito::tFloat32) newRelation.type = (ito::int32)(relation->at<ito::float32>(0,1));
            else newRelation.type = (ito::int32)(relation->at<ito::float64>(0,1));
        case 1:
            if(relation->getType() == ito::tFloat32) newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float32>(0,0));
            else newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float64>(0,0));
        break;
        case 0:
            return ito::RetVal(ito::retError, 0, tr("set relation failed due to empty vector").toAscii().data());
    }

    m_info.m_relationsList.append(newRelation);

    if(m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::addRelationName(const QString newName)
{
    ito::RetVal retval(ito::retOk);

    int idx = m_info.m_relationNames.indexOf(newName);

    if(idx < 7 && idx != -1)
    {
        retval = ito::RetVal(ito::retError, 0, tr("add relation name failed: relation as one of the restricted relations").toAscii().data());
        return retval;
    }

    if(idx == -1)
    {
        m_info.m_relationNames.append(newName);
    }
    else
    {
        m_info.m_relationNames[idx] = newName;
        retval = ito::RetVal(ito::retWarning, 0, tr("add relation name exited with warning: relation already existed").toAscii().data());
    }
    return retval;
}
//---------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::plotItemChanged(ito::int32 idx, ito::int32 flags, QVector<ito::float32> values)
{
    if(m_pContent)
    {
        return m_pContent->updateElement(idx, flags, values);
    }
    return ito::retOk;
}
//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuAddRelation()
{
    DialogAddRelation *dlg = new DialogAddRelation(m_info, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_info);

        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}
//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuDeleteRelation()
{
    DialogDeleteRelation *dlg = new DialogDeleteRelation(m_info, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_info);

        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}
//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuUpdate()
{
    if(m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }
}
//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuAutoFitCols()
{
    if(m_pContent)
    {
        m_pContent->autoFitCols();
    }
}
//---------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::clearAll(void) 
{
    m_info.m_relationsList.clear();

    if(m_pContent)
    {
        ito::DataObject dummyObject;
        m_pContent->refreshPlot(&dummyObject);
    }
    return ito::retOk;
}