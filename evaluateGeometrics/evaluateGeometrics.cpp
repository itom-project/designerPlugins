/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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
#include "plotTreeWidget.h"

#include "DataObject/dataObjectFuncs.h"
#include "DataObject/dataobj.h"

#include "dialogSettings.h"
#include "dialogDeleteRelation.h"
#include "dialogAddRelation.h"

#include <qmessagebox.h>
#include <qsharedpointer.h>
#include <qfiledialog.h>
#include <qregularexpression.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_actSetting(NULL),
    m_actSave(NULL),
    m_actAddRel(NULL),
    m_actRemoveRel(NULL),
    m_actUpdate(NULL),
    m_actAutoFitCols(NULL),
    m_actFitToObject(NULL),
    m_mnuSaveSwitch(NULL),
    m_lastFolder(""),
    m_lastAddedRelation(-1)
{
    m_pInfo = new InternalInfo();

    //m_actSave
    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip(tr("Export current data as table, tree, xml..."));
    m_mnuSaveSwitch = new QMenu("Save Switch", this);
    m_mnuSaveSwitch->addAction(tr("table"));
    m_mnuSaveSwitch->addAction(tr("tree"));
    m_mnuSaveSwitch->addAction(tr("list"));
    m_mnuSaveSwitch->addAction(tr("xml"));
    m_actSave->setMenu(m_mnuSaveSwitch);
    connect(m_mnuSaveSwitch, SIGNAL(triggered(QAction *)), this, SLOT(mnuExport(QAction *)));

    //m_actScaleSetting
    m_actSetting = new QAction(QIcon(":/itomDesignerPlugins/general/icons/settings.png"), tr("system settings"), this);
    m_actSetting->setObjectName("actScaleSetting");
    m_actSetting->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(m_actSetting, SIGNAL(triggered()), this, SLOT(mnuSetting()));

    //m_actAddRel
    m_actAddRel = new QAction(QIcon(":/evaluateGeometrics/icons/addRel.png"), tr("add relation"), this);
    m_actAddRel->setObjectName("actAddRelation");
    m_actAddRel->setToolTip(tr("Add a further relation to this table or fix a defect one."));
    m_actAddRel->setVisible(true);
    connect(m_actAddRel, SIGNAL(triggered()), this, SLOT(mnuAddRelation()));

    //m_actRemoveRel
    m_actRemoveRel = new QAction(QIcon(":/evaluateGeometrics/icons/remRel.png"), tr("remove relation"), this);
    m_actRemoveRel->setObjectName("actRemoveRelation");
    m_actRemoveRel->setToolTip(tr("Remove a relation from the table."));
    m_actRemoveRel->setVisible(true);
    connect(m_actRemoveRel, SIGNAL(triggered()), this, SLOT(mnuDeleteRelation()));

    //m_actUpdate
    m_actUpdate = new QAction(QIcon(":/itomDesignerPlugins/general/icons/upDate.png"), tr("update relation"), this);
    m_actUpdate->setObjectName("actUpdate");
    m_actUpdate->setToolTip(tr("Force update of this table."));
    connect(m_actUpdate, SIGNAL(triggered()), this, SLOT(mnuUpdate()));

    //m_actAutoFitCols
    m_actAutoFitCols = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"), tr("auto-scale columns"), this);
    m_actAutoFitCols->setObjectName("actAutoFitCols");
    m_actAutoFitCols->setToolTip(tr("Adapts columns to idle width."));
    connect(m_actAutoFitCols, SIGNAL(triggered()), this, SLOT(mnuAutoFitCols()));

    // m_actFitToObject
    m_actFitToObject = new QAction(QIcon("./itomDesignerPlugins/plot/icons/autoscal.png"), tr("Fit currently marked shape to underlying data"), this);
    m_actFitToObject->setObjectName("actFitToObject");
    m_actFitToObject->setToolTip(tr("Fit currently marked shape to underlying data"));
    connect(m_actFitToObject, SIGNAL(triggered()), this, SLOT(mnuFitToObject()));

    QToolBar *toolbar = new QToolBar(tr("basic options"), this);
    addToolBar(toolbar, "mainToolBar");

    QMenu *contextMenu = new QMenu(QObject::tr("Calculate"), this);
    contextMenu->addAction(m_actSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(toolbar->toggleViewAction());
    contextMenu->addSeparator();
    contextMenu->addAction(m_actFitToObject);

    // first block is zoom, scale settings, home
    toolbar->addAction(m_actSave);
    toolbar->addSeparator();
    toolbar->addAction(m_actSetting);
    toolbar->addAction(m_actAutoFitCols);
    toolbar->addAction(m_actUpdate);
    toolbar->addSeparator();
    toolbar->addAction(m_actAddRel);
    toolbar->addAction(m_actRemoveRel);

    m_pContent = new PlotTreeWidget(contextMenu, m_pInfo, this);
    m_pContent->setObjectName("canvasWidget");

    setFocus();
    setCentralWidget(m_pContent);

    m_pContent->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::~EvaluateGeometricsFigure()
{
    if(m_pInfo)
    {
        delete m_pInfo;
        m_pInfo = NULL;
    }

    if (m_mnuSaveSwitch)
    {
        m_mnuSaveSwitch->deleteLater();
        m_mnuSaveSwitch = NULL;
    }

    if (m_actAddRel)
    {
        m_actAddRel->deleteLater();
        m_actAddRel = NULL;
    }

    if (m_actRemoveRel)
    {
        m_actRemoveRel->deleteLater();
        m_actRemoveRel = NULL;
    }

    if (m_actUpdate)
    {
        m_actUpdate->deleteLater();
        m_actUpdate = NULL;
    }

    if (m_actSetting)
    {
        m_actSetting->deleteLater();
        m_actSetting = NULL;
    }

    if (m_actSave)
    {
        m_actSave->deleteLater();
        m_actSave = NULL;
    }

    if (m_actFitToObject)
    {
        m_actFitToObject->deleteLater();
        m_actFitToObject = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::applyUpdate()
{
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<ito::Shape> EvaluateGeometricsFigure::getGeometricShapes() const
{
    QVector<ito::Shape> shapes;

    if (m_pContent)
    {
        QHash<ito::int32, ito::Shape>::const_iterator it = m_pContent->m_rowHash.constBegin();
        int c = 0;
        while (it != m_pContent->m_rowHash.constEnd())
        {
            shapes << it.value();
            ++it;
        }
    }

    return shapes;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setGeometricShapes(QVector<ito::Shape> shapes)
{
    if (m_pContent)
    {
        m_pContent->setShapes(shapes);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool EvaluateGeometricsFigure::getContextMenuEnabled() const
{
    if (m_pContent)
    {
        return m_pContent->m_showContextMenu;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setContextMenuEnabled(bool show)
{
    if (m_pContent)
    {
        m_pContent->m_showContextMenu = show;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getValueUnit() const
{
    return m_pInfo->m_valueUnit;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setValueUnit(const QString &label)
{
    m_pInfo->m_valueUnit = label;
    if (m_pContent)
    {
        m_pContent->updateGeometricShapes();
        m_pContent->updateRelationShips(true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetValueUnit()
{
    m_pInfo->m_valueUnit = "";
    if (m_pContent)
    {
        m_pContent->updateGeometricShapes();
        m_pContent->updateRelationShips(true);
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
        QFileInfo exportFile(fileName);
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
ito::RetVal EvaluateGeometricsFigure::exportData(QString fileName, int exportFlag)
{
    ito::RetVal retVal = ito::retOk;

    QString saveFilter("*.csv");

    if ((exportFlag& 0x0F) == exportXMLTree)
    {
        saveFilter = "*.xml";
    }

    if (!fileName.isEmpty())
    {
        m_lastFolder = fileName;
    }

    if (exportXMLTree & showExportWindow)
    {
        fileName = QFileDialog::getSaveFileName(this, tr("select destination file"), m_lastFolder, saveFilter);
    }

    if (m_pContent && !fileName.isEmpty())
    {
        QFileInfo exportFile(fileName);
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
    DialogSettings *dlg = new DialogSettings(*m_pInfo, this->m_pContent->m_rowHash.size(), this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(*m_pInfo);

        m_pContent->updateGeometricShapes();
        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setRelations(QSharedPointer<ito::DataObject> importedData)
{
    int dims = 0;

    if (importedData.isNull())
    {
        return;
    }

    if ((dims = importedData->getDims()) == 0 || importedData->calcNumMats() > 1)
    {
        return;
    }

    if (importedData->getType() != ito::tFloat32)
    {
        return;
    }

    int rows = importedData->getSize(dims-2);

    m_pInfo->m_relationsList.clear();
    m_pInfo->m_relationsList.reserve(rows);

    cv::Mat * myMat = importedData->getCvPlaneMat(0);
    ito::float32* ptr = NULL;

    relationShip newRelation;
//    newRelation.secondElementRow = -1;
//    newRelation.firstElementRow = -1;
    newRelation.myWidget = NULL;

    switch(myMat->cols)
    {
        case 1:
            for (int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::uint32)ptr[0];
                newRelation.type = 0;
                newRelation.secondElementIdx = -1;
                newRelation.extValue = 0.0;
                m_pInfo->m_relationsList.append(newRelation);
            }
        break;

        case 2:
            for (int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = -1;
                newRelation.extValue = 0.0;
                m_pInfo->m_relationsList.append(newRelation);
            }
        break;

        case 3:
            for (int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = (ito::int32)ptr[2];
                newRelation.extValue = 0.0;
                m_pInfo->m_relationsList.append(newRelation);
            }
        break;

        default:
        case 4:
            for (int i = 0; i < myMat->rows; i ++)
            {
                ptr = myMat->ptr<ito::float32>(i);
                newRelation.firstElementIdx = (ito::int32)ptr[0];
                newRelation.type = (ito::uint32)ptr[1];
                newRelation.secondElementIdx = (ito::int32)ptr[2];
                newRelation.extValue = (ito::float32)ptr[3];
                m_pInfo->m_relationsList.append(newRelation);
            }
        break;
    }

    m_lastAddedRelation = m_pInfo->m_relationsList.size() -1;

    if (m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::modifyRelation(const int idx, QSharedPointer<ito::DataObject> relation)
{
    int dims = 0;

    if (relation.isNull())
    {
        return ito::RetVal(ito::retError, 0, tr("imported data was Null-pointer").toLatin1().data());
    }

    if ((dims = relation->getDims()) == 0)
    {
        return ito::RetVal(ito::retError, 0, tr("imported data not initialized").toLatin1().data());
    }

    if (relation->calcNumMats() > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("imported data has more than one plane").toLatin1().data());
    }

    if ((relation->getType() != ito::tFloat32 && relation->getType() != ito::tFloat64))
    {
        return ito::RetVal(ito::retError, 0, tr("set relation failed due to invalid object type").toLatin1().data());
    }

    int rows = relation->getSize(dims-2);

    if (rows > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("imported data has more than one row").toLatin1().data());
    }

    if (idx < 0)
    {
        return ito::RetVal(ito::retError, 0, tr("tried to access index below zero").toLatin1().data());
    }

    if (idx < m_pInfo->m_relationsList.size() - 1)
    {
        return ito::RetVal(ito::retError, 0, tr("addressed relation outside current relation list range (idx = %1, range = 0..%2)").arg(idx).arg(m_pInfo->m_relationsList.size() - 1).toLatin1().data());
    }

    relationShip newRelation;

    newRelation.myWidget = NULL;

    switch(relation->getSize(1))
    {
        case 4:
        default:
            if (relation->getType() == ito::tFloat32)
                newRelation.extValue = relation->at<ito::float32>(0,3);
            else
                newRelation.extValue = (ito::float32)(relation->at<ito::float64>(0,3));

        case 3:
            if (relation->getType() == ito::tFloat32)
                newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float32>(0,2));
            else
                newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float64>(0,2));

        case 2:
            if (relation->getType() == ito::tFloat32)
                newRelation.type = (ito::int32)(relation->at<ito::float32>(0,1));
            else
                newRelation.type = (ito::int32)(relation->at<ito::float64>(0,1));

        case 1:
            if (relation->getType() == ito::tFloat32)
                newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float32>(0,0));
            else
                newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float64>(0,0));
        break;

        case 0:
            return ito::RetVal(ito::retError, 0, tr("set relation failed due to empty vector").toLatin1().data());
    }

    m_pInfo->m_relationsList.insert(idx, newRelation);
//    m_pInfo->m_relationsList[idx] = newRelation;
    if (m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> EvaluateGeometricsFigure::getRelations(void) const
{
    if (m_pInfo->m_relationsList.size() == 0)
    {
        return QSharedPointer<ito::DataObject>(new ito::DataObject());
    }

    QSharedPointer<ito::DataObject> exportedData(new ito::DataObject(4, m_pInfo->m_relationsList.size(), ito::tFloat32));

    cv::Mat * myMat = exportedData->getCvPlaneMat(0);
    ito::float32* ptr = NULL;
    for (int i = 0; i < m_pInfo->m_relationsList.size(); i ++)
    {
        ptr = myMat->ptr<ito::float32>(i);
        ptr[0] = (ito::float32)m_pInfo->m_relationsList[i].firstElementIdx;
        ptr[1] = (ito::float32)m_pInfo->m_relationsList[i].type;
        ptr[2] = (ito::float32)m_pInfo->m_relationsList[i].secondElementIdx;
        ptr[3] = (ito::float32)m_pInfo->m_relationsList[i].extValue;
    }

    return exportedData;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> EvaluateGeometricsFigure::getCurrentRelation(void) const
{
    QModelIndex idx = m_pContent->currentIndex();
    QTreeWidgetItem *curItm = m_pContent->currentItem();

    if (m_pInfo->m_relationsList.size() == 0 || !idx.isValid() || !curItm->parent())
    {
        return QSharedPointer<ito::DataObject>(new ito::DataObject());
    }

    QSharedPointer<ito::DataObject> exportedData(new ito::DataObject(5, 1, ito::tFloat32));
    ito::float32* ptr = (ito::float32*)(*exportedData).rowPtr(0, 0);
    QVariant relId = curItm->data(0, Qt::UserRole);
    relationShip curRel = m_pContent->m_pData->m_relationsList[relId.toInt()];
    ptr[0] = curRel.firstElementIdx;
    ptr[1] = curRel.type;
    ptr[2] = curRel.secondElementIdx;
    ptr[3] = curRel.extValue;
    ptr[4] = relId.toInt();

  //  ptr[0] = curItm

    return exportedData;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::addRelation(QSharedPointer<ito::DataObject> relation)
{
    relationShip newRelation;

    if (relation.isNull() || relation->getDims() != 2 || relation->getSize(0) != 1)
    {
        return ito::RetVal(ito::retError, 0, tr("relation data object must be a 1xM float32 or float64 data object (M=1..4)").toLatin1().data());
    }
    else if ((relation->getType() != ito::tFloat32 && relation->getType() != ito::tFloat64))
    {
        return ito::RetVal(ito::retError, 0, tr("set relation failed due to invalid object type").toLatin1().data());
    }

    newRelation.myWidget = NULL;

    switch(relation->getSize(1))
    {
        case 4:
        default:
            if (relation->getType() == ito::tFloat32)
                newRelation.extValue = relation->at<ito::float32>(0,3);
            else
                newRelation.extValue = (ito::float32)(relation->at<ito::float64>(0,3));
        case 3:
            if (relation->getType() == ito::tFloat32)
                newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float32>(0,2));
            else
                newRelation.secondElementIdx = (ito::int32)(relation->at<ito::float64>(0,2));
        case 2:
            if (relation->getType() == ito::tFloat32)
                newRelation.type = (ito::int32)(relation->at<ito::float32>(0,1));
            else
                newRelation.type = (ito::int32)(relation->at<ito::float64>(0,1));
        case 1:
            if (relation->getType() == ito::tFloat32)
                newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float32>(0,0));
            else
                newRelation.firstElementIdx = (ito::int32)(relation->at<ito::float64>(0,0));
        break;
        case 0:
            return ito::RetVal(ito::retError, 0, tr("set relation failed due to empty vector").toLatin1().data());
    }

    m_pInfo->m_relationsList.append(newRelation);

    m_lastAddedRelation = m_pInfo->m_relationsList.size() -1;

    if (m_pContent)
    {
        m_pContent->updateRelationShips(false);
        m_pContent->expandAll();
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::addRelationName(const QString newName)
{
    ito::RetVal retval(ito::retOk);

    int idx = m_pInfo->m_relationNames.indexOf(newName);

    if (idx < 7 && idx != -1)
    {
        retval = ito::RetVal(ito::retError, 0, tr("add relation name failed: relation as one of the restricted relations").toLatin1().data());
        return retval;
    }

    if (idx == -1)
    {
        m_pInfo->m_relationNames.append(newName);
    }
    else
    {
        m_pInfo->m_relationNames[idx] = newName;
        retval = ito::RetVal(ito::retWarning, 0, tr("add relation name exited with warning: relation already existed").toLatin1().data());
    }
    return retval;
}

//---------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::geometricShapeChanged(int idx, ito::Shape shape)
{
    if (m_pContent)
    {
        return m_pContent->updateElement(idx, shape);
    }
    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuAddRelation()
{
    DialogAddRelation *dlg = new DialogAddRelation(*m_pInfo, this);
    if (dlg->exec() == QDialog::Accepted)
    {
//        dlg->getData(*m_pInfo);

        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuDeleteRelation()
{
    QVector<ito::Shape> shapes = getGeometricShapes();
    int setItem1 = -1, setItem2 = -1;
    int curItem = getCurrentItem();
    QSharedPointer<ito::DataObject> curRel = getCurrentRelation();
    if (curItem < 0 || !curRel || curRel->getDims() == 0)
    {
        QMessageBox::critical(NULL, "Error", "No relationship selected - select relationship first, aborting!");
        return;
    }

    DialogDeleteRelation *dlg = new DialogDeleteRelation(*m_pInfo, this);
    if (dlg->exec() == QDialog::Accepted)
    {
//        dlg->getData(*m_pInfo);

        m_pContent->updateRelationShips(false);
    }

    delete dlg;
    dlg = NULL;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuUpdate()
{
    if (m_pContent)
    {
        m_pContent->updateRelationShips(false);
    }
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuAutoFitCols()
{
    if (m_pContent)
    {
        m_pContent->autoFitCols();
    }
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuFitToObject()
{
    int currentItem = getCurrentItem();
    emit fitToObject(currentItem);
}

//---------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::clearAll(void)
{
    m_pInfo->m_relationsList.clear();

    if (m_pContent)
    {
        QVector<ito::Shape> shapes;
        m_pContent->setShapes(shapes);
    }
    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setRelationNames(const QStringList input)
{
    if(input.size() < 7)
    {
        return;
    }

    if(m_pInfo->m_relationNames.length() < input.size())
    {
        m_pInfo->m_relationNames.reserve(input.length());
    }

    for( int i = 6; i < input.length(); i++)
    {
        int idx = m_pInfo->m_relationNames.indexOf(input[i]);
        if(idx < 7 && idx != -1)
        {
            continue;
        }

        if(m_pInfo->m_relationNames.length() > i)
        {
            m_pInfo->m_relationNames[i] = input[i];
        }
        else
        {
            m_pInfo->m_relationNames.append(input[i]);
        }
    }

    while(m_pInfo->m_relationNames.length() > input.size() && m_pInfo->m_relationNames.length() > 6)
    {
        m_pInfo->m_relationNames.removeLast();
    }
    return;
}

//---------------------------------------------------------------------------------------------------------
QStringList EvaluateGeometricsFigure::getRelationNames(void) const
{
    return m_pInfo->m_relationNames;
}

//---------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::init() //called when api-pointers are transmitted, directly after construction
{
    return m_pContent->init();
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::clearRelation(const bool apply)
{
    m_pInfo->m_relationsList.clear();
}

//---------------------------------------------------------------------------------------------------------
QPixmap EvaluateGeometricsFigure::renderToPixMap(const int xsize, const int ysize, const int resolution)
{
    QSizeF curSize(xsize, ysize);
    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = QSizeF(m_pContent->width(), m_pContent->height());
    }

    QPixmap destinationImage(xsize, ysize);

    if(!m_pContent)
    {
        destinationImage.fill(Qt::red);
        return destinationImage;
    }
    destinationImage.fill(Qt::white);

    QPainter painter( &destinationImage );


    int ySpacing = m_pInfo->m_rowPrintSpacing;
    int ySpacingTp = m_pInfo->m_tpPrintSpacing;
    int xSpacing = m_pInfo->m_columnPrintSpacing;
    int yStartPos = 5;

    int linesize = m_pContent->iconSize().height() + ySpacing;

    //if(m_pContent->topLevelItemCount() > 0) yStartPos = (m_pContent->iconSize().height() - m_pContent->topLevelItem(0)->font(0).pixelSize()) / 2;

    QPoint pos(m_pContent->iconSize().width() + 4,yStartPos);
    QPoint posI(0,0);

    for (int topItem = 0; topItem < m_pContent->topLevelItemCount(); topItem++)
    {
        pos.setX(m_pContent->iconSize().width() + xSpacing);
        posI.setX(0);
        painter.setFont(  m_pContent->topLevelItem(topItem)->font(0) );
        painter.drawStaticText(pos, (QStaticText)m_pContent->topLevelItem(topItem)->text(0));
        painter.drawPixmap(posI, m_pContent->topLevelItem(topItem)->icon(0).pixmap(m_pContent->iconSize()));
        pos.setY(pos.y() + linesize);
        posI.setY(posI.y() + linesize);
        if (m_pContent->topLevelItem(topItem)->childCount() > 0)
        {
            pos.setX(30 + m_pContent->iconSize().width() + xSpacing);
            posI.setX(30);
            for (int childItem = 0; childItem < m_pContent->topLevelItem(topItem)->childCount(); childItem++)
            {
                painter.setFont(  m_pContent->topLevelItem(topItem)->child(childItem)->font(0) );
                painter.drawStaticText(pos, (QStaticText)m_pContent->topLevelItem(topItem)->child(childItem)->text(0));
                painter.drawPixmap(posI, m_pContent->topLevelItem(topItem)->child(childItem)->icon(0).pixmap(m_pContent->iconSize()));
                pos.setY(pos.y() + linesize);
                posI.setY(posI.y() + linesize);
            }
        }
        pos.setY(pos.y() + ySpacingTp);
        posI.setY(posI.y() + ySpacingTp);
    }
    pos.setX(0);
    for (int col = 1; col < m_pContent->columnCount(); col++)
    {
        pos.setX(pos.x() + m_pContent->columnWidth(col - 1) + xSpacing);
        pos.setY(yStartPos);
        for (int topItem = 0; topItem < m_pContent->topLevelItemCount(); topItem++)
        {

            painter.setFont(  m_pContent->topLevelItem(topItem)->font(col) );
            painter.drawStaticText(pos, (QStaticText)m_pContent->topLevelItem(topItem)->text(col));
            pos.setY(pos.y() + linesize);

            if (m_pContent->topLevelItem(topItem)->childCount() > 0)
            {
                for (int childItem = 0; childItem < m_pContent->topLevelItem(topItem)->childCount(); childItem++)
                {
                    painter.setFont(  m_pContent->topLevelItem(topItem)->child(childItem)->font(col) );
                     painter.drawStaticText(pos, (QStaticText)m_pContent->topLevelItem(topItem)->child(childItem)->text(col));
                    pos.setY(pos.y() + linesize);
                }
            }
            pos.setY(pos.y() + ySpacingTp);
        }
    }

    return destinationImage;
}

//---------------------------------------------------------------------------------------------------------
int EvaluateGeometricsFigure::getNumberOfDigits() const
{
    return m_pInfo->m_numberOfDigits;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setNumberOfDigits(const int val)
{
    if(val < 0 || val > 6)
    {
        return;
    }
    m_pInfo->m_numberOfDigits = val;
}

//---------------------------------------------------------------------------------------------------------
int EvaluateGeometricsFigure::getPrintRowSpacing(void) const
{
    return m_pInfo->m_rowPrintSpacing;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setPrintRowSpacing(const int val)
{
    if(val < 0 || val > 20)
    {
        return;
    }
    m_pInfo->m_rowPrintSpacing = val;
}

//---------------------------------------------------------------------------------------------------------
int EvaluateGeometricsFigure::getPrintTopLevelRowSpacing(void) const
{
    return m_pInfo->m_tpPrintSpacing;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setPrintTopLevelRowSpacing(const int val)
{
    if(val < 0 || val > 20)
    {
        return;
    }
    m_pInfo->m_tpPrintSpacing = val;
}

//---------------------------------------------------------------------------------------------------------
int EvaluateGeometricsFigure::getPrintColumnSpacing(void) const
{
    return m_pInfo->m_columnPrintSpacing;
}

//---------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setPrintColumnSpacing(const int val)
{
    if(val < 0 || val > 20)
    {
        return;
    }
    m_pInfo->m_columnPrintSpacing = val;
}

//---------------------------------------------------------------------------------------------------------
int EvaluateGeometricsFigure::getCurrentItem(void)
{
    QModelIndex idx = m_pContent->currentIndex();
    if (idx.isValid())
    {
        QString txt = m_pContent->topLevelItem(idx.row())->text(0);
        QRegularExpression re("([0-9]+)");
        auto match = re.match(txt);

        if (match.hasMatch())
        {
            return match.captured().toInt();
        }
    }
    return -1;
}

//---------------------------------------------------------------------------------------------------------
ito::Shape EvaluateGeometricsFigure::getShape(const int idx)
{
    if (m_pContent->m_rowHash.contains(idx))
        return m_pContent->m_rowHash[idx];
    else
        return ito::Shape();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::delRelation(const int idx)
{
    m_pContent->m_pData->m_relationsList.remove(idx);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
