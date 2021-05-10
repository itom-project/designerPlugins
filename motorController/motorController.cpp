/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
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

/**
* \file motorController.cpp
* \brief In this file the functions of the MotorController-Class are defined
*
*    The MotorController-Class defines a widget for generic motor monitoring and controll. The following files are 
*   needed: MotorController.cpp, MotorController.h, MotorControllerFactory.h, MotorControllerFactory.cpp
*
*\sa MotorController, MotorControllerFactory, MotorControllerFactory.h
*\author ITO
*\date    2013
*/

#include "motorController.h"
#include <QSpinBox>
#include <QLayout>
#include <iostream>
//#include <qdebug.h>
#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
MotorController::MotorController(QWidget *parent /*= 0*/)
    : QGroupBox(parent),
    m_pActuator(NULL),
    m_updateBySignal(false),
    m_baseScale(1.0),
    m_readOnly(true),
    m_needStepAdaption(false),
    m_absRelPosition(false),
    m_isUpdating(true),
    m_smallStep(0.001),
    m_bigStep(0.010),
    m_actSetUnit(NULL),
    m_actUpdatePos(NULL),
    m_unitActionGroup(NULL),
    m_mnuSetUnit(NULL),
    m_mnuSetAbsRel(NULL),
    m_actSetAbsRel(NULL),
    m_actSetAutoUpdate(NULL),
    m_mnuSetAutoUpdate(NULL),
    m_unit(mm),
    m_hasJoyStick(false),
    m_enableJoyStick(true),
    m_allowJoyStick(true),
    m_joyModeFast(true),
    m_autoUpdate(false),
    m_pStepFastNegSignalMapper(NULL),
    m_pStepSlowNegSignalMapper(NULL),
    m_pStepFastPosSignalMapper(NULL),
    m_pStepSlowPosSignalMapper(NULL)
{
    m_pStepFastNegSignalMapper = new QSignalMapper(this);
    m_pStepSlowNegSignalMapper = new QSignalMapper(this);
    m_pStepFastPosSignalMapper = new QSignalMapper(this);
    m_pStepSlowPosSignalMapper = new QSignalMapper(this);
    connect(m_pStepFastNegSignalMapper, SIGNAL(mapped(int)), this, SLOT(moveStepFastNeg(int)));
    connect(m_pStepSlowNegSignalMapper, SIGNAL(mapped(int)), this, SLOT(moveStepSlowNeg(int)));
    connect(m_pStepFastPosSignalMapper, SIGNAL(mapped(int)), this, SLOT(moveStepFastPos(int)));
    connect(m_pStepSlowPosSignalMapper, SIGNAL(mapped(int)), this, SLOT(moveStepSlowPos(int)));

    unsigned int numAxisToUse = 6;
    QPushButton *btn;

    setTitle("MotorMonitor");
    m_axisName.clear();
    m_axisName.reserve(6);
    m_axisName.append("x ");
    m_axisName.append("y ");
    m_axisName.append("z ");
    m_axisName.append("a ");
    m_axisName.append("b ");
    m_axisName.append("c ");


    m_relPosNull.resize(numAxisToUse);
    m_relPosNull.fill(0.0, numAxisToUse);

    m_curAbsPos.resize(numAxisToUse);
    m_curAbsPos.fill(0.0, numAxisToUse);

    m_posLabels.reserve(numAxisToUse);
    m_posWidgets.reserve(numAxisToUse);
    m_smallStepWidgets.reserve(numAxisToUse);
    m_largeStepWidgets.reserve(numAxisToUse);
    m_changePosButtons.reserve(numAxisToUse);

    for (unsigned int i = 0; i < numAxisToUse; ++i)
    {
        m_posLabels.append(new QLineEdit(this));
        m_posWidgets.append(new QDoubleSpinBox(this));
        m_smallStepWidgets.append(new QDoubleSpinBox(this));
        m_largeStepWidgets.append(new QDoubleSpinBox(this));

        QList<QPushButton* > buttons;
        buttons.reserve(4);
        QString bname(10);
        
        btn = new QPushButton("--", this);
        bname.sprintf("%c--", m_axisName[i][0].toLatin1());
        btn->setToolTip(bname);
        btn->setMinimumSize(24, 24);
        btn->setMaximumSize(24, 24);
        buttons.append(btn);
        connect(btn, SIGNAL(pressed()), m_pStepFastNegSignalMapper, SLOT(map()));
        m_pStepFastNegSignalMapper->setMapping(btn, i);

        btn = new QPushButton("-", this);
        bname.sprintf("%c-", m_axisName[i][0].toLatin1());
        btn->setToolTip(bname);
        btn->setMinimumSize(24, 24);
        btn->setMaximumSize(24, 24);
        buttons.append(btn);
        connect(btn, SIGNAL(pressed()), m_pStepSlowNegSignalMapper, SLOT(map()));
        m_pStepSlowNegSignalMapper->setMapping(btn, i);

        btn = new QPushButton("+", this);
        bname.sprintf("%c+", m_axisName[i][0].toLatin1());
        btn->setToolTip(bname);
        btn->setMinimumSize(24, 24);
        btn->setMaximumSize(24, 24);
        buttons.append(btn);
        connect(btn, SIGNAL(pressed()), m_pStepSlowPosSignalMapper, SLOT(map()));
        m_pStepSlowPosSignalMapper->setMapping(btn, i);

        btn = new QPushButton("++", this);
        bname.sprintf("%c++", m_axisName[i][0].toLatin1());
        btn->setToolTip(bname);
        btn->setMinimumSize(24, 24);
        btn->setMaximumSize(24, 24);
        buttons.append(btn);
        connect(btn, SIGNAL(pressed()), m_pStepFastPosSignalMapper, SLOT(map()));
        m_pStepFastPosSignalMapper->setMapping(btn, i);

        m_changePosButtons.append(buttons);
    }

    QString micronString(2, 181);
    micronString[1] = 'm';

    //QMenu *contextMenu = new QMenu(QObject::tr("motorController"), this);
    m_actSetUnit = new QAction(tr("Toggle Unit"), this);
    m_unitActionGroup = new QActionGroup(this);
    QAction *a;

    m_mnuSetUnit = new QMenu(tr("Unit Switch"), this);

    a = m_mnuSetUnit->addAction("nm");
    a->setCheckable(true);
    a->setData(nm);
    m_unitActionGroup->addAction(a);

    a = m_mnuSetUnit->addAction(micronString);
    a->setCheckable(true);
    a->setData(micron);
    m_unitActionGroup->addAction(a);

    a = m_mnuSetUnit->addAction("mm");
    a->setCheckable(true);
    a->setData(mm);
    m_unitActionGroup->addAction(a);
    a->setChecked(true);

    a = m_mnuSetUnit->addAction("m");
    a->setCheckable(true);
    a->setData(m);
    m_unitActionGroup->addAction(a);
    
    m_actSetUnit->setMenu(m_mnuSetUnit);
    connect(m_mnuSetUnit, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetUnit(QAction*)));
    

    m_actUpdatePos = new QAction(tr("Update"), this);
    
    m_actSetAbsRel = new QAction(tr("Toggle Abs/Rel"), this);
    m_mnuSetAbsRel = new QMenu(tr("AbsRel-Switch"), this);
    m_mnuSetAbsRel->addAction(tr("abs"));
    m_mnuSetAbsRel->addAction(tr("rel"));
    m_mnuSetAbsRel->addSeparator();
    m_mnuSetAbsRel->addAction(tr("origin"));
    m_actSetAbsRel->setMenu(m_mnuSetAbsRel);

    connect(m_mnuSetAbsRel, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetAbsRel(QAction*)));
    
    connect(m_actUpdatePos, SIGNAL(triggered()), this, SLOT(triggerUpdatePosition()));
    
    //QMenu *contextMenu = new QMenu(QObject::tr("motorController"), this);
    m_actSetAutoUpdate = new QAction(tr("Toggle Update (off)"), this);
    m_mnuSetAutoUpdate = new QMenu(tr("Update Switch"), this);
    m_mnuSetAutoUpdate->addAction(tr("on"));
    m_mnuSetAutoUpdate->addAction(tr("off"));
    m_actSetAutoUpdate->setMenu(m_mnuSetAutoUpdate);

    connect(m_mnuSetAutoUpdate, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetAutoUpdate(QAction*)));
    
    setContextMenuPolicy(Qt::ActionsContextMenu);
    addAction(m_actUpdatePos);
    addAction(m_actSetAutoUpdate);
    m_mnuSetAbsRel->addSeparator();
    addAction(m_actSetUnit);
    addAction(m_actSetAbsRel);

    m_numAxis = m_posWidgets.length();
    m_numVisAxis = m_posWidgets.length();

    QString unitString;

    if (m_unit != micron)
    {
        QMetaEnum me = metaObject()->enumerator( metaObject()->indexOfEnumerator("Unit") );
        unitString = me.valueToKey(m_unit);
    }
    else
    {
        unitString = micronString;
    }

    for (int i = 0; i < m_numVisAxis; i++)
    {
        m_posWidgets[i]->setReadOnly(true);
        m_posWidgets[i]->setSuffix(unitString);
        m_posWidgets[i]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        m_posWidgets[i]->setMinimum(-9999.999);
        m_posWidgets[i]->setMaximum(9999.999);
        m_posWidgets[i]->setDecimals(3);
        
        m_posLabels[i]->setText(m_axisName[i]);
        m_posLabels[i]->setReadOnly(true);
        m_posLabels[i]->setMaximumWidth(30);

        m_smallStepWidgets[i]->setSuffix(unitString);
        m_smallStepWidgets[i]->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        m_smallStepWidgets[i]->setMinimum(0.00);
        m_smallStepWidgets[i]->setMaximum(999.999);
        m_smallStepWidgets[i]->setDecimals(3);
        m_smallStepWidgets[i]->setKeyboardTracking(false);

        m_largeStepWidgets[i]->setSuffix(unitString);
        m_largeStepWidgets[i]->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        m_largeStepWidgets[i]->setMinimum(0.00);
        m_largeStepWidgets[i]->setMaximum(999.999);
        m_largeStepWidgets[i]->setDecimals(3);
        m_largeStepWidgets[i]->setKeyboardTracking(false);

        connect(m_smallStepWidgets[i], SIGNAL(valueChanged(double)), this, SLOT(guiChangedSmallStep(double)));
        connect(m_largeStepWidgets[i], SIGNAL(valueChanged(double)), this, SLOT(guiChangedLargeStep(double)));
    }

    m_axisGroups.clear();
    m_axisGroups.reserve(numAxisToUse);
    m_axisGroups.append(new QGroupBox(this));
    m_axisGroups.append(new QGroupBox(this));
    m_axisGroups.append(new QGroupBox(this));
    m_axisGroups.append(new QGroupBox(this));
    m_axisGroups.append(new QGroupBox(this));
    m_axisGroups.append(new QGroupBox(this));

    QHBoxLayout* mainLayout = new QHBoxLayout(this);

    mainLayout->setContentsMargins(2,2,2,2);

    for (unsigned int i = 0; i < numAxisToUse; i++)
    {
        QHBoxLayout* line1 = new QHBoxLayout();
        QHBoxLayout* line2 = new QHBoxLayout();
        QHBoxLayout* line3 = new QHBoxLayout();

        QVBoxLayout* colLayOut = new QVBoxLayout();

        line1->addWidget(m_posLabels[i]);
        line1->addWidget(m_posWidgets[i]);

        line1->setContentsMargins(0,0,0,0);
        line1->setSpacing(0);

        line2->addWidget(m_changePosButtons[i][0]);
        line2->addWidget(m_largeStepWidgets[i]);
        line2->addWidget(m_changePosButtons[i][3]);

        line2->setContentsMargins(0,0,0,0);
        line2->setSpacing(2);

        line3->addWidget(m_changePosButtons[i][1]);
        line3->addWidget(m_smallStepWidgets[i]);
        line3->addWidget(m_changePosButtons[i][2]);

        line3->setContentsMargins(0,0,0,0);
        line3->setSpacing(2);

        colLayOut->addLayout(line1);
        colLayOut->addLayout(line2);
        colLayOut->addLayout(line3);

        colLayOut->setContentsMargins(0,0,0,0);
        colLayOut->setSpacing(2);

        m_axisGroups[i]->setLayout(colLayOut);
        m_axisGroups[i]->setContentsMargins(1,1,1,1);

        m_axisGroups[i]->setTitle("");

        mainLayout->addWidget(m_axisGroups[i]);
    }

    this->setLayout(mainLayout);

    if (m_readOnly)
    {
        for (int i = 0; i < m_numVisAxis; i++)
        {
            m_changePosButtons[i][0]->setVisible(false);
            m_changePosButtons[i][1]->setVisible(false);
            m_changePosButtons[i][2]->setVisible(false);
            m_changePosButtons[i][3]->setVisible(false);

            m_smallStepWidgets[i]->setVisible(false);
            m_largeStepWidgets[i]->setVisible(false);
        }   
    }

#if (CONNEXION_ENABLE)
    /*
    *  Initialize the 3D mouse
    */
    m_SpwDeviceHandle = SI_NO_HANDLE;
    m_conNeedsTermination = false;
#endif

    m_timer.setInterval(1000);
    m_timer.setParent(this);
    m_timer.setSingleShot(false);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(triggerUpdatePosition()));

    resizeEvent(NULL);
    m_isUpdating = false;

    setEnabled(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::resizeEvent(QResizeEvent * event)
{
    /*
    int x = 0;
    int y = 0;
    int border = isFlat() ? 1 : 5;
    int upper = title().length() > 0 ? 10:1;
    int curpX = 0, curpY = 0;
    if (m_readOnly)
    {
        for (int i = 0; i < m_numVisAxis; i++)
        {
            curpX = 75 * x + border;
            curpY = 25 * y + border + upper;
            m_posWidgets[i]->setGeometry(curpX,  curpY, 70, 20);
            x++;
            if (x * 75 + 70 > size().width())
            {
                y++;
                x = 0;
            }
        }
        this->setMinimumHeight(curpY + 22 + border);
    }
    else
    {
        
        for (int i = 0; i < m_numVisAxis; i++)
        {
            curpX = 75 * x + border;
            curpY = 45 * y + border + upper;
            m_posWidgets[i]->setGeometry(curpX, curpY, 70, 20);
            for (int n = 0; n < 4; n++)
            {
                m_changePosButtons[i][n]->setGeometry(curpX + n * 17, curpY + 21, 17, 15);
            }
            x++;
            if (x * 75 + 70 > size().width())
            {
                y++;
                x = 0;
            }
        }
        this->setMinimumHeight(curpY + 42 + border);
    }
    */
    for (int i = 0; i < m_numVisAxis; i++)
    {
        m_axisGroups[i]->setVisible(true);
    }

    for (int i = m_numVisAxis; i < m_axisGroups.size(); i++)
    {
        m_axisGroups[i]->setVisible(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::initializeJoystick()
{
#if (CONNEXION_ENABLE)
    SiInitialize ();
    SiOpenWinInit (&m_SpwData, this->effectiveWinId());
    m_SpwDeviceHandle = SiOpen ("isoWidget", SI_ANY_DEVICE, SI_NO_MASK, SI_EVENT, &m_SpwData);

    if (m_SpwDeviceHandle == SI_NO_HANDLE)
    {
        SiTerminate ();
        m_SpwDeviceHandle = NULL;
    }
    else
    {
        m_conNeedsTermination = true;
    }

    SiSetUiMode (m_SpwDeviceHandle, SI_UI_ALL_CONTROLS);

    if (m_SpwDeviceHandle != SI_NO_HANDLE)
    {
        m_hasJoyStick = connect(this, SIGNAL(TriggerSoftJoyStickMovement(QVector<int>, QVector<double>)), m_pActuator, SLOT(startJoyStickMovement(QVector<int>, QVector<double>)));
        std::cout << "Connected to \"Spass-Stecken\"" << (m_hasJoyStick? "true" : "false") << "\n";
    }
    #endif
}

//----------------------------------------------------------------------------------------------------------------------------------
MotorController::~MotorController()
{
    m_timer.stop();
    if (m_actSetUnit)
    {
        delete m_actSetUnit;
        m_actSetUnit = NULL;
    }
    if (m_actUpdatePos)
    {
        delete m_actUpdatePos;
        m_actUpdatePos = NULL;
    }
    if (m_mnuSetUnit)
    {
        delete m_mnuSetUnit;
        m_mnuSetUnit = NULL;
    }

    if (m_actSetAbsRel)
    {
        delete m_actSetAbsRel;
        m_actSetAbsRel = NULL;
    }

    if (m_mnuSetAbsRel)
    {
        delete m_mnuSetAbsRel;
        m_mnuSetAbsRel = NULL;
    }

    if (m_actSetAutoUpdate)
    {
        delete m_actSetAutoUpdate;
        m_actSetAutoUpdate = NULL;
    }

    if (m_mnuSetAutoUpdate)
    {
        delete m_mnuSetAutoUpdate;
        m_mnuSetAutoUpdate = NULL;
    }

    removeActuator();

#if (CONNEXION_ENABLE)
    if (m_conNeedsTermination)
    {
        if (m_SpwDeviceHandle)
        {
            SiClose (m_SpwDeviceHandle);
        }
        SiTerminate();
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setActuator(QPointer<ito::AddInActuator> actuator)
{
    if (actuator.isNull())
    {
        removeActuator();
        setEnabled(false);
    }
    else
    {
        //at first increment the reference to the new actuator, then delete the old one (and decrement its reference). Finally assign the new one to this widget
        removeActuator();
        m_pActuator = actuator;

        connect(m_pActuator, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), this, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
        connect(m_pActuator, SIGNAL(destroyed()), this, SLOT(actuatorDestroyed()));
        m_updateBySignal = connect(this, SIGNAL(requestStatusAndPosition(bool, bool)), m_pActuator, SLOT(requestStatusAndPosition(bool, bool)));
        
        if (m_allowJoyStick)
        {
            initializeJoystick();
        }

        triggerUpdatePosition();

        // Use invoke or do it directly?
        // directly 
        QMap<QString, ito::Param> *paramList = NULL;
        m_pActuator->getParamList(&paramList);
        m_needStepAdaption = paramList->contains("stepwidth");
        setEnabled(true);
    }

    return;  
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::removeActuator()
{
    if (m_pActuator.isNull() == false)
    {
        disconnect(m_pActuator, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), this, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
        disconnect(m_pActuator, SIGNAL(destroyed()), this, SLOT(actuatorDestroyed()));
        disconnect(this, SIGNAL(requestStatusAndPosition(bool, bool)), m_pActuator, SLOT(requestStatusAndPosition(bool, bool)));
        m_updateBySignal = false;
        m_pActuator = NULL; //same as ito::actuator.clear(); which does not exist in Qt4
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::actuatorDestroyed()
{
    removeActuator();
    setEnabled(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::triggerUpdatePosition(void)
{
    if (m_pActuator.isNull())
    {
        return;
    }

    if (m_updateBySignal)
    {
        emit requestStatusAndPosition(true, false);
    }
    else
    {
        ito::RetVal retval;
        int axisNumbers = 0;
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        QSharedPointer<ito::Param> qsParam(new ito::Param("numaxis", ito::ParamBase::Int));
        QMetaObject::invokeMethod(m_pActuator, "getParam", Q_ARG(QSharedPointer<ito::Param>, qsParam), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while (!locker.getSemaphore()->wait(500))
        {
            retval += ito::RetVal(ito::retError, 0, tr("timeout while getting numaxis parameter").toLatin1().data());
            break;
        }

        if (!retval.containsError())
        {
            retval += locker.getSemaphore()->returnValue;
        }

        if (!retval.containsError())
        {
            axisNumbers = (*qsParam).getVal<int>();
        }

        if (axisNumbers != 0)
        {
            if (m_numVisAxis == 0)
            {
                setNumAxis(axisNumbers);
            }
            axisNumbers = axisNumbers > 6 ? 6 : axisNumbers;

            QVector<int> status(axisNumbers);
            status.fill(0);

            QVector<int> axisNo(axisNumbers);
            for (int i = 0; i < axisNumbers; i++)
            {
                axisNo[i] = i;
            }

            ItomSharedSemaphoreLocker posLocker(new ItomSharedSemaphore());
            QSharedPointer<QVector<double> > qsVector(new QVector<double>(axisNumbers, 0.0));
            QMetaObject::invokeMethod(m_pActuator, "getPos", Q_ARG(const QVector<int>, axisNo), Q_ARG(QSharedPointer<QVector<double> >, qsVector), Q_ARG(ItomSharedSemaphore*, posLocker.getSemaphore()));

            while (!posLocker.getSemaphore()->wait(5000))
            {
                retval += ito::RetVal(ito::retError, 0, tr("timeout while getting numaxis parameter").toLatin1().data());
                break;
            }

            if (!retval.containsError())
            {
                retval += posLocker.getSemaphore()->returnValue;
            }

            if (!retval.containsError())
            {
                actuatorStatusChanged(status, *qsVector);
            }
        }
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setNumAxis(const int numAxis)
{
    bool change = m_numVisAxis != numAxis ? true : false;
    if (numAxis > 0 && numAxis < (m_numAxis + 1))
    {
        m_numVisAxis = numAxis;
    }
    if (change)
    {
        if (m_numVisAxis == 1)
        {
            m_axisGroups[0]->setFlat(true);
        }
        else
        {
            m_axisGroups[0]->setFlat(false);
        }

        int i = 0;
        for (; i < m_numVisAxis; i++)
        {
            m_axisGroups[i]->setVisible(true);
        }
        for (; i < m_axisGroups.length(); i++)
        {
            m_axisGroups[i]->setVisible(false);
        }

        i = 0;


        if (!m_readOnly)
        {
            for (; i < m_numVisAxis; i++)
            {
                m_largeStepWidgets[i]->setVisible(true);
                m_smallStepWidgets[i]->setVisible(true);
                m_changePosButtons[i][0]->setVisible(true);
                m_changePosButtons[i][1]->setVisible(true);
                m_changePosButtons[i][2]->setVisible(true);
                m_changePosButtons[i][3]->setVisible(true);
            }
            for (; i < m_posWidgets.length(); i++)
            {
                m_largeStepWidgets[i]->setVisible(false);
                m_smallStepWidgets[i]->setVisible(false);
                m_changePosButtons[i][0]->setVisible(false);
                m_changePosButtons[i][1]->setVisible(false);
                m_changePosButtons[i][2]->setVisible(false);
                m_changePosButtons[i][3]->setVisible(false);
            }
        }

        resizeEvent(NULL);
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSize MotorController::sizeHint() const
{
    
    return QSize(10,10);
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    if (status.size() != positions.size())
    return;

//    bool running = false;
    QString style;
    unsigned char size = status.size();
    size = size > 6 ? 6 : size;

    for (int i = 0; i < size; i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
//            running = true;
        }
        else if ((status[i] & ito::actuatorInterrupted) || (status[i] & ito::actuatorError))
        {
            style = "background-color: red";
        }
        //else if (status[i] & ito::actuatorTimeout) //timeout is bad for dummyMotor, since the waitForDone-method always drops into a timeout
        //{
        //    style = "background-color: green";
        //}
        else
        {
            style = "background-color: ";
        }

        m_posWidgets[i]->setStyleSheet(style);
        m_curAbsPos[i] = positions[i] * m_baseScale;
        if (m_absRelPosition)
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i] - m_relPosNull[i]);
        }
        else
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i]);
        }
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setUnit(const Unit unit)
{
    double oldScale = m_baseScale;

    switch(unit)
    {
    case mm:
        m_baseScale = 1.0;
        break;
    case micron:
        m_baseScale = 1000.0;
        break;
    case nm:
        m_baseScale = 1000000.0;
        break;
    case m:
        m_baseScale = 0.001;
        break;
    case km:
        m_baseScale = 0.000001;
        break;
    default:
        return;
    }

    QString unitString;

    if (unit != micron)
    {
        QMetaEnum me = metaObject()->enumerator( metaObject()->indexOfEnumerator("Unit") );
        unitString = me.valueToKey(unit);
    }
    else
    {
        unitString = QString(2, 181);
        unitString[1] = 'm';
    }

    m_unit = unit;

    for (int i = 0; i < m_numVisAxis; i++)
    {
        m_curAbsPos[i] *= m_baseScale / oldScale;
        m_relPosNull[i] *= m_baseScale / oldScale;

        if (m_absRelPosition)
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i] - m_relPosNull[i]);            
        }
        else
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i]);    
        }
        m_posWidgets[i]->setSuffix(unitString);
        m_smallStepWidgets[i]->setSuffix(unitString);
        m_smallStepWidgets[i]->setValue(m_smallStep * m_baseScale);
        m_largeStepWidgets[i]->setSuffix(unitString);
        m_largeStepWidgets[i]->setValue(m_bigStep * m_baseScale);
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setReadOnly(const bool value)
{
    bool changed = m_readOnly != value;
    m_readOnly = value;

    if (changed)
    {
        for (int i = 0; i < m_numVisAxis; i++)
        {
            m_largeStepWidgets[i]->setVisible(!m_readOnly);
            m_smallStepWidgets[i]->setVisible(!m_readOnly);
            m_changePosButtons[i][0]->setVisible(!m_readOnly);
            m_changePosButtons[i][1]->setVisible(!m_readOnly);
            m_changePosButtons[i][2]->setVisible(!m_readOnly);
            m_changePosButtons[i][3]->setVisible(!m_readOnly);
        }

        resizeEvent(NULL);
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::mnuSetUnit(QAction* inputAction)
{
    if (inputAction)
    {
        setUnit((Unit)inputAction->data().toInt());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::mnuSetAbsRel(QAction* inputAction)
{
    if (inputAction->text() == QString(tr("abs")))
    {
        setAbsRel(false);
    }
    else if (inputAction->text() == QString(tr("rel")))
    {
        setAbsRel(true);
    }
    else if (inputAction->text() == QString(tr("origin")))
    {
        m_relPosNull = m_curAbsPos;
        setAbsRel(m_absRelPosition);
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setSmallStep(const double value)
{
    m_isUpdating = true;
    if (value > 0.0 && value < 1.0)
    {
        m_smallStep = value;
    }

    for (int i = 0; i < m_smallStepWidgets.size(); i++) 
    {
        m_smallStepWidgets[i]->setValue(m_smallStep * m_baseScale);    
    }


    m_isUpdating = false;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setBigStep(const double value)
{
    m_isUpdating = true;
    if (value > 0.0 && value < 1000.0)
    {
        m_bigStep = value;
    }

    for (int i = 0; i < m_smallStepWidgets.size(); i++) 
    {
        m_largeStepWidgets[i]->setValue(m_bigStep * m_baseScale);    
    }
    m_isUpdating = false;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setAbsRel(const bool absRel)
{
    m_absRelPosition = absRel;
    QChar axisName;
    
    char text[4] = {'%','c', ' ', 0};
    if (m_absRelPosition)
    {
        text[2] = '\'';
    }
    for (int i = 0; i < m_posWidgets.size(); i ++)
    {
        axisName = m_axisName[i][0];
        m_axisName[i].sprintf(text, axisName.toLatin1());

        m_posLabels[i]->setText(m_axisName[i]);
    }
    triggerUpdatePosition();
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::moveStepFastNeg(int index)
{
    triggerActuatorStep(index, true, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::moveStepSlowNeg(int index)
{
    triggerActuatorStep(index, false, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::moveStepFastPos(int index)
{
    triggerActuatorStep(index, true, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::moveStepSlowPos(int index)
{
    triggerActuatorStep(index, false, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::triggerActuatorStep(const int axisNo, const bool smallBig, const bool forward)
{
    if (m_pActuator.isNull())
        return;

    double step = smallBig ? m_bigStep : m_smallStep;
    step = forward ? step : -1 * step;

    bool ready = true;

    if (m_needStepAdaption)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        QSharedPointer<ito::ParamBase> qsParam(new ito::ParamBase("stepwidth", ito::ParamBase::Double, step));
        QMetaObject::invokeMethod(m_pActuator, "setParam", Q_ARG(QSharedPointer<ito::ParamBase>, qsParam), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while (!locker.getSemaphore()->wait(5000))
        {
            ready = false;
            break;
        }

        if (locker.getSemaphore()->returnValue.containsError())
        {
            ready = false;
            std::cout << locker.getSemaphore()->returnValue.errorMessage() << "\n";
        }   
    }

    if (ready)
    {
        QMetaObject::invokeMethod(m_pActuator, "setPosRel", Q_ARG(const int, axisNo), Q_ARG(const double, step), Q_ARG(ItomSharedSemaphore*, NULL));
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::guiChangedSmallStep(double value)
{
    if (m_isUpdating) return;

    setSmallStep(value / m_baseScale);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::guiChangedLargeStep(double value)
{
    if (m_isUpdating) return;

    setBigStep(value / m_baseScale);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::setAutoUpdate(const bool value)
{
    m_autoUpdate = value;

    if (m_autoUpdate == true)
    {
        m_actSetAutoUpdate->setText(tr("Toggle Update (on)"));
        m_timer.start();
    }
    else
    {
        m_actSetAutoUpdate->setText(tr("Toggle Update (off)"));
        m_timer.stop();
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MotorController::mnuSetAutoUpdate(QAction* inputAction)
{
    if (inputAction->text() == tr("on"))
    {
        setAutoUpdate(true);
    }
    else if (inputAction->text() == tr("off"))
    {
        setAutoUpdate(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
#if CONNEXION_ENABLE // Only of CONNEXION is enabled
bool MotorController::winEvent(MSG * message, long * result)
{
    std::cout << "Try to match event\n";
    if (!(m_hasJoyStick && m_enableJoyStick && m_allowJoyStick))
        return false;

    std::cout << "Got event\n";

    int            num;      /* number of button returned */
    SiSpwEvent     pEvent;    /* SpaceWare Event */ 
    SiGetEventData EData;    /* SpaceWare Event Data */
   
    /* init Window platform specific data for a call to SiGetEvent */
    SiGetEventWinInit(&EData, message->message, message->wParam, message->lParam);
  
    /* check whether msg was a 3D mouse event and process it */
    if (SiGetEvent (m_SpwDeviceHandle, 0, &EData, &pEvent) == SI_IS_EVENT)
    {
        QVector<int> axis(1);
        axis[0] = 0;
        QVector<double> vel(1);
        switch (pEvent.type)
        {
            case SI_MOTION_EVENT:
            {
                //pEvent.u.spwData.mData[SI_TX];
                //pEvent.u.spwData.mData[SI_TY];
                //pEvent.u.spwData.mData[SI_TZ];
                //pEvent.u.spwData.mData[SI_RX];
                //pEvent.u.spwData.mData[SI_RY];
                //pEvent.u.spwData.mData[SI_RZ];
                if (abs(pEvent.u.spwData.mData[SI_RY]) > 2)
                {
                    vel[0] = (m_joyModeFast ? m_bigStep : m_smallStep) *  pEvent.u.spwData.mData[SI_RY];
                    std::cout << "Gotcha\n";
                }
                else
                {
                    vel[0] = 0.0;
                }
                emit TriggerSoftJoyStickMovement(axis, vel);
                this->setEnabled(false);
            }
            break;
           
            case SI_ZERO_EVENT:
            {
                vel[0] = 0.0;
                emit TriggerSoftJoyStickMovement(axis, vel);
                this->setEnabled(false);
            }
            break;
           
            case  SI_BUTTON_EVENT:
            
            if ((num = SiButtonPressed (&pEvent)) != SI_NO_BUTTON)    
            {
                m_joyModeFast = m_joyModeFast ? false : true; 
            }
            /*
            if ((num = SiButtonReleased (&Event)) != SI_NO_BUTTON)    
            {
            SbButtonReleaseEvent(num);   // process 3D mouse button event
            }
            */
            break;
        
        } // end switch
    } /* end SiGetEvent */
    return true;
}
#endif
