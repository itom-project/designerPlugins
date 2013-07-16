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

#include "motorController.h"
#include <qspinbox.h>


//-----------------------------------------------------------------------------------------------
MotorController::MotorController(QWidget *parent /*= 0*/)
    : QGroupBox(parent),
    m_pActuator(NULL),
    m_updateBySignal(false),
    m_baseScale(1.0),
    m_readOnly(true),
    m_needStepAdaption(false),
    m_absRelPosition(false),
    m_smallStep(0.001),
    m_bigStep(0.010),
    m_actSetUnit(NULL),
    m_actUpdatePos(NULL),
    m_mnuSetUnit(NULL),
    m_mnuSetAbsRel(NULL),
    m_actSetAbsRel(NULL),
    m_unit("mm")
{
    setTitle("MotorMonitor");
    m_axisName.clear();
    m_axisName.reserve(6);
    m_axisName.append("x ");
    m_axisName.append("y ");
    m_axisName.append("z ");
    m_axisName.append("a ");
    m_axisName.append("b ");
    m_axisName.append("c ");


    m_relPosNull.resize(6);
    m_relPosNull.fill(0.0, 6);

    m_curAbsPos.resize(6);
    m_curAbsPos.fill(0.0, 6);

    m_posWidgets.clear();
    m_posWidgets.reserve(6);
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    
    m_changePosButtons.clear();
    m_changePosButtons.reserve(6);

    for(int i = 0; i < 6; i++)
    {
        QList<QPushButton* > buttons;
        buttons.reserve(4);
        QString bname(10);
        
        buttons.append(new QPushButton("--", this));
        bname.sprintf("%c--", m_axisName[i][0]);
        buttons[0]->setToolTip(bname);
        
        buttons.append(new QPushButton("-", this));
        bname.sprintf("%c-", m_axisName[i][0]);
        buttons[1]->setToolTip(bname);

        buttons.append(new QPushButton("+", this));
        bname.sprintf("%c+", m_axisName[i][0]);
        buttons[2]->setToolTip(bname);

        buttons.append(new QPushButton("++", this));
        bname.sprintf("%c++", m_axisName[i][0]);
        buttons[3]->setToolTip(bname);

        m_changePosButtons.append(buttons); 
    }

    connect(m_changePosButtons[0][0], SIGNAL(pressed()), this, SLOT(axis0BigStepMinus()));
    connect(m_changePosButtons[0][1], SIGNAL(pressed()), this, SLOT(axis0SmallStepMinus()));
    connect(m_changePosButtons[0][2], SIGNAL(pressed()), this, SLOT(axis0SmallStepPlus()));
    connect(m_changePosButtons[0][3], SIGNAL(pressed()), this, SLOT(axis0BigStepPlus()));

    connect(m_changePosButtons[1][0], SIGNAL(pressed()), this, SLOT(axis1BigStepMinus()));
    connect(m_changePosButtons[1][1], SIGNAL(pressed()), this, SLOT(axis1SmallStepMinus()));
    connect(m_changePosButtons[1][2], SIGNAL(pressed()), this, SLOT(axis1SmallStepPlus()));
    connect(m_changePosButtons[1][3], SIGNAL(pressed()), this, SLOT(axis1BigStepPlus()));

    connect(m_changePosButtons[2][0], SIGNAL(pressed()), this, SLOT(axis2BigStepMinus()));
    connect(m_changePosButtons[2][1], SIGNAL(pressed()), this, SLOT(axis2SmallStepMinus()));
    connect(m_changePosButtons[2][2], SIGNAL(pressed()), this, SLOT(axis2SmallStepPlus()));
    connect(m_changePosButtons[2][3], SIGNAL(pressed()), this, SLOT(axis2BigStepPlus()));

    connect(m_changePosButtons[3][0], SIGNAL(pressed()), this, SLOT(axis3BigStepMinus()));
    connect(m_changePosButtons[3][1], SIGNAL(pressed()), this, SLOT(axis3SmallStepMinus()));
    connect(m_changePosButtons[3][2], SIGNAL(pressed()), this, SLOT(axis3SmallStepPlus()));
    connect(m_changePosButtons[3][3], SIGNAL(pressed()), this, SLOT(axis3BigStepPlus()));

    connect(m_changePosButtons[4][0], SIGNAL(pressed()), this, SLOT(axis4BigStepMinus()));
    connect(m_changePosButtons[4][1], SIGNAL(pressed()), this, SLOT(axis4SmallStepMinus()));
    connect(m_changePosButtons[4][2], SIGNAL(pressed()), this, SLOT(axis4SmallStepPlus()));
    connect(m_changePosButtons[4][3], SIGNAL(pressed()), this, SLOT(axis4BigStepPlus()));

    connect(m_changePosButtons[5][0], SIGNAL(pressed()), this, SLOT(axis5BigStepMinus()));
    connect(m_changePosButtons[5][1], SIGNAL(pressed()), this, SLOT(axis5SmallStepMinus()));
    connect(m_changePosButtons[5][2], SIGNAL(pressed()), this, SLOT(axis5SmallStepPlus()));
    connect(m_changePosButtons[5][3], SIGNAL(pressed()), this, SLOT(axis5BigStepPlus()));

    QString micron(2, 181);
    micron[1] = 'm';

    //QMenu *contextMenu = new QMenu(QObject::tr("motorController"), this);
    m_actSetUnit = new QAction(tr("Toogle Unit"), this);
    m_mnuSetUnit = new QMenu("Unit Switch");
	m_mnuSetUnit->addAction("nm");
	m_mnuSetUnit->addAction(micron);
	m_mnuSetUnit->addAction("mm");
	m_mnuSetUnit->addAction("m");
    m_actSetUnit->setMenu(m_mnuSetUnit);
    
    m_actUpdatePos = new QAction(tr("Update"), this);
    
    m_actSetAbsRel = new QAction(tr("Toogle Abs/Rel"), this);
    m_mnuSetAbsRel = new QMenu("AbsRel-Switch");
	m_mnuSetAbsRel->addAction("abs");
	m_mnuSetAbsRel->addAction("rel");
	m_mnuSetAbsRel->addSeparator();
	m_mnuSetAbsRel->addAction("origin");
    m_actSetAbsRel->setMenu(m_mnuSetAbsRel);

    connect(m_mnuSetAbsRel, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetAbsRel(QAction*)));
    
    connect(m_mnuSetUnit, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetUnit(QAction*)));
    connect(m_actUpdatePos, SIGNAL(triggered()), this, SLOT(triggerUpdatePosition()));
    
    setContextMenuPolicy( Qt::ActionsContextMenu );
    addAction( m_actUpdatePos );
    m_mnuSetAbsRel->addSeparator();
    addAction( m_actSetUnit );
    addAction( m_actSetAbsRel );

    m_numAxis = m_posWidgets.length();
    m_numVisAxis = m_posWidgets.length();

    for(int i = 0; i < m_numVisAxis; i++)
    {
        m_posWidgets[i]->setReadOnly(true);
        m_posWidgets[i]->setPrefix(m_axisName[i]);
        m_posWidgets[i]->setSuffix(m_unit);
        m_posWidgets[i]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        m_posWidgets[i]->setMinimum(-99.999);
        m_posWidgets[i]->setMaximum(99.999);
        m_posWidgets[i]->setDecimals(3);
        
    }

    if(m_readOnly)
    {
        for(int i = 0; i < m_numVisAxis; i++)
        {
            m_changePosButtons[i][0]->setVisible(false);
            m_changePosButtons[i][1]->setVisible(false);
            m_changePosButtons[i][2]->setVisible(false);
            m_changePosButtons[i][3]->setVisible(false);
        }   
    }

    resizeEvent(NULL);
    return;
}
void MotorController::resizeEvent(QResizeEvent * event )
{
    int x = 0;
    int y = 0;
    int border = isFlat() ? 1 : 5;
    int upper = title().length() > 0 ? 10:1;
    if(m_readOnly)
    {
        for(int i = 0; i < m_numVisAxis; i++)
        {
            m_posWidgets[i]->setGeometry(75 * x + border, 25 * y + border + upper, 70, 20);
            x++;
            if(x * 75 + 70 > size().width())
            {
                y++;
                x = 0;
            }
        }
    }
    else
    {
        int curpX = 0, curpY = 0;
        for(int i = 0; i < m_numVisAxis; i++)
        {
            curpX = 75 * x + border;
            curpY = 45 * y + border + upper;
            m_posWidgets[i]->setGeometry(curpX, curpY, 70, 20);
            for(int n = 0; n < 4; n++)
            {
                m_changePosButtons[i][n]->setGeometry(curpX + n * 17, curpY + 21, 17, 15);
            }
            x++;
            if(x * 75 + 70 > size().width())
            {
                y++;
                x = 0;
            }
        }    
    }
}

MotorController::~MotorController()
{
    if(m_actSetUnit)
    {
        delete m_actSetUnit;
        m_actSetUnit = NULL;
    }
    if(m_actUpdatePos)
    {
        delete m_actUpdatePos;
        m_actUpdatePos = NULL;
    }
    if(m_mnuSetUnit)
    {
        delete m_mnuSetUnit;
        m_mnuSetUnit = NULL;
    }

    if(m_actSetAbsRel)
    {
        delete m_actSetAbsRel;
        m_actSetAbsRel = NULL;
    }

    if(m_mnuSetAbsRel)
    {
        delete m_mnuSetAbsRel;
        m_mnuSetAbsRel = NULL;
    }

    m_pActuator = NULL;
}

void MotorController::setActuator(QPointer<ito::AddInActuator> actuator)
{
    if(actuator.isNull())
    {
        m_pActuator = NULL;
        setEnabled(false);
    }
    else
    {
        m_pActuator = actuator;

        connect( m_pActuator, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>) ), this, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>) ) );

        m_updateBySignal = connect( this, SIGNAL(RequestStatusAndPosition( bool, bool) ), m_pActuator, SLOT(RequestStatusAndPosition( bool, bool) ) );

        triggerUpdatePosition();

        // Use invoke or do it directly?
        // directly 
        QMap<QString, ito::Param> *paramList = NULL;
        m_pActuator->getParamList(&paramList);
        m_needStepAdaption = paramList->contains("stepwidth");

    }

    return;  
}

void MotorController::triggerUpdatePosition(void)
{
    if(m_pActuator.isNull())
    {
        return;
    }

    if(m_updateBySignal)
    {
        emit RequestStatusAndPosition(true, false);
    }
    else
    {
        ito::RetVal retval;
        int axisNumbers = 0;
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        QSharedPointer<ito::Param> qsParam(new ito::Param("numaxis"));
        QMetaObject::invokeMethod(m_pActuator, "getParam", Q_ARG(QSharedPointer<ito::Param>, qsParam), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while (!locker.getSemaphore()->wait(500))
        {
            retval += ito::RetVal(ito::retError, 0, "timeout while getting numaxis parameter");
            break;
        }

        if(!retval.containsError())
        {
            retval += locker.getSemaphore()->returnValue;
        }

        if(!retval.containsError())
        {
            axisNumbers = (*qsParam).getVal<int>();
        }

        if(axisNumbers != 0)
        {
            if(m_numVisAxis == 0)
            {
                setNumAxis(axisNumbers);
            }
            axisNumbers = axisNumbers > 6 ? 6 : axisNumbers;

            QVector<int> status(axisNumbers);
            status.fill(0);

            QVector<int> axisNo(axisNumbers);
            for(int i = 0; i < axisNumbers; i++)
            {
                axisNo[i] = i;
            }

            ItomSharedSemaphoreLocker posLocker(new ItomSharedSemaphore());
            QSharedPointer<QVector<double>> qsVector(new QVector<double>(axisNumbers, 0.0));
            QMetaObject::invokeMethod(m_pActuator, "getPos", Q_ARG(const QVector<int>, axisNo), Q_ARG(QSharedPointer<QVector<double>>, qsVector), Q_ARG(ItomSharedSemaphore*, posLocker.getSemaphore()));

            while (!posLocker.getSemaphore()->wait(5000))
            {
                retval += ito::RetVal(ito::retError, 0, "timeout while getting numaxis parameter");
                break;
            }

            if(!retval.containsError())
            {
                retval += posLocker.getSemaphore()->returnValue;
            }

            if(!retval.containsError())
            {
                actuatorStatusChanged(status, *qsVector);
            }
        }
    }
    return;
}

void MotorController::setNumAxis(const int numAxis)
{
    bool change = m_numVisAxis != numAxis ? true : false;
    if(numAxis > 0 && numAxis < (m_numAxis + 1))
    {
        m_numVisAxis = numAxis;
    }
    if(change)
    {
        int i = 0;
        for(; i < m_numVisAxis; i++)
        {
            m_posWidgets[i]->setVisible(true);
        }
        for(; i < m_posWidgets.length(); i++)
        {
            m_posWidgets[i]->setVisible(false);
        }

        i = 0;


        if(!m_readOnly)
        {
            for(; i < m_numVisAxis; i++)
            {
                m_changePosButtons[i][0]->setVisible(true);
                m_changePosButtons[i][1]->setVisible(true);
                m_changePosButtons[i][2]->setVisible(true);
                m_changePosButtons[i][3]->setVisible(true);
            }
            for(; i < m_posWidgets.length(); i++)
            {
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

QPointer<ito::AddInActuator> MotorController::getActuator() const
{
    return QPointer<ito::AddInActuator>(NULL);
}

QSize MotorController::sizeHint() const
{
    
    return QSize(10,10);
    
}

void MotorController::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    if(status.size() != positions.size())
    return;

    bool running = false;
    QString style;
    unsigned char size = status.size();
    size = size > 6 ? 6 : size;

    for(int i = 0; i < size; i++)
    {
        if(status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
        }
        else if(status[i] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
        }
        //else if(status[i] & ito::actuatorTimeout) //timeout is bad for dummyMotor, since the waitForDone-method always drops into a timeout
        //{
        //    style = "background-color: green";
        //}
        else
        {
            style = "background-color: ";
        }

        m_posWidgets[i]->setStyleSheet(style);
        m_curAbsPos[i] = positions[i] * m_baseScale;
        if(m_absRelPosition)
        {
            m_posWidgets[i]->setValue( m_curAbsPos[i] - m_relPosNull[i]);
        }
        else
        {
            m_posWidgets[i]->setValue( m_curAbsPos[i]);
        }
    }

    return;
}

void MotorController::setUnit(const QString unit)
{
    double oldScale = m_baseScale;
    QString micron(2, 181);
    micron[1] = 'm';
    if(unit == "mm")
    {
        m_baseScale = 1.0;
    }
    else if(unit == micron)
    {
        m_baseScale = 1000.0;
    }
    else if(unit == "m")
    {
        m_baseScale = 0.001;
    }
    else if(unit == "km")
    {
        m_baseScale = 0.000001;
    }
    else if(unit == "nm")
    {
        m_baseScale = 1000000.0;
    }
    else
        return;

    m_unit = unit;

    for(int i = 0; i < m_numVisAxis; i++)
    {
        m_curAbsPos[i] *= m_baseScale / oldScale;
        m_relPosNull[i] *= m_baseScale / oldScale;

        if(m_absRelPosition)
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i] - m_relPosNull[i]);
            m_posWidgets[i]->setSuffix(unit);
        }
        else
        {
            m_posWidgets[i]->setValue(m_curAbsPos[i]);
            m_posWidgets[i]->setSuffix(unit);     
        }
    }

    return;
}

void MotorController::setReadOnly(const bool value)
{
    bool changed = m_readOnly != value;
    m_readOnly = value;

    if(changed)
    {
        for(int i = 0; i < m_numVisAxis; i++)
        {
            m_changePosButtons[i][0]->setVisible(!m_readOnly);
            m_changePosButtons[i][1]->setVisible(!m_readOnly);
            m_changePosButtons[i][2]->setVisible(!m_readOnly);
            m_changePosButtons[i][3]->setVisible(!m_readOnly);
        }

        resizeEvent(NULL);
    }
    return;
}

void MotorController::mnuSetUnit(QAction* inputAction)
{
    setUnit(inputAction->text());
}

void MotorController::mnuSetAbsRel(QAction* inputAction)
{
    if(inputAction->text() == QString("abs"))
    {
        setAbsRel(false);
    }
    else if(inputAction->text() == QString("rel"))
    {
        setAbsRel(true);
    }
    else if(inputAction->text() == QString("origin")) 
    {
        m_relPosNull = m_curAbsPos;
        setAbsRel(m_absRelPosition);
    }
    return;
}

void MotorController::setSmallStep(const double value)
{
    if(value > 0.0 && value < 1.0)
    {
        m_smallStep = value;
    }
    return;
}


void MotorController::setBigStep(const double value)
{
    if(value > 0.0 && value < 1000.0)
    {
        m_bigStep = value;
    }
    return;
}

void MotorController::setAbsRel(const bool absRel)
{
    m_absRelPosition = absRel;
    QChar axisName;
    
    char text[4] = {'%','c', ' ', 0};
    if(m_absRelPosition)
    {
        text[2] = '*';
    }
    for(int i = 0; i < m_posWidgets.size(); i ++)
    {
        axisName = m_axisName[i][0];
        m_axisName[i].sprintf(text, axisName);

        m_posWidgets[i]->setPrefix(m_axisName[i]);
    }
    triggerUpdatePosition();
}

void MotorController::triggerActuatorStep(const int axisNo, const bool smallBig, const bool forward)
{
    if(m_pActuator.isNull())
        return;

    double step = smallBig ? m_bigStep : m_smallStep;
    step = forward ? step : -1 * step;

    bool ready = true;

    if(m_needStepAdaption)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        QSharedPointer<ito::ParamBase> qsParam(new ito::ParamBase("stepwidth", ito::ParamBase::Double, step));
        QMetaObject::invokeMethod(m_pActuator, "setParam", Q_ARG(QSharedPointer<ito::ParamBase>, qsParam), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while (!locker.getSemaphore()->wait(5000))
        {
            ready = false;
            break;
        }

        if(locker.getSemaphore()->returnValue.containsError())
        {
            ready = false;
        }   
    }

    if(ready)
    {
        QMetaObject::invokeMethod(m_pActuator, "setPosRel", Q_ARG(const int, axisNo), Q_ARG(const double, step), Q_ARG(ItomSharedSemaphore*, NULL));
    }
    return;
}
