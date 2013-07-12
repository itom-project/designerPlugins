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
    m_readOnly(false),
    m_actSetUnit(NULL),
    m_actUpdatePos(NULL),
    m_mnuSetUnit(NULL),
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

    m_posWidgets.clear();
    m_posWidgets.reserve(6);
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    m_posWidgets.append(new QDoubleSpinBox(this));
    
    QString micron(2, 181);
    micron[1] = 'm';

    //QMenu *contextMenu = new QMenu(QObject::tr("motorController"), this);
    m_actSetUnit = new QAction(tr("Set Unit"), this);
    m_mnuSetUnit = new QMenu("Unit Switch");
	m_mnuSetUnit->addAction("nm");
	m_mnuSetUnit->addAction(micron);
	m_mnuSetUnit->addAction("mm");
	m_mnuSetUnit->addAction("m");
    m_actSetUnit->setMenu(m_mnuSetUnit);
    
    m_actUpdatePos = new QAction(tr("Update"), this);

    connect(m_mnuSetUnit, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetUnit(QAction*)));
    connect(m_actUpdatePos, SIGNAL(triggered()), this, SLOT(triggerUpdatePosition()));
    
    //contextMenu->addAction(m_actSetUnit);

    setContextMenuPolicy( Qt::ActionsContextMenu );
    addAction( m_actSetUnit );
    addAction( m_actUpdatePos );

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
    resizeEvent(NULL);
    return;
}
void MotorController::resizeEvent(QResizeEvent * event )
{
    int x = 0;
    int y = 0;
    int border = isFlat() ? 1 : 5;
    int upper = title().length() > 0 ? 10:1;
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

MotorController::~MotorController()
{
    if(m_actSetUnit)
    {
        delete m_actSetUnit;
    }
    if(m_actUpdatePos)
    {
        delete m_actUpdatePos;
    }
    if(m_mnuSetUnit)
    {
        delete m_mnuSetUnit;
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
        m_posWidgets[i]->setValue( positions[i] * m_baseScale );
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
        if(m_posWidgets[i]->suffix() != unit)
        {
            m_posWidgets[i]->setValue(m_posWidgets[i]->value() * m_baseScale / oldScale);
            m_posWidgets[i]->setSuffix(unit);
        }
    }

    return;
}

QString MotorController::getUnit()
{
    return m_unit;
}

bool MotorController::getReadOnly() const
{
    return m_readOnly;
}
void MotorController::setReadOnly(bool value)
{
    m_readOnly = value;
    return;
}

void MotorController::mnuSetUnit(QAction* inputAction)
{
    setUnit(inputAction->text());
}