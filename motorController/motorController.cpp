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
    m_pActuator(NULL)
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
    
    m_numAxis = m_posWidgets.length();
    m_numVisAxis = m_posWidgets.length();

    for(int i = 0; i < m_numVisAxis; i++)
    {
        m_posWidgets[i]->setReadOnly(true);
        m_posWidgets[i]->setPrefix(m_axisName[i]);
        m_posWidgets[i]->setSuffix("mm");
        m_posWidgets[i]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        m_posWidgets[i]->setMinimum(-10.0);
        m_posWidgets[i]->setMaximum(10.0);
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
        /*
        ItomSharedSemaphore* mySemaphoreLocker = new ItomSharedSemaphore();

        QSharedPointer<ito::Param> qsParam(new ito::Param("numaxis", ito::ParamBase::Int));
        QMetaObject::invokeMethod(m_pActuator, "getParam", Q_ARG(QSharedPointer<ito::Param>, qsParam), Q_ARG(ItomSharedSemaphore *, pMySemaphoreLocker.getSemaphore()));

        while (!pMySemaphoreLocker.getSemaphore()->wait(PLUGINWAIT))
        {
            retval += ito::RetVal(ito::retError, 0, "timeout while getting numaxis parameter");
            break;
        }

        retval += pMySemaphoreLocker.getSemaphore()->returnValue;

        if(retval.containsWarningOrError())
        {
            pMyMotor = NULL;
            return;
        }

        axisNumbers = (*qsParam).getVal<int>();
        */
        connect( m_pActuator, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>) ), this, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>) ) );
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
        m_posWidgets[i]->setValue( positions[i] );
    }

    return;
}
