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

#ifndef MC_H
#define MC_H

#include "common/addInInterface.h"

//#include <QWidget>
#include <QGroupBox>
#include <QPointer>
#include <QDoubleSpinBox>
#include <QAction>
#include <QMenu>

class MotorController : public QGroupBox
{
    Q_OBJECT

    Q_PROPERTY(QPointer<ito::AddInActuator> actuator READ getActuator WRITE setActuator DESIGNABLE false);
    Q_PROPERTY(int numberOfAxis READ getNumAxis WRITE setNumAxis DESIGNABLE true);
    Q_PROPERTY(QString unit READ getUnit WRITE setUnit DESIGNABLE true);
    Q_PROPERTY(bool readOnly READ getReadOnly WRITE setReadOnly DESIGNABLE true);
    //Q_PROPERTY(double min READ getMin WRITE setMin DESIGNABLE true);


public:
    MotorController(QWidget *parent = 0);
    ~MotorController();
    
    void setActuator(QPointer<ito::AddInActuator> actuator);
    QPointer<ito::AddInActuator> getActuator() const;

    void setUnit(const QString unit);
    QString getUnit();

    void setNumAxis(const int numAxis);
    int getNumAxis() const {return m_numVisAxis;};

    bool getReadOnly() const;
    void setReadOnly(bool value);

    //double getMin() const;
    //void setMin(double value);

    //double getMax() const;
    //void setMax(double value);

    virtual QSize sizeHint() const;

protected:
    QPointer<ito::AddInActuator> m_pActuator;
    void resizeEvent(QResizeEvent * event );
    
private:
    QList<QDoubleSpinBox*> m_posWidgets;
    QList<QString> m_axisName;
    int m_numAxis;
    int m_numVisAxis;
    double m_baseScale;
    bool m_updateBySignal;
    bool m_readOnly;

    QString m_unit;

    QAction  *m_actSetUnit;
    QAction  *m_actUpdatePos;
    QMenu    *m_mnuSetUnit;

public slots:

    void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
    void triggerUpdatePosition(void);
    void mnuSetUnit(QAction* inputAction);

signals:
    void RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos);
};

#endif //MC_H
