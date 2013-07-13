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
#include <QPushButton>
#include <QAction>
#include <QMenu>

class MotorController : public QGroupBox
{
    Q_OBJECT

    Q_PROPERTY(QPointer<ito::AddInActuator> actuator READ getActuator WRITE setActuator DESIGNABLE false);
    Q_PROPERTY(int numberOfAxis READ getNumAxis WRITE setNumAxis DESIGNABLE true);
    Q_PROPERTY(QString unit READ getUnit WRITE setUnit DESIGNABLE true);
    Q_PROPERTY(bool readOnly READ getReadOnly WRITE setReadOnly DESIGNABLE true);
    Q_PROPERTY(double smallStep READ getSmallStep WRITE setSmallStep DESIGNABLE true);
    Q_PROPERTY(double bigStep READ getBigStep WRITE setBigStep DESIGNABLE true);
    Q_PROPERTY(bool absRel READ getAbsRel WRITE setAbsRel DESIGNABLE true);
    //Q_PROPERTY(double min READ getMin WRITE setMin DESIGNABLE true);


public:
    MotorController(QWidget *parent = 0);
    ~MotorController();
    
    void setActuator(QPointer<ito::AddInActuator> actuator);
    QPointer<ito::AddInActuator> getActuator() const;

    void setUnit(const QString unit);
    QString getUnit() const {return m_unit;};

    void setNumAxis(const int numAxis);
    int getNumAxis() const {return m_numVisAxis;};

    bool getReadOnly() const {return m_readOnly;};
    void setReadOnly(const bool value);

    double getSmallStep() const {return m_smallStep;};
    void setSmallStep(const double value);

    double getBigStep() const {return m_bigStep;};
    void setBigStep(const double value);

    bool getAbsRel() const {return m_numVisAxis;};;
    void setAbsRel(const bool absRel);

    //double getMin() const;
    //void setMin(double value);

    //double getMax() const;
    //void setMax(double value);

    virtual QSize sizeHint() const;

protected:
    QPointer<ito::AddInActuator> m_pActuator;
    void resizeEvent(QResizeEvent * event );
    
private:
    QList<QDoubleSpinBox* > m_posWidgets;
    QList<QList<QPushButton *> > m_changePosButtons;
    QList<QString > m_axisName;
    int m_numAxis;
    int m_numVisAxis;
    double m_baseScale;
    bool m_updateBySignal;
    bool m_readOnly;
    bool m_needStepAdaption;
    bool m_absRelPosition;

    QVector<double> m_relPosNull;
    QVector<double> m_curAbsPos;

    double m_smallStep;
    double m_bigStep;

    QString m_unit;

    QAction  *m_actSetUnit;
    QAction  *m_actUpdatePos;
    QAction  *m_actSetAbsRel;

    QMenu    *m_mnuSetUnit;
    QMenu    *m_mnuSetAbsRel;
    

public slots:

    void triggerActuatorStep(const int axisNo, const bool smallBig, const bool forward);
    void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
    void triggerUpdatePosition(void);
    void mnuSetUnit(QAction* inputAction);
    void mnuSetAbsRel(QAction* inputAction);

    void axis0BigStepMinus(void){triggerActuatorStep(0, true, false);};
    void axis0BigStepPlus(void){triggerActuatorStep(0, true, true);};
    void axis0SmallStepMinus(void){triggerActuatorStep(0, false, false);};
    void axis0SmallStepPlus(void){triggerActuatorStep(0, false, true);};

    void axis1BigStepMinus(void){triggerActuatorStep(1, true, false);};
    void axis1BigStepPlus(void){triggerActuatorStep(1, true, true);};
    void axis1SmallStepMinus(void){triggerActuatorStep(1, false, false);};
    void axis1SmallStepPlus(void){triggerActuatorStep(1, false, true);};

    void axis2BigStepMinus(void){triggerActuatorStep(2, true, false);};
    void axis2BigStepPlus(void){triggerActuatorStep(2, true, true);};
    void axis2SmallStepMinus(void){triggerActuatorStep(2, false, false);};
    void axis2SmallStepPlus(void){triggerActuatorStep(2, false, true);};

    void axis3BigStepMinus(void){triggerActuatorStep(3, true, false);};
    void axis3BigStepPlus(void){triggerActuatorStep(3, true, true);};
    void axis3SmallStepMinus(void){triggerActuatorStep(3, false, false);};
    void axis3SmallStepPlus(void){triggerActuatorStep(3, false, true);};

    void axis4BigStepMinus(void){triggerActuatorStep(4, true, false);};
    void axis4BigStepPlus(void){triggerActuatorStep(4, true, true);};
    void axis4SmallStepMinus(void){triggerActuatorStep(4, false, false);};
    void axis4SmallStepPlus(void){triggerActuatorStep(4, false, true);};

    void axis5BigStepMinus(void){triggerActuatorStep(5, true, false);};
    void axis5BigStepPlus(void){triggerActuatorStep(5, true, true);};
    void axis5SmallStepMinus(void){triggerActuatorStep(5, false, false);};
    void axis5SmallStepPlus(void){triggerActuatorStep(5, false, true);};

    
signals:
    void RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos);
};

#endif //MC_H
