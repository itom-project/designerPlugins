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
* \file motorController.h
* \brief In this file the MotorController-Class is declared
*
*    The MotorController-Class defines a widget for generic motor monitoring and controll. The following files are
*   needed: MotorController.cpp, MotorController.h, MotorControllerFactory.h, MotorControllerFactory.cpp
*
*\sa MotorController, MotorControllerFactory, MotorControllerFactory.h
*\author ITO
*\date    2013
*/

#ifndef MC_H
#define MC_H

#include "common/addInInterface.h"

#include <qgroupbox.h>
#include <qpointer.h>
#include <qspinbox.h>
#include <qpushbutton.h>
#include <qaction.h>
#include <qmenu.h>
#include <qlineedit.h>
#include <qtimer.h>
#include <qactiongroup.h>
#include <qsignalmapper.h>

#if defined(CONNEXION_FOUND) //&& !_DEBUG
    #ifdef _DEBUG
        #undef _DEBUG
        #define CONNEXION_ENABLE 1
        #include <windows.h>
        #include "spwmacro.h"
        #include "si.h"
        #include "spwmath.h"
        #include "siapp.h"
        #define _DEBUG
    #else
        #undef _DEBUG
        #define CONNEXION_ENABLE 1
        #include <windows.h>
        #include "spwmacro.h"
        #include "si.h"
        #include "spwmath.h"
        #include "siapp.h"
    #endif
#else
    #define CONNEXION_ENABLE 0
#endif


class MotorController : public QGroupBox
{
    Q_OBJECT

    Q_PROPERTY(QPointer<ito::AddInActuator> actuator READ getActuator WRITE setActuator DESIGNABLE false);
    Q_PROPERTY(int numberOfAxis READ getNumAxis WRITE setNumAxis DESIGNABLE true);
    Q_PROPERTY(Unit unit READ getUnit WRITE setUnit DESIGNABLE true);
    Q_PROPERTY(bool readOnly READ getReadOnly WRITE setReadOnly DESIGNABLE true);
    Q_PROPERTY(bool autoUpdate READ getAutoUpdate WRITE setAutoUpdate DESIGNABLE true);
    Q_PROPERTY(double smallStep READ getSmallStep WRITE setSmallStep DESIGNABLE true);
    Q_PROPERTY(double bigStep READ getBigStep WRITE setBigStep DESIGNABLE true);
    Q_PROPERTY(bool absRel READ getAbsRel WRITE setAbsRel DESIGNABLE true);
    Q_PROPERTY(bool allowJoyStick READ getAllowJoyStick WRITE setAllowJoyStick DESIGNABLE true);
    //Q_PROPERTY(double min READ getMin WRITE setMin DESIGNABLE true);

    Q_CLASSINFO("prop://actuator", "Handle to the actuator to be used, not DESIGNABLE");
    Q_CLASSINFO("prop://numberOfAxis", "Number of axis to be visible");
    Q_CLASSINFO("prop://unit", "Base unit for spinboxes and movements, e.g. nm, micron, mm, m, km");
    Q_CLASSINFO("prop://readOnly", "Toggle read only");
    Q_CLASSINFO("prop://autoUpdate", "Toggle automatic motorposition update");
    Q_CLASSINFO("prop://smallStep", "Distances for the small step button, same value for plus and minus");
    Q_CLASSINFO("prop://bigStep", "Distances for the large step button, same value for plus and minus");
    Q_CLASSINFO("prop://absRel", "Toggle between absolut or relative position display. Origin can be set via context menu.");
    Q_CLASSINFO("prop://allowJoyStick", "Allow a software joystick, e.g. usb or gameport, not implemented yet.");

    Q_ENUMS(Unit);

    Q_CLASSINFO("slot://triggerActuatorStep", "")
    Q_CLASSINFO("slot://actuatorStatusChanged", "")
    Q_CLASSINFO("slot://triggerUpdatePosition", "")

    Q_CLASSINFO("signal://requestStatusAndPosition", "")
    Q_CLASSINFO("signal://TriggerSoftJoyStickMovement", "")

public:
    enum Unit { nm, micron, mm, m, km };

    //! Constructor for the class
    MotorController(QWidget *parent = 0);

    //! Destructor for the class
    ~MotorController();

    //! Set the actuator-handle
    void setActuator(QPointer<ito::AddInActuator> actuator);

    //! Retrive the current actuator-handle
    QPointer<ito::AddInActuator> getActuator() const {return QPointer<ito::AddInActuator>(NULL);};

    //! Set the metrical unit of the display
    void setUnit(const Unit unit);

    //! Read out the metrical unit of the display
    Unit getUnit() const {return m_unit;};

    //! Set the number of axis to be displayed
    void setNumAxis(const int numAxis);

    //! Retrive the number of axis currently displayed
    int getNumAxis() const {return m_numVisAxis;};

    //! Retrive readOnly status
    bool getReadOnly() const {return m_readOnly;};

    //! Retrive autoUpdate status
    bool getAutoUpdate() const {return m_autoUpdate;};

    //! Set the autoUpdate status. If true every second an update is triggered
    void setAutoUpdate(const bool value);

    bool getAllowJoyStick() const {return m_allowJoyStick;};
    void setAllowJoyStick(const bool newState){m_allowJoyStick = newState;};

    //! Toggle between display-only and additional controll functions
    void setReadOnly(const bool value);

    //! Get the small step-distance of the widget
    double getSmallStep() const {return m_smallStep;};

    //! Set the small step-distance of the widget in writeMode
    void setSmallStep(const double value);

    //! Get the large step-distance of the widget
    double getBigStep() const {return m_bigStep;};

    //! Set the large step-distance of the widget in writeMode
    void setBigStep(const double value);

    //! Retrive absolute or relative display status
    bool getAbsRel() const {return m_absRelPosition;};

    //! Toggle between absolute display and relative to a virtual coordinate display mode
    void setAbsRel(const bool absRel);

    //double getMin() const;
    //void setMin(double value);

    //double getMax() const;
    //void setMax(double value);

    virtual QSize sizeHint() const;

protected:
    //! Handle to the motor secured by QPointer
    QPointer<ito::AddInActuator> m_pActuator;

    void resizeEvent(QResizeEvent * event );
    void removeActuator();

#if CONNEXION_ENABLE
        bool winEvent(MSG * message, long * result);
#endif //CONNEXION_ENABLE

private:

#if(CONNEXION_ENABLE)
    //! Handle from the 3DConnexion device
    SiHdl m_SpwDeviceHandle;
    //! Handle from the 3DConnexion device
    SiOpenData m_SpwData;

    bool m_conNeedsTermination;

#endif

    void initializeJoystick();

    //! QList with all vertical layout, one for each axis
    QList<QGroupBox* > m_axisGroups;

    //! QList with all axis labels
    QList<QLineEdit* > m_posLabels;

    //! QList with all position-display
    QList<QDoubleSpinBox* > m_posWidgets;

    //! QList with all small step value display
    QList<QDoubleSpinBox* > m_smallStepWidgets;

    //! QList with all large step value display
    QList<QDoubleSpinBox* > m_largeStepWidgets;

    //! QList with the handles to the steps
    QList<QList<QPushButton *> > m_changePosButtons;

    //! QList with the identifiers each the axis
    QList<QString > m_axisName;

    //! Number of axis currently availeble
    int m_numAxis;

    //! Number of axis currently visible
    int m_numVisAxis;

    //! The current scaling to mm according to m_unit
    double m_baseScale;

    //! Becomes true of the position can be requested by a signal
    bool m_updateBySignal;

    //! If true, no step-Buttons are visible
    bool m_readOnly;

    //! If true, the "stepwidth" paramter is set to corresponding step width before step is triggered
    bool m_needStepAdaption;

    //! If true, position are shown relative to correspondig position
    bool m_absRelPosition;

    //! If true, ignore several signals within the gui
    bool m_isUpdating;

    //! If true, the corresponding actuator has a joystick-slot
    bool m_hasJoyStick;

    //! If true, the joystick is enabled external
    bool m_allowJoyStick;

    //! If true, the joystick is enabled by this gui element
    bool m_enableJoyStick;

    //! If true, the joystick moves fast else slow
    bool m_joyModeFast;

    //! Status for autoUpdate. Default is true
    bool m_autoUpdate;

    //! Internal trimer for autoUpdate
    QTimer m_timer;

    //! QVector with the virtual origin position
    QVector<double> m_relPosNull;

    //! QVector with last retrieved absolute position
    QVector<double> m_curAbsPos;

    //! Small step distance
    double m_smallStep;

    //! Large step distance
    double m_bigStep;

    //! Current display unit
    Unit m_unit;

    QAction  *m_actSetUnit;
    QAction  *m_actUpdatePos;
    QAction  *m_actSetAutoUpdate;
    QMenu    *m_mnuSetAutoUpdate;
    QActionGroup *m_unitActionGroup;

    QAction  *m_actSetAbsRel;

    QMenu    *m_mnuSetUnit;
    QMenu    *m_mnuSetAbsRel;

    QSignalMapper *m_pStepFastNegSignalMapper;
    QSignalMapper *m_pStepSlowNegSignalMapper;
    QSignalMapper *m_pStepFastPosSignalMapper;
    QSignalMapper *m_pStepSlowPosSignalMapper;


public slots:

    void triggerActuatorStep(const int axisNo, const bool smallBig, const bool forward);
    void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
    void triggerUpdatePosition(void);

private slots:
    void mnuSetAutoUpdate(QAction* inputAction);
    void mnuSetUnit(QAction* inputAction);
    void mnuSetAbsRel(QAction* inputAction);

    void guiChangedSmallStep(double value);
    void guiChangedLargeStep(double value);

    void actuatorDestroyed();

    void moveStepFastNeg(int index);
    void moveStepSlowNeg(int index);
    void moveStepFastPos(int index);
    void moveStepSlowPos(int index);

signals:
    void requestStatusAndPosition(bool sendActPosition, bool sendTargetPos);
    void TriggerSoftJoyStickMovement(QVector<int> axis, QVector<double> vel);
};

#endif //MC_H
