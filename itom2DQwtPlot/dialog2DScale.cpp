/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
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

#include "dialog2DScale.h"

//-----------------------------------------------------------------------------------------------
Dialog2DScale::Dialog2DScale(const InternalData &data, QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    double min,max;

    //x
    if(data.m_xaxisScaleAuto)
    {
        ui.radioAutoCalcX->setChecked(true);
    }
    else
    {
        ui.radioManualX->setChecked(true);
    }

    ui.doubleSpinMinX->setValue( data.m_xaxisMin );
    ui.doubleSpinMaxX->setValue( data.m_xaxisMax );

    //y
    if(data.m_yaxisScaleAuto)
    {
        ui.radioAutoCalcY->setChecked(true);
    }
    else
    {
        ui.radioManualY->setChecked(true);
    }

    ui.doubleSpinMinY->setValue( data.m_yaxisMin );
    ui.doubleSpinMaxY->setValue( data.m_yaxisMax );
    
    //value
    if(data.m_valueScaleAuto)
    {
        ui.radioAutoCalcValue->setChecked(true);
    }
    else
    {
        ui.radioManualValue->setChecked(true);
    }

    getDataTypeRange(data.m_dataType, min, max);

    ui.groupValue->setEnabled(data.m_dataType != ito::tRGBA32); //rgba32 data objects have no color map and can therefore not be cropped.

    ui.doubleSpinMinValue->setMinimum(min);
    ui.doubleSpinMinValue->setMaximum(max);
    ui.doubleSpinMaxValue->setMinimum(min);
    ui.doubleSpinMaxValue->setMaximum(max);
    
    ui.doubleSpinMinValue->setValue( data.m_valueMin );
    ui.doubleSpinMaxValue->setValue( data.m_valueMax );

    //ui.groupPlane->setVisible(false);
}

//-----------------------------------------------------------------------------------------------
void Dialog2DScale::getData(InternalData &data)
{
    data.m_valueScaleAuto = ui.radioAutoCalcValue->isChecked();
    data.m_valueMin = std::min(ui.doubleSpinMinValue->value(), ui.doubleSpinMaxValue->value());
    data.m_valueMax = std::max(ui.doubleSpinMinValue->value(), ui.doubleSpinMaxValue->value());
    
    data.m_xaxisScaleAuto = ui.radioAutoCalcX->isChecked();
    data.m_xaxisMin = ui.doubleSpinMinX->value();
    data.m_xaxisMax = ui.doubleSpinMaxX->value();

    data.m_yaxisScaleAuto = ui.radioAutoCalcY->isChecked();
    data.m_yaxisMin = ui.doubleSpinMinY->value();
    data.m_yaxisMax = ui.doubleSpinMaxY->value();
}

//-----------------------------------------------------------------------------------------------
void Dialog2DScale::getDataTypeRange(ito::tDataType type, double &min, double &max)
{
    switch(type)
    {
    case ito::tInt8:
        min = std::numeric_limits<ito::int8>::min();
        max = std::numeric_limits<ito::int8>::max();
        break;
    case ito::tUInt8:
    case ito::tRGBA32:
        min = std::numeric_limits<ito::uint8>::min();
        max = std::numeric_limits<ito::uint8>::max();
        break;
    case ito::tInt16:
        min = std::numeric_limits<ito::int16>::min();
        max = std::numeric_limits<ito::int16>::max();
        break;
    case ito::tUInt16:
        min = std::numeric_limits<ito::uint16>::min();
        max = std::numeric_limits<ito::uint16>::max();
        break;
    case ito::tInt32:
        min = std::numeric_limits<ito::int32>::min();
        max = std::numeric_limits<ito::int32>::max();
        break;
    case ito::tUInt32:
        min = std::numeric_limits<ito::uint32>::min();
        max = std::numeric_limits<ito::uint32>::max();
        break;
    case ito::tFloat32:
    case ito::tComplex64:
        min = -std::numeric_limits<ito::float32>::max();
        max = -min;
        break;
    case ito::tFloat64:
    case ito::tComplex128:
        min = -std::numeric_limits<ito::float64>::max();
        max = -min;
        break;
    default:
        min = max = 0.0;
    }
}