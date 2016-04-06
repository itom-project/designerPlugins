/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
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

#ifndef ITOMQWTPLOTENUMS_H
#define ITOMQWTPLOTENUMS_H

#include <qobject.h>

class ItomQwtPlotEnums : public QObject
{
    Q_OBJECT
       
    Q_ENUMS(ComplexType);
    Q_ENUMS(ModificationState);
    Q_ENUMS(ShapeType);
    Q_ENUMS(PlotPickerType);
    Q_ENUMS(MultiLineMode);    
    Q_ENUMS(ColorHandling); 
    Q_ENUMS(CurveStyle);
    Q_ENUMS(FillCurveStyle);
    Q_ENUMS(ScaleEngine);

    Q_FLAGS(ModificationMode ModificationModes)
    Q_FLAGS(ShapeType ShapeTypes)

public:
    
    enum ComplexType 
    { 
        CmplxAbs = 0, 
        CmplxImag = 1, 
        CmplxReal = 2, 
        CmplxArg = 3 
    }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value
    
    enum ModificationMode
    {
        Move = 0x01,
        Rotate = 0x02,
        Resize = 0x04
    };
    Q_DECLARE_FLAGS(ModificationModes, ModificationMode)
    
    enum MultiLineMode { AutoRowCol, FirstRow, FirstCol, MultiRows, MultiCols, MultiLayerAuto, MultiLayerCols, MultiLayerRows };
    enum ColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray};
    enum PlotPickerType { DefaultMarker, RangeMarker, ValueRangeMarker, AxisRangeMarker };
    enum CurveStyle {   NoCurve = -1,  Lines, FittedLines, Sticks, SticksHorizontal, SticksVertical, Steps, StepsRight, StepsLeft,  Dots };
    enum FillCurveStyle {   NoCurveFill = -1,  FillBaseLine, FillFromTop, FillFromBottom};
    enum ScaleEngine { Linear = 1, Log2 = 2, Log10 = 10, Log16 = 16, LogLog2 = 1002, LogLog10 = 1010, LogLog16 = 1016};

    enum ShapeType //this enum is identical to ito::Shape::ShapeType but can be used for the Qt Meta System, such that the properties can be set in QtDesigner and via Python
    {
        /*Invalid = 0, */ /*ignore invalid*/
        MultiPointPick = 0x00000001,
        Point = 0x00000002,
        Line = 0x00000004,
        Rectangle = 0x00000008,
        Square = 0x00000010,
        Ellipse = 0x00000020,
        Circle = 0x00000040,
        Polygon = 0x00000080,
    };
    Q_DECLARE_FLAGS(ShapeTypes, ShapeType)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ItomQwtPlotEnums::ModificationModes)
Q_DECLARE_OPERATORS_FOR_FLAGS(ItomQwtPlotEnums::ShapeTypes)

#endif //ITOMQWTPLOTENUMS_H