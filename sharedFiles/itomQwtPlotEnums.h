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
    Q_ENUMS(PlotPickerType);
    Q_ENUMS(MultiLineMode);    
    Q_ENUMS(ColorHandling); 
    Q_ENUMS(CurveStyle);
    Q_ENUMS(FillCurveStyle);
    Q_ENUMS(ScaleEngine);

public:
    
    enum ComplexType 
    { 
        CmplxAbs = 0, 
        CmplxImag = 1, 
        CmplxReal = 2, 
        CmplxArg = 3 
    }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value
    
    enum ModificationState
    {
        NextElementMode = 0,
        MoveGeometricElements = 1,
        RotateGeometricElements = 2,
        ResizeGeometricElements = 3,
        ModifyPoints = 4
    };
    
    enum MultiLineMode { AutoRowCol, FirstRow, FirstCol, MultiRows, MultiCols, MultiLayerAuto, MultiLayerCols, MultiLayerRows };
    enum ColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray};
    enum PlotPickerType { DefaultMarker, RangeMarker };
    enum CurveStyle {   NoCurve = -1,  Lines, FittedLines, Sticks, SticksHorizontal, SticksVertical, Steps, StepsRight, StepsLeft,  Dots };
    enum FillCurveStyle {   NoCurveFill = -1,  FillBaseLine, FillFromTop, FillFromBottom};
    enum ScaleEngine { Linear = 1, Log2 = 2, Log10 = 10, Log16 = 16, LogLog2 = 1002, LogLog10 = 1010, LogLog16 = 1016};
    
    
};

#endif //ITOMQWTPLOTENUMS_H