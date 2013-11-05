/* ********************************************************************
 i tom measurement sys*tem
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

#include "drawItem.h"

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(const QString &title)
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::~DrawItem()
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setShape(const QPainterPath &path)
{
    QwtPlotShapeItem::setShape(path);
}

//----------------------------------------------------------------------------------------------------------------------------------
