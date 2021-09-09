/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
   This file is part of the designer widget 'vtk3dVisualizer' for itom.

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

#ifndef PLUGINVERSION_H
#define PLUGINVERSION_H

#include "itom_sdk.h"

#define PLUGIN_VERSION_MAJOR 1
#define PLUGIN_VERSION_MINOR 6
#define PLUGIN_VERSION_PATCH 1
#define PLUGIN_VERSION_REVISION 0
#define PLUGIN_VERSION        CREATE_VERSION(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_VERSION_STRING CREATE_VERSION_STRING(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_COMPANY        "Institut fuer Technische Optik, University Stuttgart"
#define PLUGIN_COPYRIGHT      "(C) 2021, ITO, University Stuttgart"
#define PLUGIN_NAME           "vtk3dVisualizer"

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PLUGINVERSION_H
