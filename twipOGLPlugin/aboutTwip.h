/* ********************************************************************
#    twip designer-Plugins for itom
#    URL: http://www.twip-os.com
#    Copyright (C) 2014, twip optical solutions GmbH, 
#    Stuttgart, Germany 
#
#    This files is part of a designer-Plugin e.g. twipOGLFigure for the 
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General 
#    Public Licence and must behandled accordingly.
#
#    This plugin is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    Information under www.twip-os.com or mail to info@twip-os.com.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, University Stuttgart published under 
#    GNU General Public License as published by the Free Software 
#    Foundation. See <https://bitbucket.org/itom/> 
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef ABOUTTWIPDIALG
#define ABOUTTWIPDIALG

#include "ui_aboutTwip.h"
#include "itom_sdk.h"
#include "pluginVersion.h"
#include <qglobal.h>

#ifdef USEPCL
#include "pcl/pcl_config.h"
#endif

#include "opencv2/core/version.hpp"

#define QUOTE(x) #x

class DialogAboutTwip : public QDialog 
{
    Q_OBJECT

    public:
        DialogAboutTwip()
        {
            ui.setupUi(this);
            QString myText = ui.licence->toHtml();
            myText.replace("{ITOM_VER}", ITOM_VERSION_STRING);
            myText.replace("{PLUGIN_VER}", QUOTE(PLUGIN_VERSION_STRING));
            myText.replace("{QT_VER}", QT_VERSION_STR);
            myText.replace("{OPENCV_VER}", CV_VERSION);
            
    #ifdef USEPCL
        #ifdef PCL_REVISION_VERSION
            myText.replace("{PCL_VER}", QString("<BR>* point cloud library PCL %1.%2.%3<BR>").arg(PCL_MAJOR_VERSION).arg(PCL_MINOR_VERSION).arg(PCL_REVISION_VERSION));
        #else
            myText.replace("{PCL_VER}", QString("<BR>* point cloud library PCL %1.%2<BR>").arg(PCL_MAJOR_VERSION).arg(PCL_MINOR_VERSION));
        #endif
    #else
            myText.replace("{PCL_VER}", "");
    #endif

#ifdef USESWRANGLERGL
            
const char *lic = "The OpenGL Extension Wrangler Library<BR>\
Copyright (C) 2002-2007, Milan Ikits <milan ikits[]ieee org><BR>\
Copyright (C) 2002-2007, Marcelo E. Magallon <mmagallo[]debian org><BR>\
Copyright (C) 2002, Lev Povalahev<BR>\
All rights reserved.<BR>\
<BR>\
Redistribution and use in source and binary forms, with or without<BR>\
modification, are permitted provided that the following conditions are met:<BR>\
<BR>\
* Redistributions of source code must retain the above copyright notice,<BR>\
  this list of conditions and the following disclaimer.<BR>\
* Redistributions in binary form must reproduce the above copyright notice,<BR>\
  this list of conditions and the following disclaimer in the documentation<BR>\
  and/or other materials provided with the distribution.<BR>\
* The name of the author may be used to endorse or promote products<BR>\
  derived from this software without specific prior written permission.<BR><BR>";
            
            QString wrangler = "This programm uses the OpenGL Extension Wrangler Library with following licence: <BR><BR>";
            wrangler.append(lic);
            wrangler.append('\n');

            myText.replace("{WRANGLERGL}", wrangler);
#else
            myText.replace("{WRANGLERGL}", "");
#endif
            ui.licence->setHtml(myText);
        }

        ~DialogAboutTwip() {}

    private:
        Ui::AboutTwip ui;

    public slots:


    private slots:
        void on_pushButtonClose_clicked() {close();}

};

#endif //ABOUTTWIPDIALG