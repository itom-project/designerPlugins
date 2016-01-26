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

#ifndef ITOMQWTDOBJFIGURE_H
#define ITOMQWTDOBJFIGURE_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMQWTDOBJFIGURE_EXPORT Q_DECL_EXPORT
#else
    #define ITOMQWTDOBJFIGURE_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjFigure.h"
#include "common/retVal.h"
#include "common/itomPlotHandle.h"

#include <qsize.h>
#include <qstring.h>
#include <qpixmap.h>

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif


#ifndef DECLAREMETAPLOTHANDLE
    Q_DECLARE_METATYPE(ito::ItomPlotHandle)
    #define DECLAREMETAPLOTHANDLE
#endif

class ItomQwtPlot;


class ITOMQWTDOBJFIGURE_EXPORT ItomQwtDObjFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_CLASSINFO("slot://copyToClipBoard", "")

    Q_CLASSINFO("signal://plotItemsFinished", "Signal emitted when geometrical plotting was finished.")
    Q_CLASSINFO("signal://userInteractionDone", "")
    Q_CLASSINFO("signal://plotItemChanged", "")
    Q_CLASSINFO("signal://plotItemDeleted", "")
    Q_CLASSINFO("signal://plotItemsDeleted", "")

    DESIGNER_PLUGIN_ITOM_API

public:
    explicit ItomQwtDObjFigure(QWidget *parent = NULL);
    explicit ItomQwtDObjFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = NULL);
    virtual ~ItomQwtDObjFigure();

    
public Q_SLOTS:
    ito::RetVal copyToClipBoard();
    QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

protected Q_SLOTS:
    void sendCurrentToWorkspace();

    

protected:
    ito::RetVal exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize = QSizeF(0.0, 0.0), const int resolution = 300);
    ItomQwtPlot *m_pBaseContent;

private:

signals :
    void userInteractionDone(int type, bool aborted, QPolygonF points);
    void plotItemChanged(int idx, int flags, QVector<float> values);
    void plotItemDeleted(int idx);
    void plotItemsDeleted();
    void plotItemsFinished(int type, bool aborted);
    
};

#endif //ITOMQWTDOBJFIGURE_H