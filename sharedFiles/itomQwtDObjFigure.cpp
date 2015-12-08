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

#include "itomQwtDObjFigure.h"

#include <qbrush.h>
#include <qpalette.h>
#include <qstatusbar.h>
#include <qapplication.h>
#include <qclipboard.h>
#include <qlineedit.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include <qsharedpointer.h>

#include "DataObject/dataobj.h"
#include "itomQwtPlot.h"

#include <qwt_plot.h>
#include <qwt_plot_renderer.h>

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ItomQwtDObjFigure(QWidget *parent /*= NULL*/) :
    ito::AbstractDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent),
    m_pBaseContent(NULL)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ItomQwtDObjFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent /*= NULL*/) :
    ito::AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pBaseContent(NULL)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::~ItomQwtDObjFigure()
{
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize /*= QSizeF(0.0,0.0)*/, const int resolution /*= 300*/)
{
    QwtPlot *plot = qobject_cast<QwtPlot*>(centralWidget());
    if (!plot)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, canvas handle not initilized").toLatin1().data());
    }

    if (curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = plot->size();
    }

    QBrush curBrush = plot->canvasBackground();

    QPalette curPalette = plot->palette();

    plot->setAutoFillBackground(true);
    plot->setPalette(Qt::white);
    plot->setCanvasBackground(Qt::white);

    plot->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if (copyToClipboardNotFile)
    {
        statusBar()->showMessage(tr("copy current view to clipboard..."));

        qreal resFaktor = std::max(qRound(resolution / 72.0), 1);

        QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
        QClipboard *clipboard = QApplication::clipboard();
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFaktor, resFaktor);
        renderer.render(plot, &painter, plot->rect());
        img.setDotsPerMeterX(img.dotsPerMeterX() * resFaktor); //setDotsPerMeterXY must be set after rendering
        img.setDotsPerMeterY(img.dotsPerMeterY() * resFaktor);
        clipboard->setImage(img);

        statusBar()->showMessage(tr("copy current view to clipboard. done."), 1000);
    }
    else
    {
        renderer.renderDocument(plot, fileName, curSize, resolution);
    }

    plot->setPalette(curPalette);
    plot->setCanvasBackground(curBrush);

    plot->replot();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::copyToClipBoard()
{
    return exportCanvas(true, "");
}

//----------------------------------------------------------------------------------------------------------------------------------
QPixmap ItomQwtDObjFigure::renderToPixMap(const int xsize, const int ysize, const int resolution)
{
    QwtPlot *plot = qobject_cast<QwtPlot*>(centralWidget());
    QSizeF curSize(xsize, ysize);

    if (!plot)
    {
        QSize myRect(curSize.width(), curSize.height());
        QPixmap destinationImage(myRect);
        return destinationImage;
    }

    if (curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = plot->size();
    }
    //qreal linewidth = m_pContent->m_lineWidth;
    qreal resFaktor = resolution / 72.0 + 0.5;
    resFaktor = resFaktor < 1 ? 1 : resFaktor;
    resFaktor = resFaktor > 6 ? 6 : resFaktor;
    QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);

    QPixmap destinationImage(myRect);
    destinationImage.fill(Qt::white);

    QBrush curBrush = plot->canvasBackground();

    QPalette curPalette = plot->palette();

    plot->setAutoFillBackground(true);
    plot->setPalette(Qt::white);
    plot->setCanvasBackground(Qt::white);
    plot->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);

    QPainter painter(&destinationImage);
    painter.scale(resFaktor, resFaktor);
    renderer.render(plot, &painter, plot->rect());

    plot->setPalette(curPalette);
    plot->setCanvasBackground(curBrush);
    plot->replot();

    return destinationImage;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::sendCurrentToWorkspace()
{
    bool ok;
    QString varname = QInputDialog::getText(this, tr("Current to workspace"), tr("Indicate the python variable name for the currently visible object"), QLineEdit::Normal, "zoom_object", &ok);
    if (ok && varname != "")
    {
        QSharedPointer<ito::DataObject> obj = getDisplayed();
        const ito::DataObject *dobj = &(*obj);
        QSharedPointer<ito::ParamBase> obj_(new ito::ParamBase("displayed", ito::ParamBase::DObjPtr, (const char*)dobj));

        QApplication::setOverrideCursor(Qt::WaitCursor);

        ito::RetVal retval = apiSendParamToPyWorkspace(varname, obj_);

        QApplication::restoreOverrideCursor();

        if (retval.containsError())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Error sending data object to workspace").toLatin1().data());
            if (retval.errorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.exec();
        }
        else if (retval.containsWarning())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Error sending data object to workspace").toLatin1().data());
            if (retval.errorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }
}



