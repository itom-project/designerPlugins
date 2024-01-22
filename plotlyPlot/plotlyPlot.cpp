/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2024, Institut fuer Technische Optik (ITO),
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

#include "plotlyPlot.h"

#include <QWebEngineProfile>
#include <QWebEngineView>
#include <QtGlobal>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    #include <qwebenginedownloaditem.h>
#else
    #include <QWebEngineDownloadRequest>
#endif
#include <qfiledialog.h>
#include <qfileinfo.h>
#include <qsharedpointer.h>
#include <qstatusbar.h>
#include <qtemporaryfile.h>
#include <qdir.h>
#include <qdebug.h>
#include <qcoreapplication.h>

class PlotlyPlotPrivate
{
public:
    PlotlyPlotPrivate() : m_pWebEngineView(nullptr), m_htmlFile(nullptr)
    {
    }

    QWebEngineView* m_pWebEngineView;
    QSharedPointer<QTemporaryFile> m_htmlFile;
};

//-------------------------------------------------------------------------------------
PlotlyPlot::PlotlyPlot(
    const QString& itomSettingsFile,
    AbstractFigure::WindowMode windowMode,
    QWidget* parent /*= 0*/) :
    ito::AbstractFigure(itomSettingsFile, windowMode, parent),
    d_ptr(new PlotlyPlotPrivate())
{
    Q_D(PlotlyPlot);

    setWindowFlags(Qt::Widget); // this is important such that this main window reacts as widget

    d->m_pWebEngineView = new QWebEngineView(this);
    setContentsMargins(0, 0, 0, 0);
    d->m_pWebEngineView->setObjectName("canvasWidget");

    setCentralWidget(d->m_pWebEngineView);

    QStatusBar* statusBar = qobject_cast<QMainWindow*>(this)->statusBar();

    connect(d->m_pWebEngineView, &QWebEngineView::loadStarted, [=]() {
        statusBar->showMessage(tr("Started Loading..."));
    });

    connect(d->m_pWebEngineView, &QWebEngineView::loadFinished, [=](bool ok) {
        if (ok)
        {
            statusBar->clearMessage();
        }
        else
        {
            statusBar->showMessage(tr("Error loading the content."), 2000);
        }
    });

    connect(d->m_pWebEngineView, &QWebEngineView::loadProgress, [=](int progress) {
        statusBar->showMessage(tr("Loading (%1 %)").arg(progress));
    });

    QWebEngineProfile* profile = d->m_pWebEngineView->page()->profile();

    connect(profile, &QWebEngineProfile::downloadRequested, this, &PlotlyPlot::downloadRequested);

    setPropertyObservedObject(this);
}

//-------------------------------------------------------------------------------------
PlotlyPlot::~PlotlyPlot()
{
}

//-------------------------------------------------------------------------------------
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
void PlotlyPlot::downloadRequested(QWebEngineDownloadItem* download)
{
    QFileInfo fileInfo(download->path());
    QString newPath = QFileDialog::getSaveFileName(
            this, tr("File Location"), download->path(), tr("Image (*.%1)").arg(fileInfo.suffix()));

    if (newPath != "")
    {
        download->setPath(newPath);
        download->accept();
    }
}
#else
void PlotlyPlot::downloadRequested(QWebEngineDownloadRequest* download)
{
    QFileInfo fileInfo(download->downloadDirectory(), download->downloadFileName());
    QString newPath = QFileDialog::getSaveFileName(
        this, tr("File Location"), fileInfo.absoluteFilePath(), tr("Image (*.%1)").arg(fileInfo.suffix()));

    if (newPath != "")
    {
        fileInfo.setFile(newPath);
        download->setDownloadDirectory(fileInfo.absoluteDir().absolutePath());
        download->setDownloadFileName(fileInfo.fileName());
        download->accept();
    }
}
#endif

//-------------------------------------------------------------------------------------
void PlotlyPlot::setContextMenuEnabled(bool show)
{
}

//-------------------------------------------------------------------------------------
bool PlotlyPlot::getContextMenuEnabled() const
{
    return false;
}

//-------------------------------------------------------------------------------------
void PlotlyPlot::setHtml(const QString& html)
{
    Q_D(PlotlyPlot);

    if (html == "")
    {
        d->m_htmlFile.clear();
        d->m_pWebEngineView->setUrl(QUrl());
    }
    else
    {
        QString tempFile = QDir::tempPath() + "/itom_plotly_XXXXXX.html";
        d->m_htmlFile =
            QSharedPointer<QTemporaryFile>(new QTemporaryFile(tempFile));
        d->m_htmlFile->setAutoRemove(true);
        if (d->m_htmlFile->open())
        {
            d->m_htmlFile->write(html.toUtf8());
            d->m_htmlFile->close();
        }

        QString filename = d->m_htmlFile->fileName();
        d->m_pWebEngineView->setUrl(QUrl(filename));
    }
}

//-------------------------------------------------------------------------------------
void PlotlyPlot::clear()
{
    setHtml("");
}
