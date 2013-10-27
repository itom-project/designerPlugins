#ifndef DOBJMETADATATABLEFACTORY_H
#define DOBJMETADATATABLEFACTORY_H

#include <QtDesigner/QDesignerCustomWidgetInterface>

class dObMetaDataTableFactory : public QObject, public QDesignerCustomWidgetInterface
{
    Q_OBJECT
    Q_INTERFACES(QDesignerCustomWidgetInterface)

public:
    dObMetaDataTableFactory(QObject *parent = 0);

    bool isContainer() const;
    bool isInitialized() const;
    QIcon icon() const;
    QString domXml() const;
    QString group() const;
    QString includeFile() const;
    QString name() const;
    QString toolTip() const;
    QString whatsThis() const;
    QWidget *createWidget(QWidget *parent);
    void initialize(QDesignerFormEditorInterface *core);

private:
    bool initialized;
};

#endif // DOBJMETADATATABLEFACTORY_H
