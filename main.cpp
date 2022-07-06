#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "realsense.h"
#include "can.h"
#include "datavisualizer.h"
#include "headercontrol.h"

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QApplication app(argc, argv);

    app.setOrganizationName("Zuern Harvesting");
    app.setOrganizationDomain("zuern.de");

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject const *obj, const QUrl &objUrl)
        {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    auto realsenseWorker = new RealsenseWorker;
    engine.addImageProvider("realsense", realsenseWorker);
    engine.rootContext()->setContextProperty("Realsense", realsenseWorker);

    auto canBus = new CAN;
    engine.rootContext()->setContextProperty("CAN", canBus);

    auto datavisualizer = new DataVisualizer;
    engine.rootContext()->setContextProperty("DataVisualizer", datavisualizer);

    auto headercontrol = new Headercontrol;
    engine.rootContext()->setContextProperty("Headercontrol", headercontrol);

    QObject::connect(realsenseWorker, &RealsenseWorker::newHeight, headercontrol, &Headercontrol::updateHeight);
    QObject::connect(headercontrol, &Headercontrol::sendCanMessage, canBus, &CAN::sendCANMessage);
    QObject::connect(canBus, &CAN::newHeaderMessage, headercontrol, &Headercontrol::processHeaderMessage);

    engine.load(url);

    return QApplication::exec();
}
