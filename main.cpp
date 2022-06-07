#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "realsense.h"
#include "can.h"


int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    auto realsenseWorker = new RealsenseWorker;
    engine.addImageProvider("color", realsenseWorker);
    engine.rootContext()->setContextProperty("Realsense", realsenseWorker);

    auto canBus = new CAN;
    engine.rootContext()->setContextProperty("CAN", canBus);

    engine.load(url);

    return app.exec();
}
