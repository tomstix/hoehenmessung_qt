import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Window {
    width: 200
    height: 150
    visible: true
    title: "Sensor Calibration"

    Column {
        anchors.fill: parent
        Button {
            text: "Set Max"
            width: parent.width
            height: 50
        }
        Button {
            text: "Set Min"
            width: parent.width
            height: 50
        }
        Label {
            text: "Raw Value:\t"
            width: parent.width
        }
        Label {
            text: "Calibration status:\t"
            width: parent.width
        }
    }

    onClosing: calibrationLoader.source = ""
}