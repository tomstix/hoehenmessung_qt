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
            onClicked: Headercontrol.calibrateMax()
        }
        Button {
            text: "Set Min"
            width: parent.width
            height: 50
            onClicked: Headercontrol.calibrateMin()
        }
        Label {
            text: "Raw Value:\t" + Headercontrol.tableRaw
            width: parent.width
        }
        Label {
            text: (Headercontrol.tableCalibrated ? "Calibrated" : "Not Calibrated")
            width: parent.width
        }
    }

    onClosing: calibrationLoader.source = ""
}
