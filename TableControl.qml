import QtQuick 2.15
import QtCharts 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls 2.15

GridLayout {
    columns: 2

    Column {
        Layout.fillWidth: true
        Label {
            property string textc: "Nothing received yet"
            id: distanceLabel
            text: "Measured height:\t" + Realsense.distanceRaw.toLocaleString(Qt.locale("de_DE"), 'f', 2) + " m"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            property string textc: "Nothing received yet"
            text: "Offset:\t\t" + "0,00 m"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            property string textc: "Nothing received yet"
            text: "Table setpoint:\t" + "0,72 m"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            property string textc: "Nothing received yet"
            text: "Table length:\t" + "0,72 m"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
    }
    Column {
        Layout.fillWidth: true
        Layout.preferredWidth: 80
        Layout.preferredHeight: 160
        Button {
            text: "+"
            font.pointSize: 64
            id: plusButton
            height: parent.height / 2
            width: parent.width
            background: Rectangle {
                color: parent.down ? "#80ff80" : "#00ff00"
            }
        }
        Button {
            text: "-"
            font.pointSize: 64
            id: minusButton
            height: parent.height / 2
            width: parent.width
            background: Rectangle {
                color: parent.down ? "#ff8080" : "#ff0000"
            }
        }
    }
    CheckBox {
        Layout.fillWidth: true
        text: "Header Control active"
        checked: false
    }
    RunningPlot {
        id: heightPlot
        Layout.columnSpan: 2
        Layout.fillWidth: true
        visible: true
    }
}