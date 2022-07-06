import QtQuick 2.15
import QtCharts 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls 2.15

GridLayout {
    columns: 2

    Column {
        Layout.fillWidth: true
        Layout.preferredWidth: parent.width*0.6666
        Label {
            id: distanceLabel
            text: "Measured height:\t" + (Headercontrol.height/10).toFixed(0) + "cm"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            text: "Offset:\t\t" + (Headercontrol.tableSetpointOffset/10).toFixed(0) + "cm"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            text: "Table setpoint:\t" + (Headercontrol.tableSetpoint/10).toFixed(0) + "cm"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            text: "Table length:\t" + (Headercontrol.tableCalibrated ? ((Headercontrol.tableLength/10).toFixed(0) + "cm") : "n/A")
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignLeft
        }
        Label {
            text: "Error:\t\t" + (Headercontrol.tableCalibrated ? (((Headercontrol.tableLength - Headercontrol.tableSetpoint)/10).toFixed(0) + "cm") : "n/A")
            color: Math.abs(((Headercontrol.tableLength - Headercontrol.tableSetpoint)/10)) > 10 ? "red" : "black"
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
            autoRepeat: true
            onClicked: Headercontrol.tableSetpointOffset += 10
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
            autoRepeat: true
            onClicked: Headercontrol.tableSetpointOffset -= 10
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
