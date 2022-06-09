import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GridLayout {

    Connections {
        target: Realsense
        function onIsRunningChanged()
        {
            resolutionSelector.enabled = !Realsense.running
            startButton.enabled = !Realsense.running
            stopButton.enabled = Realsense.running
            tareButton.enabled = Realsense.running
        }
    }

    id: controlLayout
    columns: 2
    ComboBox {
        id: resolutionSelector
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        Layout.columnSpan: 2
        currentIndex: 3
        model: ListModel {
            ListElement {
                text: "424x240"
            }
            ListElement {
                text: "480x270"
            }
            ListElement {
                text: "640x360"
            }
            ListElement {
                text: "640x480"
            }
            ListElement {
                text: "1280x720"
            }
        }
        Component.onCompleted: Realsense.resolution = currentIndex
        onCurrentIndexChanged: Realsense.resolution = currentIndex
    }
    Button {
        id: startButton
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        text: "Start"
        onClicked: {
            Realsense.start()
        }
    }
    Button {
        id: stopButton
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        text: "Stop"
        onClicked: {
            Realsense.stop()
        }
        Component.onCompleted: enabled = false
    }
    LabelledSlider {
        labeltext: "min. Z Distance"
        unitSuffix: " m"
        minValue: 0.2
        maxValue: 20.0
        step: 0.1
        initialValue: 1.0
        onSliderValueChanged: {
            Realsense.pointcloudoptions.z_min = value
        }
    }
    LabelledSlider {
        labeltext: "max. Z Distance"
        unitSuffix: " m"
        minValue: 0.2
        maxValue: 20.0
        step: 0.1
        initialValue: 15.0
        onSliderValueChanged: {
            Realsense.pointcloudoptions.z_max = value
        }
    }
    LabelledSlider {
        labeltext: "X width"
        unitSuffix: " m"
        minValue: 0.2
        maxValue: 20.0
        step: 0.1
        initialValue: 5.5
        onSliderValueChanged: {
            Realsense.pointcloudoptions.x_min = -value/2.0
            Realsense.pointcloudoptions.x_max = value/2.0
        }
    }
    LabelledSlider {
        labeltext: "Voxel size"
        unitSuffix: " cm"
        minValue: 0.5
        maxValue: 5.0
        step: 0.1
        initialValue: 2.5
        onSliderValueChanged: {
            Realsense.pointcloudoptions.voxel_size = value / 100.0
        }
    }
    LabelledSlider {
        labeltext: "RANSAC Threshold"
        unitSuffix: " cm"
        minValue: 1.0
        maxValue: 10.0
        step: 1.0
        initialValue: 2.0
        onSliderValueChanged: {
            Realsense.pointcloudoptions.ransac_threshold = value / 100.0
        }
    }
    LabelledSlider {
        labeltext: "Plane angle max."
        unitSuffix: "°"
        minValue: 1.0
        maxValue: 45.0
        step: 1.0
        initialValue: 20.0
        onSliderValueChanged: {
            Realsense.pointcloudoptions.ransac_angle_max = value
        }
    }
    LabelledSlider {
        labeltext: "RANSAC iterations"
        minValue: 10
        maxValue: 2000
        step: 10
        initialValue: 200
        onSliderValueChanged: {
            Realsense.pointcloudoptions.ransac_iterations = value
        }
    }
    LabelledSlider {
        labeltext: "Filter Alpha"
        minValue: 0.001
        maxValue: 0.999
        step: 0.001
        initialValue: 0.1
        onSliderValueChanged: {
            Realsense.pointcloudoptions.ma_alpha = value
        }
    }
    Button {
        id: resetTareButton
        text: "Reset Tare"
        Layout.fillWidth: true
        onClicked: Realsense.resetTare()
        Component.onCompleted: enabled = false
        Connections {
            target: Realsense
            function onTareChanged()
            {
                resetTareButton.enabled = Realsense.tared
            }
        }
    }
    Button {
        id: tareButton
        text: "Tare"
        Layout.fillWidth: true
        onClicked: Realsense.tare()
        Component.onCompleted: enabled = false
        Connections {
            target: Realsense
            function onTareChanged()
            {
                tareButton.enabled = !Realsense.tared
            }
        }
    }
    Button {
        id: loadExtrinsicsButton
        text: "Load Extrinsics"
        Layout.fillWidth: true
        onClicked: Realsense.loadExtrinsics()
    }
    CheckBox {
        Layout.fillWidth: true
        text: "Paint Points"
        checked: true
        onToggled: Realsense.paintPoints = checked
    }
    ComboBox {
        id: imageSelector
        Layout.columnSpan: 2
        Layout.fillWidth: true
        Layout.alignment: Qt.AlignHCenter
        model: ListModel {
            ListElement {
                text: "Color Image"
            }
            ListElement {
                text: "Depth Image"
            }
        }
        onCurrentIndexChanged: image.src = currentIndex ? "image://realsense/depth/": "image://realsense/color/"
    }
    Label {
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        property string textc: "Nothing received yet"
        id: distanceLabel
        text: "Raw height: " + Realsense.distanceRaw.toLocaleString(Qt.locale("de_DE"), 'f', 2) + " m"
        horizontalAlignment: Text.AlignHCenter
    }
    Label {
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        property string textc: "Nothing received yet"
        id: frametimeLabel
        text: "Frame time: " + Realsense.frameTime.toLocaleString(Qt.locale("de_DE"), 'f', 0) + " ms"
        horizontalAlignment: Text.AlignHCenter
    }
}