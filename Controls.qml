import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQuick.Dialogs 1.3

GridLayout {
    Connections {
        target: Realsense
        function onIsRunningChanged()
        {
            resolutionSelector.enabled = !Realsense.running
            startButton.enabled = !Realsense.running
            stopButton.enabled = Realsense.running
            pauseButton.enabled = Realsense.running
            tareButton.enabled = Realsense.running
        }
    }

    FileDialog {
        id: recordFilePicker
        title: "Please choose a location"
        folder: shortcuts.desktop
        selectFolder: true
        onAccepted: {
            Realsense.recordFile = recordFilePicker.fileUrl
            close()
        }
        onRejected: {
            close()
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
            ListElement {
                text: "BAG File"
            }
        }
        Component.onCompleted: Realsense.resolution = currentIndex
        onCurrentIndexChanged: {
            if (currentIndex < 5)
            {
                Realsense.useBag = false
                Realsense.resolution = currentIndex
            }
            else if (Realsense.bagFile.toString() != "")
            {
                Realsense.useBag = true
                console.log("Using BAG File: " + Realsense.bagFile)
            }
            else
            {
                console.error("Choose BAG File first! Current file: " + Realsense.bagFile)
                currentIndex = 3
            }
        }
    }
    Button {
        id: startButton
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        Layout.preferredWidth: parent.width / 2
        text: "Start"
        onClicked: {
            Realsense.start()
        }
    }
    Button {
        id: stopButton
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        Layout.preferredWidth: parent.width / 2
        text: "Stop"
        onClicked: {
            Realsense.stop()
        }
        Component.onCompleted: enabled = false
    }
    Button {
        id: pauseButton
        Layout.alignment: Qt.AlignHCenter
        Layout.fillWidth: true
        Layout.columnSpan: 2
        text: Realsense.paused ? "Resume" : "Pause"
        Component.onCompleted: enabled = false
        onClicked: {
            Realsense.paused = !Realsense.paused
        }
        visible: Realsense.useBag
    }

    CheckBox {
        id: proccessPointsBox
        Layout.fillWidth: true
        text: "Process Points"
        checked: false
        Component.onCompleted: {
            Realsense.processPoints = checked
        }
        onToggled: {
            Realsense.processPoints = checked
        }
    }
    CheckBox {
        id: recordCheckBox
        Layout.fillWidth: true
        text: "Record"
        checked: false
        onToggled: {
            Realsense.record = checked
        }
        enabled: !Realsense.running
    }
    Label {
        Layout.fillWidth: true
        Layout.minimumHeight: 30
        id: recordFileLabel
        text: "Choose record location"
        visible: recordCheckBox.checked
        MouseArea{
            anchors.fill: parent
            onClicked: recordFilePicker.visible = true
        }
        Connections {
            target: Realsense
            function onRecordFileChanged() {
                recordFileLabel.text = Realsense.recordFile
            }
        }
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
    }
    Button {
        id: loadExtrinsicsButton
        text: "Load Extrinsics"
        Layout.fillWidth: true
        onClicked: Realsense.loadExtrinsics()
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
    }
    CheckBox {
        Layout.fillWidth: true
        text: "Paint Points"
        checked: true
        onToggled: Realsense.paintPoints = checked
        enabled: proccessPointsBox.checked
        visible: proccessPointsBox.checked
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
            ListElement {
                text: "Infrared Image"
            }
        }
        onCurrentIndexChanged: {
            if (currentIndex == 0) {
                image.src = "image://realsense/color/"
            }
            if (currentIndex == 1) {
                image.src = "image://realsense/depth/"
            }
            if (currentIndex == 2) {
                image.src = "image://realsense/infrared/"
            }
        }
    }
    Label {
        Layout.fillWidth: true
        Layout.columnSpan: 2
        property string textc: "Nothing received yet"
        id: frametimeLabel
        text: "Frame time: " + Realsense.frameTime.toLocaleString(Qt.locale("de_DE"), 'f', 0) + " ms"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignRight
    }
    TableControl {
        id: tablecontrols
        Layout.fillWidth: true
        Layout.columnSpan: 2
        visible: proccessPointsBox.checked
    }
}
