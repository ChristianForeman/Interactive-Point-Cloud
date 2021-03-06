import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from std_msgs.msg import Float32

rospy.init_node("radius_panel")

radius_pub = rospy.Publisher("radius", Float32, queue_size=10)


class TrajectoryPlaybackGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)

        # Idx slider
        self.traj_slider = QSlider(Qt.Horizontal)
        self.traj_slider.setValue(0)
        self.traj_slider.setTickPosition(QSlider.TicksBelow)
        self.traj_slider.setTickInterval(1)
        self.traj_slider.valueChanged.connect(self.slider_change)
        self.layout.addWidget(self.traj_slider)

    # Callback for slider change
    def slider_change(self):
        radius_pub.publish(self.traj_slider.value() / 100.0)


if __name__ == "__main__":
    app = QApplication([])
    gui = TrajectoryPlaybackGUI()
    gui.show()
    app.exec()
