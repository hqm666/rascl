#Using GUI to publish the cube location information
#

import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLineEdit, QPushButton, QLabel, QListWidget, QMessageBox
)


class GoalPoseGui(Node):
    def __init__(self):
        super().__init__("goal_pose_gui")
        self.pub = self.create_publisher(Point, "/goal_poses", 10)

        # 
        self.history = []  # list of dict: {"t":..., "x":..., "y":..., "z":...}

    def publish_point(self, x: float, y: float, z: float):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pub.publish(msg)

        t = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.history.append({"t": t, "x": msg.x, "y": msg.y, "z": msg.z})
        self.get_logger().info(f"Published /goal_poses: ({msg.x}, {msg.y}, {msg.z})")


class MainWindow(QWidget):
    def __init__(self, ros_node: GoalPoseGui):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Goal Pose Publisher GUI (/goal_poses)")

        # ---- UI ----
        layout = QVBoxLayout()

        row = QHBoxLayout()
        self.x_edit = QLineEdit("-230.0")
        self.y_edit = QLineEdit("50.0")
        self.z_edit = QLineEdit("15.0")
        row.addWidget(QLabel("x"))
        row.addWidget(self.x_edit)
        row.addWidget(QLabel("y"))
        row.addWidget(self.y_edit)
        row.addWidget(QLabel("z"))
        row.addWidget(self.z_edit)

        self.btn_pub = QPushButton("Publish")
        self.btn_pub.clicked.connect(self.on_publish_clicked)

        self.history_list = QListWidget()

        layout.addLayout(row)
        layout.addWidget(self.btn_pub)
        layout.addWidget(QLabel("History (latest at bottom):"))
        layout.addWidget(self.history_list)

        self.setLayout(layout)

    def on_publish_clicked(self):
        try:
            x = float(self.x_edit.text().strip())
            y = float(self.y_edit.text().strip())
            z = float(self.z_edit.text().strip())
        except ValueError:
            QMessageBox.warning(self, "Invalid input", "Please enter valid numbers for x, y, z.")
            return

        self.node.publish_point(x, y, z)

        last = self.node.history[-1]
        self.history_list.addItem(f'{last["t"]}  x={last["x"]:.3f}, y={last["y"]:.3f}, z={last["z"]:.3f}')


def main():
    rclpy.init()

    node = GoalPoseGui()

    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.resize(520, 400)
    win.show()

    # 
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(20)  # 

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
