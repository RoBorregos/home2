#!/usr/bin/env python3
"""
Navigation UI - Real-time navigation monitor and control panel.
Displays map, robot position, costmaps, and trajectories.
Provides controls for sending goals, changing maps, and monitoring nav2.
"""

import sys
import os
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Empty

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QSplitter, QToolBar,
    QStatusBar, QCheckBox, QComboBox, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QPointF, QTimer, QSize, pyqtSignal, QObject
from PyQt5.QtGui import (
    QPixmap, QPainter, QColor, QPen, QBrush, QFont,
    QMouseEvent, QWheelEvent, QPolygonF, QCursor, QImage
)


# Colors
COLOR_ROBOT = QColor(0, 200, 255, 255)
COLOR_ROBOT_FILL = QColor(0, 200, 255, 80)
COLOR_GOAL = QColor(255, 50, 50, 255)
COLOR_GOAL_FILL = QColor(255, 50, 50, 80)
COLOR_PATH = QColor(50, 255, 100, 200)
COLOR_LOCAL_COSTMAP = QColor(255, 100, 0, 60)
COLOR_GLOBAL_COSTMAP = QColor(150, 0, 255, 30)
COLOR_MAP_FREE = QColor(255, 255, 255)
COLOR_MAP_OCCUPIED = QColor(0, 0, 0)
COLOR_MAP_UNKNOWN = QColor(128, 128, 128)


class RosSignals(QObject):
    """Bridge between ROS callbacks (threads) and Qt signals (main thread)."""
    map_updated = pyqtSignal()
    path_updated = pyqtSignal()
    local_costmap_updated = pyqtSignal()
    global_costmap_updated = pyqtSignal()
    robot_pose_updated = pyqtSignal(float, float, float)  # x, y, yaw


class NavRosNode(Node):
    def __init__(self, signals):
        super().__init__('nav_ui')
        self.signals = signals

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for map topics (transient local)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriptions
        self.map_data = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.path_data = None
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        self.local_costmap_data = None
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self.local_costmap_callback, 10)

        self.global_costmap_data = None
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.global_costmap_callback, map_qos)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # TF timer
        self.create_timer(0.1, self.update_robot_pose)

    def map_callback(self, msg):
        self.map_data = msg
        self.signals.map_updated.emit()

    def path_callback(self, msg):
        self.path_data = msg
        self.signals.path_updated.emit()

    def local_costmap_callback(self, msg):
        self.local_costmap_data = msg
        self.signals.local_costmap_updated.emit()

    def global_costmap_callback(self, msg):
        self.global_costmap_data = msg
        self.signals.global_costmap_updated.emit()

    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.signals.robot_pose_updated.emit(
                self.robot_x, self.robot_y, self.robot_yaw)
        except TransformException:
            pass

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def send_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.initialpose_pub.publish(msg)


class NavCanvas(QWidget):
    """Real-time map display with robot, costmaps, and paths."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.map_pixmap = None
        self.local_costmap_pixmap = None
        self.global_costmap_pixmap = None
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Local costmap transform
        self.lc_origin_x = 0.0
        self.lc_origin_y = 0.0
        self.lc_width = 0
        self.lc_height = 0
        self.lc_resolution = 0.05

        # Global costmap transform
        self.gc_origin_x = 0.0
        self.gc_origin_y = 0.0
        self.gc_width = 0
        self.gc_height = 0
        self.gc_resolution = 0.05

        # View
        self.zoom = 1.0
        self.pan_offset = QPointF(0, 0)
        self.last_mouse_pos = None
        self.panning = False

        # Robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Goal drag
        self.dragging_goal = False
        self.goal_start = None
        self.goal_yaw = 0.0
        self.current_goal = None  # (x, y, yaw)

        # Initial pose drag
        self.dragging_initpose = False
        self.initpose_start = None
        self.initpose_yaw = 0.0

        # Path
        self.path_points = []

        # Visibility
        self.show_local_costmap = True
        self.show_global_costmap = True
        self.show_path = True

        # Mode: 'goal', 'initialpose', 'view'
        self.mode = 'goal'

        self.setMouseTracking(True)
        self.setMinimumSize(600, 400)
        self.setFocusPolicy(Qt.StrongFocus)

    def map_to_pixel(self, mx, my):
        px = (mx - self.map_origin_x) / self.map_resolution
        py = self.map_height - (my - self.map_origin_y) / self.map_resolution
        return px, py

    def pixel_to_map(self, px, py):
        mx = px * self.map_resolution + self.map_origin_x
        my = (self.map_height - py) * self.map_resolution + self.map_origin_y
        return mx, my

    def screen_to_pixel(self, sx, sy):
        px = (sx - self.pan_offset.x()) / self.zoom
        py = (sy - self.pan_offset.y()) / self.zoom
        return px, py

    def fit_to_view(self):
        if not self.map_pixmap:
            return
        w_ratio = self.width() / self.map_pixmap.width()
        h_ratio = self.height() / self.map_pixmap.height()
        self.zoom = min(w_ratio, h_ratio) * 0.95
        self.pan_offset = QPointF(
            (self.width() - self.map_pixmap.width() * self.zoom) / 2,
            (self.height() - self.map_pixmap.height() * self.zoom) / 2
        )

    def update_map(self, grid_msg):
        w, h = grid_msg.info.width, grid_msg.info.height
        self.map_resolution = grid_msg.info.resolution
        self.map_origin_x = grid_msg.info.origin.position.x
        self.map_origin_y = grid_msg.info.origin.position.y
        self.map_width = w
        self.map_height = h

        img = QImage(w, h, QImage.Format_RGB32)
        data = np.array(grid_msg.data, dtype=np.int8).reshape((h, w))
        for y in range(h):
            for x in range(w):
                v = data[y][x]
                if v == -1:
                    c = COLOR_MAP_UNKNOWN
                elif v == 0:
                    c = COLOR_MAP_FREE
                else:
                    intensity = max(0, 255 - int(v * 2.55))
                    c = QColor(intensity, intensity, intensity)
                img.setPixel(x, h - 1 - y, c.rgb())

        self.map_pixmap = QPixmap.fromImage(img)
        if self.zoom == 1.0 and self.pan_offset == QPointF(0, 0):
            self.fit_to_view()
        self.update()

    def update_costmap(self, grid_msg, is_local):
        w, h = grid_msg.info.width, grid_msg.info.height
        res = grid_msg.info.resolution
        ox = grid_msg.info.origin.position.x
        oy = grid_msg.info.origin.position.y

        img = QImage(w, h, QImage.Format_ARGB32)
        img.fill(Qt.transparent)
        data = np.array(grid_msg.data, dtype=np.int8).reshape((h, w))

        if is_local:
            color = COLOR_LOCAL_COSTMAP
        else:
            color = COLOR_GLOBAL_COSTMAP

        for y in range(h):
            for x in range(w):
                v = data[y][x]
                if v > 0 and v != -1:
                    alpha = min(255, int(v * 2))
                    c = QColor(color.red(), color.green(), color.blue(), alpha)
                    img.setPixel(x, h - 1 - y, c.rgba())

        pixmap = QPixmap.fromImage(img)
        if is_local:
            self.local_costmap_pixmap = pixmap
            self.lc_origin_x = ox
            self.lc_origin_y = oy
            self.lc_width = w
            self.lc_height = h
            self.lc_resolution = res
        else:
            self.global_costmap_pixmap = pixmap
            self.gc_origin_x = ox
            self.gc_origin_y = oy
            self.gc_width = w
            self.gc_height = h
            self.gc_resolution = res
        self.update()

    def update_path(self, path_msg):
        self.path_points = []
        for pose in path_msg.poses:
            self.path_points.append(
                (pose.pose.position.x, pose.pose.position.y))
        self.update()

    def wheelEvent(self, event: QWheelEvent):
        old_zoom = self.zoom
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.zoom = max(0.1, min(80.0, self.zoom * factor))
        mx, my = event.pos().x(), event.pos().y()
        self.pan_offset.setX(mx - (mx - self.pan_offset.x()) * self.zoom / old_zoom)
        self.pan_offset.setY(my - (my - self.pan_offset.y()) * self.zoom / old_zoom)
        self.update()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MiddleButton or (event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier):
            self.panning = True
            self.last_mouse_pos = event.pos()
            self.setCursor(QCursor(Qt.ClosedHandCursor))
        elif event.button() == Qt.LeftButton and self.map_pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            if 0 <= px < self.map_width and 0 <= py < self.map_height:
                mx, my = self.pixel_to_map(px, py)
                if self.mode == 'goal':
                    self.dragging_goal = True
                    self.goal_start = (mx, my)
                    self.goal_yaw = 0.0
                elif self.mode == 'initialpose':
                    self.dragging_initpose = True
                    self.initpose_start = (mx, my)
                    self.initpose_yaw = 0.0

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.panning and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.pan_offset += delta
            self.last_mouse_pos = event.pos()
            self.update()
        elif self.dragging_goal and self.goal_start and self.map_pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            mx, my = self.pixel_to_map(px, py)
            dx = mx - self.goal_start[0]
            dy = my - self.goal_start[1]
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                self.goal_yaw = math.atan2(dy, dx)
            self.update()
        elif self.dragging_initpose and self.initpose_start and self.map_pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            mx, my = self.pixel_to_map(px, py)
            dx = mx - self.initpose_start[0]
            dy = my - self.initpose_start[1]
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                self.initpose_yaw = math.atan2(dy, dx)
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if self.dragging_goal and self.goal_start:
            self.current_goal = (self.goal_start[0], self.goal_start[1], self.goal_yaw)
            self.dragging_goal = False
            self.goal_start = None
            self.update()
            return
        if self.dragging_initpose and self.initpose_start:
            x, y, yaw = self.initpose_start[0], self.initpose_start[1], self.initpose_yaw
            self.dragging_initpose = False
            self.initpose_start = None
            self.update()
            # Store for parent to pick up
            self._pending_initpose = (x, y, yaw)
            return
        if event.button() == Qt.MiddleButton or event.button() == Qt.LeftButton:
            self.panning = False
            self.setCursor(QCursor(Qt.ArrowCursor))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(25, 25, 30))

        if not self.map_pixmap:
            painter.setPen(QColor(100, 100, 100))
            painter.setFont(QFont("Segoe UI", 14))
            painter.drawText(self.rect(), Qt.AlignCenter, "Waiting for /map topic...")
            return

        painter.save()
        painter.translate(self.pan_offset)
        painter.scale(self.zoom, self.zoom)

        # Map
        painter.drawPixmap(0, 0, self.map_pixmap)

        # Global costmap
        if self.show_global_costmap and self.global_costmap_pixmap:
            gc_px = (self.gc_origin_x - self.map_origin_x) / self.map_resolution
            gc_py = self.map_height - (self.gc_origin_y - self.map_origin_y) / self.map_resolution - self.gc_height * (self.gc_resolution / self.map_resolution)
            scale_factor = self.gc_resolution / self.map_resolution
            painter.save()
            painter.translate(gc_px, gc_py)
            painter.scale(scale_factor, scale_factor)
            painter.drawPixmap(0, 0, self.global_costmap_pixmap)
            painter.restore()

        # Local costmap
        if self.show_local_costmap and self.local_costmap_pixmap:
            lc_px = (self.lc_origin_x - self.map_origin_x) / self.map_resolution
            lc_py = self.map_height - (self.lc_origin_y - self.map_origin_y) / self.map_resolution - self.lc_height * (self.lc_resolution / self.map_resolution)
            scale_factor = self.lc_resolution / self.map_resolution
            painter.save()
            painter.translate(lc_px, lc_py)
            painter.scale(scale_factor, scale_factor)
            painter.drawPixmap(0, 0, self.local_costmap_pixmap)
            painter.restore()

        # Path
        if self.show_path and len(self.path_points) > 1:
            painter.setPen(QPen(COLOR_PATH, 2.0 / self.zoom))
            poly = QPolygonF()
            for mx, my in self.path_points:
                px, py = self.map_to_pixel(mx, my)
                poly.append(QPointF(px, py))
            painter.drawPolyline(poly)

        # Goal
        if self.current_goal:
            gx, gy, gyaw = self.current_goal
            gpx, gpy = self.map_to_pixel(gx, gy)
            r = max(4, 7 / self.zoom)
            painter.setPen(QPen(COLOR_GOAL, 2.0 / self.zoom))
            painter.setBrush(QBrush(COLOR_GOAL_FILL))
            painter.drawEllipse(QPointF(gpx, gpy), r, r)
            arrow_len = r * 3
            ax = gpx + arrow_len * math.cos(-gyaw)
            ay = gpy + arrow_len * math.sin(-gyaw)
            painter.drawLine(QPointF(gpx, gpy), QPointF(ax, ay))
            font = QFont("Segoe UI", max(5, int(7 / self.zoom)))
            painter.setFont(font)
            painter.drawText(QPointF(gpx + r + 2 / self.zoom, gpy - r), "GOAL")

        # Goal drag preview
        if self.dragging_goal and self.goal_start:
            gx, gy = self.goal_start
            gpx, gpy = self.map_to_pixel(gx, gy)
            r = max(4, 7 / self.zoom)
            painter.setPen(QPen(COLOR_GOAL.lighter(150), 2.0 / self.zoom))
            painter.setBrush(QBrush(QColor(255, 50, 50, 40)))
            painter.drawEllipse(QPointF(gpx, gpy), r, r)
            arrow_len = r * 3
            ax = gpx + arrow_len * math.cos(-self.goal_yaw)
            ay = gpy + arrow_len * math.sin(-self.goal_yaw)
            painter.drawLine(QPointF(gpx, gpy), QPointF(ax, ay))

        # InitialPose drag preview
        if self.dragging_initpose and self.initpose_start:
            ix, iy = self.initpose_start
            ipx, ipy = self.map_to_pixel(ix, iy)
            r = max(4, 7 / self.zoom)
            painter.setPen(QPen(QColor(255, 200, 0, 200), 2.0 / self.zoom))
            painter.setBrush(QBrush(QColor(255, 200, 0, 40)))
            painter.drawEllipse(QPointF(ipx, ipy), r, r)
            arrow_len = r * 3
            ax = ipx + arrow_len * math.cos(-self.initpose_yaw)
            ay = ipy + arrow_len * math.sin(-self.initpose_yaw)
            painter.drawLine(QPointF(ipx, ipy), QPointF(ax, ay))

        # Robot
        rpx, rpy = self.map_to_pixel(self.robot_x, self.robot_y)
        r = max(5, 8 / self.zoom)
        painter.setPen(QPen(COLOR_ROBOT, 2.0 / self.zoom))
        painter.setBrush(QBrush(COLOR_ROBOT_FILL))
        painter.drawEllipse(QPointF(rpx, rpy), r, r)
        # Robot direction
        arrow_len = r * 3
        ax = rpx + arrow_len * math.cos(-self.robot_yaw)
        ay = rpy + arrow_len * math.sin(-self.robot_yaw)
        painter.setPen(QPen(COLOR_ROBOT, 2.5 / self.zoom))
        painter.drawLine(QPointF(rpx, rpy), QPointF(ax, ay))

        painter.restore()

        # HUD - mode indicator
        painter.setPen(Qt.NoPen)
        if self.mode == 'goal':
            painter.setBrush(QBrush(QColor(255, 50, 50, 150)))
        elif self.mode == 'initialpose':
            painter.setBrush(QBrush(QColor(255, 200, 0, 150)))
        else:
            painter.setBrush(QBrush(QColor(100, 100, 100, 150)))
        painter.drawRoundedRect(10, 10, 120, 28, 6, 6)
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        mode_text = {"goal": "Send Goal", "initialpose": "Set Pose", "view": "View Only"}
        painter.drawText(20, 29, mode_text.get(self.mode, ""))


class NavUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Navigation UI")
        self.setMinimumSize(1200, 750)
        self.setup_ui()
        self.apply_style()
        self.connect_ros_signals()

    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Toolbar
        toolbar = QToolBar("Main")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(20, 20))
        self.addToolBar(toolbar)

        from PyQt5.QtWidgets import QAction
        fit_action = QAction("Fit View", self)
        fit_action.setShortcut("F")
        fit_action.triggered.connect(lambda: (self.canvas.fit_to_view(), self.canvas.update()))
        toolbar.addAction(fit_action)

        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Canvas
        self.canvas = NavCanvas()
        splitter.addWidget(self.canvas)

        # Side panel
        panel = QWidget()
        panel.setMaximumWidth(300)
        panel.setMinimumWidth(260)
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(8, 8, 8, 8)
        panel_layout.setSpacing(6)

        # Mode
        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout(mode_group)
        mode_btn_layout = QHBoxLayout()
        self.btn_goal_mode = QPushButton("Send Goal")
        self.btn_goal_mode.setCheckable(True)
        self.btn_goal_mode.setChecked(True)
        self.btn_goal_mode.clicked.connect(lambda: self.set_mode('goal'))
        self.btn_initpose_mode = QPushButton("Set Pose")
        self.btn_initpose_mode.setCheckable(True)
        self.btn_initpose_mode.clicked.connect(lambda: self.set_mode('initialpose'))
        self.btn_view_mode = QPushButton("View")
        self.btn_view_mode.setCheckable(True)
        self.btn_view_mode.clicked.connect(lambda: self.set_mode('view'))
        mode_btn_layout.addWidget(self.btn_goal_mode)
        mode_btn_layout.addWidget(self.btn_initpose_mode)
        mode_btn_layout.addWidget(self.btn_view_mode)
        mode_layout.addLayout(mode_btn_layout)
        panel_layout.addWidget(mode_group)

        # Navigation controls
        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout(nav_group)

        self.btn_send_goal = QPushButton("Send Goal")
        self.btn_send_goal.clicked.connect(self.send_current_goal)
        nav_layout.addWidget(self.btn_send_goal)

        self.btn_cancel = QPushButton("Cancel Navigation")
        self.btn_cancel.setStyleSheet("QPushButton { background: #c0392b; } QPushButton:hover { background: #e74c3c; }")
        self.btn_cancel.clicked.connect(self.cancel_navigation)
        nav_layout.addWidget(self.btn_cancel)

        panel_layout.addWidget(nav_group)

        # Robot info
        info_group = QGroupBox("Robot")
        info_layout = QVBoxLayout(info_group)
        self.lbl_robot_pos = QLabel("Position: -- , --")
        self.lbl_robot_yaw = QLabel("Heading: --")
        self.lbl_goal = QLabel("Goal: none")
        info_layout.addWidget(self.lbl_robot_pos)
        info_layout.addWidget(self.lbl_robot_yaw)
        info_layout.addWidget(self.lbl_goal)
        panel_layout.addWidget(info_group)

        # Layers
        layers_group = QGroupBox("Layers")
        layers_layout = QVBoxLayout(layers_group)
        self.chk_local_costmap = QCheckBox("Local Costmap")
        self.chk_local_costmap.setChecked(True)
        self.chk_local_costmap.toggled.connect(
            lambda v: setattr(self.canvas, 'show_local_costmap', v) or self.canvas.update())
        self.chk_global_costmap = QCheckBox("Global Costmap")
        self.chk_global_costmap.setChecked(True)
        self.chk_global_costmap.toggled.connect(
            lambda v: setattr(self.canvas, 'show_global_costmap', v) or self.canvas.update())
        self.chk_path = QCheckBox("Path")
        self.chk_path.setChecked(True)
        self.chk_path.toggled.connect(
            lambda v: setattr(self.canvas, 'show_path', v) or self.canvas.update())
        layers_layout.addWidget(self.chk_local_costmap)
        layers_layout.addWidget(self.chk_global_costmap)
        layers_layout.addWidget(self.chk_path)
        panel_layout.addWidget(layers_group)

        # Map DB
        db_group = QGroupBox("RTAB-Map DB")
        db_layout = QVBoxLayout(db_group)
        self.lbl_current_db = QLabel("Current: --")
        db_layout.addWidget(self.lbl_current_db)

        panel_layout.addWidget(db_group)

        panel_layout.addStretch()

        # Connection status
        self.lbl_connection = QLabel("Waiting for topics...")
        self.lbl_connection.setStyleSheet("color: #f39c12; font-size: 11px; padding: 4px;")
        panel_layout.addWidget(self.lbl_connection)

        splitter.addWidget(panel)
        splitter.setSizes([900, 300])

        # Status bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Ready")

        # Poll for pending actions from canvas
        self.action_timer = QTimer()
        self.action_timer.timeout.connect(self.check_canvas_actions)
        self.action_timer.start(50)

    def apply_style(self):
        self.setStyleSheet("""
            QMainWindow { background: #1a1a2e; }
            QWidget { background: #16213e; color: #e0e0e0; font-family: 'Segoe UI', sans-serif; font-size: 12px; }
            QGroupBox { border: 1px solid #1a1a4e; border-radius: 5px; margin-top: 8px; padding-top: 14px; font-weight: bold; color: #7f8fa6; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { background: #0f3460; color: white; border: none; border-radius: 4px; padding: 8px 14px; font-size: 12px; }
            QPushButton:hover { background: #1a4f8a; }
            QPushButton:pressed { background: #0a2540; }
            QPushButton:checked { background: #e94560; }
            QPushButton:disabled { background: #2c2c54; color: #555; }
            QCheckBox { spacing: 6px; padding: 3px; }
            QCheckBox::indicator { width: 16px; height: 16px; border-radius: 3px; border: 1px solid #444; background: #1a1a2e; }
            QCheckBox::indicator:checked { background: #e94560; border-color: #e94560; }
            QComboBox { background: #1a1a2e; border: 1px solid #333; border-radius: 3px; padding: 4px 8px; color: #ccc; }
            QToolBar { background: #0f3460; border: none; spacing: 4px; padding: 4px; }
            QToolBar QToolButton { color: #ccc; padding: 4px 8px; }
            QToolBar QToolButton:hover { background: #1a4f8a; border-radius: 3px; }
            QStatusBar { background: #e94560; color: white; font-size: 11px; }
            QSplitter::handle { background: #1a1a4e; width: 2px; }
            QLabel { background: transparent; }
        """)

    def connect_ros_signals(self):
        signals = self.ros_node.signals
        signals.map_updated.connect(self.on_map_update)
        signals.path_updated.connect(self.on_path_update)
        signals.local_costmap_updated.connect(self.on_local_costmap_update)
        signals.global_costmap_updated.connect(self.on_global_costmap_update)
        signals.robot_pose_updated.connect(self.on_robot_pose)

    def on_map_update(self):
        if self.ros_node.map_data:
            self.canvas.update_map(self.ros_node.map_data)
            self.lbl_connection.setText("Connected")
            self.lbl_connection.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px;")

    def on_path_update(self):
        if self.ros_node.path_data:
            self.canvas.update_path(self.ros_node.path_data)

    def on_local_costmap_update(self):
        if self.ros_node.local_costmap_data:
            self.canvas.update_costmap(self.ros_node.local_costmap_data, is_local=True)

    def on_global_costmap_update(self):
        if self.ros_node.global_costmap_data:
            self.canvas.update_costmap(self.ros_node.global_costmap_data, is_local=False)

    def on_robot_pose(self, x, y, yaw):
        self.canvas.robot_x = x
        self.canvas.robot_y = y
        self.canvas.robot_yaw = yaw
        self.canvas.update()
        self.lbl_robot_pos.setText(f"Position: {x:.2f}, {y:.2f}")
        self.lbl_robot_yaw.setText(f"Heading: {math.degrees(yaw):.1f}")

    def set_mode(self, mode):
        self.canvas.mode = mode
        self.btn_goal_mode.setChecked(mode == 'goal')
        self.btn_initpose_mode.setChecked(mode == 'initialpose')
        self.btn_view_mode.setChecked(mode == 'view')
        mode_msgs = {
            'goal': 'Click and drag on map to set goal',
            'initialpose': 'Click and drag to set initial pose',
            'view': 'View only - scroll to zoom, shift+click to pan'
        }
        self.status.showMessage(mode_msgs.get(mode, ''))

    def send_current_goal(self):
        goal = self.canvas.current_goal
        if not goal:
            self.status.showMessage("No goal set. Click on the map first.")
            return
        x, y, yaw = goal
        self.ros_node.send_goal(x, y, yaw)
        self.lbl_goal.setText(f"Goal: ({x:.2f}, {y:.2f})")
        self.status.showMessage(f"Goal sent: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}")

    def cancel_navigation(self):
        # Cancel by publishing empty goal action
        try:
            cancel_client = self.ros_node.create_client(Empty, '/lifecycle_manager_navigation/manage_nodes')
        except Exception:
            pass
        self.canvas.current_goal = None
        self.canvas.path_points = []
        self.canvas.update()
        self.lbl_goal.setText("Goal: cancelled")
        self.status.showMessage("Navigation cancelled")

    def check_canvas_actions(self):
        # Check for pending goal send
        if self.canvas.current_goal and self.canvas.mode == 'goal':
            goal = self.canvas.current_goal
            self.lbl_goal.setText(f"Goal: ({goal[0]:.2f}, {goal[1]:.2f})")

        # Check for pending initial pose
        if hasattr(self.canvas, '_pending_initpose'):
            x, y, yaw = self.canvas._pending_initpose
            del self.canvas._pending_initpose
            self.ros_node.send_initialpose(x, y, yaw)
            self.status.showMessage(f"Initial pose set: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}")


def main():
    rclpy.init()
    signals = RosSignals()
    ros_node = NavRosNode(signals)

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    app.setApplicationName("Navigation UI")
    window = NavUI(ros_node)
    window.show()

    ret = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
