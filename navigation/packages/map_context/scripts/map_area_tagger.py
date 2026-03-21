#!/usr/bin/env python3
"""
Map Area Tagger - Qt UI for tagging areas and locations on a ROS2 map.
Loads a map image (.pgm/.png) and allows clicking to define areas with
named locations and polygon boundaries. Exports to areas.json format.
"""

import sys
import os
import json
import math
import yaml
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QFileDialog, QListWidget,
    QListWidgetItem, QComboBox, QGroupBox, QSplitter, QMessageBox,
    QInputDialog, QScrollArea, QToolBar, QAction, QStatusBar,
    QFrame, QStyle, QMenu, QTreeWidget, QTreeWidgetItem, QHeaderView,
    QDoubleSpinBox, QDialog, QDialogButtonBox, QFormLayout
)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal, QSize
from PyQt5.QtGui import (
    QPixmap, QPainter, QColor, QPen, QBrush, QFont, QIcon,
    QWheelEvent, QMouseEvent, QPainterPath, QPolygonF, QCursor,
    QImage
)


AREA_COLORS = [
    QColor(52, 152, 219, 80),    # Blue
    QColor(231, 76, 60, 80),     # Red
    QColor(46, 204, 113, 80),    # Green
    QColor(155, 89, 182, 80),    # Purple
    QColor(241, 196, 15, 80),    # Yellow
    QColor(230, 126, 34, 80),    # Orange
    QColor(26, 188, 156, 80),    # Teal
    QColor(236, 64, 122, 80),    # Pink
    QColor(63, 81, 181, 80),     # Indigo
    QColor(0, 150, 136, 80),     # Cyan
]

LOCATION_COLOR = QColor(255, 255, 255, 220)
POLYGON_VERTEX_COLOR = QColor(255, 200, 50, 200)


class MapCanvas(QWidget):
    """Widget for displaying the map and handling mouse interactions."""
    location_placed = pyqtSignal(float, float, float)  # mx, my, yaw
    polygon_clicked = pyqtSignal(float, float)
    mouse_moved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pixmap = None
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.map_height = 0
        self.zoom = 1.0
        self.pan_offset = QPointF(0, 0)
        self.last_mouse_pos = None
        self.panning = False
        self.areas = {}
        self.current_area = None
        self.mode = 'location'  # 'location' or 'polygon'
        self.temp_polygon = []
        # Drag-to-orient state
        self.dragging_orientation = False
        self.drag_start_map = None  # (mx, my) where click started
        self.drag_yaw = 0.0
        self.setMouseTracking(True)
        self.setMinimumSize(600, 400)
        self.setFocusPolicy(Qt.StrongFocus)

    def load_map(self, image_path, yaml_path):
        """Load map image and metadata from yaml."""
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        self.resolution = meta.get('resolution', 0.05)
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = origin[0]
        self.origin_y = origin[1]

        img = QImage(image_path)
        if img.isNull():
            return False
        self.map_height = img.height()
        self.pixmap = QPixmap.fromImage(img)
        self.fit_to_view()
        self.update()
        return True

    def fit_to_view(self):
        if not self.pixmap:
            return
        w_ratio = self.width() / self.pixmap.width()
        h_ratio = self.height() / self.pixmap.height()
        self.zoom = min(w_ratio, h_ratio) * 0.95
        self.pan_offset = QPointF(
            (self.width() - self.pixmap.width() * self.zoom) / 2,
            (self.height() - self.pixmap.height() * self.zoom) / 2
        )

    def pixel_to_map(self, px, py):
        """Convert pixel coordinates to map coordinates (meters)."""
        mx = px * self.resolution + self.origin_x
        my = (self.map_height - py) * self.resolution + self.origin_y
        return mx, my

    def map_to_pixel(self, mx, my):
        """Convert map coordinates (meters) to pixel coordinates."""
        px = (mx - self.origin_x) / self.resolution
        py = self.map_height - (my - self.origin_y) / self.resolution
        return px, py

    def screen_to_pixel(self, sx, sy):
        """Convert screen coordinates to image pixel coordinates."""
        px = (sx - self.pan_offset.x()) / self.zoom
        py = (sy - self.pan_offset.y()) / self.zoom
        return px, py

    def pixel_to_screen(self, px, py):
        """Convert image pixel coordinates to screen coordinates."""
        sx = px * self.zoom + self.pan_offset.x()
        sy = py * self.zoom + self.pan_offset.y()
        return sx, sy

    def wheelEvent(self, event: QWheelEvent):
        old_zoom = self.zoom
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.zoom *= factor
        self.zoom = max(0.1, min(50.0, self.zoom))
        # Zoom toward mouse
        mx, my = event.pos().x(), event.pos().y()
        self.pan_offset.setX(mx - (mx - self.pan_offset.x()) * self.zoom / old_zoom)
        self.pan_offset.setY(my - (my - self.pan_offset.y()) * self.zoom / old_zoom)
        self.update()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MiddleButton or (event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier):
            self.panning = True
            self.last_mouse_pos = event.pos()
            self.setCursor(QCursor(Qt.ClosedHandCursor))
        elif event.button() == Qt.LeftButton and self.pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            if 0 <= px < self.pixmap.width() and 0 <= py < self.pixmap.height():
                mx, my = self.pixel_to_map(px, py)
                if self.mode == 'location':
                    self.dragging_orientation = True
                    self.drag_start_map = (mx, my)
                    self.drag_yaw = 0.0
                elif self.mode == 'polygon':
                    self.polygon_clicked.emit(mx, my)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.panning and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.pan_offset += delta
            self.last_mouse_pos = event.pos()
            self.update()
        elif self.dragging_orientation and self.drag_start_map and self.pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            mx, my = self.pixel_to_map(px, py)
            dx = mx - self.drag_start_map[0]
            dy = my - self.drag_start_map[1]
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                self.drag_yaw = math.atan2(dy, dx)
            self.update()
        if self.pixmap:
            px, py = self.screen_to_pixel(event.pos().x(), event.pos().y())
            if 0 <= px < self.pixmap.width() and 0 <= py < self.pixmap.height():
                mx, my = self.pixel_to_map(px, py)
                self.mouse_moved.emit(mx, my)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if self.dragging_orientation and self.drag_start_map:
            mx, my = self.drag_start_map
            self.location_placed.emit(mx, my, self.drag_yaw)
            self.dragging_orientation = False
            self.drag_start_map = None
            self.update()
            return
        if event.button() == Qt.MiddleButton or event.button() == Qt.LeftButton:
            self.panning = False
            self.setCursor(QCursor(Qt.ArrowCursor))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        if not self.pixmap:
            painter.setPen(QColor(120, 120, 120))
            painter.setFont(QFont("Segoe UI", 14))
            painter.drawText(self.rect(), Qt.AlignCenter, "Load a map to begin\n(File → Open Map)")
            return

        # Draw map
        painter.save()
        painter.translate(self.pan_offset)
        painter.scale(self.zoom, self.zoom)
        painter.drawPixmap(0, 0, self.pixmap)

        # Draw areas
        color_idx = 0
        for area_name, area_data in self.areas.items():
            color = AREA_COLORS[color_idx % len(AREA_COLORS)]
            border_color = QColor(color.red(), color.green(), color.blue(), 180)
            color_idx += 1

            # Draw polygon
            if 'polygon' in area_data:
                polygon = QPolygonF()
                for pt in area_data['polygon']:
                    px, py = self.map_to_pixel(pt[0], pt[1])
                    polygon.append(QPointF(px, py))
                painter.setPen(QPen(border_color, 2.0 / self.zoom))
                painter.setBrush(QBrush(color))
                painter.drawPolygon(polygon)

                # Area label
                if len(area_data['polygon']) > 0:
                    cx = sum(p[0] for p in area_data['polygon']) / len(area_data['polygon'])
                    cy = sum(p[1] for p in area_data['polygon']) / len(area_data['polygon'])
                    lpx, lpy = self.map_to_pixel(cx, cy)
                    font = QFont("Segoe UI", max(8, int(11 / self.zoom)))
                    font.setBold(True)
                    painter.setFont(font)
                    painter.setPen(QPen(Qt.white))
                    painter.drawText(QPointF(lpx - 30 / self.zoom, lpy), area_name)

            # Draw locations
            for loc_name, loc_data in area_data.items():
                if loc_name == 'polygon':
                    continue
                x, y = loc_data[0], loc_data[1]
                qw = loc_data[6] if len(loc_data) >= 7 else 1.0
                qz = loc_data[5] if len(loc_data) >= 6 else 0.0
                yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

                px, py = self.map_to_pixel(x, y)
                r = max(3, 5 / self.zoom)

                # Location dot
                painter.setPen(QPen(border_color, 1.5 / self.zoom))
                painter.setBrush(QBrush(LOCATION_COLOR))
                painter.drawEllipse(QPointF(px, py), r, r)

                # Direction arrow
                arrow_len = r * 2.5
                ax = px + arrow_len * math.cos(-yaw)
                ay = py + arrow_len * math.sin(-yaw)
                painter.setPen(QPen(border_color, 2.0 / self.zoom))
                painter.drawLine(QPointF(px, py), QPointF(ax, ay))

                # Label with background
                font = QFont("Segoe UI", max(5, int(7 / self.zoom)))
                painter.setFont(font)
                label_x = px + r + 2 / self.zoom
                label_y = py - r
                # Draw text shadow/background for readability
                painter.setPen(QPen(QColor(0, 0, 0, 180)))
                painter.drawText(QPointF(label_x + 0.5 / self.zoom, label_y + 0.5 / self.zoom), loc_name)
                # Colored label matching area
                label_color = QColor(color.red(), color.green(), color.blue(), 255)
                label_color = label_color.lighter(180)
                painter.setPen(QPen(label_color))
                painter.drawText(QPointF(label_x, label_y), loc_name)

        # Draw drag-to-orient preview
        if self.dragging_orientation and self.drag_start_map:
            smx, smy = self.drag_start_map
            spx, spy = self.map_to_pixel(smx, smy)
            r = max(4, 6 / self.zoom)
            # Preview dot
            painter.setPen(QPen(QColor(0, 200, 255, 200), 2.0 / self.zoom))
            painter.setBrush(QBrush(QColor(0, 200, 255, 100)))
            painter.drawEllipse(QPointF(spx, spy), r, r)
            # Preview arrow
            arrow_len = r * 4
            yaw = self.drag_yaw
            arx = spx + arrow_len * math.cos(-yaw)
            ary = spy + arrow_len * math.sin(-yaw)
            painter.setPen(QPen(QColor(0, 200, 255, 220), 2.5 / self.zoom))
            painter.drawLine(QPointF(spx, spy), QPointF(arx, ary))

        # Draw temp polygon vertices
        if self.temp_polygon:
            painter.setPen(QPen(POLYGON_VERTEX_COLOR, 2.0 / self.zoom))
            painter.setBrush(QBrush(QColor(255, 200, 50, 40)))
            poly = QPolygonF()
            for pt in self.temp_polygon:
                px, py = self.map_to_pixel(pt[0], pt[1])
                poly.append(QPointF(px, py))
                r = max(4, 5 / self.zoom)
                painter.drawEllipse(QPointF(px, py), r, r)
            if len(self.temp_polygon) > 1:
                painter.drawPolyline(poly)

        painter.restore()


class MapAreaTagger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Map Area Tagger")
        self.setMinimumSize(1100, 700)
        self.areas = {}
        self.current_area = None
        self.mode = 'location'
        self.temp_polygon = []
        self.map_yaml_path = None
        self.setup_ui()
        self.apply_style()

    def setup_ui(self):
        # Central widget
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

        open_action = QAction("Open Map", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self.open_map)
        toolbar.addAction(open_action)

        load_action = QAction("Load JSON", self)
        load_action.setShortcut("Ctrl+L")
        load_action.triggered.connect(self.load_json)
        toolbar.addAction(load_action)

        save_action = QAction("Save JSON", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_json)
        toolbar.addAction(save_action)

        toolbar.addSeparator()

        new_action = QAction("New", self)
        new_action.setShortcut("Ctrl+N")
        new_action.triggered.connect(self.new_project)
        toolbar.addAction(new_action)

        fit_action = QAction("Fit View", self)
        fit_action.setShortcut("F")
        fit_action.triggered.connect(self.fit_view)
        toolbar.addAction(fit_action)

        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Map canvas
        self.canvas = MapCanvas()
        self.canvas.location_placed.connect(self.on_location_placed)
        self.canvas.polygon_clicked.connect(self.on_polygon_click)
        self.canvas.mouse_moved.connect(self.on_mouse_move)
        splitter.addWidget(self.canvas)

        # Side panel
        panel = QWidget()
        panel.setMaximumWidth(320)
        panel.setMinimumWidth(280)
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(8, 8, 8, 8)
        panel_layout.setSpacing(6)

        # Mode selector
        mode_group = QGroupBox("Mode")
        mode_layout = QHBoxLayout(mode_group)
        self.btn_location_mode = QPushButton("Add Location")
        self.btn_location_mode.setCheckable(True)
        self.btn_location_mode.setChecked(True)
        self.btn_location_mode.clicked.connect(lambda: self.set_mode('location'))
        self.btn_polygon_mode = QPushButton("Draw Polygon")
        self.btn_polygon_mode.setCheckable(True)
        self.btn_polygon_mode.clicked.connect(lambda: self.set_mode('polygon'))
        mode_layout.addWidget(self.btn_location_mode)
        mode_layout.addWidget(self.btn_polygon_mode)
        panel_layout.addWidget(mode_group)

        # Area management
        area_group = QGroupBox("Areas")
        area_layout = QVBoxLayout(area_group)

        area_btn_layout = QHBoxLayout()
        self.btn_add_area = QPushButton("+ New Area")
        self.btn_add_area.clicked.connect(self.add_area)
        self.btn_del_area = QPushButton("Delete")
        self.btn_del_area.clicked.connect(self.delete_area)
        area_btn_layout.addWidget(self.btn_add_area)
        area_btn_layout.addWidget(self.btn_del_area)
        area_layout.addLayout(area_btn_layout)

        self.area_list = QListWidget()
        self.area_list.currentItemChanged.connect(self.on_area_selected)
        area_layout.addWidget(self.area_list)
        panel_layout.addWidget(area_group)

        # Locations tree
        loc_group = QGroupBox("Locations & Polygon")
        loc_layout = QVBoxLayout(loc_group)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Name", "X", "Y", "Yaw°"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tree.setColumnWidth(1, 55)
        self.tree.setColumnWidth(2, 55)
        self.tree.setColumnWidth(3, 45)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self.tree_context_menu)
        loc_layout.addWidget(self.tree)

        loc_btn_layout = QHBoxLayout()
        self.btn_del_location = QPushButton("Delete Selected")
        self.btn_del_location.clicked.connect(self.delete_selected_item)
        self.btn_clear_polygon = QPushButton("Clear Polygon")
        self.btn_clear_polygon.clicked.connect(self.clear_polygon)
        loc_btn_layout.addWidget(self.btn_del_location)
        loc_btn_layout.addWidget(self.btn_clear_polygon)
        loc_layout.addLayout(loc_btn_layout)
        panel_layout.addWidget(loc_group)

        # Finish polygon button
        self.btn_finish_polygon = QPushButton("Finish Polygon")
        self.btn_finish_polygon.clicked.connect(self.finish_polygon)
        self.btn_finish_polygon.setVisible(False)
        panel_layout.addWidget(self.btn_finish_polygon)

        panel_layout.addStretch()

        # Coordinates display
        self.coord_label = QLabel("x: --  y: --")
        self.coord_label.setStyleSheet("color: #aaa; font-size: 11px; padding: 4px;")
        panel_layout.addWidget(self.coord_label)

        splitter.addWidget(panel)
        splitter.setSizes([780, 320])

        # Status bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Open a map (.yaml) to begin")

    def apply_style(self):
        self.setStyleSheet("""
            QMainWindow { background: #1e1e1e; }
            QWidget { background: #252526; color: #cccccc; font-family: 'Segoe UI', sans-serif; font-size: 12px; }
            QGroupBox { border: 1px solid #3c3c3c; border-radius: 4px; margin-top: 8px; padding-top: 14px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { background: #0e639c; color: white; border: none; border-radius: 3px; padding: 6px 12px; font-size: 12px; }
            QPushButton:hover { background: #1177bb; }
            QPushButton:pressed { background: #0d5689; }
            QPushButton:checked { background: #16825d; }
            QPushButton:disabled { background: #3c3c3c; color: #666; }
            QListWidget, QTreeWidget { background: #1e1e1e; border: 1px solid #3c3c3c; border-radius: 3px; }
            QListWidget::item:selected, QTreeWidget::item:selected { background: #094771; }
            QListWidget::item:hover, QTreeWidget::item:hover { background: #2a2d2e; }
            QLineEdit { background: #1e1e1e; border: 1px solid #3c3c3c; border-radius: 3px; padding: 4px 8px; color: #ccc; }
            QLineEdit:focus { border-color: #0e639c; }
            QToolBar { background: #333333; border: none; spacing: 4px; padding: 4px; }
            QToolBar QToolButton { color: #ccc; padding: 4px 8px; }
            QToolBar QToolButton:hover { background: #444; border-radius: 3px; }
            QStatusBar { background: #007acc; color: white; font-size: 11px; }
            QSplitter::handle { background: #3c3c3c; width: 2px; }
            QHeaderView::section { background: #2d2d2d; border: 1px solid #3c3c3c; padding: 3px; }
            QScrollBar:vertical { background: #1e1e1e; width: 10px; }
            QScrollBar::handle:vertical { background: #424242; border-radius: 4px; min-height: 20px; }
            QMenu { background: #252526; border: 1px solid #3c3c3c; }
            QMenu::item:selected { background: #094771; }
        """)

    def set_mode(self, mode):
        self.mode = mode
        self.canvas.mode = mode
        self.btn_location_mode.setChecked(mode == 'location')
        self.btn_polygon_mode.setChecked(mode == 'polygon')
        self.btn_finish_polygon.setVisible(mode == 'polygon')
        if mode == 'location':
            self.temp_polygon = []
            self.canvas.temp_polygon = []
            self.canvas.update()
            self.status.showMessage("Click on the map to add a location")
        else:
            self.status.showMessage("Click vertices to define polygon boundary. Click 'Finish Polygon' when done.")

    def new_project(self):
        reply = QMessageBox.question(
            self, "New Project",
            "Clear all areas and start from scratch?",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.areas = {}
            self.current_area = None
            self.temp_polygon = []
            self.canvas.areas = {}
            self.canvas.temp_polygon = []
            self.canvas.current_area = None
            self.refresh_area_list()
            self.refresh_tree()
            self.canvas.update()
            self.status.showMessage("Started new project")

    def open_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Map YAML", "", "Map files (*.yaml *.yml)")
        if not path:
            return
        self.map_yaml_path = path
        map_dir = os.path.dirname(path)
        with open(path, 'r') as f:
            meta = yaml.safe_load(f)
        image_file = meta.get('image', '')
        image_path = os.path.join(map_dir, image_file)
        if not os.path.exists(image_path):
            QMessageBox.warning(self, "Error", f"Map image not found: {image_path}")
            return
        if self.canvas.load_map(image_path, path):
            self.status.showMessage(f"Loaded: {os.path.basename(path)} | Resolution: {self.canvas.resolution}m/px")
        else:
            QMessageBox.warning(self, "Error", "Failed to load map image")

    def load_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Areas JSON", "", "JSON files (*.json)")
        if not path:
            return
        with open(path, 'r') as f:
            self.areas = json.load(f)
        self.canvas.areas = self.areas
        self.refresh_area_list()
        self.canvas.update()
        self.status.showMessage(f"Loaded {len(self.areas)} areas from {os.path.basename(path)}")

    def save_json(self):
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'maps', 'areas')
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Areas JSON", default_dir, "JSON files (*.json)")
        if not path:
            return
        if not path.endswith('.json'):
            path += '.json'
        with open(path, 'w') as f:
            json.dump(self.areas, f, indent=4)
        self.status.showMessage(f"Saved to {path}")

    def fit_view(self):
        self.canvas.fit_to_view()
        self.canvas.update()

    def add_area(self):
        name, ok = QInputDialog.getText(self, "New Area", "Area name:")
        if ok and name:
            name = name.strip().lower().replace(' ', '_')
            if name in self.areas:
                QMessageBox.warning(self, "Error", f"Area '{name}' already exists")
                return
            self.areas[name] = {}
            self.canvas.areas = self.areas
            self.refresh_area_list()
            # Select the new area
            for i in range(self.area_list.count()):
                if self.area_list.item(i).text() == name:
                    self.area_list.setCurrentRow(i)
                    break
            self.status.showMessage(f"Created area: {name}")

    def delete_area(self):
        if not self.current_area:
            return
        reply = QMessageBox.question(
            self, "Delete Area",
            f"Delete area '{self.current_area}' and all its contents?",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            del self.areas[self.current_area]
            self.current_area = None
            self.canvas.areas = self.areas
            self.refresh_area_list()
            self.refresh_tree()
            self.canvas.update()

    def on_area_selected(self, current, previous):
        if current:
            self.current_area = current.text()
            self.canvas.current_area = self.current_area
        else:
            self.current_area = None
            self.canvas.current_area = None
        self.refresh_tree()

    def on_location_placed(self, mx, my, yaw):
        if not self.current_area:
            self.status.showMessage("Select or create an area first!")
            return
        name, ok = QInputDialog.getText(self, "Location Name", "Name for this location:")
        if not ok or not name:
            return
        name = name.strip().lower().replace(' ', '_')

        # Convert yaw to quaternion (z-axis rotation only)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        self.areas[self.current_area][name] = [mx, my, 0.0, 0.0, 0.0, qz, qw]
        self.canvas.areas = self.areas
        self.refresh_tree()
        self.canvas.update()
        self.status.showMessage(f"Added '{name}' at ({mx:.2f}, {my:.2f}) yaw={math.degrees(yaw):.0f}")

    def on_polygon_click(self, mx, my):
        if not self.current_area:
            self.status.showMessage("Select or create an area first!")
            return
        self.temp_polygon.append([mx, my])
        self.canvas.temp_polygon = self.temp_polygon
        self.canvas.update()
        self.status.showMessage(f"Polygon vertex {len(self.temp_polygon)} at ({mx:.2f}, {my:.2f})")

    def finish_polygon(self):
        if not self.current_area:
            return
        if len(self.temp_polygon) < 3:
            self.status.showMessage("Need at least 3 vertices for a polygon")
            return
        self.areas[self.current_area]['polygon'] = self.temp_polygon.copy()
        self.temp_polygon = []
        self.canvas.temp_polygon = []
        self.canvas.areas = self.areas
        self.refresh_tree()
        self.canvas.update()
        self.status.showMessage(f"Polygon saved for '{self.current_area}'")

    def clear_polygon(self):
        if not self.current_area:
            return
        self.temp_polygon = []
        self.canvas.temp_polygon = []
        if 'polygon' in self.areas.get(self.current_area, {}):
            del self.areas[self.current_area]['polygon']
        self.canvas.areas = self.areas
        self.refresh_tree()
        self.canvas.update()

    def on_mouse_move(self, mx, my):
        self.coord_label.setText(f"x: {mx:.3f}  y: {my:.3f}")

    def refresh_area_list(self):
        self.area_list.clear()
        for name in self.areas:
            self.area_list.addItem(name)

    def refresh_tree(self):
        self.tree.clear()
        if not self.current_area or self.current_area not in self.areas:
            return
        area = self.areas[self.current_area]

        # Locations
        for name, data in area.items():
            if name == 'polygon':
                continue
            yaw = 0.0
            if len(data) >= 7:
                qz, qw = data[5], data[6]
                yaw = math.degrees(math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))
            item = QTreeWidgetItem([
                name,
                f"{data[0]:.2f}",
                f"{data[1]:.2f}",
                f"{yaw:.0f}"
            ])
            self.tree.addTopLevelItem(item)

        # Polygon
        if 'polygon' in area:
            poly_item = QTreeWidgetItem(["polygon", "", "", ""])
            poly_item.setForeground(0, QBrush(POLYGON_VERTEX_COLOR))
            for i, pt in enumerate(area['polygon']):
                child = QTreeWidgetItem([f"v{i}", f"{pt[0]:.2f}", f"{pt[1]:.2f}", ""])
                poly_item.addChild(child)
            self.tree.addTopLevelItem(poly_item)
            poly_item.setExpanded(True)

    def tree_context_menu(self, pos):
        item = self.tree.itemAt(pos)
        if not item or not self.current_area:
            return
        menu = QMenu(self)
        delete_action = menu.addAction("Delete")
        rename_action = menu.addAction("Rename")
        action = menu.exec_(self.tree.viewport().mapToGlobal(pos))
        if action == delete_action:
            self.delete_tree_item(item)
        elif action == rename_action:
            self.rename_tree_item(item)

    def delete_tree_item(self, item):
        name = item.text(0)
        if name in self.areas.get(self.current_area, {}):
            del self.areas[self.current_area][name]
            self.canvas.areas = self.areas
            self.refresh_tree()
            self.canvas.update()

    def rename_tree_item(self, item):
        old_name = item.text(0)
        if old_name == 'polygon':
            return
        new_name, ok = QInputDialog.getText(self, "Rename", "New name:", text=old_name)
        if ok and new_name and new_name != old_name:
            new_name = new_name.strip().lower().replace(' ', '_')
            data = self.areas[self.current_area].pop(old_name)
            self.areas[self.current_area][new_name] = data
            self.canvas.areas = self.areas
            self.refresh_tree()
            self.canvas.update()

    def delete_selected_item(self):
        item = self.tree.currentItem()
        if item:
            self.delete_tree_item(item)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            if self.mode == 'polygon' and self.temp_polygon:
                self.temp_polygon = []
                self.canvas.temp_polygon = []
                self.canvas.update()
                self.status.showMessage("Polygon cancelled")
        elif event.key() == Qt.Key_Return and self.mode == 'polygon':
            self.finish_polygon()
        super().keyPressEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Map Area Tagger")
    window = MapAreaTagger()
    window.show()

    # Auto-load map if yaml path passed as argument
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]
        if os.path.exists(yaml_path):
            map_dir = os.path.dirname(yaml_path)
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            image_path = os.path.join(map_dir, meta.get('image', ''))
            if os.path.exists(image_path):
                window.canvas.load_map(image_path, yaml_path)
                window.map_yaml_path = yaml_path

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
