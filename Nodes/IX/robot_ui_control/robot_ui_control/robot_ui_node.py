#!/usr/bin/env python3

import sys
import os
import subprocess
import threading
import time
import json
import signal
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float32, Int32
from rclpy.action import ActionClient

# 尝试导入CV Bridge，如果失败则给出提示
try:
    from cv_bridge import CvBridge
except ImportError:
    print("Warning: cv_bridge not available. Image viewing will be disabled.")
    CvBridge = None

# 尝试导入PyQt5，如果失败则给出提示
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QTabWidget, QTreeWidget, QTreeWidgetItem,
                                 QPushButton, QLabel, QTextEdit, QComboBox, QGroupBox,
                                 QScrollArea, QSplitter, QFrame, QMessageBox, QProgressBar,
                                 QCheckBox, QSpinBox, QDoubleSpinBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject, QSize
    from PyQt5.QtGui import QPixmap, QFont, QIcon, QPalette, QColor, QImage
    PYQT5_AVAILABLE = True
except ImportError:
    print("Error: PyQt5 not available. Please install it with:")
    print("sudo apt-get install python3-pyqt5")
    PYQT5_AVAILABLE = False

# 尝试导入OpenCV，如果失败则给出提示
try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    print("Warning: OpenCV not available. Image processing will be disabled.")
    OPENCV_AVAILABLE = False

# 尝试导入机械臂和爪子控制接口
try:
    from arm_control_interfaces.action import MoveArm
    from claw_control_interfaces.action import MoveClaw
except ImportError:
    MoveArm = None
    MoveClaw = None


class RobotUINode(Node):
    """机器人UI控制节点"""
    
    def __init__(self):
        super().__init__('robot_ui_node')
        
        # CV Bridge for image conversion
        if CvBridge is not None:
            self.cv_bridge = CvBridge()
        else:
            self.cv_bridge = None
        
        # 存储图像数据
        self.latest_images = {}
        
        # 创建图像订阅器（动态创建）
        self.image_subscriptions = {}
        
        # 控制器发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 初始化机械臂和爪子动作客户端
        self.init_action_clients()
        
        # 存储当前激活的控制器
        self.active_controller = None
        self.controller_processes = {}
        
        self.get_logger().info('Robot UI Node initialized')

    def subscribe_to_image_topic(self, topic_name: str):
        """订阅图像话题"""
        if self.cv_bridge is None:
            self.get_logger().warn('CV Bridge not available, cannot subscribe to image topics')
            return
            
        if topic_name not in self.image_subscriptions:
            self.image_subscriptions[topic_name] = self.create_subscription(
                Image,
                topic_name,
                lambda msg, topic=topic_name: self.image_callback(msg, topic),
                10
            )
            self.get_logger().info(f'Subscribed to image topic: {topic_name}')

    def image_callback(self, msg, topic_name):
        """图像话题回调"""
        if self.cv_bridge is None:
            return
            
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_images[topic_name] = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image from {topic_name}: {e}')

    def subscribe_to_topic(self, topic_name: str, msg_type: str):
        """订阅其他类型话题"""
        # 这里可以根据msg_type动态创建订阅器
        # 简化实现，主要处理常见的消息类型
        pass

    def get_node_list(self) -> List[str]:
        """获取当前运行的节点列表"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [node.strip() for node in result.stdout.split('\n') if node.strip()]
            return []
        except Exception as e:
            self.get_logger().error(f'Error getting node list: {e}')
            return []

    def get_topic_list(self) -> Dict:
        """获取话题列表和类型信息"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'], 
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = {}
                for line in result.stdout.split('\n'):
                    if line.strip() and '[' in line:
                        parts = line.strip().split('[')
                        topic_name = parts[0].strip()
                        msg_type = parts[1].replace(']', '').strip()
                        topics[topic_name] = msg_type
                return topics
            return {}
        except Exception as e:
            self.get_logger().error(f'Error getting topic list: {e}')
            return {}

    def start_controller(self, controller_type: str):
        """启动控制器"""
        # 停止当前控制器
        self.stop_current_controller()
        
        try:
            if controller_type == 'handle':
                process = subprocess.Popen(['ros2', 'run', 'handlecontroler', 'handle_control_node'])
                self.controller_processes['handle'] = process
                self.active_controller = 'handle'
                
            elif controller_type == 'keyboard':
                process = subprocess.Popen(['ros2', 'run', 'keyboardcontroler', 'keyboard_control_node'])
                self.controller_processes['keyboard'] = process
                self.active_controller = 'keyboard'
                
            # 其他控制器可以在这里添加
            
            self.get_logger().info(f'Started {controller_type} controller')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error starting {controller_type} controller: {e}')
            return False

    def stop_current_controller(self):
        """停止当前控制器"""
        if self.active_controller and self.active_controller in self.controller_processes:
            try:
                process = self.controller_processes[self.active_controller]
                
                # 首先尝试正常终止
                process.terminate()
                
                # 等待进程结束，如果5秒内没有结束则强制杀死
                try:
                    process.wait(timeout=3)
                    self.get_logger().info(f'Controller {self.active_controller} terminated gracefully')
                except subprocess.TimeoutExpired:
                    # 如果正常终止失败，强制杀死进程
                    self.get_logger().warning(f'Controller {self.active_controller} did not terminate gracefully, forcing kill')
                    process.kill()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.get_logger().error(f'Failed to kill controller {self.active_controller} process')
                
                # 额外使用pkill确保相关节点被完全终止
                controller_node_names = {
                    'handle': 'handle_control_node',
                    'keyboard': 'keyboard_control_node'
                }
                
                if self.active_controller in controller_node_names:
                    node_name = controller_node_names[self.active_controller]
                    try:
                        # 使用pkill杀死相关进程
                        subprocess.run(['pkill', '-f', node_name], timeout=3)
                        self.get_logger().info(f'Killed {node_name} processes with pkill')
                    except Exception as e:
                        self.get_logger().warning(f'pkill failed for {node_name}: {e}')
                
                del self.controller_processes[self.active_controller]
                self.get_logger().info(f'Stopped {self.active_controller} controller')
                
            except Exception as e:
                self.get_logger().error(f'Error stopping controller: {e}')
                
                # 即使出错也尝试清理
                try:
                    if self.active_controller in self.controller_processes:
                        del self.controller_processes[self.active_controller]
                except:
                    pass
                    
            finally:
                self.active_controller = None
        else:
            self.get_logger().info('No active controller to stop')

    def kill_node(self, node_name: str):
        """终止指定节点"""
        try:
            # 使用更强力的方式终止节点进程
            # 首先尝试使用pkill -f 匹配完整命令行
            result1 = subprocess.run(['pkill', '-f', node_name], timeout=5, capture_output=True)
            
            # 然后尝试使用pkill匹配进程名
            result2 = subprocess.run(['pkill', node_name], timeout=5, capture_output=True)
            
            # 最后尝试使用killall
            result3 = subprocess.run(['killall', node_name], timeout=5, capture_output=True)
            
            self.get_logger().info(f'Killed node: {node_name} (pkill -f: {result1.returncode}, pkill: {result2.returncode}, killall: {result3.returncode})')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error killing node {node_name}: {e}')
            return False


class ROSWorker(QObject):
    """ROS工作线程"""
    nodes_updated = pyqtSignal(list)
    topics_updated = pyqtSignal(dict)
    image_updated = pyqtSignal(str, np.ndarray)
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        
    def run(self):
        """工作线程主循环"""
        while self.running:
            try:
                # 更新节点列表
                nodes = self.ros_node.get_node_list()
                self.nodes_updated.emit(nodes)
                
                # 更新话题列表
                topics = self.ros_node.get_topic_list()
                self.topics_updated.emit(topics)
                
                # 发送图像更新
                for topic_name, image in self.ros_node.latest_images.items():
                    self.image_updated.emit(topic_name, image)
                
                time.sleep(1.0)  # 1秒更新一次
                
            except Exception as e:
                print(f"Worker error: {e}")
                
    def stop(self):
        self.running = False


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.image_labels = {}  # 存储图像显示标签
        self.is_updating_selectors = False  # 防止递归更新选择器
        
        # 机械臂与爪子控制参数
        self.arm_angle = 0.0
        self.arm_min_angle = -20.0
        self.arm_max_angle = 19.0
        self.arm_step = 1.0
        self.claw_state = 0  # 0: 抓取, 1: 释放
        
        # 设置窗口
        self.setWindowTitle('机器人控制界面')
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央widget和标签页
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # 创建标签页widget
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # 创建五个标签页
        self.create_node_manager_tab()
        self.create_topic_viewer_tab()
        self.create_content_viewer_tab()
        self.create_controller_tab()
        self.create_quick_tester_tab()  # 新增快捷测试器标签页
        
        # 启动ROS工作线程
        self.setup_ros_worker()
        
        # 设置深色主题样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2d2d2d;
                color: #ffffff;
            }
            QTabWidget::pane {
                border: 1px solid #5a5a5a;
                background-color: #3d3d3d;
            }
            QTabBar::tab {
                background-color: #4a4a4a;
                color: #ffffff;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #0078d4;
            }
            QTabBar::tab:hover {
                background-color: #5a5a5a;
            }
            QPushButton {
                background-color: #0078d4;
                color: #ffffff;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #005a9e;
            }
            QPushButton:disabled {
                background-color: #666666;
                color: #cccccc;
            }
            QTreeWidget {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                alternate-background-color: #3d3d3d;
            }
            QTreeWidget::item {
                padding: 4px;
                border-bottom: 1px solid #404040;
            }
            QTreeWidget::item:selected {
                background-color: #0078d4;
            }
            QTreeWidget::item:hover {
                background-color: #4a4a4a;
            }
            QLabel {
                color: #ffffff;
            }
            QTextEdit {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                border-radius: 4px;
                padding: 4px;
            }
            QComboBox {
                background-color: #4a4a4a;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                padding: 4px 8px;
                border-radius: 4px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                border: 2px solid #ffffff;
                border-top-color: transparent;
                border-left-color: transparent;
                border-right-color: transparent;
                width: 0px;
                height: 0px;
            }
            QComboBox QAbstractItemView {
                background-color: #4a4a4a;
                color: #ffffff;
                selection-background-color: #0078d4;
            }
            QGroupBox {
                color: #ffffff;
                border: 2px solid #5a5a5a;
                border-radius: 4px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QScrollArea {
                background-color: #2d2d2d;
                border: 1px solid #5a5a5a;
                border-radius: 4px;
            }
            /* 弹窗样式 */
            QMessageBox {
                background-color: #2d2d2d;
                color: #ffffff;
            }
            QMessageBox QLabel {
                color: #ffffff;
                font-size: 14px;
            }
            QMessageBox QPushButton {
                background-color: #0078d4;
                color: #ffffff;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
                min-width: 80px;
            }
            QMessageBox QPushButton:hover {
                background-color: #106ebe;
            }
            QMessageBox QPushButton:pressed {
                background-color: #005a9e;
            }
        """)

    def setup_ros_worker(self):
        """设置ROS工作线程"""
        self.worker_thread = QThread()
        self.worker = ROSWorker(self.ros_node)
        self.worker.moveToThread(self.worker_thread)
        
        # 连接信号
        self.worker.nodes_updated.connect(self.update_node_list)
        self.worker.topics_updated.connect(self.update_topic_list)
        self.worker.image_updated.connect(self.update_image_display)
        
        # 启动线程
        self.worker_thread.started.connect(self.worker.run)
        self.worker_thread.start()

    def init_action_clients(self):
        """初始化机械臂和爪子动作客户端"""
        try:
            if MoveArm is not None:
                self.arm_action_client = ActionClient(self, MoveArm, 'move_arm')
                self.get_logger().info('机械臂动作客户端已初始化')
            else:
                self.arm_action_client = None
                self.get_logger().warn('机械臂接口不可用，请检查arm_control_interfaces包')
                
            if MoveClaw is not None:
                self.claw_action_client = ActionClient(self, MoveClaw, 'move_claw')
                self.get_logger().info('爪子动作客户端已初始化')
            else:
                self.claw_action_client = None
                self.get_logger().warn('爪子接口不可用，请检查claw_control_interfaces包')
                
        except Exception as e:
            self.get_logger().error(f'初始化动作客户端失败: {e}')
            self.arm_action_client = None
            self.claw_action_client = None

    def create_node_manager_tab(self):
        """创建节点管理标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("节点管理器")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 刷新按钮
        refresh_btn = QPushButton("刷新节点")
        refresh_btn.clicked.connect(self.refresh_nodes)
        layout.addWidget(refresh_btn)
        
        # 节点树
        self.node_tree = QTreeWidget()
        self.node_tree.setHeaderLabels(["节点名称", "状态", "操作"])
        
        # 设置列宽
        self.node_tree.setColumnWidth(0, 400)  # 节点名称列加宽
        self.node_tree.setColumnWidth(1, 100)  # 状态列
        self.node_tree.setColumnWidth(2, 100)  # 操作列
        
        # 设置行高
        self.node_tree.setStyleSheet("""
            QTreeWidget::item {
                height: 40px;
                padding: 8px;
                border-bottom: 1px solid #404040;
            }
            QTreeWidget::item:selected {
                background-color: #0078d4;
                height: 40px;
            }
            QTreeWidget::item:hover {
                background-color: #4a4a4a;
                height: 40px;
            }
        """)
        
        layout.addWidget(self.node_tree)
        
        self.tab_widget.addTab(tab, "节点管理")

    def create_topic_viewer_tab(self):
        """创建话题查看器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("话题查看器")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 控制按钮
        btn_layout = QHBoxLayout()
        
        refresh_topics_btn = QPushButton("刷新话题")
        refresh_topics_btn.clicked.connect(self.refresh_topics)
        btn_layout.addWidget(refresh_topics_btn)
        
        show_graph_btn = QPushButton("显示RQT图形")
        show_graph_btn.clicked.connect(self.show_rqt_graph)
        btn_layout.addWidget(show_graph_btn)
        
        layout.addLayout(btn_layout)
        
        # 话题树
        self.topic_tree = QTreeWidget()
        self.topic_tree.setHeaderLabels(["话题名称", "消息类型"])
        
        # 设置列宽
        self.topic_tree.setColumnWidth(0, 400)  # 话题名称列加宽
        self.topic_tree.setColumnWidth(1, 300)  # 消息类型列
        
        # 设置行高
        self.topic_tree.setStyleSheet("""
            QTreeWidget::item {
                height: 35px;
                padding: 6px;
                border-bottom: 1px solid #404040;
            }
            QTreeWidget::item:selected {
                background-color: #0078d4;
                height: 35px;
            }
            QTreeWidget::item:hover {
                background-color: #4a4a4a;
                height: 35px;
            }
        """)
        
        layout.addWidget(self.topic_tree)
        
        self.tab_widget.addTab(tab, "话题查看")

    def create_content_viewer_tab(self):
        """创建内容查看器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("内容查看器")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 选择器
        selector_layout = QHBoxLayout()
        
        self.content_type_combo = QComboBox()
        self.content_type_combo.addItems(["图像话题", "其他话题", "URDF查看器", "导航查看器"])
        self.content_type_combo.currentTextChanged.connect(self.on_content_type_changed)
        selector_layout.addWidget(QLabel("内容类型:"))
        selector_layout.addWidget(self.content_type_combo)
        
        self.content_selector = QComboBox()
        self.content_selector.currentTextChanged.connect(self.on_content_selected)
        selector_layout.addWidget(QLabel("选择:"))
        selector_layout.addWidget(self.content_selector)
        
        layout.addLayout(selector_layout)
        
        # 内容显示区域
        self.content_display = QScrollArea()
        self.content_display.setWidgetResizable(True)
        self.content_display.setMinimumHeight(400)
        
        # 默认显示标签
        default_label = QLabel("请选择内容类型和项目进行显示")
        default_label.setAlignment(Qt.AlignCenter)
        default_label.setStyleSheet("font-size: 14px; color: #888;")
        self.content_display.setWidget(default_label)
        
        layout.addWidget(self.content_display)
        
        self.tab_widget.addTab(tab, "内容查看")

    def create_controller_tab(self):
        """创建控制器选择器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("控制器选择器")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 当前控制器状态
        status_group = QGroupBox("当前状态")
        status_layout = QVBoxLayout(status_group)
        
        self.controller_status_label = QLabel("无控制器运行")
        status_layout.addWidget(self.controller_status_label)
        
        layout.addWidget(status_group)
        
        # 控制器选择
        controller_group = QGroupBox("可用控制器")
        controller_layout = QVBoxLayout(controller_group)
        
        # 手柄控制器
        handle_layout = QHBoxLayout()
        self.handle_btn = QPushButton("启动手柄控制器")
        self.handle_btn.clicked.connect(lambda: self.start_controller('handle'))
        handle_layout.addWidget(self.handle_btn)
        handle_layout.addWidget(QLabel("使用手柄控制机器人"))
        controller_layout.addLayout(handle_layout)
        
        # 键盘控制器
        keyboard_layout = QHBoxLayout()
        self.keyboard_btn = QPushButton("启动键盘控制器")
        self.keyboard_btn.clicked.connect(lambda: self.start_controller('keyboard'))
        keyboard_layout.addWidget(self.keyboard_btn)
        keyboard_layout.addWidget(QLabel("使用键盘控制机器人"))
        controller_layout.addLayout(keyboard_layout)
        
        # 自动控制器（未完成）
        auto_layout = QHBoxLayout()
        self.auto_btn = QPushButton("启动自动控制器")
        self.auto_btn.setEnabled(False)
        self.auto_btn.setStyleSheet("background-color: #666; color: #ccc;")
        auto_layout.addWidget(self.auto_btn)
        auto_layout.addWidget(QLabel("自主导航 (即将推出)"))
        controller_layout.addLayout(auto_layout)
        
        # 网络控制器（未完成）
        network_layout = QHBoxLayout()
        self.network_btn = QPushButton("启动网络控制器")
        self.network_btn.setEnabled(False)
        self.network_btn.setStyleSheet("background-color: #666; color: #ccc;")
        network_layout.addWidget(self.network_btn)
        network_layout.addWidget(QLabel("远程网络控制 (即将推出)"))
        controller_layout.addLayout(network_layout)
        
        # 停止按钮
        stop_layout = QHBoxLayout()
        self.stop_controller_btn = QPushButton("停止当前控制器")
        self.stop_controller_btn.clicked.connect(self.stop_controller)
        self.stop_controller_btn.setStyleSheet("background-color: #d13438;")
        stop_layout.addWidget(self.stop_controller_btn)
        controller_layout.addLayout(stop_layout)
        
        layout.addWidget(controller_group)
        
        # 手动控制区域
        manual_group = QGroupBox("手动控制 (测试)")
        manual_layout = QVBoxLayout(manual_group)
        
        # 创建方向控制按钮
        direction_layout = QVBoxLayout()
        
        # 上排按钮
        top_row = QHBoxLayout()
        top_row.addStretch()
        forward_btn = QPushButton("↑")
        forward_btn.pressed.connect(lambda: self.send_velocity(0.2, 0.0, 0.0))
        forward_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        top_row.addWidget(forward_btn)
        top_row.addStretch()
        direction_layout.addLayout(top_row)
        
        # 中间排按钮
        middle_row = QHBoxLayout()
        left_btn = QPushButton("←")
        left_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.2, 0.0))
        left_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        middle_row.addWidget(left_btn)
        
        stop_btn = QPushButton("停止")
        stop_btn.clicked.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        stop_btn.setStyleSheet("background-color: #d13438;")
        middle_row.addWidget(stop_btn)
        
        right_btn = QPushButton("→")
        right_btn.pressed.connect(lambda: self.send_velocity(0.0, -0.2, 0.0))
        right_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        middle_row.addWidget(right_btn)
        direction_layout.addLayout(middle_row)
        
        # 下排按钮
        bottom_row = QHBoxLayout()
        bottom_row.addStretch()
        backward_btn = QPushButton("↓")
        backward_btn.pressed.connect(lambda: self.send_velocity(-0.2, 0.0, 0.0))
        backward_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        bottom_row.addWidget(backward_btn)
        bottom_row.addStretch()
        direction_layout.addLayout(bottom_row)
        
        manual_layout.addLayout(direction_layout)
        
        # 旋转控制
        rotation_layout = QHBoxLayout()
        rotate_left_btn = QPushButton("左转")
        rotate_left_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.0, 0.5))
        rotate_left_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        rotation_layout.addWidget(rotate_left_btn)
        
        rotate_right_btn = QPushButton("右转")
        rotate_right_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.0, -0.5))
        rotate_right_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        rotation_layout.addWidget(rotate_right_btn)
        
        manual_layout.addLayout(rotation_layout)
        
        layout.addWidget(manual_group)
        
        # 机械臂与爪子控制区域
        arm_claw_group = QGroupBox("机械臂与爪子控制")
        arm_claw_layout = QHBoxLayout(arm_claw_group)

        # 机械臂角度-1°
        arm_dec_btn = QPushButton("机械臂 -1°")
        arm_dec_btn.clicked.connect(lambda: self.send_arm_goal(self.arm_angle - self.arm_step))
        arm_claw_layout.addWidget(arm_dec_btn)

        # 机械臂角度+1°
        arm_inc_btn = QPushButton("机械臂 +1°")
        arm_inc_btn.clicked.connect(lambda: self.send_arm_goal(self.arm_angle + self.arm_step))
        arm_claw_layout.addWidget(arm_inc_btn)

        # 爪子抓取
        claw_grab_btn = QPushButton("爪子抓取")
        claw_grab_btn.clicked.connect(lambda: self.send_claw_goal(0))
        arm_claw_layout.addWidget(claw_grab_btn)

        # 爪子释放
        claw_release_btn = QPushButton("爪子释放")
        claw_release_btn.clicked.connect(lambda: self.send_claw_goal(1))
        arm_claw_layout.addWidget(claw_release_btn)

        manual_layout.addWidget(arm_claw_group)
        
        self.tab_widget.addTab(tab, "控制器")

    def create_quick_tester_tab(self):
        """创建快捷测试器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("快捷测试器")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 快速终端按钮
        terminal_group = QGroupBox("快速终端")
        terminal_layout = QVBoxLayout(terminal_group)
        
        quick_terminal_btn = QPushButton("打开终端 (管理员模式)")
        quick_terminal_btn.clicked.connect(self.open_quick_terminal)
        quick_terminal_btn.setStyleSheet("background-color: #28a745; font-size: 14px; padding: 10px;")
        terminal_layout.addWidget(quick_terminal_btn)
        
        terminal_info = QLabel("在管理员模式下打开xfce4终端并自动加载ROS环境")
        terminal_info.setStyleSheet("color: #888; font-size: 12px;")
        terminal_layout.addWidget(terminal_info)
        
        layout.addWidget(terminal_group)
        
        # 测试脚本区域
        scripts_group = QGroupBox("测试脚本")
        scripts_layout = QVBoxLayout(scripts_group)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # SingleTest 部分
        single_test_group = QGroupBox("单项测试")
        single_test_layout = QVBoxLayout(single_test_group)
        
        # 扫描 singleTest 目录中的 .sh 文件
        single_test_scripts = self.scan_test_scripts('/home/HwHiAiUser/ros/src/singleTest')
        for script_name, script_path in single_test_scripts:
            script_layout = QHBoxLayout()
            
            script_btn = QPushButton(script_name)
            script_btn.clicked.connect(lambda checked, path=script_path, name=script_name: 
                                     self.run_test_script(path, name))
            script_btn.setMinimumHeight(35)
            script_layout.addWidget(script_btn)
            
            # 添加脚本描述
            description = self.get_script_description(script_name, script_path)
            desc_label = QLabel(description)
            desc_label.setStyleSheet("color: #ccc; font-size: 11px;")
            desc_label.setWordWrap(True)
            script_layout.addWidget(desc_label)
            
            single_test_layout.addLayout(script_layout)
        
        scroll_layout.addWidget(single_test_group)
        
        # UnionTest 部分（如果存在）
        union_test_path = '/home/HwHiAiUser/ros/src/unionTest'
        if os.path.exists(union_test_path):
            union_test_group = QGroupBox("联合测试")
            union_test_layout = QVBoxLayout(union_test_group)
            
            union_test_scripts = self.scan_test_scripts(union_test_path)
            for script_name, script_path in union_test_scripts:
                script_layout = QHBoxLayout()
                
                script_btn = QPushButton(script_name)
                script_btn.clicked.connect(lambda checked, path=script_path, name=script_name: 
                                         self.run_test_script(path, name))
                script_btn.setMinimumHeight(35)
                script_btn.setStyleSheet("background-color: #17a2b8;")
                script_layout.addWidget(script_btn)
                
                description = self.get_script_description(script_name, script_path)
                desc_label = QLabel(description)
                desc_label.setStyleSheet("color: #ccc; font-size: 11px;")
                desc_label.setWordWrap(True)
                script_layout.addWidget(desc_label)
                
                union_test_layout.addLayout(script_layout)
            
            scroll_layout.addWidget(union_test_group)
        
        scroll_area.setWidget(scroll_widget)
        scripts_layout.addWidget(scroll_area)
        
        layout.addWidget(scripts_group)
        
        self.tab_widget.addTab(tab, "快捷测试")

    def scan_test_scripts(self, directory_path):
        """扫描指定目录中的.sh文件"""
        scripts = []
        try:
            if os.path.exists(directory_path):
                for filename in sorted(os.listdir(directory_path)):
                    if filename.endswith('.sh'):
                        script_path = os.path.join(directory_path, filename)
                        # 移除.sh后缀作为显示名称
                        script_name = filename[:-3]
                        scripts.append((script_name, script_path))
        except Exception as e:
            self.ros_node.get_logger().error(f'Error scanning {directory_path}: {e}')
        
        return scripts

    def get_script_description(self, script_name, script_path):
        """获取脚本描述"""
        descriptions = {
            'chassis_control_rclpy': '底盘控制节点，用于机器人移动',
            'fishbot_cartographer': '使用Cartographer进行SLAM建图',
            'fishbot_navigation2': '导航功能栈 (未完成)',
            'handlecontroler': '手柄/游戏手柄控制器',
            'keyboardcontroler': '键盘控制接口',
            'oradar_lidar-ms200_scan': 'LiDAR扫描（无可视化）',
            'oradar_lidar-ms200_scan_view': 'LiDAR扫描（带RViz可视化）',
            'sam_bot_description': '机器人URDF模型显示',
            'sensor_fusion_pkg': '传感器数据融合包',
            'task_manager': '全局任务调度器 (未完成)'
        }
        
        # 如果有预定义描述就使用，否则尝试从文件中读取
        if script_name in descriptions:
            return descriptions[script_name]
        
        # 尝试从脚本文件中提取注释
        try:
            with open(script_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip()
                    if line.startswith('#') and not line.startswith('#!/') and len(line) > 1:
                        comment = line[1:].strip()
                        if comment:
                            return comment
        except:
            pass
        
        return "测试脚本"

    def open_quick_terminal(self):
        """打开快速终端"""
        try:
            # 构建启动命令：打开终端，切换到root，然后source环境
            terminal_cmd = [
                'xfce4-terminal',
                '--hold',
                '--title', 'ROS快速终端 (管理员模式)',
                '--working-directory', '/home/HwHiAiUser/ros',
                '-e', 'sudo bash -c "cd /home/HwHiAiUser/ros && source install/setup.bash && echo \\"ROS环境已加载，准备测试\\" && bash"'
            ]
            
            subprocess.Popen(terminal_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.ros_node.get_logger().info('已在管理员模式下打开快速终端')
            
        except Exception as e:
            QMessageBox.warning(self, "错误", f"无法打开终端: {str(e)}")
            self.ros_node.get_logger().error(f'无法打开快速终端: {e}')

    def run_test_script(self, script_path, script_name):
        """运行测试脚本"""
        # 显示确认对话框
        reply = QMessageBox.question(self, '运行测试脚本', 
                                    f'是否要运行测试脚本:\n\n{script_name}\n\n路径: {script_path}',
                                    QMessageBox.Yes | QMessageBox.No, 
                                    QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            try:
                # 检查脚本文件是否存在
                if not os.path.exists(script_path):
                    QMessageBox.warning(self, "错误", f"脚本文件未找到:\n{script_path}")
                    return
                
                # 确保脚本有执行权限
                os.chmod(script_path, 0o755)
                
                # 构建终端命令：在root模式下运行脚本
                terminal_cmd = [
                    'xfce4-terminal',
                    '--hold',
                    '--title', f'运行中: {script_name}',
                    '--working-directory', '/home/HwHiAiUser/ros',
                    '-e', f'sudo bash -c "cd /home/HwHiAiUser/ros && {script_path}"'
                ]
                
                subprocess.Popen(terminal_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # 显示成功消息
                QMessageBox.information(self, "脚本已启动", 
                                      f"测试脚本 '{script_name}' 已在新终端中启动。\n\n请查看终端窗口获取输出信息。")
                
                self.ros_node.get_logger().info(f'已启动测试脚本: {script_name} at {script_path}')
                
            except Exception as e:
                QMessageBox.warning(self, "错误", f"无法运行脚本 '{script_name}':\n\n{str(e)}")
                self.ros_node.get_logger().error(f'无法运行测试脚本 {script_name}: {e}')

    def update_node_list(self, nodes):
        """更新节点列表"""
        self.node_tree.clear()
        
        for node_name in nodes:
            item = QTreeWidgetItem([node_name, "运行中"])
            
            # 设置行高 - 使用QSize对象
            item.setSizeHint(0, QSize(0, 40))
            
            # 添加终止按钮
            kill_btn = QPushButton("终止")
            kill_btn.clicked.connect(lambda checked, name=node_name: self.kill_node(name))
            kill_btn.setStyleSheet("background-color: #d13438; max-width: 60px; height: 30px;")
            
            self.node_tree.addTopLevelItem(item)
            self.node_tree.setItemWidget(item, 2, kill_btn)

    def update_topic_list(self, topics):
        """更新话题列表"""
        self.topic_tree.clear()
        
        for topic_name, msg_type in topics.items():
            # 只显示话题名称和消息类型
            item = QTreeWidgetItem([topic_name, msg_type])
            self.topic_tree.addTopLevelItem(item)
            
        # 更新内容选择器
        self.update_content_selectors(topics)

    def update_content_selectors(self, topics):
        """更新内容选择器选项"""
        if self.is_updating_selectors:
            return
            
        self.is_updating_selectors = True
        
        try:
            current_type = self.content_type_combo.currentText()
            current_selection = self.content_selector.currentText()
            
            # 暂时断开信号连接
            try:
                self.content_selector.currentTextChanged.disconnect()
            except TypeError:
                pass
            
            self.content_selector.clear()
            
            if current_type == "图像话题":
                image_topics = [topic for topic, msg_type in topics.items() 
                               if 'Image' in msg_type]
                self.content_selector.addItems(image_topics)
                
                # 订阅新的图像话题
                for topic in image_topics:
                    self.ros_node.subscribe_to_image_topic(topic)
                    
            elif current_type == "其他话题":
                other_topics = [topic for topic, msg_type in topics.items() 
                               if 'Image' not in msg_type]
                self.content_selector.addItems(other_topics)
                
                # 尝试恢复之前的选择
                if current_selection in other_topics:
                    index = self.content_selector.findText(current_selection)
                    if index >= 0:
                        self.content_selector.setCurrentIndex(index)
            elif current_type == "URDF查看器":
                self.content_selector.addItems(["机器人URDF"])
            elif current_type == "导航查看器":
                self.content_selector.addItems(["代价地图", "路径", "机器人位姿"])
            
            # 重新连接信号
            self.content_selector.currentTextChanged.connect(self.on_content_selected)
            
        finally:
            self.is_updating_selectors = False

    def update_image_display(self, topic_name, image):
        """更新图像显示"""
        if not OPENCV_AVAILABLE:
            return
            
        current_selection = self.content_selector.currentText()
        
        if (self.content_type_combo.currentText() == "图像话题" and 
            current_selection == topic_name):
            
            # 转换图像格式并显示
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            
            # 调整图像大小以适应显示
            max_width = 640
            max_height = 480
            
            scale = min(max_width/width, max_height/height)
            if scale < 1:
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # 转换为RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 创建QPixmap
            q_image = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], 
                           rgb_image.strides[0], QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            
            # 更新显示
            if topic_name not in self.image_labels:
                self.image_labels[topic_name] = QLabel()
                self.image_labels[topic_name].setAlignment(Qt.AlignCenter)
                self.image_labels[topic_name].setScaledContents(False)
            
            self.image_labels[topic_name].setPixmap(pixmap)
            self.content_display.setWidget(self.image_labels[topic_name])

    def refresh_nodes(self):
        """刷新节点列表"""
        nodes = self.ros_node.get_node_list()
        self.update_node_list(nodes)

    def refresh_topics(self):
        """刷新话题列表"""
        topics = self.ros_node.get_topic_list()
        self.update_topic_list(topics)

    def show_rqt_graph(self):
        """显示RQT图形界面"""
        try:
            subprocess.Popen(['rqt_graph'])
        except Exception as e:
            QMessageBox.warning(self, "错误", f"无法启动rqt_graph: {e}")


    def on_content_type_changed(self, content_type):
        """内容类型改变时的处理"""
        if self.is_updating_selectors:
            return
            
        # 暂时断开信号连接
        try:
            self.content_selector.currentTextChanged.disconnect()
        except TypeError:
            pass
        
        # 清空选择器
        self.content_selector.clear()
        
        if content_type == "URDF查看器":
            self.content_selector.addItems(["机器人URDF"])
        elif content_type == "导航查看器":
            self.content_selector.addItems(["代价地图", "路径", "机器人位姿"])
        
        # 重新连接信号
        self.content_selector.currentTextChanged.connect(self.on_content_selected)
        
        # 如果有默认选项，触发选择事件
        if self.content_selector.count() > 0:
            current_text = self.content_selector.currentText()
            if current_text:
                self.on_content_selected(current_text)

    def on_content_selected(self, selection):
        """内容选择改变时的处理"""
        if self.is_updating_selectors or not selection:
            return
            
        content_type = self.content_type_combo.currentText()
        
        if content_type == "URDF查看器" and selection == "机器人URDF":
            self.show_urdf_viewer()
        elif content_type == "导航查看器":
            self.show_navigation_viewer(selection)
        elif content_type == "其他话题" and selection:
            self.show_topic_with_terminal(selection)

    def show_topic_with_terminal(self, topic_name):
        """使用xfce4-terminal显示话题内容"""
        try:
            # 直接使用xfce4-terminal启动ros2 topic echo
            subprocess.Popen([
                'xfce4-terminal', 
                '--hold', 
                '--title', f'ROS2话题: {topic_name}',
                '-e', f'ros2 topic echo {topic_name}'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 在UI中显示确认信息
            info_label = QLabel(f"已为话题打开终端: {topic_name}\n\n正在显示实时数据:\nros2 topic echo {topic_name}\n\n终端窗口将单独显示。")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
            self.ros_node.get_logger().info(f'已为话题打开终端: {topic_name}')
            
        except Exception as e:
            # 如果启动失败，显示错误信息和备用方案
            error_label = QLabel(f"无法为话题打开终端: {topic_name}\n\n错误: {str(e)}\n\n请手动在终端中运行:\nros2 topic echo {topic_name}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setWordWrap(True)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)
            
            self.ros_node.get_logger().error(f'无法为话题打开终端 {topic_name}: {e}')

    def show_urdf_viewer(self):
        """显示URDF查看器"""
        try:
            # 获取sam_bot_description包的路径
            pkg_share_result = subprocess.run([
                'ros2', 'pkg', 'prefix', 'sam_bot_description'
            ], capture_output=True, text=True)
            
            if pkg_share_result.returncode == 0:
                pkg_path = pkg_share_result.stdout.strip()
                rviz_config_path = os.path.join(pkg_path, 'share', 'sam_bot_description', 'rviz', 'urdf_config.rviz')
                
                # 检查配置文件是否存在
                if os.path.exists(rviz_config_path):
                    # 启动RViz并加载URDF配置
                    subprocess.Popen(['rviz2', '-d', rviz_config_path])
                else:
                    # 如果配置文件不存在，直接启动RViz
                    subprocess.Popen(['rviz2'])
            else:
                # 如果找不到包，直接启动RViz
                subprocess.Popen(['rviz2'])
            
            info_label = QLabel("RViz URDF查看器已启动\n如果sam_bot_description正在运行，机器人模型应该可见\n如果不可见，请添加'RobotModel'显示")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
        except Exception as e:
            error_label = QLabel(f"启动URDF查看器时出错: {e}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def show_navigation_viewer(self, viewer_type):
        """显示导航查看器"""
        try:
            # 根据类型启动不同的查看器
            if viewer_type == "代价地图":
                # 启动RViz显示导航相关信息
                nav2_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
                if os.path.exists(nav2_config):
                    subprocess.Popen(['rviz2', '-d', nav2_config])
                else:
                    subprocess.Popen(['rviz2'])
            elif viewer_type == "机器人位姿":
                subprocess.Popen(['rqt_plot', '/odom/pose/pose/position/x:y'])
            elif viewer_type == "路径":
                subprocess.Popen(['rqt_plot', '/plan'])
                
            info_label = QLabel(f"{viewer_type}查看器已启动\n请查看新窗口以查看数据")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
        except Exception as e:
            error_label = QLabel(f"启动{viewer_type}查看器时出错: {e}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def start_controller(self, controller_type):
        """启动控制器"""
        success = self.ros_node.start_controller(controller_type)
        if success:
            controller_names = {'handle': '手柄', 'keyboard': '键盘'}
            display_name = controller_names.get(controller_type, controller_type)
            self.controller_status_label.setText(f"活动: {display_name}控制器")
            self.controller_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            QMessageBox.warning(self, "错误", f"无法启动{controller_type}控制器")

    def send_velocity(self, linear_x, linear_y, angular_z):
        """发送速度控制命令"""
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_z
            
            self.ros_node.cmd_vel_publisher.publish(twist)
            
        except Exception as e:
            self.ros_node.get_logger().error(f'Error sending velocity command: {e}')

    def send_arm_goal(self, angle):
        """发送机械臂动作目标"""
        if self.ros_node.arm_action_client is None or MoveArm is None:
            QMessageBox.warning(self, "错误", "机械臂动作接口不可用\n请确保已正确安装arm_control_interfaces包")
            self.ros_node.get_logger().error('机械臂动作客户端不可用')
            return
            
        # 限制角度范围
        angle = max(self.arm_min_angle, min(self.arm_max_angle, angle))
        self.arm_angle = angle
        
        try:
            # 等待服务器可用
            if not self.ros_node.arm_action_client.wait_for_server(timeout_sec=2.0):
                QMessageBox.warning(self, "错误", "机械臂动作服务器不可用\n请确保机械臂控制节点正在运行")
                self.ros_node.get_logger().error('机械臂动作服务器不可用')
                return
                
            # 创建并发送目标
            goal_msg = MoveArm.Goal()
            goal_msg.pose = angle
            
            # 发送目标
            future = self.ros_node.arm_action_client.send_goal_async(goal_msg)
            
            # 显示成功消息
            QMessageBox.information(self, "机械臂", f"已发送机械臂角度命令: {angle}°")
            self.ros_node.get_logger().info(f'发送机械臂角度: {angle}°')
            
        except Exception as e:
            QMessageBox.warning(self, "错误", f"发送机械臂命令失败:\n{str(e)}")
            self.ros_node.get_logger().error(f'发送机械臂命令失败: {e}')

    def send_claw_goal(self, command):
        """发送爪子动作目标"""
        if self.ros_node.claw_action_client is None or MoveClaw is None:
            QMessageBox.warning(self, "错误", "爪子动作接口不可用\n请确保已正确安装claw_control_interfaces包")
            self.ros_node.get_logger().error('爪子动作客户端不可用')
            return
            
        self.claw_state = command
        
        try:
            # 等待服务器可用
            if not self.ros_node.claw_action_client.wait_for_server(timeout_sec=2.0):
                QMessageBox.warning(self, "错误", "爪子动作服务器不可用\n请确保爪子控制节点正在运行")
                self.ros_node.get_logger().error('爪子动作服务器不可用')
                return
                
            # 创建并发送目标
            goal_msg = MoveClaw.Goal()
            goal_msg.command = command
            
            # 发送目标
            future = self.ros_node.claw_action_client.send_goal_async(goal_msg)
            
            # 显示成功消息
            state_str = "抓取" if command == 0 else "释放"
            QMessageBox.information(self, "爪子", f"已发送爪子命令: {state_str}")
            self.ros_node.get_logger().info(f'发送爪子命令: {state_str}')
            
        except Exception as e:
            QMessageBox.warning(self, "错误", f"发送爪子命令失败:\n{str(e)}")
            self.ros_node.get_logger().error(f'发送爪子命令失败: {e}')

    def stop_controller(self):
        """停止当前控制器"""
        if self.ros_node.active_controller:
            controller_names = {'handle': '手柄', 'keyboard': '键盘'}
            display_name = controller_names.get(self.ros_node.active_controller, self.ros_node.active_controller)
            
            # 显示确认对话框
            reply = QMessageBox.question(self, '停止控制器', 
                                        f'确定要停止{display_name}控制器吗？',
                                        QMessageBox.Yes | QMessageBox.No, 
                                        QMessageBox.Yes)
            
            if reply == QMessageBox.Yes:
                self.ros_node.stop_current_controller()
                
                # 等待一小段时间让进程完全终止
                QTimer.singleShot(1000, lambda: self.refresh_nodes())  # 1秒后刷新节点列表
                
                self.controller_status_label.setText("无控制器运行")
                self.controller_status_label.setStyleSheet("color: #888;")
                
                # 显示成功消息
                QMessageBox.information(self, "成功", "控制器已成功停止")
        else:
            QMessageBox.information(self, "信息", "当前没有运行的控制器")

    def kill_node(self, node_name):
        """终止节点"""
        reply = QMessageBox.question(self, '确认终止节点', 
                                    f'确定要终止节点: {node_name}？',
                                    QMessageBox.Yes | QMessageBox.No, 
                                    QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            success = self.ros_node.kill_node(node_name)
            if success:
                QMessageBox.information(self, "成功", f"节点 {node_name} 已成功终止")
                self.refresh_nodes()
            else:
                QMessageBox.warning(self, "错误", f"无法终止节点 {node_name}")

    def show_rqt_graph(self):
        """显示RQT图形界面"""
        try:
            subprocess.Popen(['rqt_graph'])
        except Exception as e:
            QMessageBox.warning(self, "错误", f"无法启动rqt_graph: {e}")

    def closeEvent(self, event):
        """窗口关闭事件"""        
        # 停止工作线程
        if hasattr(self, 'worker'):
            self.worker.stop()
        if hasattr(self, 'worker_thread'):
            self.worker_thread.quit()
            self.worker_thread.wait()
        
        # 停止所有控制器
        self.ros_node.stop_current_controller()
        
        event.accept()


def main(args=None):
    # 检查PyQt5是否可用
    if not PYQT5_AVAILABLE:
        print("Cannot start UI without PyQt5. Exiting.")
        return
    
    # 初始化ROS
    rclpy.init(args=args)
    
    # 创建ROS节点
    ros_node = RobotUINode()
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Control UI")
    
    # 设置应用图标（如果有的话）
    # app.setWindowIcon(QIcon('path/to/icon.png'))
    
    # 创建主窗口
    main_window = MainWindow(ros_node)
    main_window.show()
    
    # 创建ROS执行器并在单独线程中运行
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    def ros_spin():
        executor.spin()
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 处理关闭信号
    def signal_handler(sig, frame):
        main_window.close()
        app.quit()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 运行Qt应用
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        if hasattr(main_window, 'worker'):
            main_window.worker.stop()
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
