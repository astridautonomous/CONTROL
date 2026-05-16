#!/usr/bin/env python3
"""
diag_.launch.py
==================
Termianlden çalıştırmak için:
    # layer_flags.yaml'daki default ayarları kullanmak için
    ros2 launch diag_pkg diag_.launch.py

    # Terminalden kontrol etmek için
    ros2 launch diag_pkg diag_.launch.py perception:=true slam:=false
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


LAYER_NODES = {
    "perception": [
        "diag_string_node",
        "diag_image_node",
    ],
    "slam": [
        "diag_odom_node",
        "diag_laserscan_node",
        "diag_pointc2_node",
        "diag_float32ma_node",
        "diag_grid_node",
        "diag_posestamped_node",
        "diag_markerarray_node",
        "diag_int32ma_node",
        "diag_int32s_node",
        "diag_int8s_node",
    ],
    "navigation": [
        "diag_path_node",
        "diag_posearray_node",
    ],
    "control": [
        "diag_int32_node",
        "diag_int8_node",
    ],
    "emergency_brake": [
        "emergency_brake_node",
    ],
    "loop_detector": [
        "loop_detector_node",
    ],
    "odometry": [
        "odom_node",
    ],
    "camera_object_detection": [
        "velodyne_recovery_launcher_node",
    ],
}

LAYERS = ["perception", "slam", "navigation", "control", "emergency_brake", "loop_detector", "odometry", "camera_object_detection",]


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("diag_pkg")

    # Node parametreleri — sadece ros__parameters içeren temiz dosya
    params_path = os.path.join(pkg_share, "config", "diag_params.yaml")

    # Katman anahtarları — ayrı dosya (ROS2 parser bunu görmez)
    flags_path = os.path.join(pkg_share, "config", "layer_flags.yaml")

    with open(flags_path, "r") as f:
        flags_data = yaml.safe_load(f)
    yaml_flags = flags_data.get("layer_flags", {})

    # Terminal argümanı "auto" → yaml'dan oku
    flags = {}
    sources = {}
    for layer in LAYERS:
        terminal_val = LaunchConfiguration(layer).perform(context)
        if terminal_val == "auto":
            flags[layer]   = yaml_flags.get(f"enable_{layer}", False)
            sources[layer] = "yaml    "
        else:
            flags[layer]   = terminal_val.lower() == "true"
            sources[layer] = "terminal"

    print("\n" + "="*52)
    print("   diag_pkg — Katman Durumu")
    print("="*52)
    for layer in LAYERS:
        state = "✓ AKTİF " if flags[layer] else "✗ KAPALI"
        print(f"   {state}  [{sources[layer]}]  →  {layer.upper()}")
    print("="*52 + "\n")

    active_layers = [l for l in LAYERS if flags[l]]

    dashboard = Node(
        package="diag_pkg",
        executable="dashboard_node",
        name="dashboard_node",
        output="screen",
    )

    if not active_layers:
        print("   [UYARI] Hiçbir katman aktif değil!\n")
        return []

    nodes_to_start = []
    added = set()

    for layer in active_layers:
        for node_name in LAYER_NODES[layer]:
            if node_name in added:
                continue
            nodes_to_start.append(
                Node(
                    package="diag_pkg",
                    executable=node_name,
                    name=node_name,
                    parameters=[params_path],
                    output="screen",
                )
            )
            added.add(node_name)

    #print(f"   Başlatılan node sayısı: {len(nodes_to_start)}")
    #for n in nodes_to_start:
    #    print(f"     + {n.name}")
    #print()

    return [dashboard] + nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("perception", default_value="auto",
            description="Perception katmanı  [true/false/auto]"),
        DeclareLaunchArgument("slam",       default_value="auto",
            description="SLAM katmanı        [true/false/auto]"),
        DeclareLaunchArgument("navigation", default_value="auto",
            description="Navigation katmanı  [true/false/auto]"),
        DeclareLaunchArgument("control",    default_value="auto",
            description="Control katmanı     [true/false/auto]"),
        DeclareLaunchArgument("emergency_brake", default_value="auto",
            description="Emergency Brake katmanı  [true/false/auto]"),
        DeclareLaunchArgument("loop_detector",    default_value="auto",
            description="Loop Detection katmanı     [true/false/auto]"),
        DeclareLaunchArgument("odometry", default_value="auto",
            description="Odometry katmanı  [true/false/auto]"),
        DeclareLaunchArgument("camera_object_detection", default_value="auto",
            description="Kamera ile Nesne Tespiti katmanı  [true/false/auto]"),

        OpaqueFunction(function=launch_setup),
    ])