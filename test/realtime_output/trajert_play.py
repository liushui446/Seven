"""
trajert_play.py — 编队轨迹回放器
从文件夹读取 result_*.json 文件，按编队和帧号排列，支持逐帧播放/暂停/倍速/跳帧
"""

import matplotlib.pyplot as plt
import numpy as np
import json
import os
import glob
import sys
import threading
from datetime import datetime

# 中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ========================= 常量配置 =========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_FOLDER = SCRIPT_DIR               # JSON数据文件夹（脚本同目录）
FILE_PATTERN = "result_*.json"        # 文件匹配模式
CENTER_FORMATION_ID = "1"            # 锁定为画面中心的编队ID
VIEW_RANGE_METERS = 200.0            # 视窗范围（米）
DEG_TO_M = 111319.9                  # 经纬度转米系数
DEFAULT_REFRESH_MS = 50              # 默认刷新间隔(ms)，对应20fps

FORMATION_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                    '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']

# ========================= 全局状态 =========================
formation_data = {}          # {formation_id: {frame_id: {nodes...}}}
all_frame_ids = []           # 全局帧号列表（跨编队去重排序）
current_frame_idx = 0        # 当前播放的帧索引
is_playing = False           # 播放/暂停
playback_speed = 1.0         # 播放倍速
data_loaded = False          # 数据是否加载完成
viz_lock = threading.Lock()
track_history = {}           # {fid: {nid: [(east, north), ...]}}


# ========================= 数据加载 =========================
def load_data_from_folder(folder_path=".", pattern="result_*.json"):
    """
    从文件夹加载所有 result_*.json 文件，合并为 formation_data
    返回: (formation_data, sorted_global_frame_ids)
    """
    global formation_data, all_frame_ids, data_loaded

    file_path = os.path.join(folder_path, pattern)
    files = sorted(glob.glob(file_path))

    if not files:
        print(f"[错误] 未找到匹配 '{file_path}' 的文件！")
        print(f"       请将 result_*.json 文件放在脚本同目录下。")
        return False

    print(f"[加载] 找到 {len(files)} 个JSON文件：")
    for f in files:
        print(f"  {os.path.basename(f)}")

    formation_data.clear()
    all_frames_set = set()

    for filepath in files:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[警告] 跳过 {os.path.basename(filepath)}: {e}")
            continue

        # 解析 formations 结构
        if isinstance(data, dict) and "formations" in data:
            fms = data["formations"]
            if isinstance(fms, dict):
                for fid_str, frames in fms.items():
                    if not isinstance(frames, list):
                        continue
                    if fid_str not in formation_data:
                        formation_data[fid_str] = {}
                    for frame in frames:
                        if not isinstance(frame, dict):
                            continue
                        fid_val = frame.get("frame_id", 0)
                        fid_val = int(fid_val) if isinstance(fid_val, (int, float, str)) else 0
                        formation_data[fid_str][fid_val] = frame
                        all_frames_set.add(fid_val)

    if not formation_data:
        print("[错误] 未解析到任何编队数据！请检查JSON文件格式。")
        return False

    all_frame_ids = sorted(all_frames_set)

    # 打印统计
    print(f"\n[加载] 完成！编队数: {len(formation_data)}, 总帧数: {len(all_frame_ids)}")
    for fid_str in sorted(formation_data.keys(), key=lambda x: int(x) if x.isdigit() else 0):
        frames = formation_data[fid_str]
        fids = sorted(frames.keys())
        num_uavs = 0
        if fids and frames[fids[0]].get("nodes"):
            num_uavs = len(frames[fids[0]]["nodes"])
        print(f"  编队{fid_str}: {len(frames)}帧 (#{min(fids)}-#{max(fids)}), {num_uavs}个节点")

    data_loaded = True
    return True


# ========================= 坐标转换 =========================
def geo_to_relative_meters(lon, lat, center_lon, center_lat):
    """GPS经纬度 → 以中心点为原点的东北坐标系（米）"""
    lon_scale = DEG_TO_M * np.cos(np.radians(center_lat))
    east = (lon - center_lon) * lon_scale
    north = (lat - center_lat) * DEG_TO_M
    return east, north


# ========================= 可视化 =========================
def launch_viewer(refresh_ms=DEFAULT_REFRESH_MS):
    """启动编队轨迹回放窗口（带逐帧播放控制）"""
    global formation_data, all_frame_ids, current_frame_idx, is_playing, playback_speed

    if not data_loaded:
        print("[错误] 请先加载数据！")
        return

    plt.ion()
    fig = plt.figure(figsize=(14, 9))

    # ——— 主绘图区 ———
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Formation Trajectory Player')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
    ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)

    # ——— 底部控制栏 ———
    plt.subplots_adjust(bottom=0.12)

    # 帧滑块
    from matplotlib.widgets import Slider, Button
    ax_slider = plt.axes([0.12, 0.04, 0.55, 0.03])
    frame_slider = Slider(ax_slider, 'Frame', 0, len(all_frame_ids) - 1,
                          valinit=0, valfmt='%d', valstep=1)

    # 播放/暂停按钮
    ax_btn_play = plt.axes([0.12, 0.005, 0.06, 0.03])
    btn_play = Button(ax_btn_play, 'Play/Pause')

    # 倍速按钮
    btn_speeds = {}
    speed_labels = [('0.5x', 0.19), ('1x', 0.26), ('2x', 0.33), ('5x', 0.40)]
    for label, xpos in speed_labels:
        ax_b = plt.axes([xpos, 0.005, 0.05, 0.03])
        btn = Button(ax_b, label)
        btn_speeds[label] = btn

    # 跳帧输入提示
    ax_info = plt.axes([0.50, 0.005, 0.48, 0.03])
    ax_info.axis('off')
    info_text = ax_info.text(0, 0.5, '', transform=ax_info.transAxes,
                             fontsize=8, va='center', fontfamily='monospace')

    # ===== 播放状态机 =====
    timer = [None]

    def auto_play():
        """自动播放定时回调"""
        global current_frame_idx, is_playing, playback_speed
        if is_playing and data_loaded:
            if current_frame_idx < len(all_frame_ids) - 1:
                current_frame_idx += 1
                frame_slider.set_val(current_frame_idx)
        if plt.fignum_exists(fig.number):
            timer[0] = fig.canvas.new_timer(
                interval=int(refresh_ms / playback_speed))
            timer[0].add_callback(auto_play)
            timer[0].start()

    # ===== 按钮回调 =====
    def on_play_clicked(event):
        global is_playing
        is_playing = not is_playing
        btn_play.label.set_text('[PAUSE]' if is_playing else '[PLAY]')

    btn_play.on_clicked(on_play_clicked)

    def make_speed_cb(speed_str):
        def cb(event):
            global playback_speed
            playback_speed = float(speed_str.replace('x', ''))
            print(f"播放速度: {playback_speed:.1f}x")
        return cb

    for label, btn in btn_speeds.items():
        btn.on_clicked(make_speed_cb(label))

    # ===== 滑块回调 =====
    def on_slider_changed(val):
        global current_frame_idx
        current_frame_idx = int(val)
        draw_frame(current_frame_idx)

    frame_slider.on_changed(on_slider_changed)

    # ===== 键盘快捷键 =====
    def on_key_press(event):
        global is_playing, current_frame_idx, playback_speed
        if event.key == ' ':
            # 空格 = 播放/暂停
            is_playing = not is_playing
            btn_play.label.set_text('[PAUSE]' if is_playing else '[PLAY]')
        elif event.key == 'right':
            current_frame_idx = min(current_frame_idx + 1, len(all_frame_ids) - 1)
            frame_slider.set_val(current_frame_idx)
        elif event.key == 'left':
            current_frame_idx = max(current_frame_idx - 1, 0)
            frame_slider.set_val(current_frame_idx)
        elif event.key == 'up':
            current_frame_idx = min(current_frame_idx + 10, len(all_frame_ids) - 1)
            frame_slider.set_val(current_frame_idx)
        elif event.key == 'down':
            current_frame_idx = max(current_frame_idx - 10, 0)
            frame_slider.set_val(current_frame_idx)
        elif event.key in '12345':
            speeds = {'1': 0.5, '2': 1.0, '3': 2.0, '4': 5.0, '5': 10.0}
            playback_speed = speeds[event.key]
            print(f"播放速度: {playback_speed:.1f}x")

    fig.canvas.mpl_connect('key_press_event', on_key_press)

    # ===== 核心绘制函数 =====
    def draw_frame(frame_idx):
        """绘制指定帧：所有编队节点 + 编队ID标签 + 节点ID标签 + 轨迹拖尾"""
        with viz_lock:
            if not data_loaded or frame_idx >= len(all_frame_ids):
                return

            frame_id = all_frame_ids[frame_idx]

            # 清空画布
            ax.clear()
            ax.set_xlabel('East (m)')
            ax.set_ylabel('North (m)')
            ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.grid(True)
            ax.set_aspect('equal')

            # ——— 获取中心编队原点 ———
            center_lon, center_lat = 0.0, 0.0
            center_ok = False
            if CENTER_FORMATION_ID in formation_data:
                cf = formation_data[CENTER_FORMATION_ID]
                if frame_id in cf:
                    cnodes = cf[frame_id].get("nodes", [])
                    if cnodes:
                        center_lon = cnodes[0].get("lon", 0.0)
                        center_lat = cnodes[0].get("lat", 0.0)
                        center_ok = True

            if not center_ok:
                ax.set_title(f'Frame#{frame_id} — Waiting for Center Formation {CENTER_FORMATION_ID}...')
                return

            # ——— 绘制所有编队 ———
            fids = sorted(formation_data.keys(),
                          key=lambda x: int(x) if x.isdigit() else 0)

            for fi, fid_str in enumerate(fids):
                frames = formation_data[fid_str]
                if frame_id not in frames:
                    continue

                frame_data = frames[frame_id]
                nodes = frame_data.get("nodes", [])
                fm_color = FORMATION_COLORS[fi % len(FORMATION_COLORS)]

                # 初始化轨迹缓存
                if fid_str not in track_history:
                    track_history[fid_str] = {}

                east_list, north_list = [], []
                for node in nodes:
                    nid = node.get("node_id", 0)
                    lon = node.get("lon", 0.0)
                    lat = node.get("lat", 0.0)

                    # 坐标转换
                    east, north = geo_to_relative_meters(
                        lon, lat, center_lon, center_lat)
                    east_list.append(east)
                    north_list.append(north)

                    # 轨迹拖尾缓存
                    if nid not in track_history[fid_str]:
                        track_history[fid_str][nid] = []
                    track_history[fid_str][nid].append((east, north))

                    # 绘制节点
                    ax.scatter(east, north, c=fm_color, s=90, zorder=3,
                               edgecolors='black', linewidths=0.5)
                    # 节点ID
                    ax.annotate(f"{nid}", (east, north),
                                xytext=(4, 4), textcoords="offset points",
                                fontsize=7, zorder=4)

                # 编队内部连线
                for i in range(len(east_list)):
                    for j in range(i + 1, len(east_list)):
                        ax.plot([east_list[i], east_list[j]],
                                [north_list[i], north_list[j]],
                                color=fm_color, linewidth=0.4, alpha=0.35,
                                zorder=2)

                # 编队ID标签
                if east_list:
                    cx, cy = np.mean(east_list), np.mean(north_list)
                    label = f"F{fid_str}*" if fid_str == CENTER_FORMATION_ID else f"F{fid_str}"
                    ax.annotate(label, (cx, cy), fontsize=9,
                                fontweight='bold', color=fm_color,
                                ha='center', va='center',
                                bbox=dict(boxstyle='round,pad=0.15',
                                          facecolor='white', alpha=0.85),
                                zorder=5)

                # 轨迹拖尾（最近80帧）
                for nid, track in track_history[fid_str].items():
                    if len(track) > 1:
                        trail = track[-80:]
                        es = [p[0] for p in trail]
                        ns = [p[1] for p in trail]
                        ax.plot(es, ns, '-', color=fm_color,
                                linewidth=0.5, alpha=0.3, zorder=1)

            # 标题和状态栏
            total_frames = len(all_frame_ids)
            play_status = "[>]" if is_playing else "[||]"
            ax.set_title(f'{play_status} Frame#{frame_id}/{total_frames} | '
                         f'Speed:{playback_speed:.1f}x | '
                         f'Formations:{len([f for f in fids if frame_id in formation_data.get(f,{})])}')

            # 底部信息
            info_text.set_text(
                f'SPACE=Play/Pause | Left/Right=+-1f | Up/Down=+-10f | '
                f'1=0.5x 2=1x 3=2x 4=5x 5=10x | '
                f'Now: Frame#{frame_id} ({frame_idx+1}/{total_frames})')

        fig.canvas.draw_idle()

    # ===== 初始绘制 =====
    draw_frame(0)

    # ===== 启动自动播放定时器 =====
    timer[0] = fig.canvas.new_timer(interval=refresh_ms)
    timer[0].add_callback(auto_play)
    timer[0].start()

    plt.show(block=True)
    print("[回放] 窗口已关闭。")

    return fig


# ========================= 入口 =========================
def main():
    global data_loaded

    print("=" * 50)
    print("  编队轨迹回放器 — Trajectory Player")
    print("=" * 50)

    # 1. 加载数据
    if not load_data_from_folder(DATA_FOLDER, FILE_PATTERN):
        print("\n按回车退出...")
        input()
        return

    print(f"\n[提示] 数据加载完毕，共 {len(all_frame_ids)} 帧。")
    print(f"[提示] 中心编队: 编队{CENTER_FORMATION_ID}")
    print(f"[提示] 视窗范围: ±{VIEW_RANGE_METERS}米")
    print(f"\n操作说明:")
    print(f"  空格键 = 播放/暂停")
    print(f"  ← →   = 前一帧/后一帧")
    print(f"  ↑ ↓   = 前10帧/后10帧")
    print(f"  1-5   = 切换倍速 (0.5x/1x/2x/5x/10x)")
    print(f"  底部按钮 = 鼠标操作")
    print(f"  关闭窗口 = 退出\n")

    # 2. 启动回放窗口（阻塞，直到窗口关闭）
    launch_viewer(refresh_ms=DEFAULT_REFRESH_MS)

    print("程序退出。")


if __name__ == "__main__":
    main()
