from matplotlib import patches
import socket
import subprocess
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from datetime import datetime

import time
import json
import sys
import numpy as np
import threading
import queue
import os

# 中文字体设置
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 全局变量
is_paused = False
current_frame = 0
total_frames = 0
is_listening = True
result_queue = queue.Queue()
# 分离两个Socket（纯同步模式）
hSocket_cmd = None   # 发送命令用：/tmp/ClientToServerPipe
hSocket_data = None  # 监听结果用：/tmp/ServerToClientPipe

# ========================= 可视化相关全局变量 =========================
# 编队数据缓存：{ formation_id: { frame_id: { "nodes": [...], "formation_type": N } } }
formation_data = {}
# 存储初始输入命令数据（编队配置信息）
cmd_cache = {
    "formations": [],
    "cmd_info": {}
}
# 实时动画相关
ani = None
fig = None
viz_lock = threading.Lock()
# 编队颜色映射
FORMATION_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                    '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']

# ========================= 中心化可视化配置 =========================
CENTER_FORMATION_ID = "1"       # 锁定为画面中心的编队ID（可修改）
VIEW_RANGE_METERS = 150.0       # 可视化窗口范围：±150米
DEG_TO_M = 111319.9             # 经纬度转米系数

# ========================= 【新增】帧播放控制全局变量 =========================
playback_speed = 1.0            # 播放速度（1.0=正常速度，2.0=2倍速，0.5=0.5倍速）
is_playing = False              # 是否正在播放
playback_thread = None          # 播放线程
current_playback_frame = 0      # 当前播放到的帧号
max_cached_frame = 0            # 已缓存的最大帧号

# ========================= 【新增】实时模式全局变量 =========================
formation_states = {}           # {formation_id: FormationState} 各编队实时运动状态
is_realtime_running = False     # 实时循环是否运行中
realtime_frame_count = 0        # 实时帧计数器
realtime_dt = 0.1               # 仿真步长（秒）
realtime_lock = threading.Lock()
pending_commands = {}           # {formation_id: dict} 待发送的控制命令（switch/turn/add/remove）
realtime_trajectory = {}        # 轨迹累积缓存：{fid: [frame_data, ...]}，定期落盘
realtime_save_interval = 400    # 每N帧自动保存一次轨迹JSON
realtime_output_dir = None      # 实时轨迹输出目录（None=脚本目录/realtime_output）
realtime_save_counter = 1       # 文件名递增序号

# 脚本所在目录（解决工作目录与脚本目录不一致的问题）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 命令文件映射
CMD_FILE_MAP = {
    "1": "cmd_json_1.json",
    "2": "cmd_json_2.json",
    "3": "cmd_json_3.json",
    "4": "cmd_json_4.json",
    "5": "cmd_json_1_for.json",
    "6": "cmd_json_test.json",
    "7": "cmd_json_power_test.json",
    "8": "cmd_json_freq_test.json",
    "9": "cmd_json_2_format.json",
    "10": "cmd_json_addnode.json",
    "11": "cmd_formation_start.json",
    "12": "cmd_formation_cal1.json",
    "13": "cmd_formation_cal2.json",
    "14": "cmd_formation_pause.json",
    "15": "cmd_formation_end.json"
}

# Unix Socket 路径（与服务端对应）
SOCKET_CMD_PATH = "/tmp/ClientToServerPipe"   # 发送命令
SOCKET_DATA_PATH = "/tmp/ServerToClientPipe"  # 监听结果（批次模式）

# 编队类型名称映射
FORMATION_TYPE_NAMES = {
    1: "楔形", 2: "纵队", 3: "横队",
    4: "菱形", 5: "三角形", 6: "圆形"
}

# ========================= 工具函数 =========================
def geo_to_relative_meters(lon: float, lat: float, center_lon: float, center_lat: float) -> tuple:
    """GPS经纬度 → 以中心编队为原点的东北坐标系（米）"""
    lon_scale = DEG_TO_M * np.cos(np.radians(center_lat))
    east = (lon - center_lon) * lon_scale
    north = (lat - center_lat) * DEG_TO_M
    return east, north


def _get_formation_offsets(formation_type, num_nodes, interval=10.0):
    """根据编队类型计算各节点相对于编队中心的理想偏移"""
    offsets = []
    if formation_type == 2:  # 纵队
        for i in range(num_nodes):
            offsets.append((0.0, -i * interval))
    elif formation_type == 3:  # 横队
        half = (num_nodes - 1) / 2.0
        for i in range(num_nodes):
            offsets.append(((i - half) * interval, 0.0))
    elif formation_type == 4:  # 菱形
        offsets.append((0.0, 0.0))
        row = 1
        while len(offsets) < num_nodes:
            for j in range(-row, row + 1):
                if len(offsets) < num_nodes:
                    offsets.append((j * interval, -row * interval))
            row += 1
    elif formation_type == 1 or formation_type == 5:  # 楔形/三角形
        row = 0
        while len(offsets) < num_nodes:
            for j in range(-row, row + 1):
                if len(offsets) < num_nodes:
                    offsets.append((j * interval, -row * interval))
            row += 1
    else:  # 默认：纵队
        for i in range(num_nodes):
            offsets.append((0.0, -i * interval))
    return offsets[:num_nodes]


# ========================= 【新增】编队实时运动状态类 =========================
class FormationState:
    """维护单个编队的实时运动状态，负责外推和更新"""

    def __init__(self, fid, config):
        self.formation_id = fid
        self.config = config
        self.main_speed = float(config.get("init_speed", 2.0))
        self.main_heading = float(config.get("init_heading", 0.0))
        self.main_lon = float(config.get("pos_lon", 120.0))
        self.main_lat = float(config.get("pos_lat", 27.0))
        self.heading_rate = float(config.get("heading_rate", 0.0))
        self.interval = float(config.get("interval", 10.0))
        self.collision_radius = float(config.get("collision_radius", 4.0))
        self.current_formation = config.get("formation_type", 1)
        self.node_num = config.get("num_uavs", 5)
        # 从节点状态 {node_id: {"lon","lat","speed","heading","rel_x","rel_y"}}
        self.slaves = {}
        self._init_slaves()

    def _init_slaves(self):
        """从编队配置初始化从节点位置（理想队形）"""
        offsets = _get_formation_offsets(self.current_formation, self.node_num,
                                         self.interval)
        deg_per_m = 1.0 / DEG_TO_M
        main_hdg_rad = np.radians(90 - self.main_heading)
        for i in range(1, self.node_num):
            dx, dy = offsets[i]
            # 旋转偏移到大地坐标系
            lon = self.main_lon + (dx * np.cos(main_hdg_rad) - dy * np.sin(main_hdg_rad)) * deg_per_m
            lat = self.main_lat - (dx * np.sin(main_hdg_rad) + dy * np.cos(main_hdg_rad)) * deg_per_m
            self.slaves[i] = {
                "lon": round(lon, 8), "lat": round(lat, 8),
                "speed": self.main_speed, "heading": self.main_heading,
                "rel_x": 0.0, "rel_y": 0.0
            }

    def advance(self):
        """外推所有节点位置（航位推算），dt = realtime_dt"""
        global realtime_dt
        dt = realtime_dt
        hdg_rad = np.radians(self.main_heading)
        deg_per_m_lon = 1.0 / (DEG_TO_M * np.cos(np.radians(self.main_lat)))
        deg_per_m_lat = 1.0 / DEG_TO_M

        # 主船移动
        self.main_lon += self.main_speed * np.sin(hdg_rad) * dt * deg_per_m_lon
        self.main_lat += self.main_speed * np.cos(hdg_rad) * dt * deg_per_m_lat
        self.main_heading = (self.main_heading + self.heading_rate * dt) % 360.0

        # 从节点移动
        for sid, s in self.slaves.items():
            sh_rad = np.radians(s["heading"])
            cos_slat = np.cos(np.radians(s["lat"]))
            s["lon"] += s["speed"] * np.sin(sh_rad) * dt / (DEG_TO_M * cos_slat)
            s["lat"] += s["speed"] * np.cos(sh_rad) * dt * deg_per_m_lat

    def update_from_response(self, form_entry):
        """从服务端响应更新 speed/heading 等"""
        self.main_speed = form_entry.get("main_speed", self.main_speed)
        self.main_heading = form_entry.get("main_heading", self.main_heading)
        self.main_lon = form_entry.get("main_lon", self.main_lon)
        self.main_lat = form_entry.get("main_lat", self.main_lat)
        self.current_formation = form_entry.get("current_formation", self.current_formation)
        self.node_num = form_entry.get("node_num", self.node_num)

        new_slaves = {}
        for n in form_entry.get("nodes", []):
            nid = n["node_id"]
            if nid == 0:
                continue
            new_slaves[nid] = {
                "lon": n.get("lon", 0.0),
                "lat": n.get("lat", 0.0),
                "speed": n.get("speed", 0.0),
                "heading": n.get("heading", 0.0),
                "rel_x": n.get("rel_x", 0.0),
                "rel_y": n.get("rel_y", 0.0),
            }
        self.slaves = new_slaves

    def to_realtime_entry(self, commands=None):
        """构建单个编队的 realtime JSON"""
        entry = {
            "formation_id": self.formation_id,
            "main_speed": round(self.main_speed, 3),
            "main_heading": round(self.main_heading, 3),
            "main_lon": round(self.main_lon, 8),
            "main_lat": round(self.main_lat, 8),
            "slaves": [
                {"node_id": sid, "lon": round(s["lon"], 8), "lat": round(s["lat"], 8)}
                for sid, s in sorted(self.slaves.items())
            ],
            "isSwitch": False, "isTurn": False,
            "isAdd": False, "isRemove": False,
            "formation_type": 0, "heading_rate": 0.0,
            "custom_id": 0, "add_node": [], "remove_num": 0,
        }
        if commands:
            entry.update(commands)
        return entry


def _recv_until_delimiter(sock, buf_size=4096*10000):
    """从 socket 读取直到遇到 ###END### 分隔符，返回 decoded JSON 字符串"""
    data = b""
    sock.settimeout(5.0)
    while True:
        try:
            chunk = sock.recv(buf_size)
        except socket.timeout:
            break
        if not chunk:
            break
        data += chunk
        if b"\n###END###\n" in data:
            break
    return data.decode("utf-8", errors="ignore")


def send_socket_command_raw(cmd: dict):
    """发送命令并返回解析后的 JSON 响应（None 表示失败）"""
    global hSocket_cmd
    if hSocket_cmd is None:
        print("[实时] Socket未连接")
        return None
    try:
        cmd_str = json.dumps(cmd, ensure_ascii=False) + "\n###END###\n"
        hSocket_cmd.sendall(cmd_str.encode("utf-8"))
        result_str = _recv_until_delimiter(hSocket_cmd)
        if not result_str:
            return None
        delimiter = "\n###END###\n"
        pos = result_str.find(delimiter)
        json_str = result_str[:pos] if pos != -1 else result_str
        return json.loads(json_str)
    except Exception as e:
        print(f"[实时] 收发异常: {e}")
        return None


def cache_realtime_response(result):
    """将实时响应缓存到 formation_data（可视化）和 realtime_trajectory（落盘）"""
    global realtime_frame_count, max_cached_frame, realtime_trajectory
    if not result or result.get("status") != "success":
        return
    with viz_lock:
        for form_entry in result.get("formations", []):
            fid = str(form_entry.get("formation_id", "?"))
            frame_data = {
                "frame_id": realtime_frame_count,
                "formation_type": form_entry.get("current_formation", 1),
                "nodes": form_entry.get("nodes", [])
            }
            # 可视化缓存
            if fid not in formation_data:
                formation_data[fid] = {}
            formation_data[fid][realtime_frame_count] = frame_data
            # 轨迹落盘缓存
            if fid not in realtime_trajectory:
                realtime_trajectory[fid] = []
            realtime_trajectory[fid].append(frame_data)
        if realtime_frame_count > max_cached_frame:
            max_cached_frame = realtime_frame_count


def flush_realtime_trajectory():
    """将累积的轨迹数据写入 JSON 文件（格式与批次模式 result_*.json 一致）"""
    global realtime_trajectory, realtime_output_dir, realtime_save_counter
    if not realtime_trajectory:
        return
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    out_dir = realtime_output_dir or os.path.join(SCRIPT_DIR, "realtime_output")
    os.makedirs(out_dir, exist_ok=True)

    # 批次风格：单文件含所有编队 {"formations": {"1": [...], "2": [...]}}
    filename = os.path.join(out_dir, f"result_{timestamp}_+ {realtime_save_counter}.json")
    formations_obj = {}
    total_frames = 0
    for fid_str in sorted(realtime_trajectory.keys(), key=lambda x: int(x) if x.isdigit() else 0):
        frames = realtime_trajectory[fid_str]
        if not frames:
            continue
        formations_obj[fid_str] = frames
        total_frames += len(frames)

    output = {"formations": formations_obj}
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    frame_ids = [str(f["frame_id"]) for f in next(iter(formations_obj.values()))]
    print(f"[轨迹] {len(formations_obj)}编队 {total_frames}帧 "
          f"(帧#{min(frame_ids)}-#{max(frame_ids)}) → {filename}")

    realtime_trajectory.clear()
    realtime_save_counter += 1


def realtime_loop():
    """实时模式主循环线程"""
    global is_realtime_running, realtime_frame_count, formation_states, pending_commands

    print(f"\n[实时] ===== 实时仿真开始 (dt={realtime_dt}s) =====")
    print("[实时] 命令: rstop 停止 | rswitch <fid> <type> | rturn <fid> <rate>")
    print("[实时]       radd <fid> | rremove <fid> <n> | rspeed <fid> <v>")

    while is_realtime_running and is_listening:
        loop_start = time.time()

        # --- 构建命令 ---
        cmd = {"cmd": 3, "sim_type": 2, "realtime": True, "formations": []}
        with realtime_lock:
            for fid in sorted(formation_states.keys()):
                state = formation_states[fid]
                # 取出该编队的待执行控制命令
                commands = pending_commands.pop(fid, None)
                state.advance()
                cmd["formations"].append(state.to_realtime_entry(commands))

        # --- 发送 & 接收 ---
        result = send_socket_command_raw(cmd)
        if result is None:
            print("[实时] 通信失败，重试...")
            time.sleep(0.1)
            continue

        # --- 更新状态 ---
        if result.get("status") == "success":
            with realtime_lock:
                for form_entry in result.get("formations", []):
                    fid = form_entry.get("formation_id")
                    if fid is not None and fid in formation_states:
                        formation_states[fid].update_from_response(form_entry)

            realtime_frame_count += 1
            cache_realtime_response(result)

            # 每 realtime_save_interval 帧自动落盘
            if realtime_frame_count % realtime_save_interval == 0:
                flush_realtime_trajectory()

            # 每 50 帧打印一次状态
            if realtime_frame_count % 50 == 0:
                cross = result.get("cross_formation_avoidance", False)
                n_fms = len(formation_states)
                total_nodes = sum(len(s.slaves) for s in formation_states.values())
                print(f"[实时] 帧#{realtime_frame_count} | "
                      f"{n_fms}编队 {total_nodes}从节点 | "
                      f"跨编队避碰:{cross}")
        else:
            print(f"[实时] 帧#{realtime_frame_count} 失败: {result.get('message', '?')}")

        # --- 帧率控制 ---
        elapsed = time.time() - loop_start
        sleep_t = max(0, realtime_dt - elapsed)
        if sleep_t > 0:
            time.sleep(sleep_t)

    print(f"[实时] ===== 实时仿真结束 (共{realtime_frame_count}帧) =====\n")


def queue_command(fid, cmd_dict):
    """将控制命令加入待发送队列"""
    with realtime_lock:
        pending_commands[fid] = cmd_dict
    print(f"[实时] 编队{fid} 命令已排队: {list(cmd_dict.keys())}")


def visualize_formation_static(formation_id=None):
    """静态轨迹总览 — 弹出独立窗口，绘制所有UAV完整轨迹"""
    global formation_data
    if not formation_data:
        print("没有可用的编队数据！请先发送编队命令并等待监听数据。")
        return

    if formation_id is None:
        formation_id = list(formation_data.keys())[0]
        print(f"未指定编队ID，使用第一个：编队 {formation_id}")

    fid_str = str(formation_id)
    if fid_str not in formation_data:
        print(f"编队 {formation_id} 不存在！可用ID：{list(formation_data.keys())}")
        return

    frames = formation_data[fid_str]
    if not frames:
        print(f"编队 {formation_id} 无数据！")
        return

    sorted_frames = sorted(frames.items(), key=lambda x: x[0])
    frame_ids = [f[0] for f in sorted_frames]

    # 提取所有节点的轨迹
    node_tracks = {}  # {node_id: [(lon, lat), ...]}
    for fid, fdata in sorted_frames:
        for node in fdata.get("nodes", []):
            nid = node.get("node_id", 0)
            if nid not in node_tracks:
                node_tracks[nid] = []
            node_tracks[nid].append((node.get("lon", 0), node.get("lat", 0)))

    # 获取编队配置
    fm_cfg = _get_formation_config(fid_str)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f'编队 {formation_id} — 编队变换仿真结果'
                 f'（类型: {FORMATION_TYPE_NAMES.get(fm_cfg.get("formation_type", "?"), "?")}, '
                 f'{fm_cfg.get("num_uavs", "?")}架UAV）', fontsize=14)

    # ——— 左图：2D轨迹总览 ———
    ax1 = plt.subplot(2, 3, (1, 3))
    color_idx = 0
    for nid in sorted(node_tracks.keys()):
        track = node_tracks[nid]
        lons = [p[0] for p in track]
        lats = [p[1] for p in track]
        color = FORMATION_COLORS[color_idx % len(FORMATION_COLORS)]
        ax1.plot(lons, lats, '-', color=color, linewidth=1.5, alpha=0.8,
                 label=f'UAV {nid}')
        # 起点和终点标记
        if len(track) > 1:
            ax1.scatter(lons[0], lats[0], c=color, marker='o', s=40)
            ax1.scatter(lons[-1], lats[-1], c=color, marker='s', s=40)
        elif len(track) == 1:
            ax1.scatter(lons[0], lats[0], c=color, marker='o', s=40)
        color_idx += 1

    # 标记编队中心初始位置
    if fm_cfg:
        ax1.scatter(fm_cfg.get("pos_lon", 0), fm_cfg.get("pos_lat", 0),
                    c='red', marker='*', s=200, zorder=5, label='编队中心')

    ax1.set_xlabel('经度(°)'); ax1.set_ylabel('纬度(°)')
    ax1.set_title(f'UAV轨迹总览（{len(frame_ids)}帧）')
    ax1.legend(loc='upper right', fontsize=7)
    ax1.grid(True)
    ax1.set_aspect('equal')

    # ——— 右上：最后一帧编队快照 ———
    ax2 = plt.subplot(2, 3, 4)
    last_frame = sorted_frames[-1][1]
    _draw_formation_snapshot(ax2, last_frame, fm_cfg,
                             title=f'最终帧 #{sorted_frames[-1][0]} 编队快照')

    # ——— 右中：编队误差柱状图 ———
    ax3 = plt.subplot(2, 3, 5)
    errors = []
    nids_list = []
    for node in last_frame.get("nodes", []):
        nids_list.append(node.get("node_id", 0))
        errors.append(node.get("formation_error", 0))
    colors = [FORMATION_COLORS[i % len(FORMATION_COLORS)]
              for i in range(len(nids_list))]
    ax3.bar(nids_list, errors, color=colors)
    ax3.set_xlabel('节点ID'); ax3.set_ylabel('编队误差(m)')
    ax3.set_title('最终帧各节点编队保持误差')
    ax3.grid(True, axis='y')

    # ——— 右下：帧数统计 ———
    ax4 = plt.subplot(2, 3, 6)
    ax4.axis('off')
    info_lines = [
        f"编队ID: {formation_id}",
        f"编队类型: {FORMATION_TYPE_NAMES.get(fm_cfg.get('formation_type', '?'), '?')}",
        f"UAV数量: {fm_cfg.get('num_uavs', '?')}",
        f"总帧数: {len(frame_ids)}",
        f"帧范围: #{min(frame_ids)} - #{max(frame_ids)}",
        f"初始航向: {fm_cfg.get('init_heading', '?')}°",
        f"初始速度: {fm_cfg.get('init_speed', '?')} m/s",
        f"节点间距: {fm_cfg.get('interval', '?')} m",
        "",
        f"最终帧编队误差:",
    ]
    for node in last_frame.get("nodes", []):
        info_lines.append(
            f"  UAV {node.get('node_id', 0)}: "
            f"误差={node.get('formation_error', 0):.2f}m, "
            f"航向={node.get('heading', 0):.1f}°")
    info_text = "\n".join(info_lines)
    ax4.text(0.05, 0.95, info_text, transform=ax4.transAxes,
             fontsize=9, verticalalignment='top', fontfamily='monospace')

    plt.tight_layout()
    plt.show()


def _draw_formation_snapshot(ax, frame_data, fm_cfg, title="编队快照"):
    """在指定axes上绘制单个帧的编队快照"""
    nodes = frame_data.get("nodes", [])
    if not nodes:
        return

    lons = [n.get("lon", 0) for n in nodes]
    lats = [n.get("lat", 0) for n in nodes]
    nids = [n.get("node_id", 0) for n in nodes]

    # 绘制理想编队位置（target）
    ft = fm_cfg.get("formation_type",
                    frame_data.get("formation_type", 2))
    interval = fm_cfg.get("interval", 10.0)
    ideal_offsets = _get_formation_offsets(ft, len(nodes), interval)
    # 计算理想位置需基于编队中心
    center_lon = fm_cfg.get("pos_lon", np.mean(lons))
    center_lat = fm_cfg.get("pos_lat", np.mean(lats))
    # 近似转换：1°≈111km, 用interval的经纬度近似
    deg_per_m = 1.0 / 111000.0

    ideal_lons = []
    ideal_lats = []
    for dx, dy in ideal_offsets:
        ideal_lons.append(center_lon + dx * deg_per_m)
        ideal_lats.append(center_lat + dy * deg_per_m)

    # 绘制理想编队（空心标记+虚线连接）
    ax.scatter(ideal_lons, ideal_lats, c='gray', marker='o', s=60,
               facecolors='none', linewidths=1, label='理想位置', zorder=2)
    for i in range(len(ideal_lons)):
        for j in range(i + 1, len(ideal_lons)):
            ax.plot([ideal_lons[i], ideal_lons[j]],
                    [ideal_lats[i], ideal_lats[j]],
                    'gray', linewidth=0.5, linestyle='--', alpha=0.4, zorder=1)

    # 绘制实际位置（实心标记+实线连接）
    for i, (lon, lat, nid) in enumerate(zip(lons, lats, nids)):
        color = FORMATION_COLORS[nid % len(FORMATION_COLORS)]
        ax.scatter(lon, lat, c=color, marker='o', s=80, zorder=4)
        ax.annotate(f"{nid}", (lon, lat), textcoords="offset points",
                    xytext=(4, 4), fontsize=7, zorder=5)
        # 误差连线（从理想到实际）
        if i < len(ideal_lons):
            ax.plot([ideal_lons[i], lon], [ideal_lats[i], lat],
                    'r-', linewidth=0.5, alpha=0.5, zorder=3)

    # 实际节点之间的连接线
    for i in range(len(lons)):
        for j in range(i + 1, len(lons)):
            ax.plot([lons[i], lons[j]], [lats[i], lats[j]],
                    'b-', linewidth=0.3, alpha=0.3, zorder=2)

    ax.set_xlabel('经度(°)'); ax.set_ylabel('纬度(°)')
    ax.set_title(title)
    ax.grid(True)
    ax.set_aspect('equal')


def _get_formation_config(fid_str):
    """从cmd_cache获取特定编队的配置"""
    for fm in cmd_cache.get("formations", []):
        if str(fm.get("formation_id", "")) == fid_str:
            return fm
    return {}


def visualize_formation_config():
    """可视化编队初始配置 — 显示各编队的预设位置和理想编队形状"""
    formations = cmd_cache.get("formations", [])
    if not formations:
        print("没有缓存的编队配置数据！请先发送编队命令（如: 11）。")
        return

    n_fms = len(formations)
    cols = min(3, n_fms)
    rows = (n_fms + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows))
    fig.suptitle('编队初始配置 — 理想编队布局', fontsize=14)

    if n_fms == 1:
        axes = [axes]
    elif rows == 1:
        axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
    else:
        axes = axes.flatten()

    for idx, fm in enumerate(formations):
        ax = axes[idx]
        fid = fm.get("formation_id", "?")
        ftype = fm.get("formation_type", "?")
        num_uavs = fm.get("num_uavs", 0)
        interval = fm.get("interval", 10.0)
        center_lon = fm.get("pos_lon", 0)
        center_lat = fm.get("pos_lat", 0)
        heading = fm.get("init_heading", 0)

        offsets = _get_formation_offsets(ftype, num_uavs, interval)
        deg_per_m = 1.0 / 111000.0

        lons, lats, labels = [], [], []
        for i, (dx, dy) in enumerate(offsets):
            lon = center_lon + dx * deg_per_m
            lat = center_lat + dy * deg_per_m
            lons.append(lon)
            lats.append(lat)
            labels.append(str(i))

        # 绘制节点
        for i in range(len(lons)):
            color = FORMATION_COLORS[i % len(FORMATION_COLORS)]
            ax.scatter(lons[i], lats[i], c=color, marker='o', s=100, zorder=3)
            ax.annotate(labels[i], (lons[i], lats[i]),
                        textcoords="offset points", xytext=(5, 5), fontsize=8)
        # 节点间连线
        for i in range(len(lons)):
            for j in range(i + 1, len(lons)):
                ax.plot([lons[i], lons[j]], [lats[i], lats[j]],
                        'gray', linewidth=0.5, alpha=0.5, zorder=1)

        # 编队中心
        ax.scatter(center_lon, center_lat, c='red', marker='*', s=200,
                   zorder=5, label='编队中心')

        # 航向箭头
        heading_rad = np.radians(90 - heading)  # 从正北顺时针
        arrow_len = interval * num_uavs * 0.3 * deg_per_m
        ax.arrow(center_lon, center_lat,
                 arrow_len * np.cos(heading_rad),
                 arrow_len * np.sin(heading_rad),
                 head_width=interval * 0.05 * deg_per_m,
                 head_length=interval * 0.1 * deg_per_m,
                 fc='red', ec='red', alpha=0.7, zorder=4)

        ax.set_xlabel('经度(°)'); ax.set_ylabel('纬度(°)')
        ax.set_title(f'编队{fid}: {FORMATION_TYPE_NAMES.get(ftype, "?")} '
                     f'({num_uavs}UAV, 航向{heading}°)')
        ax.grid(True)
        ax.set_aspect('equal')

    # 隐藏多余的子图
    for idx in range(n_fms, len(axes)):
        axes[idx].set_visible(False)

    # 打印配置信息
    print("\n===== 编队初始配置 =====")
    cmd_info = cmd_cache.get("cmd_info", {})
    print(f"命令类型: cmd={cmd_info.get('cmd', '?')}, sim_type={cmd_info.get('sim_type', '?')}")
    for fm in formations:
        print(f"\n编队 {fm.get('formation_id', '?')}:")
        print(f"  类型: {FORMATION_TYPE_NAMES.get(fm.get('formation_type', '?'), '?')}")
        print(f"  UAV数: {fm.get('num_uavs', '?')}")
        print(f"  中心: ({fm.get('pos_lon', 0):.6f}°, {fm.get('pos_lat', 0):.6f}°)")
        print(f"  航向: {fm.get('init_heading', '?')}°  速度: {fm.get('init_speed', '?')} m/s")
        print(f"  间距: {fm.get('interval', '?')}m  碰撞半径: {fm.get('collision_radius', '?')}m")
    print("========================\n")

    plt.tight_layout()
    plt.show()

# ========================= 【核心修改】带逐帧播放的中心化可视化 =========================
def launch_formation_viewer(refresh_ms=200):
    """
    中心化实时监控 + 逐帧播放功能：
    1. 锁定指定编队为画面中心(0,0)
    2. 坐标单位：米（东/北方向）
    3. 支持1、2、3、4、5帧逐帧播放
    4. 支持播放/暂停/调速/跳帧
    """
    global ani, fig, formation_data, current_playback_frame, max_cached_frame

    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 固定米单位坐标轴
    ax.set_xlabel('东 (m)')
    ax.set_ylabel('北 (m)')
    ax.set_title(f'锁定：编队{CENTER_FORMATION_ID} | 帧#0 | 已缓存：0帧')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
    ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)

    # 轨迹拖尾缓存
    track_history = {}

    def refresh():
        """定时刷新函数：根据当前播放帧号绘制画面"""
        with viz_lock:
            if not formation_data:
                return

            # 清空画布，重置坐标轴
            ax.clear()
            ax.set_xlabel('东 (m)')
            ax.set_ylabel('北 (m)')
            ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.grid(True)
            ax.set_aspect('equal')

            # ============== 第一步：获取中心编队坐标（原点）==============
            center_lon, center_lat = 0.0, 0.0
            center_exists = False
            
            # 检查当前播放帧是否存在
            if CENTER_FORMATION_ID in formation_data:
                center_frames = formation_data[CENTER_FORMATION_ID]
                if current_playback_frame in center_frames:
                    center_frame = center_frames[current_playback_frame]
                    center_nodes = center_frame.get("nodes", [])
                    if center_nodes:
                        # 以中心编队的0号节点为坐标原点
                        leader = center_nodes[0]
                        center_lon = leader.get("lon", 0.0)
                        center_lat = leader.get("lat", 0.0)
                        center_exists = True

            if not center_exists:
                ax.set_title(f"等待中心编队{CENTER_FORMATION_ID}数据... | 已缓存：{max_cached_frame}帧")
                return

            # ============== 第二步：绘制所有编队（当前播放帧）==============
            fids = sorted(formation_data.keys(), key=lambda x: int(x) if x.isdigit() else 0)

            for fi, fid_str in enumerate(fids):
                frames = formation_data[fid_str]
                if not frames or current_playback_frame not in frames:
                    continue

                # 获取当前播放帧的数据
                current_frame_data = frames[current_playback_frame]
                nodes = current_frame_data.get("nodes", [])
                fm_color = FORMATION_COLORS[fi % len(FORMATION_COLORS)]

                if fid_str not in track_history:
                    track_history[fid_str] = {}

                east_list, north_list = [], []
                # 遍历节点，转换坐标并绘制
                for node in nodes:
                    nid = node.get("node_id", 0)
                    lon = node.get("lon", 0.0)
                    lat = node.get("lat", 0.0)

                    # 经纬度转相对米坐标
                    east, north = geo_to_relative_meters(lon, lat, center_lon, center_lat)
                    east_list.append(east)
                    north_list.append(north)

                    # 保存轨迹（用于拖尾）
                    if nid not in track_history[fid_str]:
                        track_history[fid_str][nid] = []
                    track_history[fid_str][nid].append((east, north))

                    # 绘制节点
                    ax.scatter(east, north, c=fm_color, s=80, zorder=3, edgecolors='black')
                    ax.annotate(f"{nid}", (east, north), xytext=(4, 4), fontsize=7, zorder=4)

                # 绘制编队内部连线（显示队形）
                for i in range(len(east_list)):
                    for j in range(i + 1, len(east_list)):
                        ax.plot([east_list[i], east_list[j]], [north_list[i], north_list[j]],
                                color=fm_color, linewidth=0.4, alpha=0.35)

                # 标注编队ID（中心编队特殊标记）
                if len(east_list) > 0:
                    cx = np.mean(east_list)
                    cy = np.mean(north_list)
                    label = f"编队{fid_str}(中心)" if fid_str == CENTER_FORMATION_ID else f"编队{fid_str}"
                    ax.annotate(label, (cx, cy), fontsize=9, fontweight='bold', color=fm_color,
                                ha='center', va='center',
                                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.85))

                # 绘制轨迹拖尾（最近50帧）
                for nid, track in track_history[fid_str].items():
                    if len(track) > 1:
                        trail = track[-50:]
                        es = [p[0] for p in trail]
                        ns = [p[1] for p in trail]
                        ax.plot(es, ns, '-', color=fm_color, linewidth=0.6, alpha=0.3)

            # 更新标题（显示当前帧号和已缓存帧数）
            play_status = "▶ 播放中" if is_playing else "⏸ 已暂停"
            ax.set_title(f'{play_status} | 锁定：编队{CENTER_FORMATION_ID} | 帧#{current_playback_frame} | 已缓存：{max_cached_frame}帧 | 速度：{playback_speed:.1f}x')
            fig.canvas.draw_idle()

    # 启动定时器：定时刷新画面
    timer = fig.canvas.new_timer(interval=refresh_ms)
    timer.add_callback(refresh)
    timer.start()
    plt.show(block=False)
    print(f"[可视化] 中心化监控启动 | 中心：编队{CENTER_FORMATION_ID} | 视窗：±{VIEW_RANGE_METERS}米")
    print("[播放控制] 输入 'play' 开始播放 | 'pause' 暂停 | 'speed x' 调整速度 | 'goto x' 跳转到指定帧")
    return timer

# ========================= 【新增】帧播放控制线程 =========================
def playback_worker():
    """独立的播放线程：控制帧的播放速度"""
    global current_playback_frame, max_cached_frame, is_playing
    
    while is_listening:
        if is_playing and current_playback_frame < max_cached_frame:
            # 播放下一帧
            current_playback_frame += 1
            # 根据播放速度调整等待时间
            time.sleep(0.05 / playback_speed)  # 基础帧率20fps
        else:
            # 暂停或已播放完所有缓存帧
            time.sleep(0.01)

# ========================= 【新增】更新最大缓存帧号 =========================
def update_max_cached_frame():
    """更新已缓存的最大帧号"""
    global max_cached_frame
    with viz_lock:
        for fid_str in formation_data:
            frames = formation_data[fid_str]
            if frames:
                current_max = max(frames.keys())
                if current_max > max_cached_frame:
                    max_cached_frame = current_max

# ========================= Socket通信相关函数（完全保留你的代码） =========================
def update_formation_cache(cmd: dict):
    """从编队命令中提取并缓存编队配置数据"""
    global cmd_cache
    with viz_lock:
        if "formations" in cmd:
            cmd_cache["formations"] = cmd["formations"]
            print(f"[可视化] 已缓存 {len(cmd['formations'])} 个编队的初始配置")
            for fm in cmd["formations"]:
                fid = fm.get("formation_id", "?")
                ftype = fm.get("formation_type", "?")
                num = fm.get("num_uavs", "?")
                print(f"  编队{fid}: {FORMATION_TYPE_NAMES.get(ftype, '?')} "
                      f"({num}UAV, 航向{fm.get('init_heading', '?')}°)")

        cmd_cache["cmd_info"] = {
            "sim_type": cmd.get("sim_type", "未知"),
            "cmd": cmd.get("cmd", "未知"),
        }

        # 【新增】如果是 sim_start (sim_type=1)，自动创建 FormationState
        if cmd.get("sim_type") == 1 and "formations" in cmd:
            global formation_states
            for fm in cmd["formations"]:
                fid = fm.get("formation_id")
                if fid is not None and fid not in formation_states:
                    formation_states[fid] = FormationState(fid, fm)
                    print(f"[实时] 编队{fid} 状态已初始化 ({fm.get('num_uavs', '?')}节点)")


def send_socket_command(cmd: dict) -> bool:
    """同步发送命令到Unix Socket + 读取响应"""
    global hSocket_cmd
    if hSocket_cmd is None:
        print("[发送] Socket未连接")
        return False
    try:
        cmd_str = json.dumps(cmd, ensure_ascii=False) + "\n###END###\n"
        hSocket_cmd.sendall(cmd_str.encode("utf-8"))
        print(f"[发送] 发送字节数：{len(cmd_str)}")

        result_str = _recv_until_delimiter(hSocket_cmd)
        if not result_str:
            print("[发送] 读取响应失败")
            return False

        delimiter = "\n###END###\n"
        pos = result_str.find(delimiter)
        json_str = result_str[:pos] if pos != -1 else result_str
        try:
            result_json = json.loads(json_str)
        except json.JSONDecodeError as e:
            print(f"[错误] JSON解析失败: {e}")
            return False

        with open("message.json", "w", encoding="utf-8") as f:
            json.dump(result_json, f, ensure_ascii=False, indent=2)
        print("结果已保存到 message.json")
        return True
    except (socket.error, BrokenPipeError, ConnectionRefusedError) as e:
        print(f"[发送] Socket异常: {e}，尝试重连...")
        if reconnect_cmd_socket():
            return send_socket_command(cmd)
        return False
    except Exception as e:
        print(f"[发送] 未知异常: {e}")
        return False

def listen_socket_continually():
    """监听 data socket：解析编队批次数据并缓存到 formation_data 供可视化使用"""
    global is_listening, hSocket_data, formation_data, max_cached_frame
    if hSocket_data is None:
        print("[监听] data socket未连接，退出")
        return

    cnt_result = 1
    first_data_logged = False
    hSocket_data.settimeout(1.0)
    print(f"[监听] 启动监听（Unix Socket，编队可视化模式）")

    while is_listening:
        try:
            result_str = _recv_until_delimiter(hSocket_data)
            if not result_str:
                time.sleep(0.01)
                continue

            delimiter = "\n###END###\n"
            delimiter_pos = result_str.find(delimiter)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M")
            filename = f"result_{timestamp}_+ {cnt_result}.json"
            cnt_result += 1

            json_str = result_str[:delimiter_pos] if delimiter_pos != -1 else result_str

            result_json = None
            try:
                result_json = json.loads(json_str)
                with open(filename, "w", encoding="utf-8") as f:
                    json.dump(result_json, f, ensure_ascii=False, indent=2)
            except json.JSONDecodeError as e:
                print(f"[监听] JSON解析失败: {e}")
                continue
            except IOError as e:
                print(f"[监听] 文件写入失败: {e}")

            if result_json is not None and not first_data_logged:
                first_data_logged = True
                if isinstance(result_json, dict):
                    keys = list(result_json.keys())
                    print(f"[监听] 首次数据：字典，字段={keys}")
                    if "formations" in result_json:
                        fms = result_json["formations"]
                        if isinstance(fms, dict):
                            for fid, frames in fms.items():
                                if isinstance(frames, list) and len(frames) > 0:
                                    f0 = frames[0]
                                    print(f"[监听]  编队{fid}: {len(frames)}帧, 首帧字段={list(f0.keys())}")
                                    nodes = f0.get("nodes", [])
                                    if nodes:
                                        print(f"[监听]  编队{fid} 首帧 {len(nodes)}个节点, 节点字段={list(nodes[0].keys())}")

            if result_json is not None:
                try:
                    with viz_lock:
                        if isinstance(result_json, dict) and "formations" in result_json:
                            fms = result_json["formations"]
                            if isinstance(fms, dict):
                                for fid_str, frames in fms.items():
                                    if not isinstance(frames, list):
                                        continue
                                    if fid_str not in formation_data:
                                        formation_data[fid_str] = {}
                                    cached_count = 0
                                    for frame in frames:
                                        if not isinstance(frame, dict):
                                            continue
                                        frame_id = frame.get("frame_id", 0)
                                        frame_id = int(frame_id) if isinstance(frame_id, (int, float, str)) else 0
                                        formation_data[fid_str][frame_id] = frame
                                        cached_count += 1
                                    if cached_count > 0:
                                        all_fids = sorted(formation_data[fid_str].keys())
                                        print(f"[监听] ✓ 编队{fid_str}: +{cached_count}帧 "
                                              f"(累计{len(formation_data[fid_str])}帧, 范围#{min(all_fids)}-#{max(all_fids)})")
                                        if max(all_fids) > max_cached_frame:
                                            max_cached_frame = max(all_fids)
                except Exception as cache_err:
                    print(f"[监听] 缓存异常: {cache_err}")

        except socket.timeout:
            continue
        except (socket.error, BrokenPipeError, ConnectionRefusedError) as e:
            print(f"[监听] Socket断开: {e}，重连中...")
            if reconnect_data_socket():
                continue
            else:
                break
        except Exception as outer_err:
            print(f"[监听] 未预期异常: {outer_err}")
            time.sleep(0.01)
            continue

    print("[监听] 退出")

def create_socket_conn(sock_path: str):
    """连接到 Unix Domain Socket，重试5次"""
    for i in range(5):
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect(sock_path)
            print(f"[初始化] Socket连接成功: {sock_path} ({i+1}/5)")
            return sock
        except (socket.error, FileNotFoundError, ConnectionRefusedError) as e:
            print(f"[初始化] 连接失败 {sock_path} ({i+1}/5): {e}")
            time.sleep(1)
    print(f"[初始化] Socket连接失败 {sock_path}，重试5次后退出")
    return None


def init_both_sockets() -> bool:
    """初始化两个 Unix Socket"""
    global hSocket_cmd, hSocket_data

    hSocket_cmd = create_socket_conn(SOCKET_CMD_PATH)
    if hSocket_cmd is None:
        print("[初始化] cmd socket创建失败")
        return False

    hSocket_data = create_socket_conn(SOCKET_DATA_PATH)
    if hSocket_data is None:
        print("[初始化] data socket创建失败")
        _close_socket(hSocket_cmd)
        hSocket_cmd = None
        return False

    return True


def reconnect_cmd_socket() -> bool:
    """重连 cmd socket"""
    global hSocket_cmd
    _close_socket(hSocket_cmd)
    hSocket_cmd = create_socket_conn(SOCKET_CMD_PATH)
    return hSocket_cmd is not None


def reconnect_data_socket() -> bool:
    """重连 data socket"""
    global hSocket_data
    _close_socket(hSocket_data)
    hSocket_data = create_socket_conn(SOCKET_DATA_PATH)
    return hSocket_data is not None


def _close_socket(sock):
    """安全关闭 socket"""
    if sock is not None:
        try: sock.close()
        except: pass


def send_calc_command(cmd: dict) -> dict:
    """封装发送逻辑 + 缓存编队命令数据供可视化"""
    update_formation_cache(cmd)
    send_success = send_socket_command(cmd)
    if not send_success:
        return {"status": "error", "message": "发送/读取失败", "data": None}
    return {"status": "success", "message": "发送成功，结果已保存到message.json", "data": None}

def load_cmd_from_file(file_name: str) -> dict or None:
    """加载命令文件（优先从脚本所在目录查找）"""
    global SCRIPT_DIR
    # 如果文件名不是绝对路径且当前目录找不到，尝试脚本目录
    if not os.path.isabs(file_name) and not os.path.exists(file_name):
        file_name = os.path.join(SCRIPT_DIR, file_name)
    try:
        if not os.path.exists(file_name):
            print(f"错误：文件 {file_name} 不存在")
            return None
        with open(file_name, "r", encoding="utf-8") as f:
            cmd = json.load(f)
        if not isinstance(cmd, dict):
            print(f"错误：{file_name} 不是JSON字典")
            return None
        print(f"成功加载：{file_name}")
        return cmd
    except Exception as e:
        print(f"加载文件失败：{e}")
        return None

# ========================= 主函数 =========================
def main():
    global is_listening, hSocket_cmd, hSocket_data, playback_thread, is_playing, current_playback_frame, max_cached_frame
    global is_realtime_running, realtime_frame_count, realtime_dt, formation_states, pending_commands, playback_speed
    global realtime_trajectory, realtime_save_interval, realtime_output_dir, realtime_save_counter
    exe_path = r"F:\Seven\build\bin\Debug\app.exe"

    # 初始化两个纯同步Socket
    print("[初始化] 开始创建双Socket连接（纯同步模式）...")
    if not init_both_sockets():
        print("[初始化] 双Socket初始化失败，退出")
        return

    # 启动监听线程（纯同步）
    listen_thread = threading.Thread(target=listen_socket_continually, daemon=True)
    listen_thread.start()
    print("监听线程已启动（纯同步模式，监听ServerToClientPipe）")

    # 启动播放控制线程
    playback_thread = threading.Thread(target=playback_worker, daemon=True)
    playback_thread.start()
    print("播放控制线程已启动")

    # 启动C++程序（可选）
    # proc = subprocess.Popen([exe_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # print(f"C++程序启动，PID：{proc.pid}")
    time.sleep(0.5)  # 短等待

    """交互式主逻辑"""
    print("仿真平台启动（纯同步Socket模式 + 逐帧播放功能）...")
    print("\n===== 命令输入 =====")
    print("11-15：编队命令 | plot：静态分析 | config：初始配置 | exit：退出")
    print("speed <ms>：调整刷新速度 | status：状态")
    print("===== 播放控制命令 =====")
    print("play：开始播放 | pause：暂停 | speed <x>：调整播放速度（例：speed 2.0）")
    print("goto <x>：跳转到指定帧 | reset：重置到第0帧")
    print("===== 实时仿真命令 (新增) =====")
    print("realtime：启动实时仿真循环 | rstop：停止实时仿真")
    print("rswitch <fid> <type>：队形切换 | rturn <fid> <rate>：转向")
    print("radd <fid>：添加节点 | rremove <fid> <n>：移除末尾n个节点")
    print("rspeed <fid> <v>：修改主船速度 | rheading <fid> <deg>：修改航向")
    print("rinfo：查看所有编队实时状态 | save：手动保存轨迹JSON")
    print("====================\n")

    # ====== 自动启动编队实时监控窗口 ======
    _viewer_timer = launch_formation_viewer(refresh_ms=200)
    print("[系统] 编队实时监控窗口已自动启动！发送编队命令后自动缓存数据，输入 'play' 开始逐帧播放。\n")

    try:
        while True:
            user_input = input(">> 输入命令/数字：").strip()

            if user_input.lower() in ['exit', 'quit', 'q']:
                print("退出程序...")
                break

            # ===================== 播放控制命令 =====================
            if user_input.lower() == 'play':
                is_playing = True
                print("▶ 开始播放")
                continue

            if user_input.lower() == 'pause':
                is_playing = False
                print("⏸ 已暂停")
                continue

            if user_input.lower() == 'reset':
                current_playback_frame = 0
                print("🔄 已重置到第0帧")
                continue

            if user_input.lower().startswith('speed '):
                # 调整播放速度
                parts = user_input.split()
                if len(parts) >= 2:
                    try:
                        new_speed = float(parts[1])
                        if new_speed <= 0:
                            print("播放速度必须大于0")
                            continue
                        global playback_speed
                        playback_speed = new_speed
                        print(f"播放速度已调整为 {new_speed:.1f}x")
                    except ValueError:
                        print("用法: speed <倍数>  例: speed 2.0")
                else:
                    print(f"当前播放速度: {playback_speed:.1f}x")
                    print("用法: speed <倍数>  例: speed 2.0")
                continue

            if user_input.lower().startswith('goto '):
                # 跳转到指定帧
                parts = user_input.split()
                if len(parts) >= 2:
                    try:
                        target_frame = int(parts[1])
                        if target_frame < 0 or target_frame > max_cached_frame:
                            print(f"帧号必须在0到{max_cached_frame}之间")
                            continue
                        current_playback_frame = target_frame
                        print(f"已跳转到第 {target_frame} 帧")
                    except ValueError:
                        print("用法: goto <帧号>  例: goto 100")
                else:
                    print(f"当前播放帧: {current_playback_frame}，已缓存最大帧: {max_cached_frame}")
                    print("用法: goto <帧号>  例: goto 100")
                continue

            # ===================== 【新增】实时仿真命令 =====================
            if user_input.lower() == 'realtime':
                if is_realtime_running:
                    print("[实时] 已经在运行中！输入 rstop 先停止")
                    continue
                if not formation_states:
                    print("[实时] 没有编队状态！请先用 sim_start 初始化编队 (如: 11)")
                    continue
                is_realtime_running = True
                realtime_frame_count = 0
                rt_thread = threading.Thread(target=realtime_loop, daemon=True)
                rt_thread.start()
                print("[实时] 实时仿真线程已启动")
                continue

            if user_input.lower() == 'rstop':
                is_realtime_running = False
                time.sleep(0.2)  # 等循环退出
                flush_realtime_trajectory()
                print("[实时] 已停止，剩余轨迹已落盘")
                continue

            if user_input.lower() == 'save':
                flush_realtime_trajectory()
                print("[实时] 轨迹已手动保存")
                continue

            if user_input.lower() == 'rinfo':
                if not formation_states:
                    print("[实时] 无编队状态数据")
                else:
                    print(f"\n===== 实时编队状态 ({realtime_frame_count}帧) =====")
                    for fid, st in sorted(formation_states.items()):
                        print(f"编队{fid}: 队形{st.current_formation} "
                              f"主船({st.main_lon:.6f},{st.main_lat:.6f}) "
                              f"航向{st.main_heading:.1f}° 航速{st.main_speed:.2f}m/s")
                        print(f"  从节点{len(st.slaves)}个: ", end="")
                        for sid, s in sorted(st.slaves.items()):
                            print(f"[{sid}]spd={s['speed']:.2f} hdg={s['heading']:.1f}°  ", end="")
                        print()
                    print(f"\n运行中:{is_realtime_running} | 待发命令:{list(pending_commands.keys())}")
                    print("===============================\n")
                continue

            if user_input.lower().startswith('rswitch '):
                parts = user_input.split()
                if len(parts) < 3:
                    print("用法: rswitch <formation_id> <formation_type>")
                    print("  队形类型: 1=楔形 2=纵队 3=横队 4=菱形 5=三角形 6=圆形")
                    continue
                fid = int(parts[1])
                ftype = int(parts[2])
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                queue_command(fid, {"isSwitch": True, "formation_type": ftype,
                                    "custom_id": 1, "heading_rate": 0.0,
                                    "add_node": [], "remove_num": 0, "isTurn": False,
                                    "isAdd": False, "isRemove": False})
                print(f"[实时] 编队{fid} 将在下一帧切换为 {FORMATION_TYPE_NAMES.get(ftype, f'类型{ftype}')}")
                continue

            if user_input.lower().startswith('rturn '):
                parts = user_input.split()
                if len(parts) < 3:
                    print("用法: rturn <formation_id> <heading_rate>")
                    continue
                fid = int(parts[1])
                rate = float(parts[2])
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                with realtime_lock:
                    formation_states[fid].heading_rate = rate
                queue_command(fid, {"isTurn": True, "heading_rate": rate})
                print(f"[实时] 编队{fid} 将在下一帧开始转向 rate={rate}°/s")
                continue

            if user_input.lower().startswith('radd '):
                parts = user_input.split()
                if len(parts) < 2:
                    print("用法: radd <formation_id>")
                    continue
                fid = int(parts[1])
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                st = formation_states[fid]
                new_lon = st.main_lon
                new_lat = st.main_lat - 2 * st.interval / DEG_TO_M
                add_node = {
                    "pos": {"lon_deg": round(new_lon, 8), "lat_deg": round(new_lat, 8)},
                    "speed": st.main_speed,
                    "heading": st.main_heading,
                    "join_frames": 30,
                    "rel_pos": {"x_m": 0.0, "y_m": -2 * st.interval}
                }
                queue_command(fid, {
                    "isAdd": True, "add_node": [add_node],
                    "isSwitch": True, "formation_type": st.current_formation,
                    "custom_id": 1, "heading_rate": 0.0, "remove_num": 0,
                    "isTurn": False, "isRemove": False
                })
                print(f"[实时] 编队{fid} 将在下一帧添加节点 (pos: {new_lon:.6f},{new_lat:.6f})")
                continue

            if user_input.lower().startswith('rremove '):
                parts = user_input.split()
                if len(parts) < 3:
                    print("用法: rremove <formation_id> <num>")
                    continue
                fid = int(parts[1])
                rn = int(parts[2])
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                queue_command(fid, {"isRemove": True, "remove_num": rn})
                print(f"[实时] 编队{fid} 将在下一帧移除 {rn} 个节点")
                continue

            if user_input.lower().startswith('rspeed '):
                parts = user_input.split()
                if len(parts) < 3:
                    print("用法: rspeed <formation_id> <speed_mps>")
                    continue
                fid = int(parts[1])
                spd = float(parts[2])
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                with realtime_lock:
                    formation_states[fid].main_speed = spd
                print(f"[实时] 编队{fid} 主船速度 → {spd} m/s")
                continue

            if user_input.lower().startswith('rheading '):
                parts = user_input.split()
                if len(parts) < 3:
                    print("用法: rheading <formation_id> <heading_deg>")
                    continue
                fid = int(parts[1])
                hdg = float(parts[2]) % 360
                if fid not in formation_states:
                    print(f"编队{fid}不存在！可用: {list(formation_states.keys())}")
                    continue
                with realtime_lock:
                    formation_states[fid].main_heading = hdg
                print(f"[实时] 编队{fid} 主船航向 → {hdg}°")
                continue

            # ===================== 可视化命令 =====================
            if user_input.lower() == 'plot':
                if not formation_data:
                    print("\n暂无编队数据！请先发送编队命令(11-15)等待监听数据...")
                    continue

                print(f"\n可用编队ID：{list(formation_data.keys())}")
                for fid, frames in formation_data.items():
                    sorted_fr = sorted(frames.items(), key=lambda x: x[0])
                    fids = [f[0] for f in sorted_fr]
                    num_uavs = 0
                    if sorted_fr:
                        nodes = sorted_fr[0][1].get("nodes", [])
                        num_uavs = len(nodes)
                    print(f"  编队 {fid}: {len(sorted_fr)}帧, "
                          f"#{min(fids)}-#{max(fids)}, {num_uavs}架UAV")

                fid_input = input("输入编队ID（直接回车使用默认）：").strip()
                formation_id = int(fid_input) if fid_input else None
                visualize_formation_static(formation_id=formation_id)
                continue

            if user_input.lower() == 'config':
                visualize_formation_config()
                continue

            if user_input.lower().startswith('speed') and not user_input.lower().startswith('speed '):
                # 调整实时监控刷新速度
                parts = user_input.split()
                if len(parts) >= 2:
                    try:
                        new_ms = int(parts[1])
                        if new_ms < 20:
                            new_ms = 20
                        # 重启定时器
                        global fig
                        if fig and plt.fignum_exists(fig.number):
                            _viewer_timer.stop()
                            _viewer_timer.interval = new_ms
                            _viewer_timer.start()
                            print(f"刷新速度已调整为 {new_ms}ms/帧")
                    except ValueError:
                        print("用法: speed <毫秒数>  例: speed 100")
                else:
                    print(f"当前刷新速度: {_viewer_timer.interval}ms/帧")
                    print("用法: speed <毫秒数>  例: speed 100")
                continue

            if user_input.lower() == 'status':
                total_frames = sum(len(v) for v in formation_data.values())
                print(f"\n===== 当前状态 =====")
                print(f"监听状态: {'运行中' if is_listening else '已停止'}")
                print(f"发送Socket: {'已连接' if hSocket_cmd else '未连接'}")
                print(f"监听Socket: {'已连接' if hSocket_data else '未连接'}")
                print(f"缓存编队数: {len(formation_data)}，总帧数: {total_frames}")
                for fid, frames in formation_data.items():
                    sorted_fr = sorted(frames.items(), key=lambda x: x[0])
                    if sorted_fr:
                        fids_list = [f[0] for f in sorted_fr]
                        print(f"  编队{fid}: {len(sorted_fr)}帧 "
                              f"(#{min(fids_list)}-#{max(fids_list)})")
                if cmd_cache.get("cmd_info"):
                    info = cmd_cache["cmd_info"]
                    print(f"上次命令: cmd={info.get('cmd')}, sim_type={info.get('sim_type')}")
                print(f"实时监控: 运行中 ({_viewer_timer.interval}ms刷新)")
                print(f"播放状态: {'播放中' if is_playing else '已暂停'}")
                print(f"当前播放帧: {current_playback_frame}，已缓存最大帧: {max_cached_frame}")
                print(f"播放速度: {playback_speed:.1f}x")
                print("====================\n")
                continue

            # ===================== 原有命令处理 =====================
            if not user_input:
                print("命令不能为空")
                continue
            
            # 解析命令
            cmd = None
            if user_input in CMD_FILE_MAP:
                cmd = load_cmd_from_file(CMD_FILE_MAP[user_input])
                if cmd is None:
                    continue
            else:
                try:
                    cmd = json.loads(user_input)
                    if not isinstance(cmd, dict):
                        print("命令必须是JSON字典")
                        continue
                except json.JSONDecodeError as e:
                    print(f"JSON解析错误：{e}")
                    continue
            
            # 发送命令并计时
            print(f"[发送] 命令：{cmd}")
            start = time.time()
            result = send_calc_command(cmd)
            
            end = time.time()
            print(f"[耗时] {end-start:.3f} 秒 | [结果] {result['message']}\n")

    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序异常：{e}")
    finally:
        # 停止所有线程
        is_listening = False
        listen_thread.join(timeout=2)
        print("监听线程已停止")
        
        # 关闭两个Socket句柄
        if hSocket_cmd is not None:
            try:
                hSocket_cmd.close()
                print("发送Socket句柄已关闭")
            except:
                pass
        
        if hSocket_data is not None:
            try:
                hSocket_data.close()
                print("监听Socket句柄已关闭")
            except:
                pass
        
        # 终止C++程序（可选）
        # if 'proc' in locals() and proc.poll() is None:
        #     proc.terminate()
        #     print("C++程序已终止")

    print("程序退出")

if __name__ == "__main__":
    main()