"""
UUV 编队仿真可视化系统 - Python IPC 客户端
============================================

通过 Windows 命名管道与 C++ 仿真服务端通信。

通信协议:
  - 管道名: \\\\.\\pipe\\SimCalculatorPipe
  - 消息格式: JSON + 分隔符 "\\n###END###\\n"
  - 编码: UTF-8

用法:
  python sim_client.py                    # 交互模式
  python sim_client.py --example          # 运行示例
"""

import json
import struct
import sys
import time
import argparse

try:
    import win32file
    import win32pipe
    import pywintypes
except ImportError:
    print("请安装 pywin32: pip install pywin32")
    sys.exit(1)

# ==================== 常量 ====================

PIPE_NAME = r"\\.\pipe\ClientToServerPipe"
PIPE_NAME_S2C = r"\\.\pipe\ServerToClientPipe"
JSON_DELIMITER = "\n###END###\n"
BUF_SIZE = 4096 * 10000

# 命令类型
CMD_BARRAGE = 1
CMD_DECEPTION = 2
CMD_TRANSFORMATION = 3

# 仿真状态
SIM_STARTED = 1   # 仿真准备
SIM_RUNNING = 2   # 仿真运行中
SIM_STOPPED = 3   # 仿真暂停
SIM_ENDDING = 4   # 仿真结束

# 编队类型
FORMATION_RECTANGLE = 1
FORMATION_TRIANGLE = 2
FORMATION_CIRCLE = 3
FORMATION_DIAMOND = 4
FORMATION_LINE = 5

FORMATION_NAMES = {
    1: "Rectangle", 2: "Triangle", 3: "Circle",
    4: "Diamond", 5: "Line"
}


# ==================== 管道客户端 ====================

class SimClient:
    """UUV 编队仿真 IPC 客户端（双管道）"""

    def __init__(self):
        self.handle_c2s = None   # 发送命令 + 接收即时回复
        self.handle_s2c = None   # 接收服务端推送数据

    def connect(self) -> bool:
        """连接到仿真服务端（需同时连接 c2s 和 s2c 管道）"""
        try:
            # 1. 连接 c2s 管道（发送指令、接收即时回复）
            self.handle_c2s = win32file.CreateFile(
                PIPE_NAME,
                win32file.GENERIC_READ | win32file.GENERIC_WRITE,
                0,
                None,
                win32file.OPEN_EXISTING,
                0,
                None
            )
            win32pipe.SetNamedPipeHandleState(
                self.handle_c2s,
                win32pipe.PIPE_READMODE_MESSAGE,
                None, None
            )
            print(f"[OK] 已连接 c2s 管道: {PIPE_NAME}")

            # 2. 连接 s2c 管道（接收服务端推送）
            self.handle_s2c = win32file.CreateFile(
                PIPE_NAME_S2C,
                win32file.GENERIC_READ | win32file.GENERIC_WRITE,
                0,
                None,
                win32file.OPEN_EXISTING,
                0,
                None
            )
            win32pipe.SetNamedPipeHandleState(
                self.handle_s2c,
                win32pipe.PIPE_READMODE_MESSAGE,
                None, None
            )
            print(f"[OK] 已连接 s2c 管道: {PIPE_NAME_S2C}")
            return True
        except pywintypes.error as e:
            print(f"[ERROR] 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.handle_c2s:
            win32file.CloseHandle(self.handle_c2s)
            self.handle_c2s = None
        if self.handle_s2c:
            win32file.CloseHandle(self.handle_s2c)
            self.handle_s2c = None
        print("[OK] 已断开连接")

    def send_command(self, cmd: dict) -> dict:
        """发送命令并接收响应"""
        # 序列化并添加分隔符（服务端要求 \n###END###\n）
        json_str = json.dumps(cmd, ensure_ascii=False)
        data = json_str + JSON_DELIMITER
        raw_data = data.encode('utf-8')

        # 发送（消息模式：一次 WriteFile = 一条完整消息）
        win32file.WriteFile(self.handle_c2s, raw_data)
        print(f"[SEND] {json_str[:200]}...")

        # 接收（消息模式管道，一次 ReadFile 获取完整响应）
        hr, buf = win32file.ReadFile(self.handle_c2s, BUF_SIZE)
        response_str = buf.decode('utf-8', errors='replace') if buf else "{}"

        # 去除分隔符
        delimiter_idx = response_str.find(JSON_DELIMITER)
        if delimiter_idx >= 0:
            response_str = response_str[:delimiter_idx]

        try:
            response = json.loads(response_str)
        except json.JSONDecodeError:
            response = {"status": "error", "message": f"JSON parse failed: {response_str[:200]}"}

        return response

    # ==================== 高层 API ====================

    def sim_start(self, pos_lon: float, pos_lat: float, num_uavs: int = 4,
                  formation_type: int = FORMATION_RECTANGLE,
                  interval: float = 10.0, init_speed: float = 2.0,
                  init_heading: float = 0.0, heading_rate: float = 2.0,
                  collision_radius: float = 4.0, return_frames: int = 10) -> dict:
        """
        初始化编队仿真

        Args:
            pos_lon: 主节点经度
            pos_lat: 主节点纬度
            num_uavs: 节点数量 (2~10)
            formation_type: 队形类型 (1=Rectangle, 2=Triangle, 3=Circle, 4=Diamond, 5=Line)
            interval: 节点间距 (米)
            init_speed: 初始速度 (m/s)
            init_heading: 初始航向 (度)
            heading_rate: 航向变化率 (度/秒)
            collision_radius: 碰撞半径 (米)
            return_frames: 每次返回帧数
        """
        cmd = {
            "cmd": CMD_TRANSFORMATION,
            "sim_type": SIM_STARTED,
            "pos_lon": pos_lon,
            "pos_lat": pos_lat,
            "num_uavs": num_uavs,
            "formation_type": formation_type,
            "interval": interval,
            "init_speed": init_speed,
            "init_heading": init_heading,
            "heading_rate": heading_rate,
            "collision_radius": collision_radius,
            "return_frames": return_frames
        }
        return self.send_command(cmd)

    def sim_calc(self, **kwargs) -> dict:
        """
        执行仿真计算 / 发送实时命令

        支持的实时命令:
          - switch_formation(type): 切换队形
          - turn(heading_rate): 转向
          - add_node(nodes): 添加节点
          - remove_node(num): 删除节点
          - export_json(filename=None): 导出JSON
          - export_binary(filename): 导出二进制
          - get_stats(): 获取统计
          - get_status(): 获取状态
        """
        cmd = {
            "cmd": CMD_TRANSFORMATION,
            "sim_type": SIM_RUNNING
        }

        if "switch_formation" in kwargs:
            cmd["isSwitch"] = True
            cmd["formation_type"] = kwargs["switch_formation"]

        if "turn" in kwargs:
            cmd["isTurn"] = True
            cmd["heading_rate"] = kwargs["turn"]

        if "add_node" in kwargs:
            cmd["isAdd"] = True
            cmd["add_node"] = kwargs["add_node"]

        if "remove_node" in kwargs:
            cmd["isRemove"] = True
            cmd["remove_num"] = kwargs["remove_node"]

        if "export_json" in kwargs:
            cmd["isExportJSON"] = True
            filename = kwargs["export_json"]
            if filename:
                cmd["export_filename"] = filename

        if "export_binary" in kwargs:
            cmd["isExportBinary"] = True
            cmd["export_filename"] = kwargs.get("export_binary", "trajectory.traj")

        if "get_stats" in kwargs:
            cmd["isGetStats"] = True

        if "get_status" in kwargs:
            cmd["isGetStatus"] = True

        return self.send_command(cmd)

    def sim_stop(self) -> dict:
        """暂停仿真"""
        return self.send_command({"cmd": CMD_TRANSFORMATION, "sim_type": SIM_STOPPED})

    def sim_end(self) -> dict:
        """结束仿真"""
        return self.send_command({"cmd": CMD_TRANSFORMATION, "sim_type": SIM_ENDDING})

    # ==================== 便捷方法 ====================

    def switch_formation(self, formation_type: int) -> dict:
        """切换队形"""
        name = FORMATION_NAMES.get(formation_type, "Unknown")
        print(f"\n>>> 切换队形: {name}")
        return self.sim_calc(switch_formation=formation_type)

    def turn_formation(self, heading_rate: float) -> dict:
        """改变航向"""
        print(f"\n>>> 改变航向变化率: {heading_rate} deg/s")
        return self.sim_calc(turn=heading_rate)

    def add_nodes(self, nodes: list) -> dict:
        """
        添加节点

        Args:
            nodes: 节点列表，每个节点格式:
                   {"pos": {"lon_deg": 116.4, "lat_deg": 39.9},
                    "speed": 2.0, "heading": 45.0, "join_frames": 30}
        """
        print(f"\n>>> 添加 {len(nodes)} 个节点")
        return self.sim_calc(add_node=nodes)

    def remove_nodes(self, num: int = 1) -> dict:
        """删除末尾节点"""
        print(f"\n>>> 删除 {num} 个节点")
        return self.sim_calc(remove_node=num)

    def export_trajectory_json(self, filename: str = None) -> dict:
        """导出轨迹为 JSON"""
        print(f"\n>>> 导出轨迹 (JSON)")
        return self.sim_calc(export_json=filename)

    def export_trajectory_binary(self, filename: str = "trajectory.traj") -> dict:
        """导出轨迹为二进制文件"""
        print(f"\n>>> 导出轨迹 (Binary): {filename}")
        return self.sim_calc(export_binary=filename)

    def get_stats(self) -> dict:
        """获取仿真统计"""
        print(f"\n>>> 获取仿真统计")
        return self.sim_calc(get_stats=True)

    def get_status(self) -> dict:
        """获取仿真状态"""
        print(f"\n>>> 获取仿真状态")
        return self.sim_calc(get_status=True)


# ==================== 辅助函数 ====================

def make_node(lon: float, lat: float, speed: float = 2.0,
              heading: float = 45.0, join_frames: int = 30) -> dict:
    """创建节点数据"""
    return {
        "pos": {"lon_deg": lon, "lat_deg": lat},
        "speed": speed,
        "heading": heading,
        "join_frames": join_frames
    }


def print_response(response: dict):
    """格式化打印响应"""
    status = response.get("status", "unknown")
    if status == "success":
        print(f"  [OK] {response.get('message', '')}")
    else:
        print(f"  [FAIL] {response.get('message', '')}")

    # 打印额外数据
    for key in ["data_size_bytes", "file_path", "file_size_kb",
                "frame_count", "file_saved", "message_a", "message_b"]:
        if key in response:
            print(f"  {key}: {response[key]}")

    if "data" in response:
        data = response["data"]
        if isinstance(data, dict):
            for k, v in data.items():
                if not isinstance(v, (dict, list)):
                    print(f"  {k}: {v}")
                elif isinstance(v, dict):
                    print(f"  {k}: {json.dumps(v, ensure_ascii=False)}")

    if "node_list" in response:
        print(f"  node_list: {len(response['node_list'])} nodes")


# ==================== 示例 ====================

def run_example():
    """运行完整示例"""
    client = SimClient()

    if not client.connect():
        return

    try:
        # 1. 启动编队仿真
        print("\n" + "=" * 50)
        print("  Step 1: 初始化编队仿真")
        print("=" * 50)
        resp = client.sim_start(
            pos_lon=116.397428,
            pos_lat=39.909204,
            num_uavs=4,
            formation_type=FORMATION_RECTANGLE,
            interval=10.0,
            init_speed=2.0,
            init_heading=0.0,
            return_frames=10
        )
        print_response(resp)

        # 2. 运行仿真并获取状态
        print("\n" + "=" * 50)
        print("  Step 2: 运行仿真并获取状态")
        print("=" * 50)
        resp = client.get_status()
        print_response(resp)

        # 3. 切换队形到圆形
        print("\n" + "=" * 50)
        print("  Step 3: 切换队形 -> 圆形")
        print("=" * 50)
        resp = client.switch_formation(FORMATION_CIRCLE)
        print_response(resp)

        # 4. 改变航向
        print("\n" + "=" * 50)
        print("  Step 4: 改变航向变化率")
        print("=" * 50)
        resp = client.turn_formation(heading_rate=5.0)
        print_response(resp)

        # 5. 添加节点
        print("\n" + "=" * 50)
        print("  Step 5: 添加一个新节点")
        print("=" * 50)
        new_nodes = [
            make_node(lon=116.398, lat=39.910, speed=2.5, heading=90.0, join_frames=20)
        ]
        resp = client.add_nodes(new_nodes)
        print_response(resp)

        # 6. 获取统计
        print("\n" + "=" * 50)
        print("  Step 6: 获取仿真统计")
        print("=" * 50)
        resp = client.get_stats()
        print_response(resp)

        # 7. 导出轨迹
        print("\n" + "=" * 50)
        print("  Step 7: 导出轨迹数据")
        print("=" * 50)
        resp = client.export_trajectory_json("example_trajectory.json")
        print_response(resp)

        resp = client.export_trajectory_binary("example_trajectory.traj")
        print_response(resp)

        # 8. 删除节点
        print("\n" + "=" * 50)
        print("  Step 8: 删除节点")
        print("=" * 50)
        resp = client.remove_nodes(num=1)
        print_response(resp)

        # 9. 结束仿真
        print("\n" + "=" * 50)
        print("  Step 9: 结束仿真")
        print("=" * 50)
        resp = client.sim_end()
        print_response(resp)

    finally:
        client.disconnect()


def run_interactive():
    """交互模式"""
    client = SimClient()

    if not client.connect():
        print("请先启动 C++ 仿真服务端")
        return

    print("\n" + "=" * 60)
    print("  UUV 编队仿真 - 交互式客户端")
    print("  输入 'help' 查看命令列表")
    print("=" * 60)

    try:
        while True:
            try:
                line = input("\n> ").strip()
                if not line:
                    continue

                parts = line.split()
                cmd = parts[0].lower()

                if cmd == "help":
                    print("""
命令列表:
  start <lon> <lat> [num_uavs] [formation]  - 初始化仿真
    例: start 116.397 39.909 5 1
    队形: 1=Rectangle 2=Triangle 3=Circle 4=Diamond 5=Line

  switch <formation_type>                    - 切换队形
    例: switch 3  (切换到圆形)

  turn <heading_rate>                       - 改变航向变化率
    例: turn 5.0

  add <lon> <lat> [speed] [heading] [join_frames]
    例: add 116.398 39.910 2.5 90 20

  remove [num]                              - 删除末尾节点
    例: remove 2

  status                                    - 获取仿真状态
  stats                                     - 获取仿真统计
  export [filename]                         - 导出JSON
  export_bin [filename]                     - 导出二进制
  stop                                      - 暂停仿真
  end                                       - 结束仿真
  quit                                      - 退出
""")
                elif cmd == "start":
                    lon = float(parts[1]) if len(parts) > 1 else 116.397
                    lat = float(parts[2]) if len(parts) > 2 else 39.909
                    num = int(parts[3]) if len(parts) > 3 else 4
                    fm = int(parts[4]) if len(parts) > 4 else 1
                    resp = client.sim_start(lon, lat, num, fm)
                    print_response(resp)

                elif cmd == "switch":
                    fm_type = int(parts[1]) if len(parts) > 1 else 1
                    resp = client.switch_formation(fm_type)
                    print_response(resp)

                elif cmd == "turn":
                    rate = float(parts[1]) if len(parts) > 1 else 2.0
                    resp = client.turn_formation(rate)
                    print_response(resp)

                elif cmd == "add":
                    lon = float(parts[1]) if len(parts) > 1 else 116.398
                    lat = float(parts[2]) if len(parts) > 2 else 39.91
                    speed = float(parts[3]) if len(parts) > 3 else 2.0
                    heading = float(parts[4]) if len(parts) > 4 else 45.0
                    join_frames = int(parts[5]) if len(parts) > 5 else 30
                    nodes = [make_node(lon, lat, speed, heading, join_frames)]
                    resp = client.add_nodes(nodes)
                    print_response(resp)

                elif cmd == "remove":
                    num = int(parts[1]) if len(parts) > 1 else 1
                    resp = client.remove_nodes(num)
                    print_response(resp)

                elif cmd == "status":
                    resp = client.get_status()
                    print_response(resp)

                elif cmd == "stats":
                    resp = client.get_stats()
                    print_response(resp)

                elif cmd == "export":
                    filename = parts[1] if len(parts) > 1 else None
                    resp = client.export_trajectory_json(filename)
                    print_response(resp)

                elif cmd == "export_bin":
                    filename = parts[1] if len(parts) > 1 else "trajectory.traj"
                    resp = client.export_trajectory_binary(filename)
                    print_response(resp)

                elif cmd == "stop":
                    resp = client.sim_stop()
                    print_response(resp)

                elif cmd == "end":
                    resp = client.sim_end()
                    print_response(resp)

                elif cmd == "quit":
                    break
                else:
                    print(f"未知命令: {cmd}，输入 'help' 查看帮助")

            except (ValueError, IndexError) as e:
                print(f"参数错误: {e}")
            except KeyboardInterrupt:
                break

    finally:
        client.disconnect()


# ==================== 主入口 ====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="UUV 编队仿真 - Python IPC 客户端"
    )
    parser.add_argument("--example", action="store_true",
                        help="运行示例流程")
    args = parser.parse_args()

    if args.example:
        run_example()
    else:
        run_interactive()
