# -*- coding: utf-8 -*-
"""replay_130839.py — 用 130839 实际数据重放新 DLL（app.exe 管道），统计输出航向摆动
用法: 先启动 app.exe，再运行本脚本（或本脚本自动启动）
"""
import json, time, sys, os, subprocess

BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(BASE)
APP = os.path.join(ROOT, 'build', 'bin', 'Release', 'app.exe')
RECV = os.path.join(ROOT, 'jsons', '2026-08-27_130839000_recv.json')

import pywintypes, win32file, win32pipe
PIPE_C2S = r'\\.\pipe\ClientToServerPipe'
PIPE_S2C = r'\\.\pipe\ServerToClientPipe'
DELIMITER = b'\n###END###\n'

def connect(pipe_name, tries=30):
    for i in range(tries):
        try:
            h = win32file.CreateFile(pipe_name, win32file.GENERIC_READ | win32file.GENERIC_WRITE,
                                     0, None, win32file.OPEN_EXISTING, 0, None)
            return h
        except pywintypes.error:
            time.sleep(0.5)
    return None

def send_recv(h, data):
    raw = json.dumps(data, ensure_ascii=False).encode('utf-8') + DELIMITER
    win32file.WriteFile(h, raw)
    result = win32file.ReadFile(h, 4096 * 10000)
    buf = bytes(result[1])
    s = buf.decode('utf-8', errors='ignore')
    pos = s.find('\n###END###\n')
    return json.loads(s[:pos]) if pos != -1 else json.loads(s)

def recv_only(h):
    win32pipe.SetNamedPipeHandleState(h, win32pipe.PIPE_READMODE_MESSAGE, None, None)
    buf = b''
    while DELIMITER not in buf:
        chunk = win32file.ReadFile(h, 4096 * 10000)[1]
        buf += bytes(chunk)
    s = buf.decode('utf-8', errors='ignore')
    pos = s.find('\n###END###\n')
    return json.loads(s[:pos]) if pos != -1 else None

def main():
    # 启动服务端
    print('启动 app.exe ...')
    proc = subprocess.Popen([APP], cwd=os.path.dirname(APP))
    time.sleep(3)
    try:
        h = connect(PIPE_C2S)
        if h is None:
            print('[错误] c2s 管道连接失败')
            return
        hs = connect(PIPE_S2C)
        if hs is None:
            print('[错误] s2c 管道连接失败')
            return

        lines = [json.loads(l) for l in open(RECV, encoding='utf-8', errors='replace') if l.strip()]
        print('总帧数:', len(lines))

        # 帧0: 初始化
        r0 = send_recv(h, lines[0])
        print('初始化响应 status:', r0.get('status'))

        # 逐帧重放实时帧
        headings = {1: [], 2: []}
        first_send = None
        for i, f in enumerate(lines[1:], 1):
            send_recv(h, f)                    # c2s ack
            resp = recv_only(hs)               # s2c 实时结果
            if resp is None:
                print('  [警告] f%d 无 s2c 结果' % i)
                continue
            for fm in resp.get('formations', []):
                for nd in fm.get('nodes', []):
                    if nd['node_id'] in headings:
                        headings[nd['node_id']].append(nd['heading'])
            if i % 100 == 0:
                print('  ... %d/%d' % (i, len(lines) - 1))
        print('重放完成')

        # 摆动统计（与模拟同一口径: >5°/帧 计为摆动）
        print('\n=== 新 DLL 输出摆动统计 ===')
        for sid in [1, 2]:
            hs = headings[sid]
            if len(hs) < 2:
                print('s%d: 无数据' % sid)
                continue
            chg = [min(abs(hs[i]-hs[i-1]), 360-abs(hs[i]-hs[i-1])) for i in range(1, len(hs))]
            swing = 100 * sum(1 for c in chg if c > 5) / len(chg)
            # 分段: 转向帧 76, 268 ±12
            turns = set(range(76-12, 76+13)) | set(range(268-12, 268+13))
            tot = sw = 0
            for i in range(1, len(hs)):
                if i in turns or i-1 in turns: continue
                tot += 1
                if chg[i-1] > 5: sw += 1
            st_swing = 100 * sw / max(tot, 1)
            print('s%d: 摆动 %.1f%% | 稳态摆动 %.1f%% | 最大Δ %.0f°' % (sid, swing, st_swing, max(chg)))
            print('  帧61-110: %s' % ['%.0f' % h for h in hs[61:111]])
            print('  帧300-465: %s' % ['%.0f' % h for h in hs[300:466]])
    finally:
        proc.terminate()

if __name__ == '__main__':
    main()
