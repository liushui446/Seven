# -*- coding: utf-8 -*-
"""
merge_json.py — 合并多个 JSON 文件为一个 JSON 数组
用法:
    python merge_json.py                  # 合并当前目录下所有子文件夹(recvJson/、sendJson/...)
    python merge_json.py recvJson sendJson  # 只合并指定文件夹
输出: jsons/<文件夹名>.json，按文件名排序合并
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def merge_folder(folder: Path) -> None:
    files = sorted(folder.glob('*.json'))
    if not files:
        print(f'  {folder.name}: 无 .json 文件，跳过')
        return
    merged = []
    for fp in files:
        data = json.loads(fp.read_text(encoding='utf-8'))
        if isinstance(data, list):
            merged.extend(data)
        else:
            merged.append(data)
    out = HERE / (folder.name + '.json')
    out.write_text(json.dumps(merged, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'  {folder.name}: {len(files)} 个文件 -> {len(merged)} 条记录 -> {out.name}')


if __name__ == '__main__':
    targets = [Path(a) for a in sys.argv[1:]] or [p for p in HERE.iterdir() if p.is_dir()]
    for t in targets:
        if t.is_dir():
            merge_folder(t)
        else:
            print(f'  跳过非文件夹: {t}')
    print('完成')
