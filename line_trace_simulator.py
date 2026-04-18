#!/usr/bin/env python3
"""2センサー・ライン追従ロボットの簡易シミュレータ。

- 入力: 下向きセンサー2個 (0.0〜1.0)
- 出力: 操舵コマンド (-1.0〜1.0)
- 速度: 一定
- 目標: コースを周回
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

Point = Tuple[float, float]


# ----------------------------
# 幾何ユーティリティ
# ----------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rotate(x: float, y: float, theta: float) -> Point:
    c = math.cos(theta)
    s = math.sin(theta)
    return (c * x - s * y, s * x + c * y)


def dist2(a: Point, b: Point) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def nearest_point_on_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float]:
    """点pから線分abへの最近点とパラメータt(0..1)を返す。"""
    ax, ay = a
    bx, by = b
    px, py = p
    vx = bx - ax
    vy = by - ay
    ll = vx * vx + vy * vy
    if ll == 0.0:
        return a, 0.0
    t = ((px - ax) * vx + (py - ay) * vy) / ll
    t = clamp(t, 0.0, 1.0)
    q = (ax + t * vx, ay + t * vy)
    return q, t


# ----------------------------
# コース生成
# ----------------------------
def make_circle(radius: float = 5.0, n: int = 500) -> List[Point]:
    pts: List[Point] = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((radius * math.cos(t), radius * math.sin(t)))
    return pts


def make_oval(a: float = 6.5, b: float = 4.0, n: int = 600) -> List[Point]:
    pts: List[Point] = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((a * math.cos(t), b * math.sin(t)))
    return pts


def make_bean(scale: float = 4.8, n: int = 700) -> List[Point]:
    """くびれのある閉曲線 (自己交差なし)。"""
    pts: List[Point] = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        # limacon: r = a + b cos(t)
        a = scale
        b = 0.42 * scale
        r = a + b * math.cos(t)
        x = r * math.cos(t)
        y = r * math.sin(t)
        pts.append((x, y))
    return pts


def make_rounded_rectangle(width: float = 11.0, height: float = 7.0, r: float = 1.6, n_arc: int = 90) -> List[Point]:
    """角丸長方形 (閉ループ)"""
    hw = width / 2.0
    hh = height / 2.0
    r = min(r, hw * 0.9, hh * 0.9)

    pts: List[Point] = []

    def arc(cx: float, cy: float, a0: float, a1: float, k: int) -> None:
        for i in range(k + 1):
            t = a0 + (a1 - a0) * i / k
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))

    # 上辺右向き -> 右上角 -> 右辺下向き -> ...
    pts.append((-hw + r, hh))
    pts.append((hw - r, hh))
    arc(hw - r, hh - r, math.pi / 2, 0.0, n_arc)

    pts.append((hw, -hh + r))
    arc(hw - r, -hh + r, 0.0, -math.pi / 2, n_arc)

    pts.append((-hw + r, -hh))
    arc(-hw + r, -hh + r, -math.pi / 2, -math.pi, n_arc)

    pts.append((-hw, hh - r))
    arc(-hw + r, hh - r, math.pi, math.pi / 2, n_arc)

    # 重複点を除去
    dedup: List[Point] = []
    for p in pts:
        if not dedup or dist2(p, dedup[-1]) > 1e-10:
            dedup.append(p)
    return dedup


def build_course(name: str) -> List[Point]:
    name = name.lower()
    if name == "circle":
        return make_circle()
    if name == "oval":
        return make_oval()
    if name == "bean":
        return make_bean()
    if name == "rounded_rect":
        return make_rounded_rectangle()
    raise ValueError(f"unknown course: {name}")


# ----------------------------
# シミュレーション本体
# ----------------------------
@dataclass
class SimConfig:
    dt: float = 0.02
    max_time: float = 180.0
    speed: float = 1.15
    max_yaw_rate: float = 2.8
    sensor_forward: float = 0.28
    sensor_half_width: float = 0.14
    sensor_sigma: float = 0.11
    sensor_min: float = 0.0
    sensor_max: float = 1.0
    k_p: float = 3.0
    lookahead_idx: int = 10


@dataclass
class State:
    x: float
    y: float
    yaw: float


@dataclass
class LogRow:
    t: float
    x: float
    y: float
    yaw: float
    sensor_left: float
    sensor_right: float
    cmd: float
    nearest_idx: int


def sensor_value(sensor_pos: Point, line_points: Sequence[Point], sigma: float) -> Tuple[float, int]:
    min_d2 = float("inf")
    min_idx = 0
    n = len(line_points)
    for i in range(n):
        a = line_points[i]
        b = line_points[(i + 1) % n]
        q, _ = nearest_point_on_segment(sensor_pos, a, b)
        d2 = dist2(sensor_pos, q)
        if d2 < min_d2:
            min_d2 = d2
            min_idx = i
    # 近いほど 1.0、離れるほど 0.0
    v = math.exp(-min_d2 / (2.0 * sigma * sigma))
    return v, min_idx


def estimate_tangent(line_points: Sequence[Point], idx: int, lookahead_idx: int) -> float:
    n = len(line_points)
    p0 = line_points[idx % n]
    p1 = line_points[(idx + lookahead_idx) % n]
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def run_simulation(line_points: Sequence[Point], config: SimConfig) -> Tuple[List[LogRow], bool]:
    # 初期位置: コース最初の点近傍、接線向き
    x0, y0 = line_points[0]
    init_yaw = estimate_tangent(line_points, 0, config.lookahead_idx)
    st = State(x=x0, y=y0 - 0.25, yaw=init_yaw)

    logs: List[LogRow] = []

    n = len(line_points)
    lap_started = False
    lap_done = False
    previous_idx = 0

    t = 0.0
    steps = int(config.max_time / config.dt)
    for step in range(steps):
        # センサー位置 (ロボット座標 -> ワールド)
        lf = (config.sensor_forward, +config.sensor_half_width)
        rf = (config.sensor_forward, -config.sensor_half_width)
        ldx, ldy = rotate(lf[0], lf[1], st.yaw)
        rdx, rdy = rotate(rf[0], rf[1], st.yaw)
        left_pos = (st.x + ldx, st.y + ldy)
        right_pos = (st.x + rdx, st.y + rdy)

        left_v, idx_l = sensor_value(left_pos, line_points, config.sensor_sigma)
        right_v, idx_r = sensor_value(right_pos, line_points, config.sensor_sigma)
        nearest_idx = idx_l if left_v > right_v else idx_r

        left_v = clamp(left_v, config.sensor_min, config.sensor_max)
        right_v = clamp(right_v, config.sensor_min, config.sensor_max)

        # 操舵出力: [-1, 1]
        # 左が強い(ラインが左側) -> 左に回頭したい -> 負方向
        cmd = clamp(config.k_p * (right_v - left_v), -1.0, 1.0)

        # 運動更新
        yaw_rate = config.max_yaw_rate * cmd
        st.yaw += yaw_rate * config.dt
        st.x += config.speed * math.cos(st.yaw) * config.dt
        st.y += config.speed * math.sin(st.yaw) * config.dt

        logs.append(
            LogRow(
                t=t,
                x=st.x,
                y=st.y,
                yaw=st.yaw,
                sensor_left=left_v,
                sensor_right=right_v,
                cmd=cmd,
                nearest_idx=nearest_idx,
            )
        )

        # 周回判定: 1度ある程度進んだ後で index が巻き戻る
        progressed = (nearest_idx - previous_idx) % n
        if step > 20 and progressed > n // 2:
            # 逆向き大ジャンプは無視
            progressed = 0

        if nearest_idx > n // 3:
            lap_started = True

        if lap_started and nearest_idx < n // 20 and previous_idx > n // 3:
            lap_done = True
            break

        previous_idx = nearest_idx
        t += config.dt

    return logs, lap_done


def save_csv(logs: Sequence[LogRow], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "yaw", "sensor_left", "sensor_right", "cmd", "nearest_idx"])
        for r in logs:
            w.writerow([r.t, r.x, r.y, r.yaw, r.sensor_left, r.sensor_right, r.cmd, r.nearest_idx])


def try_plot(line_points: Sequence[Point], logs: Sequence[LogRow], out_png: Path) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    lx = [p[0] for p in line_points] + [line_points[0][0]]
    ly = [p[1] for p in line_points] + [line_points[0][1]]
    rx = [r.x for r in logs]
    ry = [r.y for r in logs]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(lx, ly, "k--", lw=2.0, label="course line")
    ax.plot(rx, ry, "tab:blue", lw=1.3, label="robot path")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Line Trace Simulation")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="ライン追従シミュレータ")
    parser.add_argument("--course", choices=["circle", "oval", "bean", "rounded_rect"], default="circle")
    parser.add_argument("--speed", type=float, default=1.15, help="一定速度")
    parser.add_argument("--max-yaw-rate", type=float, default=2.8, help="操舵最大角速度 [rad/s]")
    parser.add_argument("--kp", type=float, default=3.0, help="Pゲイン")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--max-time", type=float, default=180.0)
    parser.add_argument("--out-dir", default="outputs")
    args = parser.parse_args()

    line_points = build_course(args.course)
    cfg = SimConfig(
        dt=args.dt,
        max_time=args.max_time,
        speed=args.speed,
        max_yaw_rate=args.max_yaw_rate,
        k_p=args.kp,
    )

    logs, lap_done = run_simulation(line_points, cfg)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"sim_{args.course}.csv"
    png_path = out_dir / f"sim_{args.course}.png"
    save_csv(logs, csv_path)
    plotted = try_plot(line_points, logs, png_path)

    print(f"course      : {args.course}")
    print(f"steps       : {len(logs)}")
    print(f"sim_time[s] : {logs[-1].t if logs else 0:.2f}")
    print(f"lap_done    : {lap_done}")
    print(f"log_csv     : {csv_path}")
    if plotted:
        print(f"plot_png    : {png_path}")
    else:
        print("plot_png    : (matplotlib が無いため未出力)")


if __name__ == "__main__":
    main()
