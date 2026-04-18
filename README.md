# ライントレース・シミュレータ

2つの下向きセンサー(0〜1)でラインを検出し、操舵コマンド(-1〜1)で一定速度ロボットを制御する簡易シミュレータです。

## 仕様

- センサー入力: `sensor_left`, `sensor_right` (0.0〜1.0)
- 出力: `cmd` (-1.0〜1.0)
  - `-1` に近いほど左へ回頭
  - `+1` に近いほど右へ回頭
- 速度: 一定 (`--speed`)
- 制御: P制御 `cmd = clamp(kp * (right - left), -1, 1)`

## コース種類

- `circle` : 円
- `oval` : 楕円
- `bean` : くびれ曲線(自己交差なし)
- `rounded_rect` : 角丸長方形

## 実行

```bash
python3 line_trace_simulator.py --course circle
```

主なオプション:

- `--course` : コース種別
- `--speed` : 一定速度
- `--max-yaw-rate` : 最大回頭角速度 [rad/s]
- `--kp` : Pゲイン
- `--dt` : シミュレーション刻み[s]
- `--max-time` : 最大シミュレーション時間[s]
- `--out-dir` : 出力先ディレクトリ

## 出力

`outputs/` に以下を保存します。

- `sim_<course>.csv` : 時系列ログ
  - `t, x, y, yaw, sensor_left, sensor_right, cmd, nearest_idx`
- `sim_<course>.png` : 軌跡画像 (matplotlibがある場合)

## 周回シミュレーション例

```bash
python3 line_trace_simulator.py --course circle
python3 line_trace_simulator.py --course oval
python3 line_trace_simulator.py --course bean
python3 line_trace_simulator.py --course rounded_rect
```

上記4コースでは `lap_done=True` で1周完了を確認できます。
