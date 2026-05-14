# Frontier-based Occupancy Map Demo

這是一個用 Python + Pygame 製作的 2D frontier / occupancy map 視覺化模擬。

## 安裝

```bash
pip install -r requirements.txt
```

## 執行

```bash
python main.py
```

## 操作

- `W A S D` 或方向鍵：移動 robot
- `R`：重置 occupancy map 與 robot 位置
- `ESC`：離開

## 顏色說明

- 深灰色：Unknown，尚未探索區域
- 淺灰色：Free，已知可走區域
- 黑色：Obstacle，已知障礙物
- 黃色：Frontier，已知 free 且鄰近 unknown 的格子
- 藍色圓點：Robot

## 專案結構

```text
frontier_occupancy_demo/
├── main.py
├── config.py
├── world.py
├── occupancy_map.py
├── robot.py
├── frontier.py
├── renderer.py
└── requirements.txt
```
