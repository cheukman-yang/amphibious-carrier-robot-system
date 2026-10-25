# 水陸空連携キャリアロボット統合システム

[English Version](README.md)

Raspberry Pi 5 を中核とし、車輪走行・水上スクリュー推進・DJI Tello ドローン空撮を統合した複合キャリアロボットの制御・通信システムです。

---

## 主な機能
- **水陸空キャリア統合制御**: Raspberry Pi 5 による走行系モーター・水上推進機構および小型ドローンの統轄制御
- **リアルタイム映像ストリーミング**: Tello カメラ映像の色空間補正（RGB ➔ BGR）およびステータス情報の描画オーバーレイ
- **Web テレオペレーション**: Flask / WebSocket を活用した低遅延な機体遠隔操作ダッシュボード
- **ハードウェア排他リソース管理**: Raspberry Pi 5（RP1）環境における適切な GPIO ピン割り当てとプロセス解放設計

---

## 技術スタック
- **開発言語**: Python 3, C/C++, HTML5/JavaScript
- **ライブラリ**: OpenCV, Flask, djitellopy, lgpio, NumPy
- **実行環境**: Raspberry Pi OS 64-bit (Linux), VS Code Remote-SSH

---

## 起動手順

```bash
pip install -r requirements.txt
python3 server.py
