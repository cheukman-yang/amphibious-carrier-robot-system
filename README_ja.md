# 体験型エンタメ ～海陸両用空母ロボット～

[![Award](https://img.shields.io/badge/Award-Faculty_Award_(教員賞)-gold?style=for-the-badge&logo=trophy)](README_ja.md)
[![Event](https://img.shields.io/badge/Contest-IPUT_Tokyo_2026_CCC-blue?style=for-the-badge)](README_ja.md)

[English Version](README.md)

> 🏆 **受賞実績**: 東京国際工科専門職大学（IPUT Tokyo）主催「2026 CCC」において、全出展作品（全部門総合審査）の中から**【教員賞】**を受賞。

<p align="center">
  <img src="docs/images/robot_carrier_overview.jpg" width="48%" alt="甲板着艦形態">
  <img src="docs/images/robot_flight_mode.jpg" width="48%" alt="自律着陸飛行モード">
</p>

陸・海・空がシームレスに連動する体験型自律協調システム。  
メカナムホイールによる全方位陸上走行と水上ツインスクリュー推進を備えた母艦ロボットに、小型空撮ドローン（DJI Tello）の自動離着陸・協調管制を高度に統合した複合キャリアロボットプラットフォームです。

---

## 主な機能 & 制御アーキテクチャ

- **二重不感帯・適応型速度制御による自律着陸アライメント**
  - 甲板USBカメラでドローン底面の ArUco マーカーをリアルタイム追跡。
  - メカナムホイールの物理摩擦特性に合わせ、前後移動は速度固定で行き過ぎを防止し、負荷の大きい横・斜めスライドは1秒停滞判定で段階増速。
  - 中心到達付近では微小パルス駆動へ切り替え、ピクセル単位での高精度位置合わせを実現。

<p align="center">
  <img src="docs/images/web_dashboard_alignment_01.jpg" width="80%" alt="WebダッシュボードとArUco照準画面">
</p>

- **3視点カメラ統合 & Web集中管制ダッシュボード**
  - 前方CSIカメラ（環境監視・顔認識）、甲板USBカメラ（着陸照準HUD）、ドローンカメラ（空撮パイロット視点）の3系統映像ストリームをブラウザ1画面に集約。
  - Flask / WebSocket を活用し、車体走行・ドローンのホールド操作・ワンタッチ離着陸・個別撮影を低遅延で遠隔操作。

- **エッジAI顔認識 & 自動ログ記録**
  - Haar Cascade ＋ テンプレート照合を用いた人物検知・高速顔認識。
  - 5秒周期での自動キャプチャ撮影および CSV への出席・通過タイムスタンプ記録。

- **マルチセンシング安全フェールセーフ**
  - 超音波センサーによる障害物検知時ブレーキ（高速走行時 50cm 前進ロック機構）。
  - 水センサー連動によるツインスクリュー自動駆動判定（※制御ロジック・機構実証用のPoCプロトタイプであり、実水域での防水実装は今後の課題）。
  - Raspberry Pi 5（RP1チップ）環境における適切な GPIO 排他リソース管理。

<p align="center">
  <img src="docs/images/propeller_unit.jpg" width="60%" alt="水上ツインスクリュー推進ユニット">
</p>

---

## 技術スタック & ハードウェア構成

- **言語**: Python 3, HTML5 / JavaScript, Bash
- **ライブラリ & ツール**: OpenCV, Flask, djitellopy, lgpio, NumPy
- **ハードウェア**:
  - メインコンピュータ: Raspberry Pi 5 (Raspberry Pi OS 64-bit)
  - サブマイコン: Raspberry Pi Pico
  - 走行・推進プラットフォーム: Freenove 4WD Mecanum Wheel Car Kit, トルクチューン2モーター, 水上スクリューユニット
  - 艦載機: DJI Tello
  - センサー・カメラ: CSI広角カメラ, USB甲板カメラ, 超音波センサー, 水位センサー

---

## 起動手順

```bash
# 依存ライブラリのインストール
pip install -r requirements.txt

# 管制サーバーの起動
python3 server.py
```

ブラウザから `http://<RaspberryPi_IP>:5000` にアクセスして集中管制ダッシュボードを操作します。
