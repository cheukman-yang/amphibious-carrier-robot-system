# Amphibious Land-Air Carrier Robot System

[日本語ドキュメント (Japanese README)](README_ja.md)

An integrated teleoperation and autonomous carrier system designed for multi-terrain operations. Powered by Raspberry Pi 5, this platform coordinates terrestrial locomotion, dual-screw water propulsion, and aerial reconnaissance via a DJI Tello drone with low-latency video streaming.

---

## Key Features
- **Integrated Carrier Platform**: Raspberry Pi 5 core communicating with motor controllers and microcontrollers for amphibious operations.
- **Aerial Video Pipeline**: Real-time video ingestion from DJI Tello, featuring color-space correction (RGB to BGR) and dynamic HUD overlay (battery/timestamp).
- **Web Teleoperation Dashboard**: Responsive Flask/WebSocket-based control interface accessible across local wireless tethering.
- **Hardware Concurrency & Safety**: Clean GPIO peripheral resource handling under the RP1 architecture to avoid process lockouts.

---

## Tech Stack
- **Languages**: Python 3, C/C++, HTML5/JavaScript
- **Libraries & Frameworks**: OpenCV, Flask, djitellopy, lgpio, NumPy
- **Environment**: Raspberry Pi OS 64-bit (Linux), VS Code Remote-SSH

---

## Setup & Running

```bash
pip install -r requirements.txt
python3 server.py
