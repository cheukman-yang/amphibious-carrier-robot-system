# ==============================================================================
# Raspberry Pi Pico 水検知(DO入力) ＆ LED統合制御ファームウェア (main.py)
# ==============================================================================

import machine
import time
import sys
import select

# ==========================================
# 1. 厳格なピンアサイン (物理PIN -> GP番号変換)
# ==========================================
PIN_WATER_DO = 15  # 水センサーDO      (物理PIN 20)
PIN_RED_1    = 16  # 赤色LED 1        (物理PIN 21)
PIN_RED_2    = 17  # 赤色LED 2        (物理PIN 22)
PIN_GREEN    = 18  # 緑色LED          (物理PIN 24)
PIN_YELLOW   = 19  # 黄色LED          (物理PIN 25)

# ==========================================
# 2. ハードウェア初期化
# ==========================================
# 水センサーはDO(デジタル出力)のため、ADCではなく通常のIN(入力)として設定
water_sensor = machine.Pin(PIN_WATER_DO, machine.Pin.IN)

led_red_1  = machine.Pin(PIN_RED_1,  machine.Pin.OUT)
led_red_2  = machine.Pin(PIN_RED_2,  machine.Pin.OUT)
led_green  = machine.Pin(PIN_GREEN,  machine.Pin.OUT)
led_yellow = machine.Pin(PIN_YELLOW, machine.Pin.OUT)

# Pico基板内蔵LED
try:
    led_board = machine.Pin('LED', machine.Pin.OUT)
except Exception:
    led_board = machine.Pin(25, machine.Pin.OUT)

# 初期状態: 赤LEDのみ点滅開始状態、他は消灯
led_yellow.value(0)
led_green.value(0)
led_red_1.value(1)
led_red_2.value(0)
led_board.value(1)

# 非同期シリアル受信設定 (ノンブロッキング)
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

is_wet = False
last_send_time = time.ticks_ms()
last_red_toggle = time.ticks_ms()
red_state = 1

# ==========================================
# 3. メイン制御ループ
# ==========================================
while True:
    current_time = time.ticks_ms()

    # --- A. RasPiからの緑色LEDコマンド受信 ---
    if poll_obj.poll(0):
        cmd = sys.stdin.readline().strip()
        if cmd == "LED_GREEN:1":
            led_green.value(1)
        elif cmd == "LED_GREEN:0":
            led_green.value(0)

    # --- B. 水検知センサー (デジタルDO) 判定 ---
    # 一般的な水センサーモジュールのDOは「水検知でLOW(0)」「乾燥でHIGH(1)」になります。
    # もし実機で逆に反応する場合（水没で消灯してしまう場合）は `val == 1` に変更してください。
    val = water_sensor.value()
    
    if val == 0:  # 浸水判定
        is_wet = True
        led_yellow.value(1)
    else:         # 乾燥判定
        is_wet = False
        led_yellow.value(0)

    # --- C. 赤色LED2基 ＆ 基板LED の常時交互点滅制御 (0.5秒周期) ---
    if time.ticks_diff(current_time, last_red_toggle) >= 500:
        last_red_toggle = current_time
        red_state = 1 - red_state
        led_red_1.value(red_state)
        led_red_2.value(1 - red_state)
        led_board.value(red_state)

    # --- D. RasPiへの定期ステータス送信 (0.1秒ごと) ---
    if time.ticks_diff(current_time, last_send_time) >= 100:
        last_send_time = current_time
        if is_wet:
            print("WET:1")
        else:
            print("WET:0")

    time.sleep_ms(10)
