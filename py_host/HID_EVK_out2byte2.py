#!/usr/bin/env python3
# -------------------------------------------------------------------------
# 環境需求: pip install hidapi
#
# 【Linux 權限設定提醒】
# 若在 Linux 下執行遇到 Permission Denied，請新增 udev 規則：
# 1. sudo nano /etc/udev/rules.d/99-hid-device.rules
# 2. 加入以下內容 (依據你的 VID: 0xffff, PID: 0x1000 調整):
#    SUBSYSTEM=="usb", ATTRS{idVendor}=="ffff", ATTRS{idProduct}=="1000", MODE="0666", GROUP="plugdev"
#    SUBSYSTEM=="hidraw", ATTRS{idVendor}=="ffff", ATTRS{idProduct}=="1000", MODE="0666", GROUP="plugdev"
# 3. 重新載入規則：
#    sudo udevadm control --reload-rules
#    sudo udevadm trigger
# -------------------------------------------------------------------------

# pip install hidapi

import hid
import sys

# 目標學習板的 Vendor ID 和 Product ID
TARGET_VID = 0xFFFF
TARGET_PID = 0x1000

def list_hid_devices():
    """列出系統中所有的 HID 裝置"""
    print("目前系統中的 HID 裝置:")
    devices = hid.enumerate()
    for device in devices:
        vid = device['vendor_id']
        pid = device['product_id']
        manufacturer = device['manufacturer_string'] or "Unknown"
        product = device['product_string'] or "Unknown"
        path = device['path']
        print(f"  - VID: 0x{vid:04X}, PID: 0x{pid:04X}, Manufacturer: {manufacturer}, Product: {product}, Path: {path}")
    print("-" * 60)

def control_led(led_value: int):
    """
    對目標 HID 裝置寫入資料
    參數 led_value: 控制 8 顆 LED 的狀態值 (0x00 ~ 0xFF)
    """
    device = hid.device()
    try:
        # 嘗試開啟指定的 HID 裝置
        device.open(TARGET_VID, TARGET_PID)
        print(f"[成功] 已連接到裝置 VID: 0x{TARGET_VID:04X}, PID: 0x{TARGET_PID:04X}")

        # 封包說明:
        # [Report ID (或第一個資料 byte), 預留的第二個 byte]
        # 根據原程式邏輯，第一個 byte 是使用者傳入的 led 狀態，第二個 byte 固定為 0x00
        # (註: 若裝置需要 Report ID，通常陣列的第一個元素是 Report ID = 0x00)
        data_to_write = [led_value, 0x00]
        
        # 寫入裝置
        device.write(data_to_write)
        print(f"[成功] 已送出封包: {[hex(x) for x in data_to_write]}")

    except OSError as e:
        print(f"[錯誤] 無法開啟 HID 裝置 (VID: 0x{TARGET_VID:04X}, PID: 0x{TARGET_PID:04X})。")
        print(f"       詳細原因: {e}")
        print("       請確認裝置是否已插入，且具有正確的讀寫權限 (Linux下請參考 UDEV 設定)。")
    except Exception as e:
        print(f"[錯誤] 發生未預期的錯誤: {e}")
    finally:
        # 確保最後裝置會被關閉
        device.close()

def main():
    # 1. 顯示所有連接的 HID 設備
    list_hid_devices()

    # 2. 檢查命令列參數
    if len(sys.argv) != 2:
        print("用法說明:")
        print(f"  {sys.argv[0]} <led_byte>")
        print("範例:")
        print(f"  {sys.argv[0]} 0xFF    (全亮, 16進制)")
        print(f"  {sys.argv[0]} 255     (全亮, 10進制)")
        print(f"  {sys.argv[0]} 0x01    (亮第一顆, 16進制)")
        sys.exit(1)

    # 3. 嘗試解析參數為整數 (支援 10 進制或 16 進制)
    try:
        led_value = int(sys.argv[1], 0)
        # 限制範圍在 1 byte 內
        if not (0 <= led_value <= 255):
            raise ValueError("值必須在 0 到 255 (0xFF) 之間。")
    except ValueError as e:
        print(f"[參數錯誤] {e}")
        sys.exit(1)

    # 4. 執行 LED 控制
    control_led(led_value)

if __name__ == "__main__":
    main()
