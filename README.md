# ESP-GRABER Web (ESP32-C3)


- **SSID:** `quốc bảo`
- **Mat khau:** `12345678`
- Truy cap: **http://192.168.4.1**

## Tinh nang (giong ban goc)
- **Nhan (SubGHz-R):** bat tin hieu 315 / 433.92 / 868 / 915 MHz (RCSwitch + Raw), luu EEPROM
- **Phat (SubGHz-T):** phat lai key da luu, xoa key
- **Phan tich (Analyser):** quet RSSI, hien tan so phat hien
- **Jammer:** bat/tat theo tan so chon (⚠️ chi dung cho muc dich hoc tap/hop phap)
- **Cai dat:** Auto-save, xoa EEPROM

## Cam day CC1101 (khong doi so voi ban goc)
| CC1101 | ESP32-C3 |
|--------|----------|
| SCK  | GPIO 4 |
| MISO | GPIO 6 |
| MOSI | GPIO 7 |
| CS   | GPIO 5 |
| GDO0 | GPIO 10 |

## Nap code
### Cach 1: PlatformIO (VSCode)
1. Mo folder nay bang VSCode + PlatformIO
2. `pio run --target upload`

### Cach 2: Firmware co san (tu dong build)
Repo co GitHub Actions **tu build + auto release** `.bin` moi khi push len `main`.
Vao muc **Releases** tai `firmware.bin`, roi nap bang [esptool-js](https://espressif.github.io/esptool-js/) hoac ESP Web Flasher:
```bash
esptool.py --chip esp32c3 --port COMx write_flash 0x0 firmware.bin
```

## API (cho nguoi muon tu viet UI)
| Endpoint | Mo ta |
|----------|-------|
| `GET /api/status` | Trang thai chung (mode, freq, rssi, keys...) |
| `GET /api/key` | Key vua bat duoc |
| `GET /api/keys` | Danh sach key trong EEPROM |
| `GET /api/recv?freq=433.92` | Vao che do nhan, chon tan so |
| `GET /api/save` | Luu key vua bat vao EEPROM |
| `GET /api/send?i=1` | Phat key thu i |
| `GET /api/del?i=1` | Xoa key thu i |
| `GET /api/analyzer?on=1` | Bat/tat phan tich |
| `GET /api/jam?on=1&freq=433.92` | Bat/tat jammer |
| `GET /api/clear` | Xoa toan bo EEPROM |
| `GET /api/autosave?on=1` | Bat/tat tu dong luu |
# ESP-GRABER-Web
