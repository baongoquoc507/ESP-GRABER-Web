#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>

// ===================== PINS (ESP32-C3) =====================
#define CC1101_GDO0 10
#define CC1101_CS   5
#define CC1101_SCK  4
#define CC1101_MOSI 7
#define CC1101_MISO 6

// ===================== WIFI AP =====================
const char* AP_SSID = "quốc bảo";
const char* AP_PASS = "12345678";

// ===================== RF =====================
#define DEFAULT_RF_FREQUENCY 433.92
float frequency = DEFAULT_RF_FREQUENCY;
const float frequencies[] = {315.0, 433.92, 868.0, 915.0};
const int numFrequencies = 4;
int freqIndex = 1;

const float subghz_frequency_list[] = {315.0, 433.92, 868.0, 915.0};
const int subghz_frequency_count = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
int current_scan_index = 0;
const int rssi_threshold = -85;

RCSwitch rcswitch = RCSwitch();
WebServer server(80);

// ===================== STORAGE =====================
#define MAX_DATA_LOG 512
#define MAX_KEY_COUNT 20
#define EEPROM_SIZE 2048

volatile bool recieved = false;

enum emKeys { kUnknown, kP12bt, k12bt, k24bt, k64bt, kKeeLoq, kANmotors64, kPrinceton, kRcSwitch, kStarLine, kCAME, kNICE, kHOLTEK };
enum emMode { MODE_IDLE, MODE_RECV, MODE_ANALYZER };
emMode mode = MODE_IDLE;

struct tpKeyData {
  byte keyID[9];
  int zero[2];
  int one[2];
  int prePulse[2];
  int startPause[2];
  int midlePause[2];
  byte prePulseLenth;
  byte codeLenth;
  byte firstDataIdx;
  emKeys type;
  float frequency;
  int te;
  char rawData[16];
  int bitLength;
  char preset[8];
};

byte maxKeyCount = MAX_KEY_COUNT;
byte EEPROM_key_count;
byte EEPROM_key_index = 0;
unsigned long scanTimer = 0;
bool validKeyReceived = false;
bool readRAW = true;
bool autoSave = false;
int signals = 0;
uint64_t lastSavedKey = 0;
tpKeyData keyData1;
tpKeyData txKey;
float detected_frequency = 0.0;
float last_detected_frequency = 0.0;
bool isJamming = false;

// ===================== HTML UI =====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP-GRABER Web</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--fg:#e6edf3;--mut:#8b949e;--acc:#2f81f7;--ok:#3fb950;--err:#f85149;--warn:#d29922}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;max-width:760px;margin:0 auto;padding:12px}
h1{font-size:1.3rem;margin:8px 0 14px;text-align:center}
.card{background:var(--card);border:1px solid #30363d;border-radius:10px;padding:14px;margin-bottom:12px}
h2{font-size:1rem;color:var(--acc);margin-bottom:10px;text-transform:uppercase;letter-spacing:.5px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
button{background:var(--acc);color:#fff;border:0;border-radius:8px;padding:10px 14px;font-size:.95rem;cursor:pointer;flex:1;min-width:90px}
button:hover{filter:brightness(1.15)}
button.sec{background:#21262d;border:1px solid #30363d}
button.ok{background:var(--ok)} button.danger{background:var(--err)} button.warnb{background:var(--warn);color:#111}
select,input{background:#0d1117;color:var(--fg);border:1px solid #30363d;border-radius:8px;padding:9px;font-size:.95rem;flex:1}
.status{font-family:ui-monospace,Consolas,monospace;font-size:.85rem;color:var(--mut);white-space:pre-wrap;line-height:1.5}
.kv{display:flex;justify-content:space-between;font-size:.9rem;padding:3px 0;border-bottom:1px dashed #21262d}
.kv b{color:var(--fg);font-family:ui-monospace,Consolas,monospace;word-break:break-all;text-align:right;max-width:65%}
.msg{padding:8px 10px;border-radius:8px;font-size:.85rem;margin-top:8px;display:none}
.msg.show{display:block}
.msg.ok{background:#12261e;color:var(--ok)} .msg.err{background:#2a1215;color:var(--err)}
.tabs{display:flex;gap:6px;margin-bottom:12px;position:sticky;top:0;background:var(--bg);padding:6px 0;z-index:5}
.tabs button{flex:1;padding:9px 4px;font-size:.85rem}
.tabs button.active{background:var(--ok)}
.hidden{display:none}
.jam-warn{color:var(--warn);font-size:.8rem;margin-top:6px}
</style>
</head>
<body>
<h1>📡 ESP-GRABER Web</h1>

<div class="tabs">
  <button id="tabRecv" class="active" onclick="showTab('recv')">📥 Nhận</button>
  <button id="tabSend" onclick="showTab('send')">📤 Phát</button>
  <button id="tabAn" onclick="showTab('an')">🔍 Phân tích</button>
  <button id="tabSet" onclick="showTab('set')">⚙️ Cài đặt</button>
</div>

<!-- ============ NHAN ============ -->
<div id="recv" class="card">
  <h2>📥 Nhận tín hiệu (SubGHz-R)</h2>
  <div class="row">
    <select id="recvFreq"></select>
    <button class="sec" onclick="startRecv()">▶ Bắt đầu</button>
  </div>
  <div class="kv"><span>Trạng thái</span><b id="recvState">Đang chờ...</b></div>
  <div class="kv"><span>Tín hiệu đã bắt</span><b id="recvCount">0</b></div>
  <div class="kv"><span>Key vừa nhận</span><b id="recvKey">-</b></div>
  <div class="kv"><span>Loại / Bits</span><b id="recvType">-</b></div>
  <div class="row" style="margin-top:10px">
    <button class="ok" onclick="saveKey()">💾 Lưu key</button>
  </div>
  <div id="recvMsg" class="msg"></div>
</div>

<!-- ============ PHAT ============ -->
<div id="send" class="card hidden">
  <h2>📤 Phát tín hiệu (SubGHz-T)</h2>
  <div class="row">
    <button class="sec" onclick="loadKeys()">🔄 Tải danh sách</button>
  </div>
  <div id="keyList"><div class="status">Chưa có dữ liệu. Nhấn "Tải danh sách".</div></div>
  <div id="sendMsg" class="msg"></div>
</div>

<!-- ============ PHAN TICH ============ -->
<div id="an" class="card hidden">
  <h2>🔍 Phân tích tần số (Analyser)</h2>
  <div class="row">
    <button class="ok" id="anBtn" onclick="toggleAnalyzer()">▶ Bắt đầu quét</button>
  </div>
  <div class="kv"><span>Đang quét</span><b id="anScan">-</b></div>
  <div class="kv"><span>Phát hiện tín hiệu</span><b id="anFound">Không</b></div>
  <div class="kv"><span>RSSI</span><b id="anRssi">-</b></div>
</div>

<!-- ============ JAMMER ============ -->
<div id="jam" class="card hidden">
  <h2>📢 Jammer</h2>
  <div class="row">
    <select id="jamFreq"></select>
    <button class="warnb" id="jamBtn" onclick="toggleJam()">▶ Bật</button>
  </div>
  <div class="kv"><span>Trạng thái</span><b id="jamState">TẮT</b></div>
  <div class="jam-warn">⚠️ Jammer có thể bất hợp pháp ở nhiều quốc gia. Chỉ dùng cho mục đích học tập / thử nghiệm hợp pháp trên thiết bị của bạn.</div>
</div>

<!-- ============ CAI DAT ============ -->
<div id="set" class="card hidden">
  <h2>⚙️ Cài đặt</h2>
  <div class="kv"><span>WiFi AP</span><b>quốc bảo / 12345678</b></div>
  <div class="kv"><span>IP</span><b id="ipAddr">192.168.4.1</b></div>
  <div class="kv"><span>Số key đã lưu</span><b id="keyCount">0</b></div>
  <div class="kv"><span>Auto-save</span><b id="autoSaveState">TẮT</b></div>
  <div class="row" style="margin-top:10px">
    <button class="sec" onclick="toggleAutoSave()">🔄 Bật/Tắt Auto-save</button>
    <button class="danger" onclick="clearEEPROM()">🗑️ Xóa EEPROM</button>
  </div>
  <div id="setMsg" class="msg"></div>
</div>

<script>
const $=id=>document.getElementById(id);
const FREQS=[315.0,433.92,868.0,915.0];
FREQS.forEach(f=>{[$("recvFreq"),$("jamFreq")].forEach(s=>{let o=document.createElement("option");o.value=f;o.textContent=f+" MHz";if(f===433.92)o.selected=true;s.appendChild(o);});});

function showTab(t){
  ["recv","send","an","jam","set"].forEach(x=>$("tab"+x[0].toUpperCase()+x.slice(1)).classList.toggle("hidden",x!==t));
  $("tabRecv").classList.toggle("active",t==="recv");$("tabSend").classList.toggle("active",t==="send");
  $("tabAn").classList.toggle("active",t==="an");$("tabJam").classList.toggle("active",t==="jam");
  $("tabSet").classList.toggle("active",t==="set");
  if(t==="send")loadKeys();
  if(t==="set")pollStatus();
}
function msg(id,t,ok){const e=$(id);e.textContent=t;e.className="msg show "+(ok?"ok":"err");setTimeout(()=>e.classList.remove("show"),4000);}
async function api(u){try{const r=await fetch(u);return await r.json();}catch(e){return{error:String(e)};}}

function startRecv(){api("/api/recv?freq="+$("recvFreq").value).then(pollStatus);$("recvState").textContent="Đang chờ tín hiệu...";$("recvKey").textContent="-";$("recvType").textContent="-";}

async function pollStatus(){
  const s=await api("/api/status");if(s.error)return;
  $("recvCount").textContent=s.signals;
  $("keyCount").textContent=s.keys;
  $("ipAddr").textContent=s.ip;
  $("autoSaveState").textContent=s.autosave?"BẬT":"TẮT";
  $("jamState").textContent=s.jamming?"ĐANG JAM ⚠️":"TẮT";
  $("jamBtn").textContent=s.jamming?"⏹ Dừng":"▶ Bật";
  if(s.mode==="recv"){
    $("recvState").textContent=s.valid?"✅ Đã bắt được key!":"Đang chờ tín hiệu...";
    if(s.valid){$("recvKey").textContent=s.code;$("recvType").textContent=s.type+" / "+s.bits+" bits";}
  }
  if(s.mode==="analyzer"){
    $("anBtn").textContent="⏹ Dừng quét";$("anScan").textContent=s.scanning+" MHz";
    $("anRssi").textContent=s.rssi+" dBm";
    if(s.detected>0){$("anFound").textContent="✅ "+s.detected+" MHz";}
  }else{$("anBtn").textContent="▶ Bắt đầu quét";$("anScan").textContent="-";$("anRssi").textContent="-";if(!s.jamming)$("anFound").textContent="Không";}
}

async function saveKey(){
  const r=await api("/api/save");
  if(r.ok){msg("recvMsg","✅ Đã lưu key vào EEPROM!",true);pollStatus();}
  else msg("recvMsg","❌ "+(r.error||"Không lưu được (trùng key / đầy bộ nhớ / chưa có key)"),false);
}

async function loadKeys(){
  const r=await api("/api/keys");const el=$("keyList");el.innerHTML="";
  if(!r.keys||!r.keys.length){el.innerHTML='<div class="status">EEPROM trống.</div>';return;}
  r.keys.forEach(k=>{
    const d=document.createElement("div");d.className="kv";
    d.innerHTML='<span>#'+k.i+' — '+k.type+'</span><b>'+k.code+'</b>';
    el.appendChild(d);
    const row=document.createElement("div");row.className="row";row.style.margin="6px 0 12px";
    row.innerHTML='<button class="ok" onclick="sendKey('+k.i+')">📤 Phát</button><button class="danger" onclick="delKey('+k.i+')">🗑️ Xóa</button>';
    el.appendChild(row);
  });
}
async function sendKey(i){const r=await api("/api/send?i="+i);msg("sendMsg",r.ok?"✅ Đã phát key #"+i:"❌ "+(r.error||"Lỗi phát"),!!r.ok);}
async function delKey(i){if(!confirm("Xóa key #"+i+"?"))return;const r=await api("/api/del?i="+i);msg("sendMsg",r.ok?"✅ Đã xóa key #"+i:"❌ Lỗi",!!r.ok);loadKeys();}

async function toggleAnalyzer(){
  const s=await api("/api/status");
  if(s.mode==="analyzer"){await api("/api/analyzer?on=0");}
  else{await api("/api/analyzer?on=1");$("anFound").textContent="Không";}
  pollStatus();
}
async function toggleJam(){
  const s=await api("/api/status");
  if(s.jamming){await api("/api/jam?on=0");}
  else{await api("/api/jam?on=1&freq="+$("jamFreq").value);}
  pollStatus();
}
async function toggleAutoSave(){const r=await api("/api/autosave?toggle=1");msg("setMsg",r.ok?"✅ Auto-save: "+(r.on?"BẬT":"TẮT"):"❌ Lỗi",!!r.ok);pollStatus();}
async function clearEEPROM(){
  if(!confirm("Xóa TOÀN BỘ EEPROM?"))return;
  const r=await api("/api/clear");msg("setMsg",r.ok?"✅ Đã xóa EEPROM":"❌ Lỗi",!!r.ok);pollStatus();
}

setInterval(pollStatus,1500);
pollStatus();
</script>
</body>
</html>)rawliteral";

// ===================== UTILS =====================
String getTypeName(emKeys tp) {
  switch (tp) {
    case kUnknown: return F("Unknown");
    case kP12bt: return F("Pre 12bit");
    case k12bt: return F("12bit");
    case k24bt: return F("24bit");
    case k64bt: return F("64bit");
    case kKeeLoq: return F("KeeLoq");
    case kANmotors64: return F("ANmotors");
    case kPrinceton: return F("Princeton");
    case kRcSwitch: return F("RcSwitch");
    case kStarLine: return F("StarLine");
    case kCAME: return F("CAME");
    case kNICE: return F("NICE");
    case kHOLTEK: return F("HOLTEK");
  }
  return "Unknown";
}

String keyCodeHex(tpKeyData* kd) {
  String st = "";
  for (byte i = 0; i < kd->codeLenth >> 3; i++) {
    if (kd->keyID[i] < 0x10) st += "0";
    st += String(kd->keyID[i], HEX);
    if (i < (kd->codeLenth >> 3) - 1) st += ":";
  }
  return st.length() ? st : "00";
}

void myDelayMcs(unsigned long dl) {
  if (dl > 16000) delay(dl / 1000);
  else delayMicroseconds(dl);
}

void setupCC1101() {
  ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
  ELECHOUSE_cc1101.setGDO0(CC1101_GDO0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setRxBW(270.0);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.SetRx();
}

void restoreReceiveMode() {
  if (isJamming) stopJammingInline();
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setRxBW(270.0);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.SetRx();
  rcswitch.disableReceive();
  rcswitch.enableReceive(CC1101_GDO0);
  recieved = false;
}

// ===================== EEPROM =====================
byte indxKeyInROM(tpKeyData* kd) {
  if (!kd || kd->codeLenth == 0) return 0;
  bool eq = true;
  byte* buf = (byte*)kd;
  for (byte j = 1; j <= EEPROM_key_count; j++) {
    eq = true;
    byte i = (kd->type == kKeeLoq || kd->type == kANmotors64) ? 4 : 0;
    for (; i < kd->codeLenth >> 3; i++) {
      byte eepromByte = EEPROM.read(i + j * sizeof(tpKeyData) + 2);
      if (buf[i] != eepromByte) { eq = false; break; }
    }
    if (eq) return j;
  }
  return 0;
}

bool EEPROM_AddKey(tpKeyData* kd) {
  if (!kd || kd->codeLenth == 0 || kd->frequency == 0.0) return false;
  byte indx = indxKeyInROM(kd);
  if (indx != 0) {
    EEPROM_key_index = indx;
    EEPROM.write(1, EEPROM_key_index);
    EEPROM.commit();
    return false;
  }
  if (EEPROM_key_count >= maxKeyCount) return false;
  EEPROM_key_count++;
  EEPROM_key_index = EEPROM_key_count;
  int address = EEPROM_key_index * sizeof(tpKeyData) + 2;
  if (address + sizeof(tpKeyData) > EEPROM_SIZE) { EEPROM_key_count--; return false; }
  for (byte i = 0; i < sizeof(tpKeyData); i++) EEPROM.write(address + i, ((byte*)kd)[i]);
  EEPROM.write(0, EEPROM_key_count);
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();
  return true;
}

void EEPROM_get_key(byte idx, tpKeyData* kd) {
  int address = idx * sizeof(tpKeyData) + 2;
  if (address + sizeof(tpKeyData) > EEPROM_SIZE) return;
  for (byte i = 0; i < sizeof(tpKeyData); i++) ((byte*)kd)[i] = EEPROM.read(address + i);
}

void EEPROM_delete_key(byte idx) {
  if (EEPROM_key_count == 0 || idx < 1 || idx > EEPROM_key_count) return;
  for (byte i = idx; i < EEPROM_key_count; i++) {
    tpKeyData nextKey;
    EEPROM_get_key(i + 1, &nextKey);
    int address = i * sizeof(tpKeyData) + 2;
    for (byte j = 0; j < sizeof(tpKeyData); j++) EEPROM.write(address + j, ((byte*)&nextKey)[j]);
  }
  int lastAddress = EEPROM_key_count * sizeof(tpKeyData) + 2;
  for (byte j = 0; j < sizeof(tpKeyData); j++) EEPROM.write(lastAddress + j, 0);
  EEPROM_key_count--;
  EEPROM.write(0, EEPROM_key_count);
  if (EEPROM_key_count > 0) {
    if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = EEPROM_key_count;
  } else EEPROM_key_index = 0;
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();
}

void EEPROM_clear_all() {
  EEPROM.write(0, 0);
  EEPROM.write(1, 0);
  EEPROM.commit();
  EEPROM_key_count = 0;
  EEPROM_key_index = 0;
  memset(&keyData1, 0, sizeof(tpKeyData));
}

// ===================== RECEIVE =====================
void read_rcswitch(tpKeyData* kd) {
  uint64_t decoded = rcswitch.getReceivedValue();
  if (decoded) {
    Serial.println(F("RcSwitch signal captured"));
    signals++;
    kd->frequency = frequency;
    kd->keyID[0] = decoded & 0xFF;
    kd->keyID[1] = (decoded >> 8) & 0xFF;
    kd->keyID[2] = (decoded >> 16) & 0xFF;
    kd->keyID[3] = (decoded >> 24) & 0xFF;
    kd->type = (rcswitch.getReceivedProtocol() == 1 && rcswitch.getReceivedBitlength() == 24) ? kPrinceton : kRcSwitch;
    if (rcswitch.getReceivedBitlength() <= 40 && rcswitch.getReceivedProtocol() == 11) kd->type = kCAME;
    kd->te = rcswitch.getReceivedDelay();
    kd->bitLength = rcswitch.getReceivedBitlength();
    kd->codeLenth = kd->bitLength;
    strncpy(kd->preset, String(rcswitch.getReceivedProtocol()).c_str(), sizeof(kd->preset) - 1);
    kd->preset[sizeof(kd->preset) - 1] = '\0';
    kd->rawData[0] = '\0';
    validKeyReceived = true;
    Serial.print(F("Key: ")); Serial.println(keyCodeHex(kd));
  }
  rcswitch.resetAvailable();
}

void read_raw(tpKeyData* kd) {
  delay(400);
  unsigned int* raw = rcswitch.getReceivedRawdata();
  uint64_t decoded = rcswitch.getReceivedValue();
  int transitions = 0;
  for (transitions = 0; transitions < MAX_DATA_LOG; transitions++) {
    if (raw[transitions] == 0) break;
  }
  if (transitions > 20) {
    Serial.println(F("Raw signal captured"));
    signals++;
    kd->frequency = frequency;
    kd->type = kUnknown;
    kd->te = 0;
    kd->bitLength = 0;
    strncpy(kd->preset, "0", sizeof(kd->preset) - 1);
    kd->preset[sizeof(kd->preset) - 1] = '\0';
    kd->codeLenth = transitions;
    if (decoded) {
      kd->keyID[0] = decoded & 0xFF;
      kd->keyID[1] = (decoded >> 8) & 0xFF;
      kd->keyID[2] = (decoded >> 16) & 0xFF;
      kd->keyID[3] = (decoded >> 24) & 0xFF;
      kd->type = (rcswitch.getReceivedProtocol() == 1 && rcswitch.getReceivedBitlength() == 24) ? kPrinceton : kRcSwitch;
      if (rcswitch.getReceivedBitlength() <= 40 && rcswitch.getReceivedProtocol() == 11) kd->type = kCAME;
      kd->te = rcswitch.getReceivedDelay();
      kd->bitLength = rcswitch.getReceivedBitlength();
      kd->codeLenth = kd->bitLength;
      strncpy(kd->preset, String(rcswitch.getReceivedProtocol()).c_str(), sizeof(kd->preset) - 1);
      kd->preset[sizeof(kd->preset) - 1] = '\0';
    } else {
      if (transitions >= 129 && transitions <= 137) kd->type = kStarLine;
      else if (transitions >= 133 && transitions <= 137) kd->type = kKeeLoq;
      else if (transitions >= 40 && transitions <= 60) kd->type = kCAME;
    }
    validKeyReceived = true;
    Serial.print(F("Key: ")); Serial.println(keyCodeHex(kd));
  }
  rcswitch.resetAvailable();
}

// ===================== TRANSMIT =====================
void sendSynthKey(tpKeyData* kd) {
  recieved = true;
  Serial.print(F("TX type: ")); Serial.print(getTypeName(kd->type));
  Serial.print(F(" ID: ")); Serial.println(keyCodeHex(kd));

  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(kd->frequency);
  ELECHOUSE_cc1101.setRxBW(270.0);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.SetTx();
  pinMode(CC1101_GDO0, OUTPUT);

  if (kd->type == kRcSwitch || kd->type == kPrinceton || kd->type == kCAME || kd->type == kNICE || kd->type == kHOLTEK) {
    uint64_t data = 0;
    for (int i = 0; i < kd->bitLength / 8; i++) data |= ((uint64_t)kd->keyID[i] << (i * 8));
    int protocol = atoi(kd->preset);
    if (kd->type == kCAME || kd->type == kNICE || kd->type == kHOLTEK) { protocol = 11; kd->te = 270; }
    else if (kd->type == kPrinceton) { protocol = 1; kd->te = 350; }
    rcswitch.enableTransmit(CC1101_GDO0);
    rcswitch.setProtocol(protocol);
    if (kd->te) rcswitch.setPulseLength(kd->te);
    rcswitch.setRepeatTransmit(10);
    rcswitch.send(data, kd->bitLength);
    rcswitch.disableTransmit();
  } else {
    String data = String(kd->rawData);
    if (data.length() > 3) {
      int buff_size = 0;
      int index = 0;
      while (index >= 0) { index = data.indexOf(' ', index + 1); buff_size++; }
      int* transmittimings = (int*)calloc(sizeof(int), buff_size + 1);
      int startIndex = 0;
      index = 0;
      for (size_t i = 0; i < (size_t)buff_size; i++) {
        index = data.indexOf(' ', startIndex);
        if (index == -1) transmittimings[i] = data.substring(startIndex).toInt();
        else transmittimings[i] = data.substring(startIndex, index).toInt();
        startIndex = index + 1;
      }
      transmittimings[buff_size] = 0;
      for (int nRepeat = 0; nRepeat < 2; nRepeat++) {
        unsigned int currenttiming = 0;
        bool currentlogiclevel = true;
        while (transmittimings[currenttiming]) {
          if (transmittimings[currenttiming] >= 0) currentlogiclevel = true;
          else { currentlogiclevel = false; transmittimings[currenttiming] = (-1) * transmittimings[currenttiming]; }
          digitalWrite(CC1101_GDO0, currentlogiclevel ? HIGH : LOW);
          myDelayMcs(transmittimings[currenttiming]);
          currenttiming++;
        }
        digitalWrite(CC1101_GDO0, LOW);
      }
      free(transmittimings);
    }
  }

  ELECHOUSE_cc1101.SetRx();
  recieved = false;
  Serial.println(F("TX done"));
}

// ===================== JAMMER =====================
void startJamming() {
  Serial.println(F("Jammer ON"));
  isJamming = true;
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(0);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.setRxBW(270.0);
  ELECHOUSE_cc1101.SetTx();
  ELECHOUSE_cc1101.SpiWriteReg(0x3E, 0xFF);
  ELECHOUSE_cc1101.SpiWriteReg(0x35, 0x60);
}

void stopJammingInline() {
  isJamming = false;
}

void stopJamming() {
  Serial.println(F("Jammer OFF"));
  isJamming = false;
  ELECHOUSE_cc1101.SpiWriteReg(0x35, 0x00);
  restoreReceiveMode();
}

// ===================== HTTP API =====================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void sendJson(const String& json) {
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

String modeName() {
  switch (mode) {
    case MODE_RECV: return "recv";
    case MODE_ANALYZER: return "analyzer";
    default: return "idle";
  }
}

void handleStatus() {
  String json = "{";
  json += "\"mode\":\"" + modeName() + "\",";
  json += "\"freq\":" + String(frequency, 2) + ",";
  json += "\"signals\":" + String(signals) + ",";
  json += "\"keys\":" + String(EEPROM_key_count) + ",";
  json += "\"autosave\":" + String(autoSave ? "true" : "false") + ",";
  json += "\"jamming\":" + String(isJamming ? "true" : "false") + ",";
  json += "\"valid\":" + String(validKeyReceived ? "true" : "false") + ",";
  json += "\"code\":\"" + keyCodeHex(&keyData1) + "\",";
  json += "\"type\":\"" + getTypeName(keyData1.type) + "\",";
  json += "\"bits\":" + String(keyData1.bitLength) + ",";
  int rssi = ELECHOUSE_cc1101.getRssi();
  json += "\"rssi\":" + String(rssi) + ",";
  json += "\"scanning\":" + String(subghz_frequency_list[current_scan_index], 2) + ",";
  json += "\"detected\":" + String(last_detected_frequency, 2) + ",";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\"";
  json += "}";
  sendJson(json);
}

void handleRecv() {
  if (server.hasArg("freq")) {
    float f = server.arg("freq").toFloat();
    for (int i = 0; i < numFrequencies; i++) {
      if (abs(f - frequencies[i]) < 0.01) { freqIndex = i; frequency = f; break; }
    }
  }
  stopJamming();
  setupCC1101();
  rcswitch.disableReceive();
  rcswitch.enableReceive(CC1101_GDO0);
  validKeyReceived = false;
  signals = 0;
  memset(&keyData1, 0, sizeof(tpKeyData));
  lastSavedKey = 0;
  rcswitch.resetAvailable();
  mode = MODE_RECV;
  sendJson("{\"ok\":true}");
}

void handleSave() {
  if (!validKeyReceived) { sendJson("{\"ok\":false,\"error\":\"Chua co key\"}"); return; }
  if (EEPROM_AddKey(&keyData1)) {
    validKeyReceived = false;
    signals = 0;
    memset(&keyData1, 0, sizeof(tpKeyData));
    lastSavedKey = 0;
    rcswitch.resetAvailable();
    sendJson("{\"ok\":true}");
  } else {
    if (indxKeyInROM(&keyData1) != 0) sendJson("{\"ok\":false,\"error\":\"Key da ton tai\"}");
    else if (EEPROM_key_count >= maxKeyCount) sendJson("{\"ok\":false,\"error\":\"EEPROM day (20 keys)\"}");
    else sendJson("{\"ok\":false,\"error\":\"Loi luu key\"}");
  }
}

void handleKeys() {
  String json = "{\"keys\":[";
  for (byte j = 1; j <= EEPROM_key_count; j++) {
    tpKeyData kd;
    EEPROM_get_key(j, &kd);
    if (j > 1) json += ",";
    json += "{\"i\":" + String(j) + ",\"code\":\"" + keyCodeHex(&kd) + "\",\"type\":\"" + getTypeName(kd.type) + "\",\"freq\":" + String(kd.frequency, 2) + ",\"bits\":" + String(kd.bitLength) + "}";
  }
  json += "]}";
  sendJson(json);
}

void handleSend() {
  if (!server.hasArg("i")) { sendJson("{\"ok\":false,\"error\":\"Thieu tham so\"}"); return; }
  byte idx = server.arg("i").toInt();
  if (idx < 1 || idx > EEPROM_key_count) { sendJson("{\"ok\":false,\"error\":\"Key khong ton tai\"}"); return; }
  memset(&txKey, 0, sizeof(tpKeyData));
  EEPROM_get_key(idx, &txKey);
  emMode prevMode = mode;
  mode = MODE_IDLE;
  sendSynthKey(&txKey);
  restoreReceiveMode();
  if (prevMode == MODE_RECV) mode = MODE_RECV;
  sendJson("{\"ok\":true}");
}

void handleDel() {
  if (!server.hasArg("i")) { sendJson("{\"ok\":false}"); return; }
  byte idx = server.arg("i").toInt();
  if (idx < 1 || idx > EEPROM_key_count) { sendJson("{\"ok\":false,\"error\":\"Key khong ton tai\"}"); return; }
  EEPROM_delete_key(idx);
  sendJson("{\"ok\":true}");
}

void handleAnalyzer() {
  if (server.hasArg("on") && server.arg("on") == "1") {
    stopJamming();
    setupCC1101();
    rcswitch.disableReceive();
    current_scan_index = 0;
    detected_frequency = 0.0;
    last_detected_frequency = 0.0;
    scanTimer = millis();
    mode = MODE_ANALYZER;
    sendJson("{\"ok\":true}");
  } else {
    if (mode == MODE_ANALYZER) mode = MODE_IDLE;
    restoreReceiveMode();
    sendJson("{\"ok\":true}");
  }
}

void handleJam() {
  if (server.hasArg("on") && server.arg("on") == "1") {
    if (server.hasArg("freq")) {
      float f = server.arg("freq").toFloat();
      for (int i = 0; i < numFrequencies; i++) {
        if (abs(f - frequencies[i]) < 0.01) { freqIndex = i; frequency = f; break; }
      }
    }
    mode = MODE_IDLE;
    rcswitch.disableReceive();
    startJamming();
    sendJson("{\"ok\":true}");
  } else {
    stopJamming();
    sendJson("{\"ok\":true}");
  }
}

void handleAutoSave() {
  autoSave = !autoSave;
  sendJson(String("{\"ok\":true,\"on\":") + (autoSave ? "true}" : "false}"));
}

void handleClear() {
  EEPROM_clear_all();
  sendJson("{\"ok\":true}");
}

void handleNotFound() { server.send(404, "text/plain", "404 Not Found"); }

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP-GRABER Web starting..."));

  EEPROM.begin(EEPROM_SIZE);
  byte read_count = EEPROM.read(0);
  EEPROM_key_count = (read_count <= MAX_KEY_COUNT) ? read_count : 0;
  EEPROM_key_index = EEPROM.read(1);
  if (EEPROM_key_count > 0 && EEPROM_key_index <= EEPROM_key_count && EEPROM_key_index > 0) {
    EEPROM_get_key(EEPROM_key_index, &keyData1);
  } else {
    EEPROM_key_count = 0;
    EEPROM_key_index = 0;
    memset(&keyData1, 0, sizeof(tpKeyData));
  }

  setupCC1101();
  rcswitch.enableReceive(CC1101_GDO0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("AP IP: "));
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/recv", HTTP_GET, handleRecv);
  server.on("/api/save", HTTP_GET, handleSave);
  server.on("/api/keys", HTTP_GET, handleKeys);
  server.on("/api/send", HTTP_GET, handleSend);
  server.on("/api/del", HTTP_GET, handleDel);
  server.on("/api/analyzer", HTTP_GET, handleAnalyzer);
  server.on("/api/jam", HTTP_GET, handleJam);
  server.on("/api/autosave", HTTP_GET, handleAutoSave);
  server.on("/api/clear", HTTP_GET, handleClear);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("Web server started"));
}

void loop() {
  server.handleClient();

  if (mode == MODE_RECV) {
    if (rcswitch.available()) {
      if (!readRAW) read_rcswitch(&keyData1);
      else read_raw(&keyData1);
      if (validKeyReceived && autoSave && (lastSavedKey != keyData1.keyID[0] || keyData1.keyID[0] == 0)) {
        if (EEPROM_AddKey(&keyData1)) {
          lastSavedKey = keyData1.keyID[0];
          validKeyReceived = false;
          signals = 0;
          memset(&keyData1, 0, sizeof(tpKeyData));
          rcswitch.resetAvailable();
        }
      }
    }
  } else if (mode == MODE_ANALYZER) {
    if (millis() - scanTimer >= 250) {
      int rssi = ELECHOUSE_cc1101.getRssi();
      if (rssi >= rssi_threshold) {
        detected_frequency = subghz_frequency_list[current_scan_index];
        if (detected_frequency != last_detected_frequency) {
          last_detected_frequency = detected_frequency;
          Serial.print(F("Signal at ")); Serial.print(last_detected_frequency); Serial.println(F(" MHz"));
        }
      } else {
        detected_frequency = 0.0;
      }
      current_scan_index = (current_scan_index + 1) % subghz_frequency_count;
      ELECHOUSE_cc1101.setMHZ(subghz_frequency_list[current_scan_index]);
      ELECHOUSE_cc1101.SetRx();
      delayMicroseconds(3500);
      scanTimer = millis();
    }
  }
}
