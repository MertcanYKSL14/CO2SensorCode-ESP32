
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include "Adafruit_CCS811.h"


const char* WIFI_SSID = "Sms Wifi 2.4";         
const char* WIFI_PASS = "SerdarMakinaWifi14*-";
const char* HOSTNAME  = "hava";            
const char* AP_SSID   = "HavaKalitesi";     
const char* AP_PASS   = "12345678";        
const char* WEB_PASS  = "";                  

constexpr uint8_t  SDA_PIN  = 21;
constexpr uint8_t  SCL_PIN  = 22;
constexpr uint8_t  CCS_ADDR = 0x5A;          
constexpr uint32_t I2C_HZ   = 50000;         

#define FAN_ENABLED true                     
constexpr uint8_t  FAN_PIN           = 26;
constexpr bool     RELAY_ACTIVE_LOW  = true;       
constexpr uint16_t FAN_ON_ECO2       = 1000;       
constexpr uint16_t FAN_OFF_ECO2      = 700;        
constexpr uint16_t FAN_ON_TVOC       = 660;        
constexpr uint16_t FAN_OFF_TVOC      = 220;        
constexpr uint32_t FAN_MIN_STATE_MS  = 30000;      
constexpr bool     FAN_ON_IF_SENSOR_FAIL = true;   

constexpr uint32_t WARMUP_MS        = 20UL * 60UL * 1000UL;  
constexpr uint8_t  AVG_WINDOW       = 30;                    
constexpr uint16_t HIST_SIZE        = 120;                  
constexpr uint32_t HIST_INTERVAL_MS = 15000;                 
constexpr uint32_t CPU_MHZ          = 160;                   
constexpr bool     WIFI_MODEM_SLEEP = false;                 


constexpr uint32_t WDT_TIMEOUT_S         = 30;                   
constexpr uint32_t WIFI_RESTART_AFTER_MS = 3UL * 60UL * 1000UL;   
constexpr uint32_t AP_RETRY_AFTER_MS     = 5UL * 60UL * 1000UL;   
constexpr uint32_t MDNS_REFRESH_MS       = 15UL * 60UL * 1000UL;  
constexpr uint32_t MIN_FREE_HEAP         = 15000;                 



const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0b0e14">
<title>Hava Kalitesi Monitörü</title>
<style>
:root{--bg:#0b0e14;--card:#151922;--card2:#1c212c;--line:#262c3a;--tx:#e8ecf3;--mut:#8b94a7;--acc:#4fc3f7}
@media (prefers-color-scheme:light){:root{--bg:#f1f4f9;--card:#fff;--card2:#eef1f6;--line:#dde2ec;--tx:#1a2030;--mut:#5d6679;--acc:#0288d1}}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--tx);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif}
.wrap{max-width:780px;margin:0 auto}
header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;gap:10px}
h1{font-size:20px;margin:0}
.live{display:flex;align-items:center;gap:8px;color:var(--mut);font-size:13px;white-space:nowrap}
.dot{width:10px;height:10px;border-radius:50%;background:#ef4444}
.dot.on{background:#22c55e;box-shadow:0 0 0 4px rgba(34,197,94,.2)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:12px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:16px;text-align:center}
.lbl{color:var(--mut);font-size:12px;letter-spacing:.06em;text-transform:uppercase}
.big{font-size:54px;font-weight:700;line-height:1.1;margin:6px 0}
.unit{font-size:16px;color:var(--mut);font-weight:400;margin-left:4px}
.chip{display:inline-block;padding:5px 16px;border-radius:999px;font-weight:600;font-size:14px;color:#fff;background:#555}
.sub{color:var(--mut);font-size:12.5px;margin-top:10px}
table{width:100%;border-collapse:separate;border-spacing:0 4px;margin-top:10px;font-size:14px}
td{padding:7px 10px;background:var(--card2);color:var(--mut)}
td:first-child{border-radius:8px 0 0 8px;font-weight:600;color:var(--tx);width:45%;text-align:left}
td:last-child{border-radius:0 8px 8px 0;text-align:right}
tr.act td{color:var(--tx);outline:2px solid var(--acc);outline-offset:-2px}
.nokta{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:8px}
.banner{display:none;margin-bottom:12px;padding:10px 14px;border-radius:12px;font-size:14px;text-align:center}
.banner.warn{display:block;background:rgba(234,179,8,.15);border:1px solid #ca8a04}
.banner.err{display:block;background:rgba(239,68,68,.15);border:1px solid #dc2626}
.tabs{display:flex;gap:6px;justify-content:center;margin-bottom:10px}
button{font:inherit;color:var(--mut);background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:6px 14px;cursor:pointer}
button.sel{color:#fff;background:var(--acc);border-color:var(--acc)}
canvas{display:block;width:100%;height:210px}
.seg{display:flex;gap:6px;justify-content:center;margin-top:12px}
.note,.foot{color:var(--mut);font-size:12px;text-align:center;margin-top:10px}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>Hava Kalitesi Monitörü</h1>
    <div class="live"><span class="dot" id="dot"></span><span id="liveTxt">Bağlanıyor…</span></div>
  </header>

  <div class="banner" id="banner"></div>

  <div class="grid">
    <div class="card">
      <div class="lbl">eCO₂</div>
      <div class="big"><span id="eco2">--</span><span class="unit">ppm</span></div>
      <span class="chip" id="eChip">—</span>
      <div class="sub" id="eSub">30 sn ortalama</div>
      <table id="eTab"></table>
    </div>
    <div class="card">
      <div class="lbl">TVOC</div>
      <div class="big"><span id="tvoc">--</span><span class="unit">ppb</span></div>
      <span class="chip" id="tChip">—</span>
      <div class="sub" id="tSub">30 sn ortalama</div>
      <table id="tTab"></table>
    </div>
  </div>

  <div class="card" id="fanCard" style="display:none;margin-bottom:12px">
    <div class="lbl">Fan</div>
    <div style="margin-top:8px"><span class="chip" id="fanChip">—</span></div>
    <div class="seg">
      <button id="m_auto" onclick="fanMode('auto')">Otomatik</button>
      <button id="m_on" onclick="fanMode('on')">Açık</button>
      <button id="m_off" onclick="fanMode('off')">Kapalı</button>
    </div>
  </div>

  <div class="card" style="margin-bottom:12px">
    <div class="tabs">
      <button id="tab_e" class="sel" onclick="setTab('e')">eCO₂</button>
      <button id="tab_t" onclick="setTab('t')">TVOC</button>
    </div>
    <canvas id="cv"></canvas>
  </div>

  <div class="foot" id="foot"></div>
</div>

<script>
const $=id=>document.getElementById(id);
// [alt sinir, ust sinir, etiket, renk, gosterilecek aralik]
const ECO=[[0,800,'İyi','#22c55e','400 – 800'],
           [800,1000,'Orta','#ca8a04','800 – 1000'],
           [1000,1500,'Kötü','#f97316','1000 – 1500'],
           [1500,1e9,'Çok kötü','#ef4444','1500 ve üzeri']];
const TV=[[0,65,'Çok iyi','#16a34a','0 – 65'],
          [65,220,'İyi','#22c55e','65 – 220'],
          [220,660,'Orta','#ca8a04','220 – 660'],
          [660,1e9,'Kötü','#ef4444','660 ve üzeri']];
let D=null,tab='e';

const lvl=(t,v)=>{for(let i=0;i<t.length;i++)if(v>=t[i][0]&&v<t[i][1])return i;return t.length-1;};
function tablo(id,t,v){
  const a=lvl(t,v);
  $(id).innerHTML=t.map((r,i)=>`<tr${i===a?' class="act"':''}><td><span class="nokta" style="background:${r[3]}"></span>${r[2]}</td><td>${r[4]}</td></tr>`).join('');
}
function chip(id,t,v){const r=t[lvl(t,v)],e=$(id);e.textContent=r[2];e.style.background=r[3];}
function sure(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h>0?`${h} sa ${m} dk`:`${m} dk`;}
function sinyal(r){return r>-60?'Çok iyi':r>-70?'İyi':r>-80?'Orta':'Zayıf';}

function ciz(){
  if(!D)return;
  const cv=$('cv'),dpr=window.devicePixelRatio||1,W=cv.clientWidth,H=cv.clientHeight;
  cv.width=Math.round(W*dpr);cv.height=Math.round(H*dpr);
  const x=cv.getContext('2d');x.scale(dpr,dpr);x.clearRect(0,0,W,H);
  const data=tab==='e'?D.hE:D.hT,T=tab==='e'?ECO:TV;
  const cs=getComputedStyle(document.body);
  const mut=cs.getPropertyValue('--mut').trim(),acc=cs.getPropertyValue('--acc').trim();
  const pl=42,pr=8,pt=8,pb=20;
  x.font='11px system-ui,Arial';x.fillStyle=mut;
  if(data.length<2){x.fillText('Veri toplanıyor…',pl,H/2);return;}
  const lo=tab==='e'?400:0;
  let hi=Math.max(...data)*1.1;
  hi=Math.ceil(Math.max(hi,tab==='e'?1000:220)/50)*50;
  const Y=v=>pt+(H-pt-pb)*(1-(v-lo)/(hi-lo));
  const X=i=>pl+i*(W-pl-pr)/(data.length-1);
  // durum bantlari
  T.forEach(r=>{
    const a=Math.max(r[0],lo),b=Math.min(r[1],hi);
    if(a<b){x.globalAlpha=.13;x.fillStyle=r[3];x.fillRect(pl,Y(b),W-pl-pr,Y(a)-Y(b));}
  });
  x.globalAlpha=1;
  // yatay cizgiler ve etiketler
  x.strokeStyle=cs.getPropertyValue('--line').trim();x.lineWidth=1;x.fillStyle=mut;
  for(let k=0;k<=4;k++){
    const v=lo+(hi-lo)*k/4,y=Y(v);
    x.beginPath();x.moveTo(pl,y);x.lineTo(W-pr,y);x.stroke();
    x.fillText(Math.round(v),4,y+4);
  }
  x.fillText(`-${Math.round(data.length*D.interval/60)} dk`,pl,H-5);
  x.textAlign='right';x.fillText('şimdi',W-pr,H-5);x.textAlign='left';
  // cizgi
  x.strokeStyle=acc;x.lineWidth=2;x.lineJoin='round';x.beginPath();
  data.forEach((v,i)=>{i?x.lineTo(X(i),Y(v)):x.moveTo(X(i),Y(v));});
  x.stroke();
}
function setTab(t){
  tab=t;
  $('tab_e').classList.toggle('sel',t==='e');
  $('tab_t').classList.toggle('sel',t==='t');
  ciz();
}
async function fanMode(m){
  try{await fetch('/fan?mode='+m,{cache:'no-store'});}catch(e){}
  guncelle();
}
function ekranaYaz(){
  $('eco2').textContent=D.eco2;$('tvoc').textContent=D.tvoc;
  chip('eChip',ECO,D.eco2);chip('tChip',TV,D.tvoc);
  tablo('eTab',ECO,D.eco2);tablo('tTab',TV,D.tvoc);
  $('eSub').textContent=`30 sn ort. · Anlık ${D.eco2Raw} · Min ${D.eco2Min} · Maks ${D.eco2Max}`;
  $('tSub').textContent=`30 sn ort. · Anlık ${D.tvocRaw} · Min ${D.tvocMin} · Maks ${D.tvocMax}`;
  const b=$('banner');
  if(!D.online){
    b.className='banner err';
    b.textContent='Sensörden veri alınamıyor. Bağlantıları kontrol edin (WAK → GND, SDA/SCL, 3V3).';
  }else if(D.warm>0){
    b.className='banner warn';
    b.textContent=`Sensör ısınıyor (kalan ≈ ${Math.ceil(D.warm/60)} dk). Bu sürede değerler kararsız olabilir.`;
  }else b.className='banner';
  $('fanCard').style.display=D.fanEnabled?'block':'none';
  if(D.fanEnabled){
    const c=$('fanChip');c.textContent=D.fan?'AÇIK':'KAPALI';c.style.background=D.fan?'#22c55e':'#64748b';
    ['auto','on','off'].forEach(m=>$('m_'+m).classList.toggle('sel',D.fanMode===m));
  }
  const wifi=D.ap?'Erişim noktası modu':`Wi-Fi: ${sinyal(D.rssi)} (${D.rssi} dBm)`;
  $('foot').textContent=`${wifi} · Çalışma: ${sure(D.uptime)} · Güncelleme: ${new Date().toLocaleTimeString('tr-TR')}`;
  $('dot').classList.add('on');$('liveTxt').textContent='Canlı';
  ciz();
}
async function guncelle(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    if(!r.ok)throw new Error(r.status);
    D=await r.json();
    ekranaYaz();
  }catch(e){
    $('dot').classList.remove('on');$('liveTxt').textContent='Bağlantı yok';
  }
}
window.addEventListener('resize',ciz);
guncelle();setInterval(guncelle,2000);
</script>
</body>
</html>
)rawliteral";


Adafruit_CCS811 ccs;
WebServer server(80);


struct Rolling {
  uint16_t buf[AVG_WINDOW];
  uint8_t  n = 0, idx = 0;
  uint32_t sum = 0;
  void add(uint16_t v) {
    if (n < AVG_WINDOW) { buf[idx] = v; sum += v; n++; }
    else { sum -= buf[idx]; buf[idx] = v; sum += v; }
    idx = (idx + 1) % AVG_WINDOW;
  }
  uint16_t avg() const { return n ? (uint16_t)(sum / n) : 0; }
  bool full() const { return n >= AVG_WINDOW; }
};

Rolling rollE, rollT;
uint16_t eco2Raw = 400, tvocRaw = 0, eco2Avg = 400, tvocAvg = 0;
uint16_t eco2Min = 65535, eco2Max = 0, tvocMin = 65535, tvocMax = 0;

uint16_t histE[HIST_SIZE], histT[HIST_SIZE];
uint16_t histHead = 0, histCount = 0;

bool     sensorBegun = false;
uint32_t lastGoodRead = 0, lastInitTry = 0, lastPoll = 0, lastHist = 0, lastWifiCheck = 0;

enum FanModeSel : uint8_t { MODE_AUTO = 0, MODE_ON = 1, MODE_OFF = 2 };
FanModeSel fanMode = MODE_AUTO;
bool     fanOn = false;
uint32_t fanChangedAt = 0;

bool apMode = false;


volatile bool mdnsRestartPending = false;
uint32_t wifiDownSince = 0, apSince = 0, lastMdnsRefresh = 0, lastHeapLog = 0;


bool sensorOnline() {
  return sensorBegun && lastGoodRead != 0 && (millis() - lastGoodRead) < 10000UL;
}

bool warmingUp() { return millis() < WARMUP_MS; }

bool initSensor() {
  lastInitTry = millis();
  sensorBegun = ccs.begin(CCS_ADDR);
  Serial.println(sensorBegun ? "CCS811 baslatildi."
                             : "UYARI: CCS811 bulunamadi! WAK->GND, SDA/SCL ve adresi kontrol et.");
  return sensorBegun;
}

void addHistory(uint16_t e, uint16_t t) {
  histE[histHead] = e;
  histT[histHead] = t;
  histHead = (histHead + 1) % HIST_SIZE;
  if (histCount < HIST_SIZE) histCount++;
}

void pollSensor() {
  if (millis() - lastPoll < 500UL) return;
  lastPoll = millis();

  if (sensorBegun && ccs.available()) {
    if (ccs.readData() == 0) {
      uint16_t e = ccs.geteCO2();
      uint16_t t = ccs.getTVOC();
      if (e <= 8192 && t <= 1187) {         
        eco2Raw = e; tvocRaw = t;
        rollE.add(e); rollT.add(t);
        eco2Avg = rollE.avg(); tvocAvg = rollT.avg();
        lastGoodRead = millis();
        if (rollE.full()) {
          if (eco2Avg < eco2Min) eco2Min = eco2Avg;
          if (eco2Avg > eco2Max) eco2Max = eco2Avg;
          if (tvocAvg < tvocMin) tvocMin = tvocAvg;
          if (tvocAvg > tvocMax) tvocMax = tvocAvg;
        }
        Serial.printf("eCO2: %u ppm | TVOC: %u ppb\n", e, t);
      }
    }
  }


  if (!sensorOnline() && (millis() - lastInitTry) >= 30000UL) initSensor();
}


void setFanOutput(bool on) {
  fanOn = on;
  fanChangedAt = millis();
#if FAN_ENABLED
  digitalWrite(FAN_PIN, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
#endif
}

void updateFan() {
#if FAN_ENABLED
  bool want = fanOn;
  if (fanMode == MODE_ON)       want = true;
  else if (fanMode == MODE_OFF) want = false;
  else if (!sensorOnline())     want = FAN_ON_IF_SENSOR_FAIL;
  else if (!fanOn && (eco2Avg >= FAN_ON_ECO2 || tvocAvg >= FAN_ON_TVOC)) want = true;
  else if (fanOn && eco2Avg <= FAN_OFF_ECO2 && tvocAvg <= FAN_OFF_TVOC)  want = false;

  bool manual = (fanMode != MODE_AUTO);
  if (want != fanOn && (manual || (millis() - fanChangedAt) >= FAN_MIN_STATE_MS)) {
    setFanOutput(want);
    Serial.printf("Fan: %s\n", fanOn ? "ACIK" : "KAPALI");
  }
#endif
}

const char* fanModeName() {
  switch (fanMode) {
    case MODE_ON:  return "on";
    case MODE_OFF: return "off";
    default:       return "auto";
  }
}

bool yetkili() {
  if (WEB_PASS[0] == '\0') return true;
  if (server.authenticate("admin", WEB_PASS)) return true;
  server.requestAuthentication();
  return false;
}

void appendHistory(String& s, const uint16_t* h) {
  uint16_t start = (histHead + HIST_SIZE - histCount) % HIST_SIZE;   
  for (uint16_t i = 0; i < histCount; i++) {
    if (i) s += ',';
    s += String(h[(start + i) % HIST_SIZE]);
  }
}

void handleRoot() {
  if (!yetkili()) return;
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

void handleData() {
  if (!yetkili()) return;

  unsigned long warm = warmingUp() ? (WARMUP_MS - millis()) / 1000UL : 0;
  char head[480];
  snprintf(head, sizeof(head),
    "{\"eco2\":%u,\"eco2Raw\":%u,\"tvoc\":%u,\"tvocRaw\":%u,"
    "\"eco2Min\":%u,\"eco2Max\":%u,\"tvocMin\":%u,\"tvocMax\":%u,"
    "\"online\":%s,\"warm\":%lu,\"fanEnabled\":%s,\"fan\":%s,\"fanMode\":\"%s\","
    "\"rssi\":%d,\"uptime\":%lu,\"ap\":%s,\"interval\":%lu,\"hE\":[",
    (unsigned)eco2Avg, (unsigned)eco2Raw, (unsigned)tvocAvg, (unsigned)tvocRaw,
    (unsigned)(eco2Min == 65535 ? eco2Avg : eco2Min), (unsigned)(eco2Max == 0 ? eco2Avg : eco2Max),
    (unsigned)(tvocMin == 65535 ? tvocAvg : tvocMin), (unsigned)tvocMax,
    sensorOnline() ? "true" : "false", warm,
    FAN_ENABLED ? "true" : "false", fanOn ? "true" : "false", fanModeName(),
    apMode ? 0 : (int)WiFi.RSSI(), (unsigned long)(millis() / 1000UL),
    apMode ? "true" : "false", (unsigned long)(HIST_INTERVAL_MS / 1000UL));

  String json;
  json.reserve(1800);
  json += head;
  appendHistory(json, histE);
  json += "],\"hT\":[";
  appendHistory(json, histT);
  json += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleFan() {
  if (!yetkili()) return;
#if FAN_ENABLED
  String m = server.arg("mode");
  if      (m == "auto") fanMode = MODE_AUTO;
  else if (m == "on")   fanMode = MODE_ON;
  else if (m == "off")  fanMode = MODE_OFF;
  else { server.send(400, "application/json", "{\"ok\":false}"); return; }
  updateFan();
  server.send(200, "application/json", "{\"ok\":true}");
#else
  server.send(404, "application/json", "{\"ok\":false}");
#endif
}

void handleNotFound() {
  server.send(404, "text/plain", "Sayfa bulunamadi");
}


void startMdns() {
  MDNS.end();
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\n", HOSTNAME);
  } else {
    Serial.println("mDNS baslatilamadi!");
  }
  lastMdnsRefresh = millis();
}

void onWifiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) mdnsRestartPending = true;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  apMode = true;
  apSince = millis();
  Serial.printf("WiFi'ye baglanilamadi. Kendi agim acildi: '%s' (sifre: %s)\n", AP_SSID, AP_PASS);
  Serial.print("Adres: http://");
  Serial.println(WiFi.softAPIP());
}

void connectWifi() {
  WiFi.persistent(false);      
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(WIFI_MODEM_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi'ye baglaniliyor");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000UL) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Baglandi. Adres: http://");
    Serial.println(WiFi.localIP());
  } else {
    startAccessPoint();
  }
}

void startWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms     = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic  = true;
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   
}

void healthCheck() {
  uint32_t now = millis();


  if (now - lastHeapLog > 60000UL) {
    lastHeapLog = now;
    uint32_t freeHeap = ESP.getFreeHeap();
    Serial.printf("Bos heap: %u (min: %u)\n", (unsigned)freeHeap, (unsigned)ESP.getMinFreeHeap());
    if (freeHeap < MIN_FREE_HEAP) {
      Serial.println("Bellek az, yeniden baslatiliyor");
      delay(100);
      ESP.restart();
    }
  }

  if (mdnsRestartPending && !apMode) {
    mdnsRestartPending = false;
    startMdns();
  }


  if (apMode) {
    if (!apSince) apSince = now;
    if (WiFi.softAPgetStationNum() == 0 && now - apSince > AP_RETRY_AFTER_MS) {
      Serial.println("AP modunda uzun sure kalindi, ana ag icin yeniden baslatiliyor");
      delay(100);
      ESP.restart();
    }
    return;
  }


  if (WiFi.status() == WL_CONNECTED) {
    wifiDownSince = 0;
    if (now - lastMdnsRefresh > MDNS_REFRESH_MS) startMdns();   
    return;
  }

  if (!wifiDownSince) wifiDownSince = now;
  uint32_t down = now - wifiDownSince;

  if (now - lastWifiCheck > 15000UL) {
    lastWifiCheck = now;
    Serial.printf("WiFi yok (%lu sn)...\n", (unsigned long)(down / 1000));
    if (down > 60000UL) { WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); }
    else                { WiFi.reconnect(); }
  }

  if (down > WIFI_RESTART_AFTER_MS) {
    Serial.println("WiFi geri gelmedi, yeniden baslatiliyor");
    delay(100);
    ESP.restart();
  }
}


void setup() {
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Hava Kalitesi Monitoru v3 ===");
  Serial.printf("Son reset nedeni: %d\n", (int)esp_reset_reason());   

#if FAN_ENABLED
  pinMode(FAN_PIN, OUTPUT);
#endif
  setFanOutput(false);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(1000);
  initSensor();

  WiFi.onEvent(onWifiEvent);
  connectWifi();

  if (!apMode) startMdns();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/fan", handleFan);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web sunucu basladi.");

  startWatchdog();   
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  pollSensor();
  updateFan();

  if (rollE.n > 0 && millis() - lastHist >= HIST_INTERVAL_MS) {
    lastHist = millis();
    addHistory(eco2Avg, tvocAvg);
  }

  healthCheck();
  delay(10);
}
