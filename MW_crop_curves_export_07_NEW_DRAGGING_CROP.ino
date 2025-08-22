// Forward declaration to keep Arduino's prototype generator happy
struct Packet;

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

/*** ===================== DEBUG/FLAGS ===================== ***/
static const bool DEBUG_ENABLED      = true;   // <-- später auf false setzen
static const bool USE_ZERO_INJECTION = true;   // true: erstes Paket = t:0,x:0,v:0,a:0
#define DBG(...) do{ if(DEBUG_ENABLED){ Serial.printf(__VA_ARGS__); } }while(0)
/*** ======================================================== ***/

// --------- Hardware / Geometrie ----------
#define AS5600_ADDR 0x36
#define SDA_PIN 1
#define SCL_PIN 2


const float RAD_DURCHMESSER_MM = 36.2f;
const float WHEEL_CIRCUMFERENCE = (RAD_DURCHMESSER_MM / 1000.0f) * (float)M_PI; // m/U
const int   ENCODER_STEPS       = 4096; // AS5600

// --------- WLAN-AP ----------
const char* ssid     = "Wagen-AP";
const char* password = "12345678";
WebServer server(80);

// --------- Frequenzen ----------
const uint32_t SAMPLE_US = 5000;  // 200 Hz interne Messung
const uint32_t PACKET_MS = 50;    // 20 Hz Pakete vorbereiten (ESP)  (=T_Datenerfassung)

// --------- Modi ----------
enum Mode { MODE_MANUAL=0, MODE_INCLINE=1 };
// *** WICHTIG: Automodus stillgelegt — wir starten im MANUAL
volatile Mode modeCurrent = MODE_MANUAL;

// --------- FSM (Schiefe Ebene) ----------
enum AutoState { CALIBRATING=0, IDLE=1, ARMED=2, RUNNING=3, STOP_PENDING=4 };
volatile AutoState astate = CALIBRATING;

// Schwellen/Zeiten
float EMA_ALPHA        = 0.25f;
float START_V_MS       = 0.03f;
float STOP_V_MS        = 0.02f;
float MIN_RUN_TIME_S   = 0.12f;
float ARM_TIME_REQ     = 0.10f;
float START_TIME_REQ   = 0.02f;
float STOP_TIME_REQ    = 0.05f;
float STOP_DELTA_S_MAX = 0.0020f;

// Auto-Kalibrierung
const float CAL_TIME_S       = 0.60f;
const float CAL_MARGIN_STOP  = 0.010f;
const float CAL_EXTRA_START  = 0.020f;
float cal_elapsed_s = 0.0f;
float cal_noise_max = 0.0f;

float MAX_SPEED_MPS = 3.0f;

// --------- Mess-/Zustandsgrößen ----------
volatile bool  measuring=false;
volatile float pos_m=0, vel_ms=0, acc_ms2=0;
float lastVel_ms=0;
uint16_t lastRaw=0;

float vel_abs_filt=0;
float runTime_s=0, calmTime_s=0, overStart_s=0, underStop_s=0;
float stopWindow_s=0, stopWindow_deltaS=0;
volatile int trimCountToSend=0;

static float vel_display = 0.0f;
const float DISPLAY_ALPHA = 0.15f;

// Zeitmarken/Flags
uint32_t lastSampleUs=0, lastPacketMs=0;
unsigned long measureStartMs = 0;
bool firstPacketAfterStart = false;

// Baseline der Position beim Scharfstellen
float pos_at_arm = 0.0f;

/*** ---------- Glättung (zentrierte Differenzen) ---------- ***/
volatile int SMOOTH_HALF = 15;
const int    SMOOTH_HALF_MIN = 1;
const int    SMOOTH_HALF_MAX = 30;

struct Sample { float t; float x; };
const int SAMPLE_BUF_SIZE = 2048;
Sample sampBuf[SAMPLE_BUF_SIZE];
volatile int sampHead = 0;
volatile unsigned long sampCount = 0;

inline void sampReset(){ sampHead = 0; sampCount = 0; }
inline void sampPush(float t, float x){
  sampBuf[sampHead] = {t, x};
  sampHead = (sampHead + 1) % SAMPLE_BUF_SIZE;
  if(sampCount < 0xFFFFFFFFUL) sampCount++;
}
inline const Sample& sampRel(int rel){
  return sampBuf[(sampHead + rel + SAMPLE_BUF_SIZE) % SAMPLE_BUF_SIZE];
}

float t_meas_s = 0.0f;
bool  smoothHasT0 = false;
float smoothT0 = 0.0f, smoothX0 = 0.0f;

/*** ---------- Paket-Buffer ---------- ***/
struct Packet { float t, x, v, a; };

const int PKT_BUF_SIZE = 256;
Packet pktBuf[PKT_BUF_SIZE];
volatile int pktHead=0, pktTail=0;

inline int pktCount(){ int c = pktHead - pktTail; if(c < 0) c += PKT_BUF_SIZE; return c; }
inline void pktClear(){ pktHead = pktTail = 0; }
inline void pushPacket(float t,float x,float v,float a){
  int next=(pktHead+1)%PKT_BUF_SIZE;
  if(next==pktTail){ DBG("[BUF] DROP t=%.3f x=%.5f v=%.5f a=%.5f\n",(double)t,(double)x,(double)v,(double)a); return; }
  pktBuf[pktHead] = Packet{t,x,v,a}; pktHead=next;
}
inline bool popPacket(Packet &out){
  if(pktTail==pktHead) return false;
  out=pktBuf[pktTail]; pktTail=(pktTail+1)%PKT_BUF_SIZE; return true;
}

// ---------- „Vorpuffer“ ----------
Packet preStartPkt{0,0,0,0};
uint32_t lastPrePktMs = 0;

// ---------- HTML/JS ----------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='utf-8'/>
<meta name='viewport' content='width=device-width,initial-scale=1'/>
<title>Wagen Messung</title>
<style>
body{font-family:system-ui,Arial,sans-serif;margin:12px}
.row{display:flex;gap:8px;align-items:center;margin:6px 0;flex-wrap:wrap}
#toggleBtn{font-size:2em;padding:20px;margin-top:20px}
button,select,input{font-size:1rem}
.chartWrapper{margin-bottom:6px;width:100%}
.chartWrapper canvas{width:100%!important;height:100%!important;display:block}
.ctrl{border:1px solid #ddd; border-radius:8px; padding:8px; margin:6px 0; display:none}
.ctrl .row{margin:4px 0}
.ctrl label{margin-right:10px}
.eq{font-family:ui-monospace,Consolas,monospace; padding:2px 6px; background:#f6f6f6; border-radius:4px}
#meta{font-size:0.95rem;opacity:.8}
.hr{height:1px;background:#ddd;margin:10px 0}
/* Overlay-Guides über allen Charts */
#chartsArea{position:relative; user-select:none; -webkit-user-select:none;}
.guide{position:absolute;top:0;bottom:0;display:none}
.guide .hit{position:absolute;top:0;bottom:0;width:22px;transform:translateX(-11px);
  cursor:ew-resize;touch-action:none}
#guideL{border-left:2px solid #ff0066}
#guideR{border-left:2px solid #0066ff}

/* Horizontale Y-Guides (pro Chart: Top/Bot) */
.hguide{position:absolute;left:0;right:0;display:none}
.hguide .hit{position:absolute;left:0;right:0;height:22px;transform:translateY(-11px);
  cursor:ns-resize;touch-action:none}
.yTop{border-top:2px solid #ff6600}
.yBot{border-top:2px solid #00aa88}

/* ===== Sichtbare, pro-Chart Crop-Linien + Griffe ===== */
/* vertikale Segmente je Chart (sichtbar nur in Plotflächen) */
.vline { position:absolute; width:0; pointer-events:none; }
.vline.L { border-left:2px solid #ff0066; }
.vline.R { border-left:2px solid #0066ff; }

/* Griffe mit Doppelpfeil (↔) – mittig auf der Plotfläche */
.vgrip, .ygrip{
  position:absolute;
  background:rgba(0,0,0,.18);
  border-radius:6px;
  box-shadow:0 1px 2px rgba(0,0,0,.2);
  pointer-events:none; /* Drag macht die unsichtbare .hit-Schicht */
  display:none;
}
.vgrip{ width:26px; height:20px; transform:translate(-13px,-10px); }
.vgrip::before, .vgrip::after{ content:''; position:absolute; top:50%; width:0; height:0;
  border-top:5px solid transparent; border-bottom:5px solid transparent; transform:translateY(-50%); }
.vgrip::before{ left:6px;  border-right:7px solid #fff; }
.vgrip::after { right:6px; border-left:7px solid #fff; }

/* Y-Griffe (↕) */
.ygrip{ width:20px; height:26px; transform:translate(-10px,-13px); }
.ygrip::before, .ygrip::after{ content:''; position:absolute; left:50%; width:0; height:0;
  border-left:5px solid transparent; border-right:5px solid transparent; transform:translateX(-50%); }
.ygrip::before{ top:6px;    border-bottom:7px solid #fff; }
.ygrip::after { bottom:6px; border-top:7px solid #fff; }

/* Die hohen, globalen X-Guides dienen nur als Drag-Hit-Zone, nicht sichtbar */
#guideL, #guideR { border-left:0; }
#guideL .hit, #guideR .hit { width:24px; }
</style></head><body>
<h2>Position, Geschwindigkeit, Beschleunigung</h2>

<div class='row'>
  <label for='smoothSel'><b>Glättung:</b></label>
  <select id='smoothSel'>
    <option value='1'>±1</option><option value='2'>±2</option><option value='3'>±3</option>
    <option value='4'>±4</option><option value='5'>±5</option><option value='6'>±6</option>
    <option value='7'>±7</option><option value='8'>±8</option><option value='9'>±9</option>
    <option value='10'>±10</option><option value='11'>±11</option><option value='12'>±12</option>
    <option value='13'>±13</option><option value='14'>±14</option><option value='15' selected>±15</option>
    <option value='16'>±16</option><option value='17'>±17</option><option value='18'>±18</option>
    <option value='19'>±19</option><option value='20'>±20</option><option value='21'>±21</option>
    <option value='22'>±22</option><option value='23'>±23</option><option value='24'>±24</option>
    <option value='25'>±25</option><option value='26'>±26</option><option value='27'>±27</option>
    <option value='28'>±28</option><option value='29'>±29</option><option value='30'>±30</option>
  </select>
  <span id='status'></span>
</div>

<div class='row'>
  <label><input type='checkbox' id='cbPos' checked> x(t)</label>
  <label><input type='checkbox' id='cbVel' checked> v(t)</label>
  <label><input type='checkbox' id='cbAcc'> a(t)</label>
</div>

<div class='row'>
  <label><input type='checkbox' id='cbXGuides' checked> X-Guides anzeigen</label>
  <label><input type='checkbox' id='cbYGuides'> Y-Guides anzeigen</label>
  <label><input type='checkbox' id='cbFits'> Ausgleichskurven</label>
</div>

<div class='row'>
  <button id='toggleBtn' onclick='toggleMeasure()'>Start</button>
  <button id='cropXBtn' disabled>Crop x</button>
  <button id='cropYBtn' disabled>Crop y</button>
  <button id='btnExportCSV'>Export CSV</button>
  <!-- Neue einzelne PNG-Buttons (iOS-freundlich, je ein Download pro Klick) -->
  <button id='btnPNGpos' disabled>PNG x</button>
  <button id='btnPNGvel' disabled>PNG v</button>
  <button id='btnPNGacc' disabled>PNG a</button>
</div>

<div id='meta'></div>
<div class='hr'></div>

<!-- Charts + Overlay-Guides -->
<div id='chartsArea'>
  <!-- x(t) -->
  <div id='posWrapper' class='chartWrapper'><canvas id='posChart'></canvas></div>
  <div id='posCtrl' class='ctrl'></div>

  <!-- v(t) -->
  <div id='velWrapper' class='chartWrapper'><canvas id='velChart'></canvas></div>
  <div id='velCtrl' class='ctrl'></div>

  <!-- a(t) -->
  <div id='accWrapper' class='chartWrapper'><canvas id='accChart'></canvas></div>
  <div id='accCtrl' class='ctrl'></div>

  <!-- vertikale X-Guides (nur Hit-Zone) -->
  <div id='guideL' class='guide'><div class='hit'></div></div>
  <div id='guideR' class='guide'><div class='hit'></div></div>
  <!-- horizontale Y-Guides: je Chart Top/Bot (nur Hit-Zone) -->
  <div id='guidePosTop' class='hguide yTop'><div class='hit'></div></div>
  <div id='guidePosBot' class='hguide yBot'><div class='hit'></div></div>
  <div id='guideVelTop' class='hguide yTop'><div class='hit'></div></div>
  <div id='guideVelBot' class='hguide yBot'><div class='hit'></div></div>
  <div id='guideAccTop' class='hguide yTop'><div class='hit'></div></div>
  <div id='guideAccBot' class='hguide yBot'><div class='hit'></div></div>
</div>


<script>
/* ===== Polyfills (alte WebViews) ===== */
if (!Math.log10) { Math.log10 = function(x){ return Math.log(x) / Math.LN10; }; }
(function(){
  var raf = window.requestAnimationFrame || window.webkitRequestAnimationFrame ||
            window.mozRequestAnimationFrame || function(cb){ return setTimeout(cb,16); };
  window.requestAnimationFrame = raf;
})();

/* ===== MiniChart (ES5, ohne moderne Syntax) ===== */
function has(o,k){ return o && Object.prototype.hasOwnProperty.call(o,k); }
function get2(o,a,b){ return o && has(o,a) ? o[a] : b; }
function formatTick(v, step){
  var s = Math.abs(step);
  var d = (s>=1)?0 : (s>=0.1?1 : (s>=0.01?2 : (s>=0.001?3 : (s>=0.0001?4 : 5))));
  var x = Math.abs(v) < Math.pow(10,-d) ? 0 : v;
  return (d===0) ? String(Math.round(x)) : x.toFixed(d);
}
function niceTicks(min, max, targetCount){
  if(!isFinite(min) || !isFinite(max) || min===max){ var v=isFinite(min)?min:0; min=v-1; max=v+1; }
  var span=max-min, rough=span/Math.max(1,targetCount||6);
  var pow10=Math.pow(10, Math.floor(Math.log10(Math.max(1e-12,rough))));
  var cands=[1,2,2.5,5,10];
  var bestStep=cands[0]*pow10, best=1e9, i;
  for(i=0;i<cands.length;i++){
    var st=cands[i]*pow10, n=span/st, diff=Math.abs(n-(targetCount||6));
    if(diff<best){best=diff; bestStep=st;}
  }
  var start=Math.ceil(min/bestStep)*bestStep, ticks=[];
  for(var v=start; v<=max+1e-12; v+=bestStep) ticks.push(+v.toFixed(12));
  return {ticks:ticks, step:bestStep};
}

function MiniScale(horizontal){ this.horizontal=horizontal; this.min=undefined; this.max=undefined; this._dmin=0; this._dmax=1; this._pmin=0; this._pmax=1; }
MiniScale.prototype.setDomain=function(min,max){ this._dmin=min; this._dmax=max; };
MiniScale.prototype.setRange=function(pmin,pmax){ this._pmin=pmin; this._pmax=pmax; };
MiniScale.prototype.getPixelForValue=function(v){
  var min=(this.min!==undefined)?this.min:this._dmin;
  var max=(this.max!==undefined)?this.max:this._dmax;
  if(max===min) return (this._pmin+this._pmax)/2;
  var t=(v-min)/(max-min);
  return this._pmin + t*(this._pmax - this._pmin);
};
MiniScale.prototype.getValueForPixel=function(px){
  var min=(this.min!==undefined)?this.min:this._dmin;
  var max=(this.max!==undefined)?this.max:this._dmax;
  if(this._pmax===this._pmin) return min;
  var t=(px - this._pmin)/(this._pmax - this._pmin);
  return min + t*(max - min);
};

function MiniChart(canvas, opts){
  this.canvas=canvas;
  this.ctx=canvas.getContext('2d');
  this.data={ datasets:[{ data:[], label:'', borderColor:'#06f' }] };
  this.overlay=[]; // Ausgleichskurve (x,y)-Paare
  this.overlayColor='#a0f'; // violett
  this.options=opts||{};
  // Basisränder: werden als Untergrenze benutzt
  this._baseMarg={left:60, right:16, top:8, bottom:28};
  this._marg={left:this._baseMarg.left, right:this._baseMarg.right, top:this._baseMarg.top, bottom:this._baseMarg.bottom};
  this.chartArea={left:0,right:0,top:0,bottom:0};
  this.scales={ x:new MiniScale(true), y:new MiniScale(false) };
  if(opts && opts.datasets && opts.datasets[0] && has(opts.datasets[0],'label')){
    this.data.datasets[0].label=opts.datasets[0].label;
  }
  var self=this;
  window.addEventListener('resize', function(){ self.resize(); });
  this.resize();
}
MiniChart.prototype.resize=function(){
  var parent=this.canvas.parentElement;
  var w=parent.clientWidth||300, h=parent.clientHeight||200;
  var dpr=window.devicePixelRatio||1;
  this.canvas.width=Math.round(w*dpr);
  this.canvas.height=Math.round(h*dpr);
  this.canvas.style.width=w+'px';
  this.canvas.style.height=h+'px';
  this.ctx.setTransform(dpr,0,0,dpr,0,0);
  this._layout();
  this.update();
};
MiniChart.prototype._layout=function(){
  var w=this.canvas.clientWidth, h=this.canvas.clientHeight, m=this._marg;
  this.chartArea.left=m.left; this.chartArea.right=w-m.right;
  this.chartArea.top=m.top; this.chartArea.bottom=h-m.bottom;
  this.scales.x.setRange(this.chartArea.left, this.chartArea.right);
  this.scales.y.setRange(this.chartArea.bottom, this.chartArea.top);
};
MiniChart.prototype._computeDomains=function(){
  var arr=this.data.datasets[0].data;
  var xmin=0,xmax=1,ymin=0,ymax=1, i;
  var sx=get2(this.options,'scales',{}), sxX=get2(sx,'x',{}), sxY=get2(sx,'y',{});
  if(arr.length>0){
    xmin = (has(sxX,'min') && sxX.min!==undefined) ? sxX.min : arr[0].x;
    xmax = (has(sxX,'max') && sxX.max!==undefined) ? sxX.max : arr[arr.length-1].x;
    if(!(has(sxX,'min') && sxX.min!==undefined) || !(has(sxX,'max') && sxX.max!==undefined)){
      for(i=0;i<arr.length;i++){ var x=arr[i].x; if(x<xmin)xmin=x; if(x>xmax)xmax=x; }
    }
    ymin = (has(sxY,'min') && sxY.min!==undefined) ? sxY.min : arr[0].y;
    ymax = (has(sxY,'max') && sxY.max!==undefined) ? sxY.max : arr[0].y;
    for(i=0;i<arr.length;i++){ var y=arr[i].y; if(y<ymin)ymin=y; if(y>ymax)ymax=y; }
    if(ymax===ymin){ ymin-=1; ymax+=1; }
    if(xmax===xmin){ xmin-=1; xmax+=1; }
  }
  this.scales.x.min = (has(sxX,'min')?sxX.min:undefined);
  this.scales.x.max = (has(sxX,'max')?sxX.max:undefined);
  this.scales.y.min = (has(sxY,'min')?sxY.min:undefined);
  this.scales.y.max = (has(sxY,'max')?sxY.max:undefined);
  this._domain={xmin:xmin,xmax:xmax,ymin:ymin,ymax:ymax};
  this.scales.x.setDomain(xmin,xmax);
  this.scales.y.setDomain(ymin,ymax);
};
MiniChart.prototype.update=function(){
  // 1) Erste Layout-Runde + Domains
  this._layout();
  this._computeDomains();

  var ctx=this.ctx, ca=this.chartArea, xS=this.scales.x, yS=this.scales.y;
  var xmin=this._domain.xmin, xmax=this._domain.xmax;
  var ymin=this._domain.ymin, ymax=this._domain.ymax;

  // 2) Ticks berechnen
  var xtInfo=niceTicks(xmin, xmax, 8);
  var ytInfo=niceTicks(ymin, ymax, 6);
  var xt=xtInfo.ticks, xs=xtInfo.step, yt=ytInfo.ticks, ys=ytInfo.step;

  // 3) Adaptiv: benötigte Ränder messen
  ctx.save();
  ctx.font='12px system-ui, Arial';
  var maxYTickW = 0;
  var i;
  for(i=0;i<yt.length;i++){
    var s = formatTick(yt[i], ys);
    var w = ctx.measureText(s).width;
    if(w>maxYTickW) maxYTickW = w;
  }
  var yLabel=get2(this.options,'yLabel','');
  var yTitlePad = yLabel ? 18 : 10;
  var leftNeeded = Math.ceil(yTitlePad + maxYTickW + 10);

  var tickFontSize = 12;
  var bottomNeeded = Math.ceil(
      tickFontSize + 6
    + tickFontSize + 8
  );
  ctx.restore();

  this._marg.left   = Math.max(this._baseMarg.left, leftNeeded);
  this._marg.bottom = Math.max(this._baseMarg.bottom, bottomNeeded);

  // 4) Mit neuen Margins neu layouten
  this._layout();
  ca=this.chartArea;

  // 5) Zeichnen
  ctx.clearRect(0,0,this.canvas.clientWidth,this.canvas.clientHeight);

  ctx.save(); ctx.lineWidth=1;

  /* horizontale Grid + y-Labels */
  ctx.strokeStyle='#eee'; ctx.fillStyle='#555'; ctx.font='12px system-ui, Arial';
  ctx.textAlign='right'; ctx.textBaseline='middle';
  for(i=0;i<yt.length;i++){
    var vy=yt[i], y=yS.getPixelForValue(vy);
    ctx.beginPath(); ctx.moveTo(ca.left,y); ctx.lineTo(ca.right,y); ctx.stroke();
    ctx.fillText(formatTick(vy, ys), ca.left-8, y);
  }

  /* vertikale Grid + x-Labels */
  ctx.textAlign='center'; ctx.textBaseline='top';
  var j;
  for(j=0;j<xt.length;j++){
    var vx=xt[j], x=xS.getPixelForValue(vx);
    ctx.strokeStyle='#f2f2f2';
    ctx.beginPath(); ctx.moveTo(x,ca.top); ctx.lineTo(x,ca.bottom); ctx.stroke();
    ctx.fillStyle='#555';
    ctx.fillText(formatTick(vx, xs), x, ca.bottom+6);
  }

  /* Achsenrahmen */
  ctx.strokeStyle='#ddd';
  ctx.beginPath(); ctx.moveTo(ca.left,ca.bottom); ctx.lineTo(ca.right,ca.bottom); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(ca.left,ca.top); ctx.lineTo(ca.left,ca.bottom); ctx.stroke();
  ctx.restore();

  /* Nulllinien (gestrichelt) */
  ctx.save(); ctx.setLineDash([4,4]); ctx.strokeStyle='#999';
  if(ymin<=0 && ymax>=0){ var y0=yS.getPixelForValue(0); ctx.beginPath(); ctx.moveTo(ca.left,y0); ctx.lineTo(ca.right,y0); ctx.stroke(); }
  if(xmin<=0 && xmax>=0){ var x0=xS.getPixelForValue(0); ctx.beginPath(); ctx.moveTo(x0,ca.top); ctx.lineTo(x0,ca.bottom); ctx.stroke(); }
  ctx.setLineDash([]); ctx.restore();

  /* Datenlinie + Overlay – innerhalb der Plotfläche clippen */
  var ds=this.data.datasets[0], arr=ds.data, k;

  // Clip auf chartArea, damit außerhalb liegende Werte (z. B. < yMin) nicht gezeichnet werden
  ctx.save();
  ctx.beginPath();
  ctx.rect(ca.left, ca.top, ca.right - ca.left, ca.bottom - ca.top);
  ctx.clip();

  // Daten
  if(arr.length>0){
    ctx.beginPath();
    for(k=0;k<arr.length;k++){
      var p=arr[k];
      var xx=xS.getPixelForValue(p.x), yy=yS.getPixelForValue(p.y);
      if(k===0) ctx.moveTo(xx,yy); else ctx.lineTo(xx,yy);
    }
    ctx.lineWidth=2; ctx.strokeStyle=ds.borderColor||'#06f'; ctx.stroke();
  }

// Ausgleichskurve (Overlay) – gestrichelt
var ov=this.overlay;
if(ov && ov.length>0){
  ctx.beginPath();
  for(var q=0;q<ov.length;q++){
    var pt=ov[q];
    var ox=xS.getPixelForValue(pt.x), oy=yS.getPixelForValue(pt.y);
    if(q===0) ctx.moveTo(ox,oy); else ctx.lineTo(ox,oy);
  }
  ctx.lineWidth=2;
  ctx.strokeStyle=this.overlayColor;
  ctx.setLineDash([6,4]);   // gestrichelt
  ctx.stroke();
  ctx.setLineDash([]);      // zurücksetzen
}
ctx.restore();


/* Legende oben rechts */
ctx.save();
ctx.font='12px system-ui, Arial';
ctx.textAlign='left';
ctx.textBaseline='middle';

var ds = this.data.datasets[0];
var label = (ds && ds.label) ? ds.label : 'Daten';

// Position der Legende innerhalb der Plotfläche
var lx = ca.right - 120;   // 120 px vom rechten Rand
var ly = ca.top + 14;      // 14 px unter dem oberen Rand

// Datenlinie (durchgezogen)
ctx.strokeStyle = (ds && ds.borderColor) ? ds.borderColor : '#06f';
ctx.setLineDash([]);
ctx.beginPath(); ctx.moveTo(lx,ly); ctx.lineTo(lx+24,ly); ctx.stroke();
ctx.fillStyle='#000';
ctx.fillText(label, lx+30, ly);

// Overlay/Fit (gestrichelt) nur wenn vorhanden
if(ov && ov.length>0){
  ly += 16;
  ctx.strokeStyle = this.overlayColor || '#a0f';
  ctx.setLineDash([6,4]);
  ctx.beginPath(); ctx.moveTo(lx,ly); ctx.lineTo(lx+24,ly); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle='#000';
  ctx.fillText('Fit', lx+30, ly);
}
ctx.restore();


  /* Achsentitel */
  var xLabel='t [s]';
  ctx.save(); ctx.fillStyle='#444'; ctx.font='12px system-ui, Arial';

  if(yLabel){
    var yCenter = (ca.top+ca.bottom)/2;
    var xLeft   = this._marg.left - (maxYTickW + 10 + (yLabel?8:0));
    if(xLeft < 4) xLeft = 4;
    ctx.save();
    ctx.translate(xLeft, yCenter);
    ctx.rotate(-Math.PI/2);
    ctx.textAlign='center';
    ctx.textBaseline='middle';
    ctx.fillText(yLabel,0,0);
    ctx.restore();
  }

  ctx.textAlign='center';
  ctx.textBaseline='bottom';
  ctx.fillText(xLabel,(ca.left+ca.right)/2, this.canvas.clientHeight-4);
  ctx.restore();
};

/* API-Kompat-Schicht zu deinem Code */
function createChart(canvas,label,color,yLabel){
  var ch=new MiniChart(canvas,{});
  ch.data.datasets[0].label=label;
  ch.data.datasets[0].borderColor=color;
  ch.options.scales={x:{}, y:{}};
  ch.options.yLabel=yLabel;
  ch.resize=ch.resize.bind(ch);
  ch.update=ch.update.bind(ch);
  return ch;
}

/* ======== Deine App-Variablen (ES5: nur var!) ======== */
var posFull=[], velFull=[], accFull=[];
var running=false, prevRunning=false;

/* X/Y Cropping Steuerung */
var isCropped=false;          // X-Crop aktiv?
var yIsCropped=false;         // Y-Crop aktiv?
var cropL=null, cropR=null;   // X-Guides Zeit
var cbXGuides=document.getElementById('cbXGuides');
var cbYGuides=document.getElementById('cbYGuides');
var cropXBtn=document.getElementById('cropXBtn');
var cropYBtn=document.getElementById('cropYBtn');

var cbFits=document.getElementById('cbFits');

var chartsArea=document.getElementById('chartsArea');
var guideL=document.getElementById('guideL');
var guideR=document.getElementById('guideR');

var posChart=createChart(document.getElementById('posChart'),'x (m)','blue','x [m]');
var velChart=createChart(document.getElementById('velChart'),'v (m/s)','green','v [m/s]');
var accChart=createChart(document.getElementById('accChart'),'a (m/s²)','red','a [m/s²]');

/* ======= Y-Guides ======= */
var guidePosTop=document.getElementById('guidePosTop'), guidePosBot=document.getElementById('guidePosBot');
var guideVelTop=document.getElementById('guideVelTop'), guideVelBot=document.getElementById('guideVelBot');
var guideAccTop=document.getElementById('guideAccTop'), guideAccBot=document.getElementById('guideAccBot');

var chartDefs = [
  { key:'pos', chart:posChart, full:posFull, gTop:guidePosTop, gBot:guidePosBot, yTop:null, yBot:null, ctrl:document.getElementById('posCtrl') },
  { key:'vel', chart:velChart, full:velFull, gTop:guideVelTop, gBot:guideVelBot, yTop:null, yBot:null, ctrl:document.getElementById('velCtrl') },
  { key:'acc', chart:accChart, full:accFull, gTop:guideAccTop, gBot:guideAccBot, yTop:null, yBot:null, ctrl:document.getElementById('accCtrl') }
];

/* === Wrapper-Referenzen (für per-Chart Linien/Griffe) === */
for(var i=0;i<chartDefs.length;i++){
  chartDefs[i].wrapper = document.getElementById(
    chartDefs[i].key==='pos' ? 'posWrapper' :
    (chartDefs[i].key==='vel' ? 'velWrapper' : 'accWrapper')
  );
}

/* === Pro-Chart X-Linien & Griffe erzeugen === */
function ensurePerChartVLines(){
  for(var i=0;i<chartDefs.length;i++){
    var d = chartDefs[i];
    if(!d.vL){
      var vL = document.createElement('div'); vL.className='vline L';
      var vR = document.createElement('div'); vR.className='vline R';
      var gL = document.createElement('div'); gL.className='vgrip';
      var gR = document.createElement('div'); gR.className='vgrip';
      d.wrapper.style.position='relative';
      d.wrapper.appendChild(vL); d.wrapper.appendChild(vR);
      d.wrapper.appendChild(gL); d.wrapper.appendChild(gR);
      d.vL=vL; d.vR=vR; d.vGripL=gL; d.vGripR=gR;
    }
  }
}
ensurePerChartVLines();

/* === Pro-Chart Y-Linien & Griffe erzeugen === */
function ensurePerChartYLines(){
  for(var i=0;i<chartDefs.length;i++){
    var d = chartDefs[i];
    if(!d.yLTop){
      var yTop = document.createElement('div');
      var yBot = document.createElement('div');
      yTop.className='hline top'; yBot.className='hline bot';
      yTop.style.position='absolute'; yBot.style.position='absolute';
      yTop.style.height='0'; yBot.style.height='0';
      yTop.style.borderTop='2px solid #ff6600';
      yBot.style.borderTop='2px solid #00aa88';
      var gT = document.createElement('div'); gT.className='ygrip';
      var gB = document.createElement('div'); gB.className='ygrip';
      d.wrapper.appendChild(yTop); d.wrapper.appendChild(yBot);
      d.wrapper.appendChild(gT); d.wrapper.appendChild(gB);
      d.yLTop = yTop; d.yLBot = yBot; d.yGripTop=gT; d.yGripBot=gB;
    }
  }
}
ensurePerChartYLines();

/* ======= Ausgleichskurven: UI-Factory pro Chart ======= */
function buildFitControls(def){
  var el = def.ctrl;
  function makeParamRow(name, defVal){
    return ""
      + "<div class='row param-"+name+"' style='display:none;align-items:center;gap:6px'>"
      + "<label>"+name+":</label>"
      + "<input class='min-"+name+"' type='number' value='-10' style='width:70px'>"
      + "<input class='rng-"+name+"' type='range' min='-10' max='10' step='0.01' value='"+defVal+"' style='flex:1'>"
      + "<input class='max-"+name+"' type='number' value='10' style='width:70px'>"
      + "<input class='val-"+name+"' type='number' value='"+defVal+"' style='width:90px'>"
      + "</div>";
  }
  var html = ""
    + "<div class='row'><b>Ausgleich:</b> "
    + "<select class='fitType'>"
    + "<option value='quadratisch'>quadratisch</option>"
    + "<option value='linear'>linear</option>"
    + "<option value='exponentiell'>exponentiell</option>"
    + "<option value='sinus'>sinus</option>"
    + "</select>"
    + " <span class='eq'></span></div>"
    + makeParamRow('a',1) + makeParamRow('b',0) + makeParamRow('c',0)
    + makeParamRow('m',1) + makeParamRow('w',1) + makeParamRow('phi',0);
  el.innerHTML = html;

  function showParam(name, show){
    var row = el.querySelector(".row.param-"+name);
    if(row) row.style.display = show ? 'flex':'none';
  }

  /* State & Helpers */
  def.fit = { type:'quadratisch', params:{a:1,b:0,c:0,m:1,w:1,phi:0} };

  function setEquation(){
    var eq = el.querySelector(".eq");
    var t = def.fit.type;
    var s = (t==='quadratisch') ? "y = a·x² + b·x + c"
          : (t==='linear')      ? "y = m·x + b"
          : (t==='exponentiell')? "y = a·e^(b·x) + c"
          :                       "y = a·sin(w·x − φ) + c";
    eq.textContent = s;
  }

  // Sichtbarkeit inkl. c für exponentiell & sinus
  function updateVisibleParams(){
    var t = def.fit.type;
    showParam('a', t==='quadratisch' || t==='exponentiell' || t==='sinus');
    showParam('b', t==='quadratisch' || t==='linear' || t==='exponentiell');
    showParam('c', t==='quadratisch' || t==='exponentiell' || t==='sinus');
    showParam('m', t==='linear');
    showParam('w', t==='sinus');
    showParam('phi', t==='sinus');
  }

  function syncStep(name){
    var minEl=el.querySelector(".min-"+name);
    var maxEl=el.querySelector(".max-"+name);
    var rngEl=el.querySelector(".rng-"+name);
    if(!minEl || !maxEl || !rngEl) return;
    var min=parseFloat(minEl.value), max=parseFloat(maxEl.value);
    if(max===min){ max=min+1; maxEl.value=max; }
    var step=(max-min)/1000.0;
    rngEl.min=min; rngEl.max=max; rngEl.step=step;
  }
  function clamp(v,min,max){ return v<min?min:(v>max?max:v); }

  function setParamBoundsAndValue(name, min, max, val){
    var minEl=el.querySelector(".min-"+name);
    var maxEl=el.querySelector(".max-"+name);
    var rngEl=el.querySelector(".rng-"+name);
    var valEl=el.querySelector(".val-"+name);
    if(!minEl||!maxEl||!rngEl||!valEl) return;
    minEl.value = String(min);
    maxEl.value = String(max);
    syncStep(name);
    var v = clamp(val, min, max);
    rngEl.value = String(v);
    valEl.value = String(v);
    def.fit.params[name]=v;
  }

  function applyDefaultsForType(t){
    if(t==='quadratisch'){
      // a=0..1 -> 0.25; b=0..1 -> 0; c=0..1 -> 0
      setParamBoundsAndValue('a', 0, 1, 0.25);
      setParamBoundsAndValue('b', 0, 1, 0);
      setParamBoundsAndValue('c', 0, 1, 0);
    }else if(t==='linear'){
      // m=0..2 -> 0.5; b=0..1 -> 0
      setParamBoundsAndValue('m', 0, 2, 0.5);
      setParamBoundsAndValue('b', 0, 1, 0);
    }else if(t==='exponentiell'){
      // a=-1..1 -> 0.5; b=0..1 -> 0; c=0..1 -> 0
      setParamBoundsAndValue('a', -1, 1, 0.5);
      setParamBoundsAndValue('b', 0, 1, 0);
      setParamBoundsAndValue('c', 0, 1, 0);
    }else{
      // sinus: a=0..1 -> 0.2; w=0..10 -> 1; phi=0..2π -> 0; c=0..1 -> 0
      setParamBoundsAndValue('a', 0, 1, 0.2);
      setParamBoundsAndValue('w', 0, 10, 1);
      setParamBoundsAndValue('phi', 0, 6.283185307179586, 0); // 2π
      setParamBoundsAndValue('c', 0, 1, 0);
    }
  }

  function wireParam(name){
    var minEl=el.querySelector(".min-"+name);
    var maxEl=el.querySelector(".max-"+name);
    var rngEl=el.querySelector(".rng-"+name);
    var valEl=el.querySelector(".val-"+name);
    if(!minEl||!maxEl||!rngEl||!valEl) return;

    function applyFromRange(){
      var v=parseFloat(rngEl.value);
      valEl.value=v; def.fit.params[name]=v; refreshCharts();
    }
    function applyFromVal(){
      var min=parseFloat(minEl.value), max=parseFloat(maxEl.value);
      var v=parseFloat(valEl.value);
      v=clamp(v,min,max);
      valEl.value=v; rngEl.value=v; def.fit.params[name]=v; refreshCharts();
    }
    function applyRangeChange(){
      syncStep(name);
      var min=parseFloat(minEl.value), max=parseFloat(maxEl.value);
      var v=clamp(parseFloat(rngEl.value),min,max);
      rngEl.value=v; valEl.value=v; def.fit.params[name]=v; refreshCharts();
    }
    function applyBoundsChange(){
      syncStep(name);
      var min=parseFloat(minEl.value), max=parseFloat(maxEl.value);
      var v=clamp(parseFloat(rngEl.value),min,max);
      rngEl.value=v; valEl.value=v; def.fit.params[name]=v; refreshCharts();
    }

    syncStep(name);
    rngEl.addEventListener('input', applyFromRange);
    valEl.addEventListener('change', applyFromVal);
    rngEl.addEventListener('change', applyRangeChange);
    minEl.addEventListener('change', applyBoundsChange);
    maxEl.addEventListener('change', applyBoundsChange);
  }

  var sel = el.querySelector(".fitType");
  sel.addEventListener('change', function(){
    def.fit.type = sel.value;
    setEquation();
    updateVisibleParams();
    applyDefaultsForType(def.fit.type);
    refreshCharts();
  });

  setEquation();
  updateVisibleParams();
  // Defaults beim Start (quadratisch)
  applyDefaultsForType('quadratisch');

  wireParam('a'); wireParam('b'); wireParam('c');
  wireParam('m'); wireParam('w'); wireParam('phi');
}

/* Fit-Eval */
function evalModel(t, pars, x){
  if(t==='quadratisch') return pars.a*x*x + pars.b*x + pars.c;
  if(t==='linear')      return pars.m*x + pars.b;
  if(t==='exponentiell')return pars.a*Math.exp(pars.b*x) + pars.c;
  return pars.a*Math.sin(pars.w*x - pars.phi) + (pars.c||0); // sinus + Offset
}

/* Fit-Overlay berechnen */
function buildOverlayFor(def, xMin, xMax){
  if(!cbFits.checked || !def.fit) return [];
  var typ = def.fit.type, pars = def.fit.params;
  if(!(xMax>xMin)) return [];
  var N = 256;
  var out = [];
  var dx = (xMax - xMin) / (N-1);
  for(var i=0;i<N;i++){
    var x = xMin + i*dx;
    var y = evalModel(typ, pars, x);
    out.push({x:x,y:y});
  }
  return out;
}

/* ======= Y-Range Helpers ======= */
function getYRangeFor(def){
  var arr = def.full;
  if(arr.length===0) return {min:-1,max:1};
  var mn=arr[0].y, mx=arr[0].y, i;
  for(i=1;i<arr.length;i++){ var v=arr[i].y; if(v<mn) mn=v; if(v>mx) mx=v; }
  if(mx===mn){ mn-=1; mx+=1; }
  return {min:mn,max:mx};
}

/* ======= Canvas-Offset Helpers (Y) ======= */
function getCanvasOffsetsFor(def){
  var cont = chartsArea.getBoundingClientRect();
  var can  = def.chart.canvas.getBoundingClientRect();
  return { contTop:cont.top, canvasTopInCont:(can.top - cont.top) };
}
function plotBoundsYPxFor(def){
  var area = def.chart.chartArea;
  var o = getCanvasOffsetsFor(def);
  return { top:o.canvasTopInCont + area.top, bottom:o.canvasTopInCont + area.bottom };
}
function displayValueToPxYFor(def, v){
  var o = getCanvasOffsetsFor(def);
  var pxInCanvas = def.chart.scales.y.getPixelForValue(v);
  return o.canvasTopInCont + pxInCanvas;
}
function pxToDisplayValueYFor(def, pxCont){
  var o = getCanvasOffsetsFor(def);
  var pxInCanvas = pxCont - o.canvasTopInCont;
  return def.chart.scales.y.getValueForPixel(pxInCanvas);
}

/* ======= Sichtbarkeit / Layout ======= */
function clearCharts(){
  posFull.length=0; velFull.length=0; accFull.length=0;
  posChart.data.datasets[0].data.length=0;
  velChart.data.datasets[0].data.length=0;
  accChart.data.datasets[0].data.length=0;
  posChart.overlay=[]; velChart.overlay=[]; accChart.overlay=[];
  posChart.update(); velChart.update(); accChart.update();

  // Reset Crops & Guides
  isCropped=false; yIsCropped=false;
  cropL=null; cropR=null;
  setXGuidesVisible(false);
  setYGuidesVisible(false);
  cropXBtn.textContent='Crop x'; cropYBtn.textContent='Crop y';
  cropXBtn.disabled=true; cropYBtn.disabled=true;

  // Reset Y guide limits
  for(var i=0;i<chartDefs.length;i++){ chartDefs[i].yTop=null; chartDefs[i].yBot=null; }

  // Fit-Steuerungen zurücksetzen (Anzeige steuert cbFits)
  applyFitsVisibility();

  updateExportButtons();
}
function applyVisibility(){
  var posVis=document.getElementById('cbPos').checked;
  var velVis=document.getElementById('cbVel').checked;
  var accVis=document.getElementById('cbAcc').checked;
  document.getElementById('posWrapper').style.display=posVis?'block':'none';
  document.getElementById('velWrapper').style.display=velVis?'block':'none';
  document.getElementById('accWrapper').style.display=accVis?'block':'none';
  var visCount=(posVis?1:0)+(velVis?1:0)+(accVis?1:0); if(!visCount) visCount=1;
  var h = visCount==1 ? '400px' : (visCount==2 ? '240px' : '160px');
  if(posVis) document.getElementById('posWrapper').style.height=h;
  if(velVis) document.getElementById('velWrapper').style.height=h;
  if(accVis) document.getElementById('accWrapper').style.height=h;

  applyFitsVisibility();

  posChart.resize(); velChart.resize(); accChart.resize();
  updateGuidePositions();        // X
  updateYGuidePositionsAll();    // Y
  updateExportButtons();
}
document.getElementById('cbPos').addEventListener('change',applyVisibility);
document.getElementById('cbVel').addEventListener('change',applyVisibility);
document.getElementById('cbAcc').addEventListener('change',applyVisibility);

/* Glättung + Start */
document.getElementById('smoothSel').addEventListener('change', function(e){
  fetch('/setsmooth?n='+e.target.value);
});
function toggleMeasure(){ fetch('/toggle'); }

/* Checkboxen: Guides ein/aus */
cbXGuides.addEventListener('change', function(){
  setXGuidesVisible(cbXGuides.checked);
  updateGuidePositions();
});
cbYGuides.addEventListener('change', function(){
  if(cbYGuides.checked){ updateYGuidePositionsAll(); } else { setYGuidesVisible(false); }
});

/* Ausgleichskurven Sichtbarkeit */
cbFits.addEventListener('change', function(){
  applyFitsVisibility();
  refreshCharts();
});
function applyFitsVisibility(){
  for(var i=0;i<chartDefs.length;i++){
    var def=chartDefs[i];
    var wrapper=document.getElementById(def.key==='pos'?'posWrapper':(def.key==='vel'?'velWrapper':'accWrapper'));
    var visible = (wrapper.style.display!=='none') && cbFits.checked;
    def.ctrl.style.display = visible ? 'block' : 'none';
  }
}

/* Crop-Buttons */
cropXBtn.addEventListener('click', function(){
  isCropped = !isCropped;
  cropXBtn.textContent = isCropped ? 'Crop x aufheben' : 'Crop x';
  setXGuidesVisible(cbXGuides.checked); // sichtbar lassen, bis Checkbox aus
  refreshCharts();
});
cropYBtn.addEventListener('click', function(){
  yIsCropped = !yIsCropped;
  cropYBtn.textContent = yIsCropped ? 'Crop y aufheben' : 'Crop y';
  setYGuidesVisible(cbYGuides.checked); // sichtbar lassen, bis Checkbox aus
  refreshCharts();
});

/* Status-Polling */
function pollStatus(){
  fetch('/status').then(function(r){return r.json();}).then(function(s){
    var wasRunning = running;
    running=s.running;
    document.getElementById('status').innerText=s.text;
    document.getElementById('meta').innerText='Start: '+s.start_th+' m/s, Stop: '+s.stop_th+' m/s';
    var b=document.getElementById('toggleBtn'); b.disabled=false; b.innerText=running?'Stopp':'Start';
    if(!wasRunning && running){ clearCharts(); }
    if(wasRunning && !running){
      if(posFull.length>1){
        initGuidesFromData();            // X
        initYGuidesFromDataAll();        // Y
        cropXBtn.disabled=false;
        cropYBtn.disabled=false;
        setXGuidesVisible(cbXGuides.checked);
        setYGuidesVisible(cbYGuides.checked);
        updateGuidePositions();
        updateYGuidePositionsAll();
      }
    }
    updateExportButtons();
  })["catch"](function(e){ console.log(e); });
}

/* X-Guides initialisieren (Zeitbereich) */
function getTimeRange(){
  var arr=posFull; if(arr.length===0) return {min:0,max:1};
  return {min:arr[0].x, max:arr[arr.length-1].x};
}
function initGuidesFromData(){
  var tr=getTimeRange(); var min=tr.min, max=tr.max, span=max-min;
  cropL = min + 0.10*span; cropR = min + 0.90*span;
}

/* Anzeige-Zeit vs. Original-Zeit */
function originalToDisplay(t){ return isCropped ? (t - cropL) : t; }
function displayToOriginal(t){ return isCropped ? (cropL + t) : t; }

/* Pixel<->Zeit Mapping über posChart */
function getCanvasOffsets(){
  var cont = chartsArea.getBoundingClientRect();
  var can  = posChart.canvas.getBoundingClientRect();
  return {contLeft:cont.left, canvasLeftInCont: (can.left - cont.left)};
}
function pxToDisplayTime(pxCont){
  var o=getCanvasOffsets(); var pxInCanvas = pxCont - o.canvasLeftInCont;
  return posChart.scales.x.getValueForPixel(pxInCanvas);
}
function displayTimeToPx(tDisp){
  var o=getCanvasOffsets(); var pxInCanvas = posChart.scales.x.getPixelForValue(tDisp);
  return o.canvasLeftInCont + pxInCanvas;
}
function plotBoundsPx(){
  var area = posChart.chartArea;
  var o=getCanvasOffsets();
  return {left: o.canvasLeftInCont + area.left, right: o.canvasLeftInCont + area.right};
}

/* Guides positionieren (X) – per-Chart Segmente + Griffe */
function updateGuidePositions(){
  if(cropL===null || cropR===null) return;

  // Position der (unsichtbaren) großen Drag-Guides
  var leftPx  = displayTimeToPx( originalToDisplay(cropL) );
  var rightPx = displayTimeToPx( originalToDisplay(cropR) );
  guideL.style.left = leftPx+'px';
  guideR.style.left = rightPx+'px';

  // Pro Chart: kurze Liniensegmente + Griffe mittig in der Plotfläche
  for(var i=0;i<chartDefs.length;i++){
    var d = chartDefs[i];
    var visible = (d.wrapper.style.display!=='none') && cbXGuides.checked;
    if(!visible){
      if(d.vL){ d.vL.style.display='none'; d.vR.style.display='none'; }
      if(d.vGripL){ d.vGripL.style.display='none'; d.vGripR.style.display='none'; }
      continue;
    }

    var ca = d.chart.chartArea;       // in Canvas-Koordinaten
    var canRect = d.chart.canvas.getBoundingClientRect();
    var wrapRect= d.wrapper.getBoundingClientRect();

    // X in Canvas-Koordinaten (Display-Zeit!)
    var xL_canvas = d.chart.scales.x.getPixelForValue( originalToDisplay(cropL) );
    var xR_canvas = d.chart.scales.x.getPixelForValue( originalToDisplay(cropR) );

    // X relativ zum Wrapper
    var xL = (canRect.left - wrapRect.left) + xL_canvas;
    var xR = (canRect.left - wrapRect.left) + xR_canvas;

    // Y-Top/Bottom relativ zum Wrapper (Plotfläche)
    var yTop = (canRect.top - wrapRect.top) + ca.top;
    var yBot = (canRect.top - wrapRect.top) + ca.bottom;
    var midY = (yTop + yBot) / 2;

    // Setzen/zeigen
    d.vL.style.display='block'; d.vR.style.display='block';
    d.vL.style.left=xL+'px'; d.vR.style.left=xR+'px';
    d.vL.style.top=yTop+'px'; d.vR.style.top=yTop+'px';
    d.vL.style.height=(yBot - yTop)+'px';
    d.vR.style.height=(yBot - yTop)+'px';

    d.vGripL.style.display='block'; d.vGripR.style.display='block';
    d.vGripL.style.left=xL+'px'; d.vGripR.style.left=xR+'px';
    d.vGripL.style.top=midY+'px'; d.vGripR.style.top=midY+'px';
  }
}

/* Drag-Handling (X) – ohne Sprung */
var dragging=null, pointerId=null;
function startDrag(which,e){
  dragging=which; pointerId=e.pointerId;
  var el=(which==='L'?guideL:guideR);
  if(el.setPointerCapture) el.setPointerCapture(pointerId);
}
function onDrag(e){
  if(!dragging) return;
  var cont = chartsArea.getBoundingClientRect();
  var b=plotBoundsPx();
  var px = Math.max(b.left, Math.min(b.right, e.clientX - cont.left));
  var tDisp = pxToDisplayTime(px);
  var tOrg  = displayToOriginal(tDisp);
  var tr=getTimeRange(), min=tr.min, max=tr.max;
  if(dragging==='L'){ cropL = Math.max(min, Math.min(tOrg, (cropR!==null?cropR:max))); }
  else{ cropR = Math.min(max, Math.max(tOrg, (cropL!==null?cropL:min))); }
  updateGuidePositions();
}
function endDrag(){
  if(!dragging) return;
  dragging=null; pointerId=null;
  if(!running) refreshCharts();
}
guideL.querySelector('.hit').addEventListener('pointerdown', function(e){ e.preventDefault(); startDrag('L',e); });
guideR.querySelector('.hit').addEventListener('pointerdown', function(e){ e.preventDefault(); startDrag('R',e); });
window.addEventListener('pointermove', onDrag);
window.addEventListener('pointerup', endDrag);
window.addEventListener('pointercancel', endDrag);

/* Drag-Handling (Y) */
var draggingY = null, pointerIdY = null;
function startDragY(idx, which, e){
  draggingY = {idx:idx, which:which};
  pointerIdY = e.pointerId;
  var el = (which==='Top'? chartDefs[idx].gTop : chartDefs[idx].gBot);
  if(el.setPointerCapture) el.setPointerCapture(pointerIdY);
}
function onDragY(e){
  if(!draggingY || !cbYGuides.checked) return;
  var d = chartDefs[draggingY.idx];
  var cont = chartsArea.getBoundingClientRect();
  var b = plotBoundsYPxFor(d);
  var pxCont = e.clientY - cont.top;
  var py = Math.max(b.top, Math.min(b.bottom, pxCont));
  var v = pxToDisplayValueYFor(d, py);

  if(draggingY.which==='Top'){
    d.yTop = Math.max(v, (d.yBot!==null ? d.yBot : v));
  }else{
    d.yBot = Math.min(v, (d.yTop!==null ? d.yTop : v));
  }
  updateYGuidePositionsAll();
}
function endDragY(){
  if(!draggingY) return;
  draggingY=null; pointerIdY=null;
  if(!running) refreshCharts();
}
(function(){
  var pairs = [
    [0,'Top',document.querySelector('#guidePosTop .hit')],
    [0,'Bot',document.querySelector('#guidePosBot .hit')],
    [1,'Top',document.querySelector('#guideVelTop .hit')],
    [1,'Bot',document.querySelector('#guideVelBot .hit')],
    [2,'Top',document.querySelector('#guideAccTop .hit')],
    [2,'Bot',document.querySelector('#guideAccBot .hit')]
  ];
  for(var i=0;i<pairs.length;i++){
    (function(idx,which,el){
      el.addEventListener('pointerdown', function(e){ e.preventDefault(); startDragY(idx,which,e); });
    })(pairs[i][0], pairs[i][1], pairs[i][2]);
  }
  window.addEventListener('pointermove', onDragY);
  window.addEventListener('pointerup', endDragY);
  window.addEventListener('pointercancel', endDragY);
})();

/* ===== Fit-Controls erstellen ===== */
for(var i=0;i<chartDefs.length;i++){ buildFitControls(chartDefs[i]); }

/* Charts neu zeichnen (inkl. X/Y-Crop & Fit-Overlay) */
function refreshCharts(){
  var charts=[posChart,velChart,accChart];
  var full=[posFull,velFull,accFull];

  for(var i=0;i<charts.length;i++){
    var ch=charts[i], src=full[i], out;

    // X-Crop anwenden?
    if(isCropped && cropL!==null && cropR!==null){
      out=[];
      for(var k=0;k<src.length;k++){
        var p=src[k];
        if(p.x>=cropL && p.x<=cropR){ out.push({x:p.x - cropL, y:p.y}); }
      }
      ch.options.scales.x.min = 0;
      ch.options.scales.x.max = undefined;
    }else{
      out = src.slice(0);
      ch.options.scales.x.min = undefined;
      ch.options.scales.x.max = undefined;
    }

    // Y-Crop Datenfilter (nur Anzeige; CSV bleibt unverändert)
    var defCrop = chartDefs[i];
    var yMin = null, yMax = null;
    if(yIsCropped && defCrop.yTop!==null && defCrop.yBot!==null){
      yMin = Math.min(defCrop.yTop, defCrop.yBot);
      yMax = Math.max(defCrop.yTop, defCrop.yBot);
      var outY=[];
      for(var m=0;m<out.length;m++){
        var q=out[m];
        if(q.y>=yMin && q.y<=yMax) outY.push(q);
      }
      out = outY;
    }

    ch.data.datasets[0].data = out;

    // ===== Y-Achse festlegen
    if(yIsCropped && yMin!==null && yMax!==null){
      ch.options.scales.y.min = yMin;
      ch.options.scales.y.max = yMax;
    }else{
      var baseR = getYRangeFor(defCrop);
      ch.options.scales.y.min = baseR.min;
      ch.options.scales.y.max = baseR.max;
    }

    // Overlay (Fit) – nur im sichtbaren X-Bereich
    var xMin = (out.length>0) ? out[0].x : 0;
    var xMax = (out.length>0) ? out[out.length-1].x : 1;
    ch.overlay = buildOverlayFor(defCrop, xMin, xMax);

    ch.update();
  }

  setXGuidesVisible(cbXGuides.checked); // sichtbar bis Checkbox aus
  updateGuidePositions();
  if(cbYGuides.checked){ updateYGuidePositionsAll(); } else { setYGuidesVisible(false); }

  updateExportButtons();
}

/* Daten holen */
function fetchData(){
  if(!running) return;
  fetch('/data').then(function(r){return r.json();}).then(function(arr){
    for(var i=0;i<arr.length;i++){
      var d=arr[i];
      posFull.push({x:d.time, y:d.pos});
      velFull.push({x:d.time, y:d.vel});
      accFull.push({x:d.time, y:d.acc});
    }
    if(!isCropped && !yIsCropped){ refreshCharts(); }
  })["catch"](function(e){ console.log(e); });
}

/* Y-Guides anzeigen/initialisieren */
function setXGuidesVisible(show){
  var disp = show ? 'block' : 'none';
  guideL.style.display=disp;
  guideR.style.display=disp;
  // pro-Chart sichtbare Linien/Griffe
  for(var i=0;i<chartDefs.length;i++){
    var d=chartDefs[i];
    if(d.vL){ d.vL.style.display=disp; d.vR.style.display=disp; }
    if(d.vGripL){ d.vGripL.style.display=disp; d.vGripR.style.display=disp; }
  }
}
function setYGuidesVisible(show){
  var disp = show ? 'block' : 'none';
  for(var i=0;i<chartDefs.length;i++){
    chartDefs[i].gTop.style.display=disp;
    chartDefs[i].gBot.style.display=disp;
    // pro-Chart sichtbare Segmente + Griffe
    if(chartDefs[i].yLTop){ chartDefs[i].yLTop.style.display=disp; }
    if(chartDefs[i].yLBot){ chartDefs[i].yLBot.style.display=disp; }
    if(chartDefs[i].yGripTop){ chartDefs[i].yGripTop.style.display=disp; }
    if(chartDefs[i].yGripBot){ chartDefs[i].yGripBot.style.display=disp; }
  }
}
function updateYGuidePositionsAll(){
  if(!cbYGuides.checked){
    setYGuidesVisible(false);
    return;
  }
  for(var i=0;i<chartDefs.length;i++){
    var d=chartDefs[i];
    var visible = (d.wrapper.style.display!=='none');
    if(!visible || d.yTop===null || d.yBot===null){
      if(d.yLTop){ d.yLTop.style.display='none'; }
      if(d.yLBot){ d.yLBot.style.display='none'; }
      if(d.yGripTop){ d.yGripTop.style.display='none'; }
      if(d.yGripBot){ d.yGripBot.style.display='none'; }
      continue;
    }

    var ca = d.chart.chartArea;
    var canRect = d.chart.canvas.getBoundingClientRect();
    var wrapRect= d.wrapper.getBoundingClientRect();

    // X-Plotbereich (für Linienbreite & Griff-Mitte)
    var xLeft  = (canRect.left - wrapRect.left) + ca.left;
    var xRight = (canRect.left - wrapRect.left) + ca.right;
    var xMid   = (xLeft + xRight) / 2;
    var width  = Math.max(0, xRight - xLeft);

    // Y-Positionen in Wrapper-Koordinaten
    var chartsRect = chartsArea.getBoundingClientRect();
    var yTopDisp = displayValueToPxYFor(d, d.yTop) - wrapRect.top + chartsRect.top - chartsRect.top;
    var yBotDisp = displayValueToPxYFor(d, d.yBot) - wrapRect.top + chartsRect.top - chartsRect.top;

    // Clampen auf Plotfläche
    var yMin = (canRect.top - wrapRect.top)+ca.top;
    var yMax = (canRect.top - wrapRect.top)+ca.bottom;
    yTopDisp = Math.max(yMin, Math.min(yMax, yTopDisp));
    yBotDisp = Math.max(yMin, Math.min(yMax, yBotDisp));

    // Linien setzen
    d.yLTop.style.display='block'; d.yLBot.style.display='block';
    d.yLTop.style.left = xLeft+'px'; d.yLTop.style.width = width+'px'; d.yLTop.style.top = yTopDisp+'px';
    d.yLBot.style.left = xLeft+'px'; d.yLBot.style.width = width+'px'; d.yLBot.style.top = yBotDisp+'px';

    // Griffe mittig
    d.yGripTop.style.display='block'; d.yGripBot.style.display='block';
    d.yGripTop.style.left = xMid+'px'; d.yGripTop.style.top = yTopDisp+'px';
    d.yGripBot.style.left = xMid+'px'; d.yGripBot.style.top = yBotDisp+'px';
  }
  setYGuidesVisible(true);
}
function initYGuidesFromDataAll(){
  for(var i=0;i<chartDefs.length;i++){
    var d=chartDefs[i], yr=getYRangeFor(d), span=yr.max-yr.min;
    d.yBot = yr.min + 0.10*span;
    d.yTop = yr.min + 0.90*span;
  }
  updateYGuidePositionsAll();
}

/* ======= EXPORT: CSV & einzelne PNGs ======= */
function downloadBlob(filename, blob){
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  setTimeout(function(){ URL.revokeObjectURL(url); document.body.removeChild(a); }, 0);
}
function nowStamp(){
  var d=new Date();
  function pad(n){return (n<10?'0':'')+n;}
  return d.getFullYear()+"-"+pad(d.getMonth()+1)+"-"+pad(d.getDate())+"_"+pad(d.getHours())+pad(d.getMinutes())+pad(d.getSeconds());
}
// 3 signifikante Stellen
function sig3(x){
  if(x===null || x===undefined || x==="" || !isFinite(x)) return "";
  var n = Number(x);
  if(n===0) return "0";
  return n.toPrecision(3);
}
function exportCSV(){
  if(posFull.length===0) return;
  var lines = [];
  lines.push("time_s,position_m,velocity_mps,acceleration_mps2");
  var L = posFull.length;
  for(var i=0;i<L;i++){
    var t = posFull[i].x;
    if(isCropped && cropL!==null && cropR!==null){
      if(t<cropL || t>cropR) continue;
      t = t - cropL;
    }
    var p = posFull[i].y;
    var v = (i<velFull.length)?velFull[i].y:"";
    var a = (i<accFull.length)?accFull[i].y:"";
    lines.push(sig3(t)+","+sig3(p)+","+sig3(v)+","+sig3(a));
  }
  var csv = lines.join("\n");
  downloadBlob("daten_"+nowStamp()+".csv", new Blob([csv], {type:"text/csv"}));
}

/* Einzelnes Diagramm als PNG exportieren (ein Klick = ein Download, iOS-freundlich) */
function exportPNGOne(chart, label){
  try{
    var dataURL = chart.canvas.toDataURL('image/png');
    var bstr = atob(dataURL.split(',')[1]);
    var len = bstr.length;
    var u8 = new Uint8Array(len);
    for (var j=0;j<len;j++){ u8[j]=bstr.charCodeAt(j); }
    downloadBlob(label+"_"+nowStamp()+".png", new Blob([u8], {type:'application/octet-stream'}));
  }catch(e){ console.log("PNG export failed:", e); }
}

/* Export-Buttons aktivieren/deaktivieren passend zur Sichtbarkeit & Datenlage */
function updateExportButtons(){
  var btnPos = document.getElementById('btnPNGpos');
  var btnVel = document.getElementById('btnPNGvel');
  var btnAcc = document.getElementById('btnPNGacc');
  var posVisible = (document.getElementById('posWrapper').style.display!=='none') && posFull.length>0;
  var velVisible = (document.getElementById('velWrapper').style.display!=='none') && velFull.length>0;
  var accVisible = (document.getElementById('accWrapper').style.display!=='none') && accFull.length>0;
  btnPos.disabled = !posVisible;
  btnVel.disabled = !velVisible;
  btnAcc.disabled = !accVisible;
}

document.getElementById('btnExportCSV').addEventListener('click', exportCSV);
document.getElementById('btnPNGpos').addEventListener('click', function(){ exportPNGOne(posChart,'pos'); });
document.getElementById('btnPNGvel').addEventListener('click', function(){ exportPNGOne(velChart,'vel'); });
document.getElementById('btnPNGacc').addEventListener('click', function(){ exportPNGOne(accChart,'acc'); });

/* ======= Loops ======= */
setInterval(pollStatus,150);
setInterval(fetchData,100);
(function(){
  applyVisibility();
  pollStatus();
})();
</script>

</body></html>
)rawliteral";

// ---------- Sensor ----------
static inline uint16_t readAS5600Raw(){
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // RAW_ANGLE high
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if(Wire.available()<2) return lastRaw;
  uint16_t hi=Wire.read(), lo=Wire.read();
  return ((hi & 0x0F)<<8)|lo;
}

// ---------- HTTP ----------
void handleRoot(){ server.send_P(200,"text/html",INDEX_HTML); }

// /data: gibt ALLE bis dahin angefallenen Pakete (Array) zurück und leert den Buffer
void handleData(){
  int before = pktCount();
  String out="["; bool first=true;
  Packet p; int sent=0;
  while(popPacket(p)){
    if(!first) out+=","; first=false;
    out+="{\"time\":"+String(p.t,3)+",\"pos\":"+String(p.x,5)+",\"vel\":"+String(p.v,5)+",\"acc\":"+String(p.a,5)+"}";
    sent++;
  }
  out+="]";
  server.send(200,"application/json",out);
  DBG("[/data] sent=%d  before=%d  after=%d\n", sent, before, pktCount());
}

void handleToggle(){
  if(modeCurrent==MODE_MANUAL){
    measuring=!measuring;
    if(measuring){
      pos_m=vel_ms=acc_ms2=0; lastVel_ms=0; vel_display=0;
      lastRaw=readAS5600Raw();
      pktClear(); sampReset();
      t_meas_s = 0.0f;
      smoothHasT0=false; smoothT0=0.0f; smoothX0=0.0f;
      measureStartMs = millis();
      lastPacketMs   = measureStartMs;
      DBG("[MANUAL] START  measureStartMs=%lu  buffer cleared\n", measureStartMs);
    } else {
      DBG("[MANUAL] STOP]\n");
    }
  }
  server.send(200,"application/json",String("{\"running\":")+(measuring?"true":"false")+"}");
}

void handleSetMode(){
  String m=server.hasArg("m")?server.arg("m"):"manual";
  modeCurrent=(m=="incline")?MODE_INCLINE:MODE_MANUAL;
  if(modeCurrent==MODE_INCLINE){
    astate = CALIBRATING; cal_elapsed_s = 0; cal_noise_max=0;
    measuring=false;
    pos_m=vel_ms=acc_ms2=0; lastVel_ms=0; vel_display=0;
    lastRaw=readAS5600Raw();
    pktClear(); sampReset();
    smoothHasT0=false; smoothT0=0.0f; smoothX0=0.0f;
    DBG("[MODE] INCLINE -> CALIBRATING, buffer cleared\n");
  } else {
    astate = IDLE; measuring=false;
    pktClear(); sampReset();
    smoothHasT0=false; smoothT0=0.0f; smoothX0=0.0f;
    DBG("[MODE] MANUAL, buffer cleared\n");
  }
  server.send(200,"text/plain","OK");
}

void handleStatus(){
  String text = measuring ? "Manuell: Messung läuft" : "Manuell";
  String j="{\"mode\":\"manual\""
           +String(",\"running\":")+(measuring?"true":"false")
           +String(",\"text\":\"")+text+"\",\"start_th\":"+String(START_V_MS,3)
           +String(",\"stop_th\":")+String(STOP_V_MS,3)
           +String(",\"trim\":")+String(trimCountToSend)
           +String(",\"smooth\":")+String(SMOOTH_HALF)+"}";
  trimCountToSend=0;
  server.send(200,"application/json",j);
}

void handleRecal(){
  if(modeCurrent==MODE_INCLINE){
    astate = CALIBRATING; cal_elapsed_s=0; cal_noise_max=0; measuring=false;
    pktClear(); sampReset();
    smoothHasT0=false; smoothT0=0.0f; smoothX0=0.0f;
    DBG("[RECAL] -> CALIBRATING, buffer cleared\n");
  }
  server.send(200,"text/plain","OK");
}

void handleSetSmooth(){
  if(server.hasArg("n")){
    int n = server.arg("n").toInt();
    if(n<SMOOTH_HALF_MIN) n=SMOOTH_HALF_MIN;
    if(n>SMOOTH_HALF_MAX) n=SMOOTH_HALF_MAX;
    SMOOTH_HALF = n;
    DBG("[SMOOTH] set to ±%d samples\n", n);
  }
  server.send(200,"text/plain","OK");
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000);

  WiFi.softAP(ssid,password);
  Serial.println(WiFi.softAPIP());

  server.on("/",handleRoot);
  server.on("/data",handleData);
  server.on("/toggle",handleToggle);
  server.on("/status",handleStatus);
  server.on("/setmode",handleSetMode);
  server.on("/recal",handleRecal);
  server.on("/setsmooth",handleSetSmooth);
  server.begin();

  lastRaw=readAS5600Raw();
  lastSampleUs=micros();

  astate=CALIBRATING; // irrelevant im MANUAL-Zweig
  DBG("[BOOT] Ready. Buffer size=%d\n", PKT_BUF_SIZE);
}

// ---------- Loop ----------
void loop(){
  server.handleClient();
  uint32_t nowUs=micros();

  if(nowUs-lastSampleUs >= SAMPLE_US){
    float dt_s=(nowUs-lastSampleUs)/1000000.0f;
    lastSampleUs += SAMPLE_US;

    uint16_t raw=readAS5600Raw();
    int32_t rawDiff=(int32_t)raw - (int32_t)lastRaw;
    if(rawDiff >  ENCODER_STEPS/2) rawDiff -= ENCODER_STEPS;
    if(rawDiff < -ENCODER_STEPS/2) rawDiff += ENCODER_STEPS;
    lastRaw=raw;

    float delta_m=(rawDiff/(float)ENCODER_STEPS)*WHEEL_CIRCUMFERENCE;
    float v_now = delta_m/dt_s;
    float a_now = (v_now - vel_ms)/dt_s;

    vel_display = (1.0f - DISPLAY_ALPHA) * vel_display + DISPLAY_ALPHA * v_now;

    float v_abs = fabsf(v_now);
    vel_abs_filt=(1.0f-EMA_ALPHA)*vel_abs_filt + EMA_ALPHA*v_abs;

    if(modeCurrent==MODE_INCLINE){
      // ungenutzt in dieser UI
    } else {
      if(measuring){
        pos_m  += delta_m;
        acc_ms2 = a_now;
        vel_ms  = v_now;
        t_meas_s += dt_s; sampPush(t_meas_s, pos_m);
      } else {
        acc_ms2 = 0;
      }
    }

    lastVel_ms = measuring ? vel_ms : (0.5f*lastVel_ms + 0.5f*v_now);
  }

  uint32_t nowMs = millis();
  if(measuring && nowMs-lastPacketMs >= PACKET_MS){
    lastPacketMs += PACKET_MS;

    int N = SMOOTH_HALF; if(N < 1) N = 1;
    if((int)sampCount >= (2*N + 1)){
      const Sample& sL = sampRel(-(1 + 2*N));
      const Sample& sC = sampRel(-(1 + N));
      const Sample& sR = sampRel(-1);

      float v_out=0.0f, a_out=0.0f;

      float dtLR = (sR.t - sL.t);
      if(dtLR != 0.0f) v_out = (sR.x - sL.x) / dtLR;

      float dtLC = (sC.t - sL.t), dtCR = (sR.t - sC.t);
      float v_left  = (dtLC != 0.0f) ? (sC.x - sL.x)/dtLC : 0.0f;
      float v_right = (dtCR != 0.0f) ? (sR.x - sC.x)/dtCR : 0.0f;
      float t_mid_L = 0.5f*(sL.t + sC.t);
      float t_mid_R = 0.5f*(sC.t + sR.t);
      float dt_mid  = (t_mid_R - t_mid_L);
      if(dt_mid != 0.0f) a_out = (v_right - v_left) / dt_mid;

      if(!smoothHasT0){ smoothHasT0 = true; smoothT0 = sC.t; smoothX0 = sC.x; }

      float t_out = sC.t - smoothT0;
      float x_out = sC.x - smoothX0;

      pushPacket(t_out, x_out, v_out, a_out);
    }
  }
}
