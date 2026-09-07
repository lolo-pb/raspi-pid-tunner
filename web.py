"""FastAPI and single-page web UI for the Raspberry Pi PID tuner."""

from __future__ import annotations

import argparse
import asyncio
import os
import socket
from contextlib import asynccontextmanager
from typing import Literal

import uvicorn
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, PlainTextResponse
from pydantic import BaseModel

from tuner import MavlinkTuner, TunerError


class GainUpdate(BaseModel):
    axis: Literal["roll", "pitch", "yaw"]
    rate_p: float
    rate_i: float
    rate_d: float
    angle_p: float


class RestoreRequest(BaseModel):
    snapshot: Literal["previous", "initial"]


class ModeRequest(BaseModel):
    mode: Literal["STABILIZE"]


class ForceDisarmRequest(BaseModel):
    confirmation: str


class RunStartRequest(BaseModel):
    axis: Literal["roll", "pitch", "yaw"]


def _call(operation, *args):
    try:
        return operation(*args)
    except TunerError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


def _web_urls(host: str, port: int) -> list[str]:
    if host not in {"0.0.0.0", "::"}:
        return [f"http://{host}:{port}"]

    addresses: set[str] = set()
    try:
        addresses.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    try:
        import fcntl
        import struct

        for _, interface_name in socket.if_nameindex():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as interface:
                    packed = fcntl.ioctl(
                        interface.fileno(),
                        0x8915,  # Linux SIOCGIFADDR
                        struct.pack("256s", interface_name.encode()[:15]),
                    )
                    addresses.add(socket.inet_ntoa(packed[20:24]))
            except OSError:
                continue
    except (ImportError, OSError):
        pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            addresses.add(probe.getsockname()[0])
    except OSError:
        pass

    usable = sorted(address for address in addresses if not address.startswith("127.") and address != "0.0.0.0")
    if not usable:
        usable = ["127.0.0.1"]
    return [f"http://{address}:{port}" for address in usable]


def create_app(tuner: MavlinkTuner, host: str = "0.0.0.0", port: int = 8000) -> FastAPI:
    web_urls = _web_urls(host, port)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        print("PID tuner UI:\n  " + "\n  ".join(web_urls), flush=True)
        tuner.start()
        try:
            yield
        finally:
            tuner.close()

    application = FastAPI(title="ArduPilot Bench PID Tuner", lifespan=lifespan)

    @application.middleware("http")
    async def reject_cross_origin_controls(request: Request, call_next):
        origin = request.headers.get("origin")
        host = request.headers.get("host")
        if request.method not in {"GET", "HEAD"} and origin and host:
            allowed_origins = {f"http://{host}", f"https://{host}"}
            if origin.rstrip("/") not in allowed_origins:
                return PlainTextResponse("Cross-origin control request rejected", status_code=403)
        return await call_next(request)

    @application.get("/", response_class=HTMLResponse)
    def index():
        return HTML

    @application.get("/api/status")
    def status():
        result = tuner.status()
        result["web_urls"] = web_urls
        return result

    @application.get("/api/parameters")
    def parameters(axis: Literal["roll", "pitch", "yaw"]):
        return _call(tuner.gains, axis)

    @application.post("/api/parameters/apply")
    def apply_parameters(update: GainUpdate):
        return _call(
            tuner.apply_gains,
            update.axis,
            {
                "rate_p": update.rate_p,
                "rate_i": update.rate_i,
                "rate_d": update.rate_d,
                "angle_p": update.angle_p,
            },
        )

    @application.post("/api/parameters/restore")
    def restore_parameters(request: RestoreRequest):
        return _call(tuner.restore_parameters, request.snapshot)

    @application.post("/api/control/mode")
    def set_mode(request: ModeRequest):
        return _call(tuner.set_stabilize_mode)

    @application.post("/api/control/arm")
    def arm():
        return _call(tuner.arm)

    @application.post("/api/control/disarm")
    def disarm():
        return _call(tuner.disarm, False)

    @application.post("/api/control/force-disarm")
    def force_disarm(request: ForceDisarmRequest):
        if request.confirmation != "FORCE DISARM":
            raise HTTPException(status_code=400, detail="Confirmation must be exactly FORCE DISARM")
        return _call(tuner.disarm, True)

    @application.post("/api/runs/start")
    def start_run(request: RunStartRequest):
        return _call(tuner.start_run, request.axis)

    @application.post("/api/runs/stop")
    def stop_run():
        return _call(tuner.stop_run)

    @application.get("/api/runs")
    def runs():
        return tuner.list_runs()

    @application.get("/api/runs/{run_id}")
    def run(run_id: str):
        return _call(tuner.get_run, run_id)

    @application.get("/api/runs/{run_id}/csv", response_class=PlainTextResponse)
    def run_csv(run_id: str):
        try:
            content = tuner.run_csv(run_id)
        except TunerError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return PlainTextResponse(
            content,
            media_type="text/csv",
            headers={"Content-Disposition": f'attachment; filename="{run_id}.csv"'},
        )

    @application.websocket("/ws/telemetry")
    async def telemetry(websocket: WebSocket):
        origin = websocket.headers.get("origin")
        host = websocket.headers.get("host")
        if origin and host and origin.rstrip("/") not in {f"http://{host}", f"https://{host}"}:
            await websocket.close(code=1008)
            return
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(tuner.telemetry())
                await asyncio.sleep(0.05)
        except (WebSocketDisconnect, RuntimeError):
            return

    return application


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ArduPilot Bench PID Tuner</title>
  <style>
    :root { color-scheme: dark; --bg:#0b1118; --panel:#131d28; --line:#2b3b4d; --text:#e9f0f7; --muted:#90a3b7; --blue:#55a7ff; --orange:#ffad5a; --green:#58d68d; --red:#ff6262; }
    * { box-sizing:border-box; }
    body { margin:0; background:var(--bg); color:var(--text); font:15px/1.4 system-ui,sans-serif; }
    header { padding:16px 22px; background:#111923; border-bottom:1px solid var(--line); position:sticky; top:0; z-index:2; }
    h1,h2 { margin:0 0 12px; } h1 { font-size:20px; } h2 { font-size:17px; }
    .danger { margin-top:10px; padding:9px 12px; background:#4a1717; border:1px solid #a33; border-radius:6px; font-weight:700; }
    main { max-width:1400px; margin:auto; padding:16px; display:grid; gap:14px; }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); gap:14px; }
    section { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:14px; min-width:0; }
    .row { display:flex; flex-wrap:wrap; align-items:center; gap:9px; margin:8px 0; }
    label { color:var(--muted); } input,select,button { font:inherit; color:var(--text); background:#0c141e; border:1px solid #40546a; border-radius:5px; padding:7px 9px; }
    input { width:110px; } button { cursor:pointer; } button:hover { border-color:var(--blue); } button:disabled { cursor:not-allowed; opacity:.45; }
    button.primary { background:#164b78; } button.safe { background:#174b34; } button.danger-button { background:#761f1f; border-color:#d34b4b; }
    .badge { display:inline-block; padding:3px 8px; border-radius:99px; background:#3a2630; } .badge.ok { background:#164b34; }
    .muted { color:var(--muted); } .error { color:#ff8585; white-space:pre-wrap; } .good { color:var(--green); }
    canvas { width:100%; height:250px; display:block; background:#0b121a; border:1px solid var(--line); border-radius:5px; }
    table { width:100%; border-collapse:collapse; } th,td { padding:7px; border-bottom:1px solid var(--line); text-align:left; font-size:13px; }
    pre { max-height:300px; overflow:auto; background:#0b121a; padding:10px; border-radius:5px; }
    .legend span { margin-right:16px; } .dot { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:5px; }
    #forceHold { display:none; } #runTable { overflow:auto; }
  </style>
</head>
<body>
<header>
  <h1>ArduPilot Bench PID Tuner</h1>
  <span id="connectionBadge" class="badge">Disconnected</span>
  <span id="vehicleSummary" class="muted"></span>
  <span class="muted"> · Network UI: </span><span id="networkUrls" class="good"></span>
  <div class="danger">Powered propellers: secure the one-axis rig, keep people clear, and retain a physical transmitter and power cutoff.</div>
</header>
<main>
  <div class="grid">
    <section>
      <h2>Vehicle control</h2>
      <div class="row"><b>Mode:</b> <span id="mode">UNKNOWN</span> <b>State:</b> <span id="armed">DISARMED</span></div>
      <div id="batterySummary" class="muted"></div>
      <div class="row">
        <button id="stabilize">Set Stabilize</button>
        <button id="arm" class="safe">Arm</button>
        <button id="disarm">Disarm</button>
      </div>
      <div class="row">
        <button id="enableForce" class="danger-button">Enable force disarm</button>
        <button id="forceHold" class="danger-button">Hold 3 seconds: FORCE DISARM</button>
      </div>
      <div id="statusError" class="error"></div>
      <div id="statusText" class="muted"></div>
    </section>

    <section>
      <h2>Axis and gains</h2>
      <div class="row"><label for="axis">Axis</label><select id="axis"><option>roll</option><option>pitch</option><option>yaw</option></select></div>
      <div class="row"><label>Rate P <input id="rate_p" type="number" min="0" step="any"></label><label>Rate I <input id="rate_i" type="number" min="0" step="any"></label></div>
      <div class="row"><label>Rate D <input id="rate_d" type="number" min="0" step="any"></label><label>Angle P <input id="angle_p" type="number" min="0" step="any"></label></div>
      <div class="row"><button id="apply" class="primary">Review and apply</button><button id="undo">Undo last apply</button><button id="restore">Restore connection snapshot</button></div>
      <div id="gainMessage" class="muted"></div>
    </section>

    <section>
      <h2>Push-and-release test</h2>
      <p class="muted">Arm in Stabilize, press Start, use the rig mechanism to push and release the selected axis from outside the propeller envelope, then press Stop.</p>
      <div class="row"><button id="start" class="primary">Start recording</button><button id="stop">Stop and analyze</button><span id="recording" class="badge">Idle</span></div>
      <div id="runMessage" class="error"></div>
    </section>
  </div>

  <section>
    <h2>Live response — <span id="liveAxis">roll</span></h2>
    <div class="legend"><span><i class="dot" style="background:#55a7ff"></i>Target</span><span><i class="dot" style="background:#ffad5a"></i>Actual</span></div>
    <p class="muted">Angle (degrees)</p><canvas id="angleChart"></canvas>
    <p class="muted">Angular rate (degrees/second)</p><canvas id="rateChart"></canvas>
  </section>

  <div class="grid">
    <section><h2>Latest metrics</h2><pre id="metrics">No completed run selected.</pre></section>
    <section>
      <h2>Compare runs</h2>
      <div class="row"><select id="compareA"></select><select id="compareB"></select><button id="compare">Compare</button></div>
      <p class="muted">Angle error aligned at detected release (seconds)</p><canvas id="compareChart"></canvas>
    </section>
  </div>

  <section><h2>Saved runs</h2><div id="runTable"></div></section>
</main>
<script>
const $ = id => document.getElementById(id);
const fields = ['rate_p','rate_i','rate_d','angle_p'];
let live = [], forceTimer = null, forceEnabledUntil = 0, gainsReady = false;

async function request(path, options={}) {
  if (options.body && typeof options.body !== 'string') {
    options.headers = {'Content-Type':'application/json'};
    options.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, options);
  const text = await response.text();
  let body = text;
  try { body = text ? JSON.parse(text) : {}; } catch (_) {}
  if (!response.ok) throw new Error(body.detail || text || response.statusText);
  return body;
}

function showError(element, error) { $(element).textContent = error ? String(error.message || error) : ''; }

async function refreshStatus() {
  try {
    const status = await request('/api/status');
    $('connectionBadge').textContent = status.connected ? 'Connected' : 'Disconnected';
    $('connectionBadge').className = 'badge' + (status.connected ? ' ok' : '');
    $('vehicleSummary').textContent = ` ${status.vehicle} · ArduPilot ${status.firmware} · ${status.device} @ ${status.baud}`;
    $('networkUrls').textContent = (status.web_urls || [location.origin]).join(' · ');
    $('mode').textContent = status.mode; $('armed').textContent = status.armed ? 'ARMED' : 'DISARMED';
    $('armed').className = status.armed ? 'error' : 'good';
    $('recording').textContent = status.recording ? `Recording ${status.recording_axis}` : 'Idle';
    $('recording').className = 'badge' + (status.recording ? ' ok' : '');
    $('statusError').textContent = status.last_error || (status.parameter_writes_locked ? 'Parameter writes locked after failed rollback.' : '');
    $('statusText').textContent = (status.status_text || []).slice(-3).join(' · ');
    $('apply').disabled = !status.connected || status.armed || status.recording || !status.parameters_ready || status.parameter_writes_locked;
    $('undo').disabled = !status.connected || status.armed || status.recording || status.parameter_writes_locked;
    $('restore').disabled = !status.connected || status.armed || status.recording || status.parameter_writes_locked;
    $('stabilize').disabled = !status.connected || status.recording;
    $('arm').disabled = !status.connected || status.armed || status.mode !== 'STABILIZE' || status.recording;
    $('disarm').disabled = !status.connected || !status.armed;
    $('enableForce').disabled = !status.connected || !status.armed;
    $('start').disabled = !status.connected || !status.armed || status.mode !== 'STABILIZE' || status.recording || !status.parameters_ready;
    $('stop').disabled = !status.recording;
    if (!status.connected) gainsReady=false;
    if (status.parameters_ready && !gainsReady) { gainsReady=true; await loadGains(); }
  } catch (error) { showError('statusError', error); }
}

async function loadGains() {
  $('liveAxis').textContent = $('axis').value;
  try {
    const result = await request(`/api/parameters?axis=${$('axis').value}`);
    fields.forEach(field => $(field).value = result.values[field] ?? '');
    $('gainMessage').textContent = result.ready ? 'Values read from the flight controller.' : 'Waiting for all PID parameters…';
  } catch (error) { showError('gainMessage', error); }
}

async function control(path, body={}) {
  showError('statusError', null);
  try { await request(path, {method:'POST', body}); await refreshStatus(); }
  catch (error) { showError('statusError', error); }
}

$('stabilize').onclick = () => control('/api/control/mode', {mode:'STABILIZE'});
$('arm').onclick = () => { if(confirm('The secured rig is clear. Arm the motors?')) control('/api/control/arm'); };
$('disarm').onclick = () => control('/api/control/disarm');
$('axis').onchange = () => { live=[]; loadGains(); };

$('apply').onclick = async () => {
  if (fields.some(field => $(field).value.trim() === '')) { $('gainMessage').textContent='Every gain needs a value.'; return; }
  const body = {axis:$('axis').value}; fields.forEach(field => body[field] = Number($(field).value));
  if (fields.some(field => !Number.isFinite(body[field]) || body[field] < 0)) { $('gainMessage').textContent='Gains must be finite, nonnegative numbers.'; return; }
  const description = fields.map(field => `${field}: ${body[field]}`).join('\n');
  if (!confirm(`Apply these gains while disarmed?\n\n${description}`)) return;
  try { await request('/api/parameters/apply', {method:'POST', body}); $('gainMessage').textContent='Gains applied and verified.'; await loadGains(); }
  catch (error) { showError('gainMessage', error); }
};
$('undo').onclick = async () => { if (confirm('Restore the values from before the last apply?')) await restoreSnapshot('previous'); };
$('restore').onclick = async () => { if (confirm('Restore all PID values captured when this FC connected?')) await restoreSnapshot('initial'); };
async function restoreSnapshot(snapshot) {
  try { await request('/api/parameters/restore', {method:'POST', body:{snapshot}}); $('gainMessage').textContent=`Restored ${snapshot} snapshot.`; await loadGains(); }
  catch (error) { showError('gainMessage', error); }
}

$('start').onclick = async () => {
  try { const run=await request('/api/runs/start',{method:'POST',body:{axis:$('axis').value}}); $('runMessage').textContent=`Recording ${run.id}`; await refreshStatus(); }
  catch(error){ showError('runMessage',error); }
};
$('stop').onclick = async () => {
  try { const run=await request('/api/runs/stop',{method:'POST'}); $('runMessage').textContent=`Saved ${run.id}`; showMetrics(run.metrics); await refreshRuns(); await refreshStatus(); }
  catch(error){ showError('runMessage',error); }
};

$('enableForce').onclick = () => {
  if (prompt('Type FORCE DISARM to reveal the guarded control:') !== 'FORCE DISARM') return;
  forceEnabledUntil = Date.now() + 15000; $('forceHold').style.display='inline-block';
  setTimeout(() => { if (Date.now() >= forceEnabledUntil) $('forceHold').style.display='none'; }, 15100);
};
$('forceHold').onpointerdown = () => {
  if (Date.now() >= forceEnabledUntil) return;
  $('forceHold').textContent='Keep holding…';
  forceTimer=setTimeout(async()=>{ forceTimer=null; $('forceHold').style.display='none'; $('forceHold').textContent='Hold 3 seconds: FORCE DISARM'; await control('/api/control/force-disarm',{confirmation:'FORCE DISARM'}); },3000);
};
function cancelForce() { if(forceTimer){clearTimeout(forceTimer);forceTimer=null;} $('forceHold').textContent='Hold 3 seconds: FORCE DISARM'; }
$('forceHold').onpointerup=cancelForce; $('forceHold').onpointerleave=cancelForce; $('forceHold').onpointercancel=cancelForce;

function showMetrics(metrics) { $('metrics').textContent = metrics ? JSON.stringify(metrics,null,2) : 'Run could not be analyzed.'; }

function drawChart(canvas, series) {
  const ratio=window.devicePixelRatio||1, width=canvas.clientWidth, height=canvas.clientHeight;
  canvas.width=width*ratio; canvas.height=height*ratio;
  const ctx=canvas.getContext('2d'); ctx.scale(ratio,ratio); ctx.clearRect(0,0,width,height);
  const points=series.flatMap(item=>item.points).filter(point=>Number.isFinite(point.x)&&Number.isFinite(point.y));
  if(points.length<2){ctx.fillStyle='#90a3b7';ctx.fillText('Waiting for data…',12,22);return;}
  let minX=Math.min(...points.map(p=>p.x)), maxX=Math.max(...points.map(p=>p.x));
  let minY=Math.min(...points.map(p=>p.y)), maxY=Math.max(...points.map(p=>p.y));
  if(maxX===minX)maxX=minX+1; if(maxY===minY){maxY+=1;minY-=1;} const pad=(maxY-minY)*.1;minY-=pad;maxY+=pad;
  const left=48,right=10,top=10,bottom=25, px=x=>left+(x-minX)/(maxX-minX)*(width-left-right), py=y=>top+(maxY-y)/(maxY-minY)*(height-top-bottom);
  ctx.strokeStyle='#2b3b4d';ctx.fillStyle='#90a3b7';ctx.font='11px system-ui';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=minY+(maxY-minY)*i/4,sy=py(y);ctx.beginPath();ctx.moveTo(left,sy);ctx.lineTo(width-right,sy);ctx.stroke();ctx.fillText(y.toFixed(1),3,sy+4);}
  series.forEach(item=>{ctx.strokeStyle=item.color;ctx.lineWidth=2;ctx.beginPath();let started=false;item.points.forEach(point=>{if(!Number.isFinite(point.y))return;const x=px(point.x),y=py(point.y);started?ctx.lineTo(x,y):ctx.moveTo(x,y);started=true;});ctx.stroke();});
}

function updateLive(data) {
  const axis=$('axis').value, now=Date.now()/1000;
  const voltage=Number.isFinite(data.battery_voltage_v)?`${data.battery_voltage_v.toFixed(2)} V`:'voltage unavailable';
  const remaining=Number.isFinite(data.battery_remaining_percent)?` · ${data.battery_remaining_percent}%`:'';
  $('batterySummary').textContent=`Battery: ${voltage}${remaining}`;
  live.push({t:now,targetAngle:data.target_angles_deg[axis],actualAngle:data.angles_deg[axis],targetRate:data.target_rates_deg_s[axis],actualRate:data.rates_deg_s[axis]});
  live=live.filter(point=>point.t>=now-15); const start=live[0]?.t||now;
  drawChart($('angleChart'),[
    {color:'#55a7ff',points:live.map(p=>({x:p.t-start,y:p.targetAngle}))},
    {color:'#ffad5a',points:live.map(p=>({x:p.t-start,y:p.actualAngle}))}
  ]);
  drawChart($('rateChart'),[
    {color:'#55a7ff',points:live.map(p=>({x:p.t-start,y:p.targetRate}))},
    {color:'#ffad5a',points:live.map(p=>({x:p.t-start,y:p.actualRate}))}
  ]);
}

async function refreshRuns() {
  try {
    const runs=await request('/api/runs');
    $('runTable').innerHTML='<table><thead><tr><th>Started</th><th>Axis</th><th>Status</th><th>Samples</th><th>Actions</th></tr></thead><tbody>'+runs.map(run=>`<tr><td>${run.started_at}</td><td>${run.axis}</td><td>${run.status}</td><td>${run.sample_count}</td><td><button data-view="${run.id}">View</button> <a href="/api/runs/${run.id}/csv">CSV</a></td></tr>`).join('')+'</tbody></table>';
    document.querySelectorAll('[data-view]').forEach(button=>button.onclick=async()=>showMetrics((await request(`/api/runs/${button.dataset.view}`)).metrics));
    for(const select of [$('compareA'),$('compareB')]){const selected=select.value;select.innerHTML=runs.map(run=>`<option value="${run.id}">${run.started_at} · ${run.axis}</option>`).join('');if(runs.some(run=>run.id===selected))select.value=selected;}
    if(runs.length>1&&$('compareA').value===$('compareB').value)$('compareB').selectedIndex=1;
  } catch(error){ $('runTable').textContent=error.message; }
}

$('compare').onclick=async()=>{
  try{
    const [a,b]=await Promise.all([request(`/api/runs/${$('compareA').value}`),request(`/api/runs/${$('compareB').value}`)]);
    const toPoints=run=>{const release=run.metrics?.release_time_s||0;return run.samples.map(sample=>({x:sample.t-release,y:sample.angle_error_deg}));};
    drawChart($('compareChart'),[{color:'#55a7ff',points:toPoints(a)},{color:'#ffad5a',points:toPoints(b)}]);
  }catch(error){showError('runMessage',error);}
};

function connectWebSocket(){const protocol=location.protocol==='https:'?'wss':'ws';const socket=new WebSocket(`${protocol}://${location.host}/ws/telemetry`);socket.onmessage=event=>updateLive(JSON.parse(event.data));socket.onclose=()=>setTimeout(connectWebSocket,1000);}
loadGains(); refreshStatus(); refreshRuns(); connectWebSocket(); setInterval(refreshStatus,1000);
</script>
</body>
</html>
"""


default_host = os.getenv("PID_TUNER_HOST", "0.0.0.0")
default_port = int(os.getenv("PID_TUNER_PORT", "8000"))
default_tuner = MavlinkTuner(
    os.getenv("PID_TUNER_DEVICE", "/dev/serial0"),
    int(os.getenv("PID_TUNER_BAUD", "921600")),
    os.getenv("PID_TUNER_DATA_DIR", "runs"),
)
app = create_app(default_tuner, default_host, default_port)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArduPilot one-axis bench PID tuner")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--data-dir", default="runs")
    arguments = parser.parse_args()
    runtime_tuner = MavlinkTuner(arguments.device, arguments.baud, arguments.data_dir)
    uvicorn.run(
        create_app(runtime_tuner, arguments.host, arguments.port),
        host=arguments.host,
        port=arguments.port,
    )
