"""FastAPI tuning panel for the dual-hand teleop service.

Real left/right tabs, live commanded-vs-actual bars per finger, retargeting and
drive tuning, and guided open/fist calibration. Served on 127.0.0.1:<port>.
"""
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel
import uvicorn


class ParamReq(BaseModel):
    hand: str
    key: str
    value: float


class JointReq(BaseModel):
    hand: str
    idx: int
    field: str
    value: float


class HandCmd(BaseModel):
    hand: str
    cmd: str


def build_app(service):
    app = FastAPI(title="DexHand Teleop Panel")

    @app.get("/", response_class=HTMLResponse)
    def index():
        return _HTML

    @app.get("/api/state")
    def state():
        return JSONResponse(service.snapshot())

    @app.post("/api/param")
    def param(req: ParamReq):
        w = service.workers.get(req.hand)
        if not w:
            return {"ok": False}
        w.set_param(req.key, req.value)
        return {"ok": True}

    @app.post("/api/joint")
    def joint(req: JointReq):
        w = service.workers.get(req.hand)
        if not w:
            return {"ok": False}
        w.set_joint_param(req.idx, req.field, req.value)
        return {"ok": True}

    @app.post("/api/capture")
    def capture(req: HandCmd):
        w = service.workers.get(req.hand)
        if not w:
            return {"ok": False}
        return {"ok": True, **w.capture(req.cmd)}

    @app.post("/api/command")
    def command(req: HandCmd):
        w = service.workers.get(req.hand)
        if not w:
            return {"ok": False}
        w.command(req.cmd)
        return {"ok": True}

    return app


def run_panel(service, port: int = 8090):
    app = build_app(service)
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")


_HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>灵巧手遥操调参面板</title>
<style>
*{box-sizing:border-box} body{margin:0;background:#0a0e17;color:#dfe7f3;font-family:Segoe UI,Microsoft YaHei,Arial,sans-serif}
.app{max-width:1040px;margin:0 auto;padding:16px}
.tabs{display:flex;gap:10px;margin-bottom:14px}
.tab{flex:1;height:50px;border:1px solid #243049;background:#0e1626;color:#8aa;border-radius:12px;font-size:20px;font-weight:700;cursor:pointer}
.tab.active{background:linear-gradient(135deg,#1f9ff0,#3b6bff);color:#fff;border:0}
.tab:disabled{opacity:.35;cursor:not-allowed}
.bar{display:flex;justify-content:space-between;align-items:center;background:#0e1626;border:1px solid #1d2840;border-radius:12px;padding:10px 16px;margin-bottom:14px;font-size:14px;color:#9fb0c8}
.bar b{color:#46d6ff}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#f55;margin-right:6px}
.dot.on{background:#3ed67b}
.card{background:#0b1220;border:1px solid #1c2740;border-radius:14px;padding:16px 18px;margin-bottom:14px}
.card h2{margin:0 0 12px;font-size:15px;color:#9fb0c8;font-weight:700}
.frow{display:grid;grid-template-columns:90px 1fr 1fr;gap:12px;align-items:center;margin:9px 0}
.frow .nm{font-weight:700;color:#cdd8ea}
.track{height:18px;border-radius:9px;background:#1b2236;position:relative;overflow:hidden}
.track>span{position:absolute;left:0;top:0;height:100%;border-radius:9px}
.cmd>span{background:linear-gradient(90deg,#2b8cff,#46d6ff)}
.act>span{background:linear-gradient(90deg,#ff7a59,#ffd166)}
.lab{font-size:11px;color:#7e8ba3;margin-bottom:3px;display:flex;justify-content:space-between}
.srow{display:grid;grid-template-columns:130px 1fr 60px;gap:12px;align-items:center;margin:10px 0}
.srow label{color:#bac6da;font-weight:600;font-size:14px}
input[type=range]{width:100%;accent-color:#46c7ff}
input[type=number]{width:100%;background:#0a111e;border:1px solid #243049;color:#eef6ff;border-radius:7px;padding:6px 8px}
.val{text-align:right;color:#8fd}
.btns{display:flex;gap:10px;flex-wrap:wrap}
button.act-btn{border:0;border-radius:9px;padding:10px 16px;font-weight:700;cursor:pointer}
.b-cap{background:#1f6feb;color:#fff}.b-warn{background:#b5532a;color:#fff}.b-ghost{background:#16203a;color:#bcd;border:1px solid #2a3550}
.adv{margin-top:8px}.adv table{width:100%;border-collapse:collapse;font-size:12px}
.adv th,.adv td{padding:4px 6px;border-bottom:1px solid #18223a;text-align:center}
.adv input{width:64px}
.note{font-size:12px;color:#6f7d96;margin-top:6px}
.pill{font-size:12px;padding:2px 8px;border-radius:999px;background:#16203a;color:#9fb0c8;margin-left:6px}
.pill.warn{background:#3a2a16;color:#ffd166}
</style></head>
<body><div class="app">
  <div class="tabs" id="tabs"></div>
  <a id="viz3d" href="#" target="_blank" style="display:none;text-decoration:none;margin-bottom:14px;padding:13px 16px;border-radius:12px;background:linear-gradient(135deg,#125a3a,#1f9ff0);color:#fff;font-weight:700;font-size:15px">&#129482; &#25171;&#24320; 3D &#20223;&#30495;&#35270;&#22270;&#65288;&#30475;&#26144;&#23556;&#23545;&#19981;&#23545; + &#28789;&#24039;&#25163;&#23454;&#26102;&#29366;&#24577;&#65289; &rarr;</a>
  <div class="bar">
    <div><span class="dot" id="dot"></span><span id="status">连接中…</span></div>
    <div>速率 <b id="rate">0</b> Hz · 包 <b id="pkts">0</b> · 写入 <span id="we" class="pill">?</span></div>
  </div>

  <div class="card"><h2>手指：指令(蓝) vs 实际电机位置(橙) · 0–1000</h2><div id="fingers"></div></div>

  <div class="card"><h2>重映射参数（实时生效）</h2>
    <div class="srow"><label>缩放 scaling</label><input id="p_scaling" type="range" min="0.5" max="2" step="0.01"><div class="val" id="v_scaling"></div></div>
    <div class="srow"><label>低通 low_pass</label><input id="p_low_pass_alpha" type="range" min="0.02" max="1" step="0.01"><div class="val" id="v_low_pass_alpha"></div></div>
    <div class="srow"><label>寄存器平滑</label><input id="p_smoothing_alpha" type="range" min="0.05" max="1" step="0.01"><div class="val" id="v_smoothing_alpha"></div></div>
    <div class="srow"><label>死区 deadband</label><input id="p_deadband" type="range" min="0" max="40" step="1"><div class="val" id="v_deadband"></div></div>
    <div class="note">缩放=人手/机器手尺寸比；低通越小越平滑但越滞后。</div>
  </div>

  <div class="card"><h2>平滑 / 防突兀（实时生效）</h2>
    <div class="srow"><label>限速 max_step</label><input id="p_max_step" type="range" min="0" max="200" step="5"><div class="val" id="v_max_step"></div></div>
    <div class="srow"><label>捏合触发 project</label><input id="p_project_dist" type="range" min="0.004" max="0.05" step="0.001"><div class="val" id="v_project_dist"></div></div>
    <div class="srow"><label>捏合释放 escape</label><input id="p_escape_dist" type="range" min="0.006" max="0.06" step="0.001"><div class="val" id="v_escape_dist"></div></div>
    <div class="note">限速=每帧寄存器最大变化(越小越柔、越滞后)；捏合触发越小，对捏越不“突兀强拉”(仅最后这点距离才助力合拢)。escape 要略大于 project。</div>
  </div>

  <div class="card"><h2>速度 / 力（一次性下发 6–11 / 12–17）</h2>
    <div class="srow"><label>速度 speed</label><input id="p_speed" type="range" min="0" max="1000" step="10"><div class="val" id="v_speed">600</div></div>
    <div class="srow"><label>力 force</label><input id="p_force" type="range" min="0" max="1000" step="10"><div class="val" id="v_force">400</div></div>
  </div>

  <div class="card"><h2>行程微调 / 控制</h2>
    <div class="note" style="margin-top:0;margin-bottom:12px">映射本身<b style="color:#46d6ff">免标定</b>。这里只调「关节角 → 电机行程(0–1000)」的范围/方向，都是<b>可选</b>的——默认 URDF 极限即可用，仅当“满握没到满闭合 / 张开有残留 / 个别指方向反”时才用。</div>
    <div class="btns">
      <button class="act-btn b-cap" onclick="cap('open')">捕获张开</button>
      <button class="act-btn b-cap" onclick="cap('fist')">捕获握拳</button>
      <button class="act-btn b-ghost" onclick="cmd('reset')">重置滤波</button>
      <button class="act-btn b-warn" onclick="cmd('relax')">松开(归零)</button>
      <span class="pill" id="capst"></span>
    </div>
    <div class="note">用满行程:张开手→捕获张开;用力握拳→捕获握拳。两者都采到后,把你的张开~握拳范围对齐到电机 0–1000 满行程。</div>
    <div class="adv">
      <button type="button" class="act-btn b-ghost" style="margin-top:12px" onclick="var a=document.getElementById('jwrap');a.style.display=a.style.display==='none'?'block':'none'">逐关节高级 ▾</button>
      <div id="jwrap" style="display:none">
        <table style="margin-top:10px"><thead><tr><th>reg</th><th>关节</th><th>qmin</th><th>qmax</th><th>out_lo</th><th>out_hi</th><th>反向</th></tr></thead>
        <tbody id="jtbody"></tbody></table>
      </div>
    </div>
  </div>
</div>
<script>
const FNAMES=['拇指旋转','拇指弯曲','食指','中指','无名指','小指'];
let hands=[], cur=null, dragging=false;
const $=id=>document.getElementById(id);
['p_scaling','p_low_pass_alpha','p_smoothing_alpha','p_max_step','p_project_dist','p_escape_dist','p_deadband','p_speed','p_force'].forEach(id=>{
  const el=$(id); el.addEventListener('input',()=>{dragging=true; const v=$('v_'+id.slice(2)); if(v)v.textContent=el.value;});
  el.addEventListener('change',()=>{dragging=false; sendParam(id.slice(2), parseFloat(el.value));});
});
async function sendParam(key,value){if(!cur)return; await fetch('/api/param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({hand:cur,key,value})});}
async function sendJoint(idx,field,value){if(!cur)return; await fetch('/api/joint',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({hand:cur,idx,field,value})});}
async function cap(pose){if(!cur)return; const r=await(await fetch('/api/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({hand:cur,cmd:pose})})).json(); $('capst').textContent='已采:'+(r.have||[]).join('/');}
async function cmd(c){if(!cur)return; await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({hand:cur,cmd:c})});}
function fingersHtml(){let h='';for(let i=0;i<6;i++){h+=`<div class="frow"><div class="nm">${FNAMES[i]}</div>
  <div><div class="lab"><span>指令</span><span id="cv${i}">0</span></div><div class="track cmd"><span id="cb${i}"></span></div></div>
  <div><div class="lab"><span>实际/力</span><span id="av${i}">-</span></div><div class="track act"><span id="ab${i}"></span></div></div></div>`;}
  $('fingers').innerHTML=h;}
function jointsHtml(js){let h='';js.forEach((j,i)=>{h+=`<tr><td>${i}</td><td style="text-align:left">${j.name}</td>
  <td><input type="number" step="0.01" value="${j.qmin}" onchange="sendJoint(${i},'qmin',parseFloat(this.value))"></td>
  <td><input type="number" step="0.01" value="${j.qmax}" onchange="sendJoint(${i},'qmax',parseFloat(this.value))"></td>
  <td><input type="number" step="1" value="${j.out_lo}" onchange="sendJoint(${i},'out_lo',parseFloat(this.value))"></td>
  <td><input type="number" step="1" value="${j.out_hi}" onchange="sendJoint(${i},'out_hi',parseFloat(this.value))"></td>
  <td><input type="checkbox" ${j.invert?'checked':''} onchange="sendJoint(${i},'invert',this.checked?1:0)"></td></tr>`;});
  $('jtbody').innerHTML=h;}
function buildTabs(names){hands=names; $('tabs').innerHTML=['right','left'].map(n=>{
  const on=names.includes(n); return `<button class="tab ${n===cur?'active':''}" ${on?'':'disabled'} onclick="selTab('${n}')">${n==='right'?'右手':'左手'}</button>`;}).join('');}
function selTab(n){cur=n; document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active')); event&&event.target&&event.target.classList.add('active'); applyParams=true;}
let applyParams=true, lastJointsKey='';
async function tick(){
  let s; try{s=await(await fetch('/api/state')).json();}catch(e){return;}
  const names=Object.keys(s.hands); if(!hands.length||hands.join()!=names.join()){if(!cur)cur=names[0]; buildTabs(names);}
  $('pkts').textContent=s.packets;
  if(s.meshcat_url){const v=$('viz3d'); v.href=s.meshcat_url; v.style.display='block';}
  const h=s.hands[cur]; if(!h)return;
  $('dot').className='dot'+(h.connected?' on':''); $('status').textContent=cur.toUpperCase()+(h.connected?` 跟踪中 (age ${h.age}s)`:' 无数据');
  $('rate').textContent=h.rate; $('we').textContent=h.write_enabled?'已连真机':'空跑(dry-run)'; $('we').className='pill'+(h.write_enabled?'':' warn');
  const fb=h.feedback;
  for(let i=0;i<6;i++){const reg=h.registers[i]||0; $('cb'+i).style.width=(reg/10)+'%'; $('cv'+i).textContent=reg;
    if(fb){const mp=fb.motor_pos[i]||0; $('ab'+i).style.width=(mp/10)+'%';
      const force=fb.force_g[i*2]||0; $('av'+i).textContent=mp+' / '+force+'g';}
    else {$('ab'+i).style.width='0%'; $('av'+i).textContent='-';}}
  if(applyParams && !dragging){
    for(const k of ['scaling','low_pass_alpha','smoothing_alpha','max_step','project_dist','escape_dist','deadband']){const el=$('p_'+k); if(el&&h[k]!=null){el.value=h[k]; const v=$('v_'+k); if(v)v.textContent=(+h[k]).toFixed((k==='deadband'||k==='max_step')?0:(k.endsWith('_dist')?3:2));}}
    applyParams=false;
  }
  const jk=JSON.stringify(h.joints.map(j=>j.name)); if(jk!==lastJointsKey){jointsHtml(h.joints); lastJointsKey=jk;}
}
fingersHtml(); setInterval(tick,200); tick();
</script></body></html>"""
