import * as THREE from 'three';
import {installPlanner,robotColor} from './planner-ui.js';
let plannerUI=null;
import {OrbitControls} from './vendor/OrbitControls.js';
const $=id=>document.getElementById(id);

/* ───────── 파라미터 정의 (layered_surface_v2) ───────── */
const defaults={layered:true,resolution:.1,percentile:10,minpoints:3,zmin:-1,zmax:5,roll:0,pitch:0,
 surfacegap:.08,surfacethickness:.12,surfaceslope:25,linkstep:.15,linkslope:25,seedheight:0,seedauto:true,seedradius:.5,maxgap:1.2,
 robotheight:1.5,obstacleheight:.22,obstaclepoints:4,wallpadding:0,
 interpolate:false,maxheight:.2,maxslope:20,recover:true,recoverheight:.15};
const N=(key,label,min,max,step,hint,dep)=>({key,label,min,max,step,hint,dep});
const C=(key,label,hint,dep)=>({key,label,check:true,hint,dep});
const groups=[
 {id:'pass',title:'통과 가능성 (장애물 · 머리 위)',open:true,hint:'바닥 표면과 별개로 전체 점군에서 판정합니다. 바닥이 있어도 막힐 수 있습니다(빨강 = 막힘, 바닥 없음 아님).',items:[
  N('robotheight','로봇 높이 (m)',.3,3,.1,'바닥 위 이 높이까지의 점을 머리 위 장애물로 검사합니다.'),
  N('obstacleheight','장애물 시작 높이 (m)',.05,1,.01,'바닥 면보다 이 높이 이상 위의 점부터 장애물로 셉니다.'),
  N('obstaclepoints','장애물 최소 점 개수',1,20,1,'셀 안에서 이 개수 이상이면 막힘.'),
  N('wallpadding','벽 주변 여유 (m)',0,.5,.05,'통과 가능성에만 적용됩니다. 바닥 표면은 바뀌지 않습니다.')]},
 {id:'floor',title:'바닥 표면 선택 · 연결',open:true,hint:'층별 후보 표면 중 기준 높이의 표면을 고르고, 이웃 셀과 이어지는 영역만 채택합니다.',items:[
  C('seedauto','기준 바닥 높이 자동 선택','가장 많은 낮은 평면을 기준으로 삼습니다.'),
  N('seedheight','기준 바닥 Z (m)',-10,10,.05,'자동 선택을 끄면 이 높이 부근의 표면에서 시작합니다.',p=>p.seedauto),
  N('seedradius','기준 높이 허용 오차 (m)',.1,2,.05,'이 범위 안에서 기준 높이에 가장 가까운 신뢰할 수 있는 지면을 찾습니다.'),
  N('surfaceslope','표면 최대 경사 (°)',5,60,1,'후보 면의 국소 평면 경사 한계.'),
  N('linkstep','이웃 높이 단차 (m)',0,.5,.01,'인접 셀 사이 허용 높이 차. 계단·턱을 끊으려면 줄이세요.'),
  N('linkslope','이웃 연결 경사 (°)',5,60,1),
  N('maxgap','최대 연결 거리 (m)',.1,3,.05,'듬성한 조각도 이 거리 안이면 같은 바닥으로 잇습니다. 보간과는 별개입니다.')]},
 {id:'cand',title:'격자 · 후보 표면',items:[
  N('resolution','격자 크기 (m)',.02,.5,.01,'작을수록 세밀하지만 빈 셀이 늘고 계산이 느립니다.'),
  N('minpoints','최소 점 개수',1,30,1,'미만인 셀은 지지 부족으로 제외.'),
  N('percentile','대표 높이 분위수 (%)',0,50,1,'후보 대역 안에서 낮은 쪽 몇 %를 대표 높이로 삼을지.'),
  N('surfacegap','층 분리 간격 (m)',.02,.3,.01,'세로로 이만큼 떨어지면 다른 층으로 나눕니다.'),
  N('surfacethickness','후보 대역 두께 (m)',.04,.3,.01,'한 표면으로 묶는 수평 대역의 두께.')]},
 {id:'z',title:'높이 범위 · 수평 보정',hint:'Z 범위는 바닥 후보 높이만 제한합니다. 장애물 점은 범위와 상관없이 전체 점군을 씁니다.',items:[
  N('zmin','바닥 후보 Z 최솟값 (m)',-10,10,.05),
  N('zmax','바닥 후보 Z 최댓값 (m)',-10,15,.05),
  N('roll','X축 회전 (°)',-30,30,.1),
  N('pitch','Y축 회전 (°)',-30,30,.1,'회전한 좌표에서 계산합니다.')]},
 {id:'interp',title:'보간 · 복구 (선택)',hint:'비어 있는 바닥을 추정으로 메웁니다. 추정면은 관측이 아니며 주행 안전을 보장하지 않습니다.',items:[
  C('interpolate','빈 바닥 보간 사용','주변 지지점의 삼각형 안에서만 채웁니다.'),
  N('maxheight','지지점 높이 차이 (m)',0,1,.01,null,p=>!p.interpolate),
  N('maxslope','보간 허용 경사 (°)',0,60,1,null,p=>!p.interpolate),
  C('recover','약한 바닥 후보 복구','점 분포가 부족해도 주변의 신뢰할 수 있는 바닥면과 높이가 맞으면 복구합니다. 보간과 독립적으로 적용됩니다.'),
  N('recoverheight','복구 허용 높이 차이 (m)',.02,.4,.01,null,p=>!p.recover)]}
];
const specs={};for(const g of groups)for(const it of g.items)specs[it.key]=it;
const presetKeep=['resolution','robotheight','zmin','zmax','roll','pitch'];
const presets={
 default:{},
 strict:{minpoints:5,surfaceslope:18,linkstep:.1,linkslope:18,maxgap:.6,wallpadding:.15,obstaclepoints:3,maxslope:15,maxheight:.12},
 sparse:{minpoints:1,surfacethickness:.18,surfaceslope:35,linkstep:.25,linkslope:35,maxgap:2,interpolate:true,maxheight:.3,maxslope:30}
};

/* ───────── 저장 / 복구 (localStorage) ───────── */
const KEY='elevationLab.v2';
function readStore(){try{const o=JSON.parse(localStorage.getItem(KEY));return o&&typeof o==='object'?o:{};}catch{return {};}}
function sanitize(src){
 const out={...defaults};if(!src||typeof src!=='object')return out;
 for(const key of Object.keys(defaults)){
  if(key==='layered')continue;const v=src[key],d=defaults[key];
  if(typeof v!==typeof d)continue;
  if(typeof v==='number'){if(!Number.isFinite(v))continue;const s=specs[key];
   if(key==='zmin'||key==='zmax'){if(Math.abs(v)>1e4)continue;}else if(s&&(v<s.min||v>s.max))continue;}
  out[key]=v;
 }
 return out;
}
const store=readStore();
let params=sanitize(store.params),sig=typeof store.sig==='string'?store.sig:null;
function saveStore(){try{localStorage.setItem(KEY,JSON.stringify({params,sig}));}catch{}}

/* ───────── 상태 ───────── */
let info=null,result=null,mode='ground',pending=false,processing=false,generation=0,timer,busyLoad=false,fitPending=false;
let raw,mesh,rawBox=null,cellBox=null,visFlags=null,gridHelper,axesHelper,stats={};

/* ───────── 파라미터 UI ───────── */
const controls={};
function fmt(v,step){const d=step>=1?0:Math.min(3,Math.max(0,Math.ceil(-Math.log10(step)-1e-9)));return (+v).toFixed(d);}
for(const g of groups){
 const box=document.createElement('details');box.className='param-group';box.open=!!g.open;
 const sum=document.createElement('summary');sum.textContent=g.title;box.append(sum);
 if(g.hint){const h=document.createElement('p');h.className='hint';h.textContent=g.hint;box.append(h);}
 for(const it of g.items){
  const row=document.createElement('div');
  if(it.check){
   row.className='check';const id='p-'+it.key;
   row.innerHTML=`<label for="${id}"><input id="${id}" type="checkbox"> ${it.label}</label>${it.hint?`<small id="h-${it.key}">${it.hint}</small>`:''}`;
   const c=row.querySelector('input');if(it.hint)c.setAttribute('aria-describedby','h-'+it.key);
   c.onchange=()=>{params[it.key]=c.checked;refreshDeps();changed();};
   controls[it.key]={row,c};
  }else{
   row.className='param';const k=it.key;
   row.innerHTML=`<div class="param-head"><label for="n-${k}">${it.label}</label><input id="n-${k}" type="number" min="${it.min}" max="${it.max}" step="${it.step}" aria-describedby="r-${k}${it.hint?' h-'+k:''}"></div>`+
    `<input id="p-${k}" type="range" min="${it.min}" max="${it.max}" step="${it.step}" aria-label="${it.label} 슬라이더" aria-describedby="r-${k}">`+
    `<div class="param-range" id="r-${k}"><span>최소 ${it.min}</span><span>최대 ${it.max}</span></div><small class="param-err" id="e-${k}" role="alert"></small>${it.hint?`<small id="h-${k}">${it.hint}</small>`:''}`;
   const n=row.querySelector('input[type=number]'),r=row.querySelector('input[type=range]'),err=row.querySelector('.param-err');
   const setErr=m=>{err.textContent=m;n.setAttribute('aria-invalid',m?'true':'false');};
   n.oninput=()=>{const v=n.valueAsNumber;if(!Number.isFinite(v)||v<it.min||v>it.max){setErr(`${it.min} ~ ${it.max} 사이 값을 입력하세요`);return;}setErr('');params[k]=v;r.value=v;changed();};
   n.onchange=()=>{if(n.getAttribute('aria-invalid')==='true'){n.value=params[k];setErr('');}};
   r.oninput=()=>{n.value=r.value;setErr('');params[k]=+r.value;changed();};
   controls[k]={row,n,r,setErr};
  }
  box.append(row);
 }
 $('parameters').append(box);
}
function refreshDeps(){for(const [k,it] of Object.entries(specs)){if(!it.dep)continue;const off=!!it.dep(params),c=controls[k];c.row.classList.toggle('off',off);for(const el of [c.n,c.r,c.c])if(el)el.disabled=off;}}
function syncParams(){
 for(const [k,v] of Object.entries(params)){const c=controls[k];if(!c)continue;if(typeof v==='boolean')c.c.checked=v;else{c.n.value=v;c.r.value=v;c.setErr('');}}
 refreshDeps();
}
function setZLimits(lo,hi){for(const k of ['zmin','zmax']){const s=specs[k],c=controls[k];s.min=Math.min(-10,lo);s.max=Math.max(15,hi);for(const el of [c.n,c.r]){el.min=s.min;el.max=s.max;}c.row.querySelector('.param-range').innerHTML=`<span>최소 ${s.min}</span><span>최대 ${s.max}</span>`;}}

/* ───────── 상태 표시 ───────── */
function status(text,error=false){$('status').textContent=text;$('status').classList.toggle('error',error);}
function setState(kind,text){const s=$('state');s.className='pill '+kind;s.textContent=text;}
function setExport(on){document.querySelectorAll('[data-export]').forEach(b=>b.disabled=!on);}
function setBusy(on){$('progress').hidden=!on;$('apply').disabled=on;}
function markStale(on){$('stale').hidden=!on;$('viewport').classList.toggle('stale',on);}
function validate(){
 if(!(params.zmin<params.zmax))return '바닥 후보 Z 최솟값은 최댓값보다 작아야 합니다.';
 if(!(params.obstacleheight<params.robotheight))return '장애물 시작 높이는 로봇 높이보다 작아야 합니다.';
 return null;
}
function changed(){
 generation++;pending=true;setExport(false);saveStore();clearTimeout(timer);plannerUI?.invalidateSurface();
 const bad=validate();
 if(result)markStale(true);
 if(bad){setState('error','입력 확인');status(bad,true);return;}
 setState('stale','변경됨');
 if($('auto').checked){status('파라미터 변경됨 · 잠시 후 자동 계산');timer=setTimeout(processMap,350);}
 else status('파라미터 변경됨 · "다시 계산"을 눌러 반영하세요');
}
async function api(url,options){const r=await fetch(url,options);if(!r.ok){let e;try{e=await r.json();}catch{}throw Error(e?.error||`HTTP ${r.status}`);}return r;}
function setPreset(name){
 const keep=Object.fromEntries(presetKeep.map(k=>[k,params[k]]));
 params={...defaults,...keep,...presets[name]};syncParams();changed();
 if(!validate()){clearTimeout(timer);processMap();}
}
document.querySelectorAll('[data-preset]').forEach(b=>b.onclick=()=>setPreset(b.dataset.preset));
$('apply').onclick=()=>{clearTimeout(timer);processMap();};
$('auto').onchange=()=>{if($('auto').checked&&pending)processMap();};
$('togglepanel').onclick=()=>{const hidden=document.body.classList.toggle('panel-hidden');$('togglepanel').setAttribute('aria-expanded',String(!hidden));};
$('pickfile').onclick=()=>$('file').click();
$('importbtn').onclick=()=>$('config').click();

/* ───────── 계산 ───────── */
async function processMap(){
 clearTimeout(timer);
 if(!info?.loaded||busyLoad)return;
 const bad=validate();if(bad){setState('error','입력 확인');status(bad,true);return;}
 if(processing){pending=true;return;}
 processing=true;pending=false;setExport(false);setBusy(true);setState('busy','계산 중');
 const version=generation,snapshot={...params,layered:true};
 status('전체 점군에서 바닥 표면 후보와 통과 가능성을 계산하는 중…');
 try{
  const r=await api('/api/process',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({revision:info.revision,parameters:snapshot})});
  const next=await r.json();
  if(version!==generation){pending=true;return;}
  result=next;result.blocked=Array.isArray(next.blocked)?next.blocked:next.reason.map(v=>v===6);
  result.clearance=Array.isArray(next.clearance)?next.clearance:next.reason.map(()=>null);
  plannerUI?.setResult(result);buildCells();transformRaw();helpers();paint();renderLegend();
  if(fitPending){view('iso');fitPending=false;}
  markStale(false);setExport(true);setState('ok','최신');$('pick').hidden=true;
  summarize();
 }catch(e){if(version===generation){setState('error','오류');status(e.message,true);}}
 finally{processing=false;setBusy(false);if(pending&&$('auto').checked&&!busyLoad&&!validate())processMap();}
}
function summarize(){
 const m=result.meta,n=v=>v==null?'—':v.toLocaleString(),rs=m.reasons||[];
 const surf=m.surface_cells??m.valid_cells;
 $('surf').textContent=n(surf);$('trav').textContent=n(m.traversable_cells);$('blk').textContent=n(m.blocked_surface_cells);$('cand').textContent=n(m.candidate_surfaces);
 $('area').textContent=m.area_m2!=null?m.area_m2.toFixed(2)+' m²':'—';
 $('range').textContent=stats.z?stats.z.map(v=>v.toFixed(2)).join(' ~ ')+' m':'바닥 없음';
 $('elapsed').textContent=m.elapsed_ms!=null?m.elapsed_ms+' ms':'—';
 $('algo').textContent=[m.algorithm,m.obstacle_source&&`장애물: ${m.obstacle_source}`].filter(Boolean).join(' · ');
 status(`입력 ${n(m.input_points)}점 · 바닥 표면 ${n(surf)}셀 (관측 ${n(m.observed_cells)} · 복구 ${n(m.recovered_cells)} · 보간 ${n(m.interpolated_cells)})\n통과 후보 ${n(m.traversable_cells)} · 막힌 바닥 ${n(m.blocked_surface_cells)} · 바닥 없는 장애물 ${n(rs[6])}\n기준 바닥 Z ${m.seed_height_m?.toFixed(3)??'—'} m · 후보 ${n(m.candidate_surfaces)}개 중 선택한 지면만 표시`);
}

/* ───────── Three.js 장면 ───────── */
const viewport=$('viewport');
const scene=new THREE.Scene();scene.background=new THREE.Color('#0b121a');
const camera=new THREE.PerspectiveCamera(45,1,.01,10000);camera.up.set(0,0,1);
let renderer;
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});}catch(e){setState('error','오류');status('WebGL을 사용할 수 없습니다. GPU 가속을 지원하는 브라우저에서 열어 주세요.',true);throw e;}
renderer.setPixelRatio(Math.min(devicePixelRatio,2));viewport.append(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.1;orbit.screenSpacePanning=true;
const content=new THREE.Group();scene.add(content);

// 색맹 친화 순차 팔레트(저→고: 파랑→노랑, 밝기 단조 증가). 빨강은 '막힘' 전용.
const RAMP=['#4a74c9','#5d8fb5','#8a9a96','#b5a67c','#dcbc5a','#fff176'];
const PAL=RAMP.map(c=>new THREE.Color(c));
const BLOCKED=new THREE.Color('#ff3b3b'),NOFLOOR=new THREE.Color('#e040fb'),NOCLR=new THREE.Color('#6b7785');
const REASON_HEX=['#009e73','#f0e442','#cc79a7','#e69f00','#9a7bff','#b0b0b0','#e040fb','#56d6f0'];
const REASON_COL=REASON_HEX.map(c=>new THREE.Color(c));
const REASON_TEXT=['관측된 바닥 표면','지지 부족 (점 수 부족)','연결 끊김 · 높이 불연속','너무 가파르거나 거친 면','선택되지 않은 다른 표면 후보','보간으로 추정한 바닥 (미관측)','바닥 없음 · 장애물만 감지','제외 후보에서 복구한 바닥'];
function ramp(t,out){t=Math.max(0,Math.min(1,t))*(PAL.length-1);const i=Math.min(PAL.length-2,Math.floor(t));return out.copy(PAL[i]).lerp(PAL[i+1],t-i);}
function dispose(o){if(!o)return;content.remove(o);o.geometry.dispose();o.material.dispose();}

function transformRaw(){
 if(!raw)return;const p=result?.meta?.parameters||params;
 const rx=new THREE.Matrix4().makeRotationX(THREE.MathUtils.degToRad(p.roll||0)),ry=new THREE.Matrix4().makeRotationY(THREE.MathUtils.degToRad(p.pitch||0));
 raw.matrixAutoUpdate=false;raw.matrix.copy(ry.multiply(rx));raw.matrixWorldNeedsUpdate=true;
}
function buildCells(){
 dispose(mesh);mesh=null;cellBox=null;stats={};
 const cells=result.cells;if(!cells.length)return;
 const res=result.meta.resolution;
 mesh=new THREE.InstancedMesh(new THREE.PlaneGeometry(res*.98,res*.98),new THREE.MeshBasicMaterial({side:THREE.DoubleSide}),cells.length);
 visFlags=new Uint8Array(cells.length);
 const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];let zlo=Infinity,zhi=-Infinity,clo=Infinity,chi=-Infinity;
 cells.forEach(([x,y,z],i)=>{
  const r=result.reason[i];
  if(r===0||r===5||r===7){zlo=Math.min(zlo,z);zhi=Math.max(zhi,z);if(x<lo[0])lo[0]=x;if(x>hi[0])hi[0]=x;if(y<lo[1])lo[1]=y;if(y>hi[1])hi[1]=y;if(z<lo[2])lo[2]=z;if(z>hi[2])hi[2]=z;}
  const c=result.clearance[i];if(c!=null){clo=Math.min(clo,c);chi=Math.max(chi,c);}
 });
 if(zlo<=zhi){stats.z=[zlo,zhi];cellBox=new THREE.Box3(new THREE.Vector3(...lo),new THREE.Vector3(...hi));}
 if(clo<=chi)stats.c=[clo,chi];
 mesh.frustumCulled=false;content.add(mesh);
}
const M=new THREE.Matrix4(),COL=new THREE.Color();
function paint(){
 if(raw){raw.visible=mode==='raw'||$('overlay').checked;raw.material.opacity=mode==='raw'?1:+$('opacity').value;raw.material.size=+$('pointsize').value;}
 document.querySelectorAll('[data-mode]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.mode===mode)));
 $('diagwrap').hidden=mode!=='rejected';
 $('viewlabel').textContent={ground:'① 바닥 표면 (높이)',passage:'② 통과 가능성',rejected:'③ 추론 진단',raw:'원본 점군'}[mode];
 $('viewhelp').textContent={ground:'막힌 바닥도 포함 · 높이로 색칠',passage:'빨강 = 막힘(바닥은 있음) · 안전 보장 아님',rejected:$('diagkind').value==='reason'?'표면 출처별 색':'머리 위 여유 높이',raw:'계산에 쓴 전체 점군(표시용 샘플)'}[mode];
 if(!mesh||!result){return;}
 mesh.visible=mode!=='raw';if(mode==='raw')return;
 const {cells,reason,blocked,clearance}=result;
 const showI=$('showinterp').checked,showR=$('showrec').checked,showO=$('obstacles').checked,diag=$('diagkind').value;
 const [z0,z1]=stats.z||[0,1],[c0,c1]=stats.c||[0,1];
 const robotCodes=plannerUI?.codes();
 if(mode==='passage'&&robotCodes)$('viewhelp').textContent='로봇 기준 차단 영역 · 셀 클릭으로 사유 확인 · 단차는 이동 간선에서 검사';
 for(let i=0;i<cells.length;i++){
  const [x,y,z]=cells[i],r=reason[i],floor=r===0||r===5||r===7;
  const shown=floor&&(r!==5||showI)&&(r!==7||showR);
  let s=0;
  if(mode==='ground'){if(shown){s=1;ramp((z-z0)/(z1-z0||1),COL);}}
  else if(mode==='passage'){
   if(shown){s=1;if(robotCodes?robotCodes[i]!==0:blocked[i]){if(robotCodes)COL.set(robotColor(robotCodes[i]));else COL.copy(BLOCKED);}else ramp((z-z0)/(z1-z0||1),COL);}
   else if(r===6&&showO){s=.5;COL.copy(NOFLOOR);}
  }else{ // 진단: 모든 셀
   s=r===6?.5:1;
   if(diag==='reason')COL.copy(REASON_COL[r]||NOCLR);
   else{const c=clearance[i];if(c==null)COL.copy(NOCLR);else ramp((c-c0)/(c1-c0||1),COL);}
  }
  visFlags[i]=s?1:0;
  M.makeScale(s,s,s||1).setPosition(x,y,z+.002);mesh.setMatrixAt(i,M);mesh.setColorAt(i,COL);
 }
 mesh.instanceMatrix.needsUpdate=true;if(mesh.instanceColor)mesh.instanceColor.needsUpdate=true;
}

/* ───────── 범례 ───────── */
const GRAD=`linear-gradient(90deg,${RAMP.join(',')})`;
function renderLegend(){
 const L=$('legend');L.hidden=mode==='raw';if(mode==='raw')return;L.replaceChildren();
 const add=(tag,cls,html)=>{const e=document.createElement(tag);if(cls)e.className=cls;e.innerHTML=html;L.append(e);return e;};
 const sw=(c,t,small)=>add('div','lg',`<i class="${small?'small':''}" style="background:${c}"></i><span>${t}</span>`);
 const grad=(title,a,b,note)=>{add('h3','',title);add('div','gradient','').style.background=GRAD;add('div','legend-range',`<span>${a}</span><span>${b}</span>`);if(note)add('div','legend-note',note);};
 const z=stats.z?stats.z.map(v=>v.toFixed(2)+' m'):['—','—'];
 if(mode==='ground'){
  grad('바닥 높이 (Z)',z[0],z[1],'막힌 바닥도 같은 높이색으로 칠합니다. 후보에서 제외된 셀과 바닥을 못 찾은 장애물은 이 보기에 없습니다.');
 }else if(mode==='passage'){
  add('h3','','통과 가능성 (후보)');
  const reasons=plannerUI?.legend();
  if(reasons){for(const r of reasons)sw(r.color,`${r.label} <b>${r.count.toLocaleString()}</b>`);add('div','legend-note','중복 사유는 위쪽 색 우선 · 수는 중복 포함. 셀 클릭: 전체 사유 및 원인 연결선.');}
  else sw('#ff3b3b','막힘 — 로봇 판정 갱신 후 상세 색상 표시');
  add('div','gradient','').style.background=GRAD;add('div','legend-range',`<span>${z[0]}</span><span>${z[1]}</span>`);
  add('div','legend-note','위 막대 = 막히지 않은 바닥 (높이색). 안전을 보장하지 않는 후보입니다.');
  sw('#e040fb','<b>작은 칸</b> — 바닥 없음, 장애물만 감지',true);
 }else if(stats){
  if($('diagkind').value==='reason'){
   add('h3','','표시 셀 분류 (셀 수)');const rs=result?.meta?.reasons||[];
   REASON_TEXT.forEach((t,i)=>sw(REASON_HEX[i],`${i} ${t} <b>${(rs[i]??0).toLocaleString()}</b>`,i===6));
   add('div','legend-note','색과 번호로 구분합니다. 6번은 작은 칸으로 표시하며 바닥 높이는 참고값입니다.');
  }else{
   const c=stats.c?stats.c.map(v=>v.toFixed(2)+' m'):['—','—'];
   grad('머리 위 여유 높이',c[0],c[1]);sw('#6b7785','측정 안 됨 / 해당 없음');
  }
 }
}

/* ───────── 카메라 / 보조 요소 ───────── */
function robustRawBox(pos){
 const b=new THREE.Box3();if(!pos.length)return b;const n=pos.length/3,lo=[],hi=[];
 for(let a=0;a<3;a++){const v=new Float32Array(n);for(let i=0;i<n;i++)v[i]=pos[i*3+a];v.sort();lo.push(v[Math.floor(n*.01)]);hi.push(v[Math.min(n-1,Math.floor(n*.99))]);}
 return b.set(new THREE.Vector3(...lo),new THREE.Vector3(...hi));
}
function dataBox(){
 if(mode!=='raw'&&cellBox)return cellBox.clone();
 if(rawBox){const b=rawBox.clone();if(raw)b.applyMatrix4(raw.matrix);return b;}
 return cellBox?cellBox.clone():new THREE.Box3(new THREE.Vector3(-5,-5,-1),new THREE.Vector3(5,5,1));
}
function view(type){
 const b=dataBox(),k=content.scale.z;b.min.z*=k;b.max.z*=k;
 const c=b.getCenter(new THREE.Vector3()),s=b.getSize(new THREE.Vector3());
 const r={top:Math.hypot(s.x,s.y),front:Math.hypot(s.x,s.z)}[type]??s.length();
 const half=Math.max(r/2,.5),vf=THREE.MathUtils.degToRad(camera.fov),hf=2*Math.atan(Math.tan(vf/2)*camera.aspect);
 const dist=half/Math.sin(Math.min(vf,hf)/2)*1.08;
 const dir={top:[0,-.0001,1],front:[0,-1,0],iso:[1,-1,.9]}[type]||[1,-1,.9];
 orbit.target.copy(c);camera.position.copy(c).add(new THREE.Vector3(...dir).normalize().multiplyScalar(dist));
 camera.near=Math.max(.001,dist/5000);camera.far=Math.max(1000,dist*40);camera.updateProjectionMatrix();orbit.maxDistance=dist*20;orbit.update();
}
function helpers(){
 for(const h of [gridHelper,axesHelper]){if(h){content.remove(h);h.geometry?.dispose();h.material?.dispose();h.dispose?.();}}
 const b=dataBox(),s=b.getSize(new THREE.Vector3()),c=b.getCenter(new THREE.Vector3());
 const len=Math.min(400,Math.max(10,Math.ceil(Math.max(s.x,s.y)/2)*2+2));
 gridHelper=new THREE.GridHelper(len,len,0x4d6b84,0x26394a);gridHelper.rotation.x=Math.PI/2;
 gridHelper.position.set(Math.round(c.x),Math.round(c.y),b.min.z-.02);gridHelper.visible=$('grid').checked;content.add(gridHelper);
 axesHelper=new THREE.AxesHelper(1);axesHelper.position.set(b.min.x,b.min.y,b.min.z-.01);content.add(axesHelper);
}
new ResizeObserver(()=>{const w=viewport.clientWidth,h=viewport.clientHeight;if(!w||!h)return;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h);}).observe(viewport);
renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);});
$('fit').onclick=$('iso').onclick=()=>view('iso');$('top').onclick=()=>view('top');$('front').onclick=()=>view('front');

/* ───────── 보기 설정 ───────── */
function setMode(m){mode=m;$('pick').hidden=true;paint();renderLegend();}
$('modes').onclick=e=>{const b=e.target.closest('[data-mode]');if(b)setMode(b.dataset.mode);};
document.addEventListener('keydown',e=>{if(e.target.closest('input,select,textarea')||e.ctrlKey||e.metaKey||e.altKey)return;const m={1:'ground',2:'passage',3:'rejected',4:'raw'}[e.key];if(m)setMode(m);});
for(const id of ['overlay','opacity','pointsize','showinterp','showrec','obstacles'])$(id).oninput=paint;
$('diagkind').onchange=()=>{paint();renderLegend();};
$('exaggeration').onchange=()=>{content.scale.z=+$('exaggeration').value;};
$('grid').onchange=()=>{if(gridHelper)gridHelper.visible=$('grid').checked;};

/* ───────── 셀 클릭 상세 ───────── */
const raycaster=new THREE.Raycaster();let down;
renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
renderer.domElement.addEventListener('pointerup',e=>{
 if(!mesh?.visible||!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4)return;
 const r=renderer.domElement.getBoundingClientRect();
 raycaster.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);
 const hit=raycaster.intersectObject(mesh).find(h=>visFlags[h.instanceId]);
 if(!hit){$('pick').hidden=true;return;}
 if(e.button===0&&plannerUI?.click(hit.instanceId))return;
 describe(hit.instanceId);
});
function describe(i){
 const [x,y,z]=result.cells[i],r=result.reason[i],b=result.blocked[i],c=result.clearance[i],floor=r===0||r===5||r===7,p=result.meta.parameters||params;
 const surface=floor?`있음 — ${REASON_TEXT[r]}`:r===6?'없음 — 바닥을 찾지 못함 (아래 Z는 참고값)':`채택 안 됨 — ${REASON_TEXT[r]}`;
 const pass=r===6?'장애물 감지 (바닥 없음)':!floor?'판정 대상 아님 (채택된 바닥이 아님)':b?'막힘 — 장애물·머리 위·벽 여유':'통과 후보 (안전 보장 아님)';
 const head=c==null?'측정 안 됨':`${c.toFixed(2)} m (설정 로봇 높이 ${(+p.robotheight).toFixed(2)} m)`;
 const t=$('pick');
 t.textContent=`바닥 표면: ${surface}\n통과 가능성: ${pass}\n머리 위 여유: ${head}\nX ${x.toFixed(3)} · Y ${y.toFixed(3)} · Z ${z.toFixed(3)} m\n점 ${result.counts[i]} · 높이 분산 ${result.roughness[i].toFixed(3)} m`;
 t.textContent+=plannerUI?.detail(i)||'';t.classList.toggle('blocked',!!b);t.hidden=false;
}

/* ───────── 파일 불러오기 ───────── */
async function loadInfo(next){
 info=next;result=null;setExport(false);dispose(mesh);mesh=null;cellBox=null;markStale(false);
 $('filename').textContent=info.name;$('fileinfo').textContent=`${info.points.toLocaleString()}점 · 표시 ${info.display_points.toLocaleString()}점 · 로컬 처리`;
 const r=await api('/api/points');const pos=new Float32Array(await r.arrayBuffer());dispose(raw);
 const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
 const col=new Float32Array(pos.length),c=new THREE.Color(),[lo,hi]=info.bounds;
 for(let i=0;i<pos.length;i+=3){ramp((pos[i+2]-lo[2])/(hi[2]-lo[2]||1),c);col[i]=c.r*.85;col[i+1]=c.g*.85;col[i+2]=c.b*.85;}
 geo.setAttribute('color',new THREE.BufferAttribute(col,3));
 raw=new THREE.Points(geo,new THREE.PointsMaterial({vertexColors:true,size:+$('pointsize').value,sizeAttenuation:false,transparent:true,opacity:.35,depthWrite:false}));
 content.add(raw);rawBox=robustRawBox(pos);transformRaw();
 $('empty').hidden=true;$('pick').hidden=true;content.scale.z=1;$('exaggeration').value='1';
 setZLimits(Math.floor((lo[2]-1)*10)/10,Math.ceil((hi[2]+1)*10)/10);
 const newSig=`${info.name}|${info.points}|${lo.map(v=>v.toFixed(2))}|${hi.map(v=>v.toFixed(2))}`;
 if(newSig!==sig||!(params.zmin<params.zmax)){params.zmin=Math.floor((lo[2]-.05)*20)/20;params.zmax=Math.ceil((hi[2]+.05)*20)/20;}
 sig=newSig;saveStore();syncParams();helpers();fitPending=true;view('iso');generation++;pending=true;paint();renderLegend();
}
$('file').onchange=async e=>{
 const file=e.target.files[0];if(!file)return;
 if(file.size>512*1024**2){setState('error','오류');status('512MB 이하의 파일을 선택하세요',true);e.target.value='';return;}
 busyLoad=true;generation++;clearTimeout(timer);setExport(false);setBusy(true);setState('busy','불러오는 중');status('PCD를 불러오는 중…');$('file').disabled=true;
 try{
  const r=await api('/api/load',{method:'POST',headers:{'Content-Type':'application/octet-stream','X-Filename':encodeURIComponent(file.name)},body:file});
  const next=await r.json();try{next.name=decodeURIComponent(next.name);}catch{}
  await loadInfo(next);
 }catch(err){setState('error','오류');status(err.message,true);}
 finally{busyLoad=false;$('file').disabled=false;setBusy(false);e.target.value='';if(info?.loaded)processMap();}
};

/* ───────── 저장 / JSON 불러오기 ───────── */
for(const button of document.querySelectorAll('[data-export]'))button.onclick=async()=>{
 if(!result)return;button.disabled=true;
 try{
  const r=await api('/api/export',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:result.token,format:button.dataset.export})});
  const url=URL.createObjectURL(await r.blob()),a=document.createElement('a');
  a.href=url;a.download=r.headers.get('Content-Disposition')?.match(/filename="([^"]+)"/)?.[1]||'elevation';a.click();setTimeout(()=>URL.revokeObjectURL(url),10000);
 }catch(e){setState('error','오류');status(e.message,true);}
 finally{if(!pending)button.disabled=false;}
};
$('config').onchange=async e=>{
 try{
  const data=JSON.parse(await e.target.files[0].text()),src=data.parameters||data;
  if(!src||typeof src!=='object')throw Error('유효한 파라미터 JSON이 아닙니다');
  const next=sanitize({...params,...src});
  params={...next,zmin:Number.isFinite(src.zmin)?src.zmin:params.zmin,zmax:Number.isFinite(src.zmax)?src.zmax:params.zmax};
  syncParams();changed();if(!validate())processMap();
 }catch(err){setState('error','오류');status(err.message,true);}
 finally{e.target.value='';}
};

plannerUI=installPlanner({api,content,setMode,repaint:()=>{paint();renderLegend();}});

/* ───────── 시작 ───────── */
syncParams();setExport(false);setState('idle','대기');
(async()=>{
 try{const r=await api('/api/info');const next=await r.json();
  if(next.loaded){setBusy(true);await loadInfo(next);setBusy(false);await processMap();}
  else status('＋ PCD 열기 버튼으로 맵을 불러오세요.');
 }catch(e){setBusy(false);setState('error','오류');status(e.message,true);}
})();
