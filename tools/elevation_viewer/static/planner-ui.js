import * as THREE from 'three';
export function installPlanner({api,content,setMode,repaint}){
 const root=document.createElement('section');root.className='block robot-block';
 root.innerHTML=`<h2>로봇 · A* 경로 계획</h2><p class="hint">원형 로봇의 오프라인 검증입니다. 지도를 클릭해 출발점과 목표점을 선택하세요.</p><div class="robot-fields"></div><label class="robot-check"><input id="robot-strict" type="checkbox" checked> 몸체 아래 미관측 지면 차단</label><div class="robot-actions"><button id="robot-evaluate">통과 판정 갱신</button><button id="robot-example">예시 지점</button><button id="robot-start">출발점 선택</button><button id="robot-goal">목표점 선택</button><button id="robot-plan" class="primary">A* 경로 찾기</button><button id="robot-clear">경로 지우기</button></div><label class="robot-check"><input id="robot-show" type="checkbox" checked> 로봇 기준 차단 영역 표시</label><div id="robot-status" role="status">먼저 지면을 계산하세요.</div><button id="robot-download" disabled>경로 JSON 저장</button><p class="hint">빨강: 로봇 기준 차단. 단차는 셀 사이 이동에서 검사합니다. 전체 높이와 몸체 하부 여유는 지면 추출 설정과 독립적입니다.</p>`;
 document.getElementById('parameters').closest('section').before(root);
 const fields=[['radius','몸체 반경 (m)',.18,0,2,.01],['margin','안전 여유 (m)',.03,0,1,.01],['height','전체 높이 (m)',.5,.1,3,.05],['bodyclearance','하부 여유 (m)',.15,.02,1,.01],['maxstep','최대 단차 (m)',.12,0,1,.01],['maxslope','최대 경사 (°)',25,0,85,1],['inferredcost','보간 추가 비용',3,0,50,.5],['recoveredcost','복구 추가 비용',1,0,50,.5]];
 let stored={};try{stored=JSON.parse(localStorage.getItem('elevationRobot.v1'))||{};}catch{}
 for(const [key,label,value,min,max,step] of fields){const v=Number.isFinite(stored[key])&&stored[key]>=min&&stored[key]<=max?stored[key]:value;const el=document.createElement('label');el.htmlFor='robot-'+key;el.innerHTML=`${label}<input id="robot-${key}" type="number" min="${min}" max="${max}" step="${step}" value="${v}">`;root.querySelector('.robot-fields').append(el);}
 const $=id=>document.getElementById('robot-'+id);let result=null,analysis=null,start=null,goal=null,selecting=null,route=null,version=0,busy=false;
 const drawings=new THREE.Group();content.add(drawings);
 $('strict').checked=stored.strictunknown!==false;
 const status=(t,error=false)=>{$('status').textContent=t;$('status').style.color=error?'#ffa08c':'#b4d4e7';};
 function robot(){const p={strictunknown:$('strict').checked};for(const [key,,v,min,max]of fields){const n=$(key).valueAsNumber;if(!Number.isFinite(n)||n<min||n>max)throw Error(key+'의 입력 범위를 확인하세요');p[key]=n;}try{localStorage.setItem('elevationRobot.v1',JSON.stringify(p));}catch{}return p;}
 function clean(){while(drawings.children.length){const o=drawings.children[0];drawings.remove(o);o.geometry.dispose();o.material.dispose();}}
 function draw(){clean();for(const [p,color]of [[start,0x2fe5ae],[goal,0xffcc42]]){if(!p)continue;const m=new THREE.Mesh(new THREE.SphereGeometry(.12,12,8),new THREE.MeshBasicMaterial({color,depthTest:false}));m.position.set(p[0],p[1],p[2]+.13);m.renderOrder=10;drawings.add(m);}
  if(route?.ok){const geometry=new THREE.BufferGeometry().setFromPoints(route.path.map(p=>new THREE.Vector3(p[0],p[1],p[2]+.04)));const line=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:0xffed69,depthTest:false}));line.renderOrder=9;drawings.add(line);} $('download').disabled=!route?.ok;
 }
 function invalidate(message='로봇 설정 변경됨 · 통과 판정을 갱신하세요.'){version++;analysis=null;route=null;draw();repaint();status(message);}
 for(const [key]of fields)$(key).oninput=()=>invalidate();$('strict').onchange=()=>invalidate();$('show').onchange=repaint;
 async function request(plan=false){
  if(!result){status('먼저 최신 지면을 계산하세요.',true);return;}if(busy)return;let p;try{p=robot();}catch(e){status(e.message,true);return;}
  if(plan&&(!start||!goal)){status('출발점과 목표점을 먼저 선택하세요.',true);return;}
  const v=version,token=result.token;busy=true;status(plan?'A* 경로 탐색 중…':'원본 점군에서 로봇 통과 가능성을 검사하는 중…');$('plan').disabled=$('evaluate').disabled=$('example').disabled=true;
  try{const r=await api(plan?'/api/plan':'/api/robot',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token,robot:p,start:start?.slice(0,2),goal:goal?.slice(0,2)})});const next=await r.json();if(v!==version||token!==result?.token)return;
   analysis=next;setMode('passage');repaint();
   if(plan){route=next;draw();status(next.ok?`경로 ${next.length_m.toFixed(2)} m · 최대 경사 ${next.max_slope_deg.toFixed(1)}° · 최대 단차 ${next.max_step_m.toFixed(2)} m\n보간 ${(next.inferred_ratio*100).toFixed(1)}% · 복구 ${(next.recovered_ratio*100).toFixed(1)}% · ${next.elapsed_ms} ms`:next.error,!next.ok);}
   else status(`로봇 통과 후보 ${next.allowed_cells.toLocaleString()}셀. 출발점·목표점을 선택하세요.`);
  }catch(e){status(e.message,true);}finally{busy=false;$('plan').disabled=$('evaluate').disabled=$('example').disabled=false;}
 }
 $('evaluate').onclick=()=>request();$('plan').onclick=()=>request(true);
 $('example').onclick=async()=>{if(!analysis)await request();if(!analysis?.suggested?.length){status('예시 지점을 찾지 못했습니다. 반경·미관측 지면 조건을 확인하세요.',true);return;}version++;[start,goal]=analysis.suggested;route=null;draw();status('같은 통과 후보 영역에 예시 지점을 골랐습니다. A* 경로 찾기를 누르세요.');};
 for(const which of ['start','goal'])$(which).onclick=()=>{selecting=which;setMode('ground');status((which==='start'?'출발점':'목표점')+'으로 사용할 바닥 셀을 클릭하세요.');};
 $('clear').onclick=()=>{version++;start=goal=route=selecting=null;draw();status('경로와 선택 지점을 지웠습니다.');};
 $('download').onclick=()=>{if(!route?.ok)return;const blob=new Blob([JSON.stringify({source:result.meta.source,ground_token:result.token,robot:robot(),start,goal,...route},null,2)],{type:'application/json'});const u=URL.createObjectURL(blob);const a=document.createElement('a');a.href=u;a.download='terrain_path.json';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000);};
 return {setResult(r){result=r;start=goal=route=analysis=null;selecting=null;version++;draw();status('로봇 설정 후 통과 판정을 갱신하세요.');},invalidateSurface(){result=null;start=goal=null;invalidate('지면 설정 변경됨 · 다시 계산한 뒤 경로를 계획하세요.');},
  click(i){if(!selecting||!result)return false;const code=result.reason[i];if(![0,5,7].includes(code)){status('관측·복구·보간된 바닥 셀을 선택하세요.',true);return true;}version++;if(selecting==='start')start=result.cells[i];else goal=result.cells[i];selecting=null;route=null;draw();status(`출발점 ${start?start.slice(0,2).map(v=>v.toFixed(2)).join(', '):'미선택'} / 목표점 ${goal?goal.slice(0,2).map(v=>v.toFixed(2)).join(', '):'미선택'}`);return true;},
  codes(){return $('show').checked?analysis?.cell_codes:null;},detail(i){if(!analysis)return '';const code=analysis.cell_codes[i];return '\n로봇 판정: '+(code?Object.entries(analysis.reason_labels).filter(([bit])=>code&+bit).map(([,t])=>t).join(', '):'통과 후보')+` · 국소 경사 ${analysis.cell_slopes[i].toFixed(1)}°`;}};
}
