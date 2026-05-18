"use strict";
/* app.js — UI, drawing, and state for multi-object anomaly detector.
   Depends on engine.js being loaded first. */

const COLORS = { tx:"#ffb84d", rx:"#64d2a5", object:"#ff6b7a", det:"#d98cff", est:"#77a7ff", accent:"#e1f56f", muted:"#77828a", grid:"rgba(255,255,255,0.08)", white:"#f4f0e8" };
const OBJ_COLORS = ["#ff6b7a","#ff9e4d","#ffde6b","#7aefb2","#7ac8ff","#c49eff","#ff7ab8","#b0ff7a"];

const els = {};
["roomPreset","roomName","roomDimensions","detectionStatus","detectionCount","objectCount","objectCountOut",
 "reflectivity","reflectivityOut","noise","noiseOut","scatterObjects","animateTarget",
 "captureBaseline","clearBaseline","baselineStatus","cfarThreshold","cfarThresholdOut",
 "cfarGuard","cfarGuardOut","minLinks","minLinksOut",
 "resetView","viewYaw","viewPitch","viewZoom","viewPanX","viewPanY",
 "viewYawOut","viewPitchOut","viewZoomOut","viewPanXOut","viewPanYOut",
 "sceneCanvas","csiCanvas","cirCanvas","anomalyCanvas",
 "receiverMetrics","detectionList","detectionSummary","linkSummary","csiSummary","delaySummary"
].forEach(id => { els[id] = document.getElementById(id); });
els.receiverButtons = document.querySelectorAll("[data-rx]");

let state = {
  presetKey: "lab", selectedRx: 0,
  objects: [], objectCount: 3, reflectivity: 0.56, noise: 0.04,
  baseline: null, // array of N CSI arrays (one per link)
  cfarThreshold: 3.5, cfarGuard: 2, minLinks: 1,
  view: { yaw:-28, pitch:24, zoom:1, panX:0, panY:0 },
  time: 0, detections: [], anomalyPerLink: []
};

function initObjects(room, count) {
  const objs = [];
  const defaults = room.defaultObjects || [];
  for (let i = 0; i < count; i++) {
    if (i < defaults.length) { objs.push([...defaults[i]]); }
    else { objs.push(randomPoint(room)); }
  }
  return objs;
}

function randomPoint(room) {
  if (room.targetCorridor) return randomCorridorPt(room);
  const b = room.objectBounds || room.size.map(m => [m*0.15, m*0.85]);
  return b.map(([lo,hi]) => lo + Math.random()*(hi-lo));
}

function randomCorridorPt(room) {
  const c = room.targetCorridor;
  const t = c.minT + Math.random()*(c.maxT - c.minT);
  const ctr = add(c.base, scale(c.axis, t));
  const sA = normalize([c.direction[1], -c.direction[0], 0]);
  const sB = normalize(cross(c.direction, sA));
  const mr = c.baseRadius*(1-t) + c.endRadius*t;
  const r = Math.sqrt(Math.random())*mr*0.92, a = Math.random()*Math.PI*2;
  return add(ctr, add(scale(sA, Math.cos(a)*r), scale(sB, Math.sin(a)*r)));
}

/* ── Formatting ── */
function fmtM(v) { return Math.abs(v)>=1000 ? `${(v/1000).toFixed(1)} km` : Math.abs(v)>=100 ? `${v.toFixed(1)} m` : `${v.toFixed(2)} m`; }
function fmtNs(s) { return Math.abs(s)>=1e-3 ? `${(s*1e3).toFixed(3)} ms` : Math.abs(s)>=1e-6 ? `${(s*1e6).toFixed(3)} µs` : `${(s*1e9).toFixed(3)} ns`; }

/* ── Drawing helpers ── */
function setupCanvas(c) {
  const r = c.getBoundingClientRect(), p = devicePixelRatio||1;
  c.width = Math.round(r.width*p); c.height = Math.round(r.height*p);
  const ctx = c.getContext("2d"); ctx.setTransform(p,0,0,p,0,0); return ctx;
}
function drawLine(ctx,a,b,col,w) { ctx.strokeStyle=col; ctx.lineWidth=w; ctx.setLineDash([]); ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke(); }
function drawDash(ctx,a,b,col,w) { ctx.strokeStyle=col; ctx.lineWidth=w; ctx.setLineDash([7,7]); ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke(); ctx.setLineDash([]); }
function drawNode(ctx,pt,col,label,r,hollow,dx,dy) {
  dx=dx||r+7; dy=dy||4;
  ctx.save(); ctx.shadowColor=col; ctx.shadowBlur=16;
  ctx.fillStyle=hollow?"#111317":col; ctx.strokeStyle=col; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(pt.x,pt.y,r,0,Math.PI*2); hollow?ctx.stroke():ctx.fill();
  ctx.shadowBlur=0; ctx.fillStyle=COLORS.white; ctx.font="12px Inter,system-ui,sans-serif";
  ctx.fillText(label,pt.x+dx,pt.y+dy); ctx.restore();
}
function plotFrame(ctx,w,h,pad) {
  ctx.fillStyle="#101419"; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle=COLORS.grid; ctx.lineWidth=1;
  for (let i=0;i<=4;i++) { const y=pad+(i/4)*(h-pad*2); drawLine(ctx,{x:pad,y},{x:w-pad,y},COLORS.grid,1); }
  ctx.strokeStyle="rgba(255,255,255,0.18)"; ctx.strokeRect(pad,pad,w-pad*2,h-pad*2);
}
function axisLabels(ctx,xl,yl,w,h) {
  ctx.fillStyle="rgba(244,240,232,0.58)"; ctx.font="11px Inter,system-ui,sans-serif";
  ctx.textAlign="left"; ctx.fillText(yl,11,18); ctx.textAlign="right"; ctx.fillText(xl,w-12,h-10); ctx.textAlign="left";
}

/* ── Scene drawing ── */
function drawScene(room) {
  const canvas=els.sceneCanvas, rect=canvas.getBoundingClientRect(), ratio=devicePixelRatio||1;
  canvas.width=Math.round(rect.width*ratio); canvas.height=Math.round(rect.height*ratio);
  const ctx=canvas.getContext("2d"); ctx.setTransform(ratio,0,0,ratio,0,0);
  ctx.clearRect(0,0,rect.width,rect.height);
  const bounds=room.viewSize||room.size, vMin=room.viewMin||[0,0,0], vCenter=add(vMin,scale(bounds,0.5));
  const margin=58, sf=Math.min(rect.width-margin*2,rect.height-margin*2)/Math.max(...bounds)*state.view.zoom;
  const yaw=state.view.yaw*Math.PI/180, pitch=state.view.pitch*Math.PI/180;
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const project=(p)=>{
    const dx=p[0]-vCenter[0],dy=p[1]-vCenter[1],dz=p[2]-vCenter[2];
    const xY=dx*cy-dy*sy, yY=dx*sy+dy*cy, yP=yY*cp-dz*sp, zP=yY*sp+dz*cp;
    return {x:rect.width/2+state.view.panX+xY*sf, y:rect.height/2+state.view.panY+yP*0.18*sf-zP*sf};
  };

  // Earth / room
  if (room.earthCenter) {
    const ce=project(room.earthCenter), ed=project([room.earthCenter[0]+room.earthRadius,room.earthCenter[1],room.earthCenter[2]]);
    const er=Math.max(8,Math.abs(ed.x-ce.x));
    ctx.fillStyle="rgba(50,105,158,0.22)"; ctx.strokeStyle="rgba(107,176,220,0.5)"; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.arc(ce.x,ce.y,er,0,Math.PI*2); ctx.fill(); ctx.stroke();
    ctx.fillStyle="rgba(244,240,232,0.58)"; ctx.font="12px Inter,system-ui,sans-serif"; ctx.fillText("Earth",ce.x+er+8,ce.y+4);
  } else {
    const [sx,sy2,sz]=room.size;
    const corners=[[0,0,0],[sx,0,0],[sx,sy2,0],[0,sy2,0],[0,0,sz],[sx,0,sz],[sx,sy2,sz],[0,sy2,sz]].map(project);
    const edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
    ctx.fillStyle="rgba(255,255,255,0.025)"; ctx.beginPath();
    [0,1,2,3].forEach((idx,ord)=>ord?ctx.lineTo(corners[idx].x,corners[idx].y):ctx.moveTo(corners[idx].x,corners[idx].y));
    ctx.closePath(); ctx.fill();
    edges.forEach(([a,b])=>drawLine(ctx,corners[a],corners[b],"rgba(255,255,255,0.13)",1));
  }

  const txP=project(room.tx), rxPs=room.receivers.map(project);
  const links = room.links;

  // Primary link lines (Tx→Rx)
  rxPs.forEach((rp)=>{ drawDash(ctx,txP,rp,"rgba(255,184,77,0.25)",1.5); });
  // Inter-node link lines (Rx↔Rx)
  for (let i = 0; i < room.receivers.length; i++) {
    for (let j = i+1; j < room.receivers.length; j++) {
      drawDash(ctx, rxPs[i], rxPs[j], "rgba(100,210,165,0.18)", 1);
    }
  }

  // Objects (ground truth)
  state.objects.forEach((obj,oi)=>{
    const op=project(obj);
    ctx.globalAlpha=0.4;
    rxPs.forEach(rp=>{ drawLine(ctx,op,rp,"rgba(100,210,165,0.12)",1); });
    drawLine(ctx,txP,op,"rgba(255,107,122,0.15)",1);
    ctx.globalAlpha=1;
    drawNode(ctx,op,OBJ_COLORS[oi%OBJ_COLORS.length],`Obj ${oi+1}`,7,false,10,14);
  });

  // Detection position estimates
  state.detections.forEach((det,i)=>{
    if (det.estimatedPos) {
      const ep = project(det.estimatedPos);
      const pulsePhase = (state.time/800 + i*0.7) % (Math.PI*2);
      const pulseR = 12 + Math.sin(pulsePhase)*4;
      ctx.save();
      ctx.strokeStyle = `rgba(217,140,255,${0.4+det.confidence*0.5})`;
      ctx.lineWidth = 2; ctx.setLineDash([3,4]);
      ctx.beginPath(); ctx.arc(ep.x, ep.y, pulseR, 0, Math.PI*2); ctx.stroke();
      ctx.beginPath(); ctx.arc(ep.x, ep.y, pulseR+8, 0, Math.PI*2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#d98cff"; ctx.beginPath(); ctx.arc(ep.x,ep.y,4,0,Math.PI*2); ctx.fill();
      ctx.font = "bold 11px Inter,system-ui,sans-serif";
      ctx.fillText(`D${i+1}`, ep.x+pulseR+10, ep.y+4);
      ctx.restore();
    }
  });

  // Nodes
  drawNode(ctx,txP,COLORS.tx,"GPS Tx",11,false,12,-12);
  rxPs.forEach((p,i)=>drawNode(ctx,p,COLORS.rx,`LEO ${i+1}`,9));

  // Info text
  const nLinks = links.length;
  ctx.fillStyle="rgba(244,240,232,0.72)"; ctx.font="13px Inter,system-ui,sans-serif";
  const infoText = state.baseline ? `Baseline active | ${nLinks} links | ${state.objects.length} obj | CFAR σ=${state.cfarThreshold}` : `No baseline | ${nLinks} links | ${state.objects.length} objects`;
  ctx.fillText(infoText, 18, 26);
}

/* ── CSI plot (shows ΔH if baseline exists, raw H otherwise) ── */
function drawCsi(csi, deltaCsi) {
  const data = deltaCsi || csi;
  const ctx=setupCanvas(els.csiCanvas);
  const {width:w,height:h}=els.csiCanvas.getBoundingClientRect();
  const pad=34; ctx.clearRect(0,0,w,h); plotFrame(ctx,w,h,pad);
  const maxA=Math.max(...data.map(v=>v.amp))*1.08||1;
  const bw=(w-pad*2)/data.length;

  // Bars
  data.forEach((v,i)=>{
    const x=pad+i*bw, ah=(v.amp/maxA)*(h-pad*2);
    ctx.fillStyle=deltaCsi?"rgba(217,140,255,0.65)":"rgba(100,210,165,0.72)";
    ctx.fillRect(x+1,h-pad-ah,Math.max(1,bw-2),ah);
  });
  // Phase line
  ctx.strokeStyle=COLORS.accent; ctx.lineWidth=2; ctx.beginPath();
  data.forEach((v,i)=>{
    const x=pad+i*bw+bw/2, y=pad+((Math.PI-v.phase)/(Math.PI*2))*(h-pad*2);
    i?ctx.lineTo(x,y):ctx.moveTo(x,y);
  });
  ctx.stroke();
  axisLabels(ctx,"subcarrier",deltaCsi?"ΔH amplitude":"H amplitude",w,h);
}

/* ── CIR plot with CFAR threshold overlay ── */
function drawCir(cir, peaks) {
  const ctx=setupCanvas(els.cirCanvas);
  const {width:w,height:h}=els.cirCanvas.getBoundingClientRect();
  const pad=34; ctx.clearRect(0,0,w,h); plotFrame(ctx,w,h,pad);
  const vis=cir.slice(0,128);
  const maxM=Math.max(...vis.map(v=>v.mag))*1.1||1;

  // CIR line
  ctx.strokeStyle=COLORS.est; ctx.lineWidth=2; ctx.beginPath();
  vis.forEach((v,i)=>{
    const x=pad+(i/(vis.length-1))*(w-pad*2), y=h-pad-(v.mag/maxM)*(h-pad*2);
    i?ctx.lineTo(x,y):ctx.moveTo(x,y);
  });
  ctx.stroke();

  // CFAR peaks
  peaks.forEach(pk=>{
    if (pk.bin>=128) return;
    const x=pad+(pk.bin/(vis.length-1))*(w-pad*2);
    const y=h-pad-(pk.mag/maxM)*(h-pad*2);
    ctx.fillStyle="#d98cff"; ctx.beginPath(); ctx.arc(x,y,5,0,Math.PI*2); ctx.fill();
    ctx.fillStyle="#d98cff"; ctx.font="bold 10px Inter,system-ui,sans-serif";
    ctx.fillText(`${pk.timeNs.toFixed(1)}ns`,x+7,y-6);
    // Threshold line segment
    const ty=h-pad-(pk.threshold/maxM)*(h-pad*2);
    ctx.strokeStyle="rgba(255,158,77,0.5)"; ctx.lineWidth=1; ctx.setLineDash([3,3]);
    ctx.beginPath(); ctx.moveTo(x-12,ty); ctx.lineTo(x+12,ty); ctx.stroke(); ctx.setLineDash([]);
  });

  const label = peaks.length ? `${peaks.length} peak${peaks.length>1?"s":""}` : "no peaks";
  els.delaySummary.textContent = label;
  axisLabels(ctx,"delay bins","ΔCIR magnitude",w,h);
}

/* ── Anomaly strength bar chart ── */
function drawAnomaly() {
  const ctx=setupCanvas(els.anomalyCanvas);
  const {width:w,height:h}=els.anomalyCanvas.getBoundingClientRect();
  const pad=24; ctx.clearRect(0,0,w,h);
  ctx.fillStyle="#101419"; ctx.fillRect(0,0,w,h);
  const nLinks = state.anomalyPerLink.length;
  const maxA=Math.max(...state.anomalyPerLink,0.01)*1.2;
  const bw=(w-pad*2)/nLinks;
  const linkCols=["#64d2a5","#77a7ff","#e1f56f","#ff9e4d","#d98cff","#ff6b7a"];
  const room = roomPresets[state.presetKey];
  state.anomalyPerLink.forEach((a,i)=>{
    const x=pad+i*bw+bw*0.1, barW=bw*0.8;
    const barH=(a/maxA)*(h-pad*2);
    ctx.fillStyle=linkCols[i%linkCols.length]; ctx.globalAlpha=0.7;
    ctx.fillRect(x,h-pad-barH,barW,barH);
    ctx.globalAlpha=1; ctx.fillStyle=COLORS.white; ctx.font="bold 9px Inter,system-ui,sans-serif";
    ctx.textAlign="center";
    ctx.fillText(room.links[i]?.label || `L${i+1}`,x+barW/2,h-5);
    if (a > 0.0001) ctx.fillText(a.toFixed(3),x+barW/2,h-pad-barH-5);
    ctx.textAlign="left";
  });
}

/* ── Detection list ── */
function updateDetections() {
  const list = els.detectionList;
  if (!state.detections.length) {
    list.innerHTML = `<div class="no-detections">${state.baseline?"No anomalies above threshold":"Capture baseline to enable detection"}</div>`;
    els.detectionCount.textContent = "0 detected";
    els.detectionSummary.textContent = state.baseline ? "listening…" : "no baseline";
    els.detectionStatus.textContent = state.baseline ? "Scanning" : "No baseline";
    return;
  }
  els.detectionCount.textContent = `${state.detections.length} detected`;
  els.detectionSummary.textContent = `${state.detections.length} anomal${state.detections.length>1?"ies":"y"}`;
  els.detectionStatus.textContent = `${state.detections.length} detected`;
  const room = roomPresets[state.presetKey];
  list.innerHTML = state.detections.map((d,i) => {
    const posText = d.estimatedPos ? `Pos ≈ (${d.estimatedPos.map(v=>room.unitMode==="km"?(v/1000).toFixed(0)+"km":v.toFixed(2)+"m").join(", ")})` : "";
    const linkLabels = d.links.map(l => room.links[l]?.label || `L${l+1}`).join(", ");
    return `<div class="detection-item">
      <div class="det-icon" style="background:${OBJ_COLORS[i%OBJ_COLORS.length]}">${i+1}</div>
      <div class="det-info">
        Range ≈ ${fmtM(d.range)} <span>${linkLabels} | SNR ${d.snr.toFixed(1)}${posText ? " | "+posText : ""}</span>
      </div>
      <div class="det-confidence" style="color:${d.confidence>=0.66?"#5beba0":d.confidence>=0.33?"#ffde6b":"#ff9e4d"}">${(d.confidence*100).toFixed(0)}%</div>
    </div>`;
  }).join("");
}

/* ── Link metrics ── */
function updateMetrics(room) {
  els.receiverMetrics.innerHTML = room.links.map((link,i) => {
    const bl = dist(link.tx, link.rx);
    const anom = state.anomalyPerLink[i] || 0;
    const type = i < room.receivers.length ? "primary" : "inter";
    return `<div class="metric">
      <strong>${link.label}</strong>
      <dl>
        <div><dt>baseline</dt><dd>${fmtM(bl)}</dd></div>
        <div><dt>anomaly</dt><dd>${anom.toFixed(4)}</dd></div>
        <div><dt>peaks</dt><dd>${(state._peaksPerLink||[])[i]||0}</dd></div>
      </dl>
    </div>`;
  }).join("");
}

/* ── Controls ── */
function readViewControls() {
  state.view = { yaw:+els.viewYaw.value, pitch:+els.viewPitch.value, zoom:+els.viewZoom.value, panX:+els.viewPanX.value, panY:+els.viewPanY.value };
  els.viewYawOut.textContent=`${state.view.yaw}°`; els.viewPitchOut.textContent=`${state.view.pitch}°`;
  els.viewZoomOut.textContent=`${state.view.zoom.toFixed(2)}x`; els.viewPanXOut.textContent=state.view.panX; els.viewPanYOut.textContent=state.view.panY;
}

function readControls() {
  readViewControls();
  state.objectCount = +els.objectCount.value;
  state.reflectivity = +els.reflectivity.value;
  state.noise = +els.noise.value;
  state.cfarThreshold = +els.cfarThreshold.value;
  state.cfarGuard = +els.cfarGuard.value;
  state.minLinks = +els.minLinks.value;
  els.objectCountOut.textContent = state.objectCount;
  els.reflectivityOut.textContent = state.reflectivity.toFixed(2);
  els.noiseOut.textContent = `${(state.noise*100).toFixed(1)}%`;
  els.cfarThresholdOut.textContent = state.cfarThreshold.toFixed(1);
  els.cfarGuardOut.textContent = state.cfarGuard;
  els.minLinksOut.textContent = state.minLinks;
  // Adjust object count
  while (state.objects.length < state.objectCount) state.objects.push(randomPoint(roomPresets[state.presetKey]));
  while (state.objects.length > state.objectCount) state.objects.pop();
}

function resetView() {
  state.view = {yaw:-28,pitch:24,zoom:1,panX:0,panY:0};
  els.viewYaw.value=state.view.yaw; els.viewPitch.value=state.view.pitch;
  els.viewZoom.value=state.view.zoom; els.viewPanX.value=state.view.panX; els.viewPanY.value=state.view.panY;
  render();
}

/* ── Main render ── */
function render() {
  if (els.animateTarget.checked) {
    const room = roomPresets[state.presetKey];
    const t = state.time / 1000;
    state.objects = state.objects.map((obj, oi) => {
      const b = room.objectBounds || room.size.map((m,idx) => [m*0.15, m*0.85]);
      return b.map(([lo,hi], idx) => {
        const phase = [0.8,0.65,1.1][idx] * (1 + oi*0.3);
        const ctr = (lo+hi)/2, rad = (hi-lo)*(idx===2?0.15:0.22);
        return ctr + rad * Math.sin(t*phase + idx + oi*2.1);
      });
    });
  }

  readControls();
  const room = roomPresets[state.presetKey];
  const links = room.links;
  const nLinks = links.length;
  const refs = state.objects.map(() => state.reflectivity);
  const allPeaks = [];
  let selCsi, selDelta, selCir, selPeaks;

  state._peaksPerLink = [];
  state.anomalyPerLink = [];

  for (let li = 0; li < nLinks; li++) {
    const rawCsi = generateCsiForLink(links[li], state.objects, refs, state.noise, state.time);
    const csi = sanitizeCsiForLink(rawCsi, links[li]);
    const delta = subtractCsi(csi, state.baseline ? state.baseline[li] : null);
    const cir = computeCir(delta);
    const peaks = state.baseline ? cfarDetect(cir, state.cfarGuard, state.cfarThreshold) : [];
    allPeaks.push(peaks);
    state._peaksPerLink.push(peaks.length);
    state.anomalyPerLink.push(state.baseline ? anomalyStrength(cir) : 0);
    if (li === state.selectedRx) { selCsi=csi; selDelta=state.baseline?delta:null; selCir=cir; selPeaks=peaks; }
  }

  state.detections = state.baseline ? associateDetections(allPeaks, state.minLinks) : [];
  // Position estimation for each detection
  state.detections.forEach(det => {
    if (det.peaks && det.peaks.length >= 2) {
      const result = solvePositionFromDetection(det, links, room);
      if (result) det.estimatedPos = result.position;
    }
  });

  drawScene(room);
  drawCsi(selCsi, selDelta);
  drawCir(selCir, selPeaks);
  drawAnomaly();
  updateDetections();
  updateMetrics(room);
  const selLink = links[state.selectedRx];
  const profile = getPhysicalProfile(selLink?.profileKey);
  const profileSummary = `${(profile.carrierHz/1e9).toFixed(1)} GHz / ${profile.subcarrierCount} sc`;
  els.csiSummary.textContent = `${profileSummary} / ${selLink?.label||"Link "+state.selectedRx}${state.baseline?" (dH)":""}`;
  els.linkSummary.textContent = `${nLinks} links (${room.receivers.length} primary + ${nLinks - room.receivers.length} inter)`;
}

/* ── Baseline capture ── */
function captureBaseline() {
  const room = roomPresets[state.presetKey];
  const links = room.links;
  // Capture with ZERO objects to get the empty-room fingerprint on ALL links
  state.baseline = links.map(link => sanitizeCsiForLink(generateCsiForLink(link, [], [], state.noise, state.time), link));
  els.captureBaseline.classList.add("baseline-active");
  els.captureBaseline.textContent = `✓ Baseline (${links.length} links)`;
  els.baselineStatus.textContent = "active";
  render();
}

function clearBaseline() {
  state.baseline = null;
  els.captureBaseline.classList.remove("baseline-active");
  els.captureBaseline.textContent = "📡 Capture Baseline (empty room)";
  els.baselineStatus.textContent = "—";
  render();
}

/* ── Preset switching ── */
function applyPreset(key) {
  state.presetKey = key;
  const room = roomPresets[key];
  state.objects = initObjects(room, state.objectCount);
  state.baseline = null;
  els.captureBaseline.classList.remove("baseline-active");
  els.captureBaseline.textContent = "📡 Capture Baseline (empty room)";
  els.baselineStatus.textContent = "—";
  els.roomName.textContent = room.name;
  els.roomDimensions.textContent = room.description || `${room.size[0]} × ${room.size[1]} × ${room.size[2]} m`;
  render();
}

/* ── Event binding ── */
els.roomPreset.addEventListener("change", e => applyPreset(e.target.value));
[els.objectCount,els.reflectivity,els.noise,els.cfarThreshold,els.cfarGuard,els.minLinks].forEach(el => el.addEventListener("input", render));
[els.viewYaw,els.viewPitch,els.viewZoom,els.viewPanX,els.viewPanY].forEach(el => el.addEventListener("input", render));
els.scatterObjects.addEventListener("click", () => { state.objects = initObjects(roomPresets[state.presetKey], state.objectCount).map(() => randomPoint(roomPresets[state.presetKey])); render(); });
els.resetView.addEventListener("click", resetView);
els.captureBaseline.addEventListener("click", captureBaseline);
els.clearBaseline.addEventListener("click", clearBaseline);
els.receiverButtons.forEach(btn => {
  btn.addEventListener("click", () => {
    state.selectedRx = +btn.dataset.rx;
    els.receiverButtons.forEach(b => b.classList.toggle("active", b===btn));
    render();
  });
});
window.addEventListener("resize", render);

let lastFrameTime = performance.now();
function tick(time) { 
  const dt = time - lastFrameTime;
  lastFrameTime = time;
  if (els.animateTarget.checked) {
    state.time += dt;
    render();
  }
  requestAnimationFrame(tick); 
}

/* ── Init ── */
state.objects = initObjects(roomPresets.lab, state.objectCount);
els.roomName.textContent = roomPresets.lab.name;
render();
requestAnimationFrame(tick);
