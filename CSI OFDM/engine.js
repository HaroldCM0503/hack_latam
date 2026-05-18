"use strict";

/* ── Physical constants ── */
const SPEED_OF_LIGHT = 299_792_458;
const EARTH_RADIUS_M = 6_371_000;
const GPS_ALTITUDE_M = 20_200_000;
const IRIDIUM_ALTITUDE_M = 780_000;
const DEFAULT_PROFILE_KEY = "scaleB";

const PHYSICAL_PROFILES = {
  scaleA: {
    key: "scaleA",
    label: "Scale A: LEO constellation",
    carrierHz: 2_300_000_000,
    bandwidthHz: 100_000_000,
    subcarrierCount: 512,
    nodeSpeedMps: 7_500,
    targetSpeedMps: 12_000,
    staticExtraMeters: [650, 1_900, 4_800, 13_000, 32_000],
    staticAmplitudes: [0.08, 0.045, 0.026, 0.014, 0.008],
    targetReflectionGain: 2.4e6,
    noiseScale: 0.45,
    maxBistaticRangeM: Infinity,
    cirPad: 1024,
    mode: "leo"
  },
  scaleB: {
    key: "scaleB",
    label: "Scale B: ESP32 lab bench",
    carrierHz: 2_400_000_000,
    bandwidthHz: 20_000_000,
    subcarrierCount: 52,
    nodeSpeedMps: 0,
    targetSpeedMps: 1.2,
    staticExtraMeters: [0.75, 1.45, 2.85, 4.7, 6.4],
    staticAmplitudes: [0.18, 0.11, 0.07, 0.045, 0.03],
    targetReflectionGain: 9.5,
    noiseScale: 0.75,
    maxBistaticRangeM: 60,
    sfoSlopeRad: 0.075,
    phaseOffsetRad: 0.9,
    cirPad: 256,
    mode: "esp32"
  }
};

const SUBCARRIER_COUNT = PHYSICAL_PROFILES.scaleB.subcarrierCount;
const CIR_PAD = PHYSICAL_PROFILES.scaleB.cirPad;

/* ── Vector math ── */
function add(a, b) { return a.map((v, i) => v + b[i]); }
function sub(a, b) { return a.map((v, i) => v - b[i]); }
function scale(a, s) { return a.map(v => v * s); }
function len(a) { return Math.hypot(a[0], a[1], a[2]); }
function dist(a, b) { return len(sub(a, b)); }
function dot(a, b) { return a.reduce((s, v, i) => s + v * b[i], 0); }
function normalize(a) { const l = Math.max(len(a), 1e-9); return scale(a, 1 / l); }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function lerp(a, b, t) { return a.map((v, i) => v + (b[i] - v) * t); }
function centroid(pts) { return scale(pts.reduce((s, p) => add(s, p), [0, 0, 0]), 1 / pts.length); }
function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function seededNoise(i, salt) {
  const r = Math.sin(i * 127.1 + salt * 311.7) * 43758.5453;
  return r - Math.floor(r);
}

function getPhysicalProfile(key) {
  return PHYSICAL_PROFILES[key] || PHYSICAL_PROFILES[DEFAULT_PROFILE_KEY];
}

function profileForLink(link) {
  return getPhysicalProfile(link?.profileKey || DEFAULT_PROFILE_KEY);
}

function subcarrierSpacing(profile) {
  return profile.bandwidthHz / profile.subcarrierCount;
}

function subcarrierFrequency(profile, k) {
  const center = (profile.subcarrierCount - 1) / 2;
  return profile.carrierHz + (k - center) * subcarrierSpacing(profile);
}

function complexAdd(a, b) { return { re: a.re + b.re, im: a.im + b.im }; }

function complexRotate(v, theta) {
  const c = Math.cos(theta), s = Math.sin(theta);
  return { re: v.re * c - v.im * s, im: v.re * s + v.im * c };
}

function gaussianPair(seedA, seedB) {
  const u1 = Math.max(seededNoise(seedA, seedB), 1e-9);
  const u2 = seededNoise(seedA + 17.17, seedB + 3.31);
  const mag = Math.sqrt(-2 * Math.log(u1));
  const ang = 2 * Math.PI * u2;
  return [mag * Math.cos(ang), mag * Math.sin(ang)];
}

/* ── Geometry helpers ── */
function earthPoint(alt, latD, lonD) {
  const c = [30e6, 30e6, 30e6], r = EARTH_RADIUS_M + alt;
  const lat = latD * Math.PI / 180, lon = lonD * Math.PI / 180;
  return [c[0] + r * Math.cos(lat) * Math.cos(lon), c[1] + r * Math.cos(lat) * Math.sin(lon), c[2] + r * Math.sin(lat)];
}

function boundsFromPoints(pts, pad) {
  const mn = [0, 1, 2].map(a => Math.min(...pts.map(p => p[a])) - pad);
  const mx = [0, 1, 2].map(a => Math.max(...pts.map(p => p[a])) + pad);
  return { min: mn, max: mx, size: mx.map((v, i) => v - mn[i]), ranges: mx.map((v, i) => [mn[i], v]) };
}

function makeTargetCorridor(rxs, tx) {
  const base = centroid(rxs), axis = sub(tx, base);
  const br = Math.max(...rxs.map(r => dist(r, base))) * 1.22;
  return { base, apex: tx, axis, axisLength: len(axis), direction: normalize(axis), baseRadius: br, endRadius: 300_000, minT: 0.04, maxT: 0.3 };
}

/* ── Orbit constants ── */
const GNSS_TX = earthPoint(GPS_ALTITUDE_M, 13, 48);
const IRIDIUM_RX = [earthPoint(IRIDIUM_ALTITUDE_M, 10, 42), earthPoint(IRIDIUM_ALTITUDE_M, 22, 49), earthPoint(IRIDIUM_ALTITUDE_M, 5, 56)];
const ORBIT_CORRIDOR = makeTargetCorridor(IRIDIUM_RX, GNSS_TX);
/* Full bounds for the camera view (includes GPS for context) */
const ORBIT_VIEW_BOUNDS = boundsFromPoints([...IRIDIUM_RX, GNSS_TX], 700_000);
/* Tight bounds around LEO constellation only — objects stay here */
const LEO_VICINITY_PAD = 500_000; // 500 km around LEO cluster
const LEO_OBJECT_BOUNDS = boundsFromPoints(IRIDIUM_RX, LEO_VICINITY_PAD);

/* ── Build link definitions for a preset ── */
function buildLinks(room) {
  const links = [];
  const profileKey = room.profileKey || DEFAULT_PROFILE_KEY;
  /* Primary links: main Tx → each Rx */
  room.receivers.forEach((rx, i) => {
    links.push({ tx: room.tx, rx, label: `Tx→R${i + 1}`, id: i });
  });
  /* Inter-node links: each Rx pair acts as Tx↔Rx */
  for (let i = 0; i < room.receivers.length; i++) {
    for (let j = i + 1; j < room.receivers.length; j++) {
      links.push({ tx: room.receivers[i], rx: room.receivers[j], label: `R${i + 1}↔R${j + 1}`, id: links.length });
    }
  }
  links.forEach(link => { link.profileKey = profileKey; });
  return links;
}

/* ── Room presets ── */
const roomPresets = {
  lab: { profileKey: "scaleB", name: "Lab bench", size: [6, 4, 3], tx: [0.55, 0.45, 2.55], receivers: [[5.35, 0.55, 2.45], [5.2, 3.45, 2.25], [0.75, 3.25, 0.75]], defaultObjects: [[3.05, 2.05, 1.35], [1.8, 1.2, 0.9], [4.1, 2.8, 1.7]] },
  classroom: { profileKey: "scaleB", name: "Classroom", size: [9, 7, 3.2], tx: [0.7, 0.65, 2.75], receivers: [[8.25, 0.8, 2.55], [8.0, 6.2, 2.65], [1.0, 6.1, 0.85]], defaultObjects: [[4.7, 3.5, 1.35], [2.3, 5.1, 1.0], [6.8, 2.0, 2.1]] },
  warehouse: { profileKey: "scaleB", name: "Warehouse", size: [14, 10, 5], tx: [1.0, 1.1, 4.25], receivers: [[12.8, 1.2, 4.05], [12.4, 8.85, 4.2], [1.2, 8.7, 1.25]], defaultObjects: [[7.2, 5.2, 2.2], [3.5, 7.8, 1.5], [10.1, 3.3, 3.0]] },
  orbit: {
    profileKey: "scaleA",
    name: "Iridium-like LEO cluster", size: [60e6, 60e6, 60e6], unitMode: "km",
    description: "GPS 20,200 km | LEO 780 km | objects near LEO",
    earthCenter: [30e6, 30e6, 30e6], earthRadius: EARTH_RADIUS_M,
    tx: GNSS_TX, receivers: IRIDIUM_RX,
    defaultObjects: [
      lerp(ORBIT_CORRIDOR.base, ORBIT_CORRIDOR.apex, 0.03),
      lerp(ORBIT_CORRIDOR.base, ORBIT_CORRIDOR.apex, 0.05),
      lerp(ORBIT_CORRIDOR.base, ORBIT_CORRIDOR.apex, 0.07)
    ],
    objectBounds: LEO_OBJECT_BOUNDS.ranges,
    viewMin: ORBIT_VIEW_BOUNDS.min, viewSize: ORBIT_VIEW_BOUNDS.size,
    targetCorridor: { ...ORBIT_CORRIDOR, minT: 0.01, maxT: 0.10 },
    initialGuess: lerp(ORBIT_CORRIDOR.base, ORBIT_CORRIDOR.apex, 0.05)
  }
};
/* Pre-build link arrays for each preset */
Object.keys(roomPresets).forEach(k => { roomPresets[k].links = buildLinks(roomPresets[k]); });

/* ── Free-space path loss ── */
function fsGain(d, carrierHz) {
  const wavelength = SPEED_OF_LIGHT / carrierHz;
  return (wavelength / (4 * Math.PI * Math.max(d, 1e-3))) ** 2;
}
function complexPolar(r, a) { return { re: r * Math.cos(a), im: r * Math.sin(a) }; }

function orbitalTangent(pos, salt) {
  const center = roomPresets?.orbit?.earthCenter || [30e6, 30e6, 30e6];
  const radial = normalize(sub(pos, center));
  let tangent = cross(radial, [0, 0, 1]);
  if (len(tangent) < 1e-6) tangent = cross(radial, [0, 1, 0]);
  return scale(normalize(tangent), salt >= 0 ? 1 : -1);
}

function nodeVelocity(pos, profile, salt) {
  if (profile.mode !== "leo") return [0, 0, 0];
  return scale(orbitalTangent(pos, salt), profile.nodeSpeedMps);
}

function targetState(obj, profile, index, timeSec) {
  const position = Array.isArray(obj) ? obj : (obj.position || obj.pos || [0, 0, 0]);
  if (!Array.isArray(obj) && obj.velocity) return { position, velocity: obj.velocity };
  if (profile.mode !== "leo") return { position, velocity: [0, 0, 0] };

  const tangent = orbitalTangent(position, index % 2 ? -1 : 1);
  const wobble = normalize([Math.sin(index + 0.7), Math.cos(index * 1.9 + timeSec * 0.1), 0.35]);
  return { position, velocity: scale(normalize(add(tangent, scale(wobble, 0.18))), profile.targetSpeedMps) };
}

function bistaticDopplerHz(profile, txPos, rxPos, targetPos, txVel, rxVel, targetVel) {
  const uTxToTarget = normalize(sub(targetPos, txPos));
  const uTargetToRx = normalize(sub(rxPos, targetPos));
  const txProjection = dot(sub(targetVel, txVel), uTxToTarget);
  const rxProjection = dot(sub(targetVel, rxVel), uTargetToRx);
  return (profile.carrierHz / SPEED_OF_LIGHT) * (txProjection + rxProjection);
}

/* ── CSI generation for ONE link (arbitrary tx/rx pair) with multiple objects ── */
function generateCsiForLink(link, objects, reflectivities, noiseLvl, time) {
  const profile = profileForLink(link);
  const txPos = link.tx, rxPos = link.rx, linkId = link.id || 0;
  const txVel = link.txVel || nodeVelocity(txPos, profile, 1);
  const rxVel = link.rxVel || nodeVelocity(rxPos, profile, -1);
  const bl = dist(txPos, rxPos);
  const baseDelay = bl / SPEED_OF_LIGHT;
  const directLoss = fsGain(bl, profile.carrierHz);
  const norm = Math.max(directLoss, 1e-24);
  const directAmp = directLoss / norm;
  const center = (profile.subcarrierCount - 1) / 2;
  const timeSec = (time || 0) / 1000;
  const sfoSlope = profile.mode === "esp32"
    ? profile.sfoSlopeRad * (0.65 * Math.sin(timeSec * 1.7 + linkId * 0.9) + 0.35 * Math.sin(timeSec * 4.1 + linkId))
    : 0;
  const phaseOffset = profile.mode === "esp32"
    ? profile.phaseOffsetRad * Math.sin(timeSec * 2.3 + linkId * 1.37)
    : 0;
  const values = [];

  for (let k = 0; k < profile.subcarrierCount; k++) {
    const freq = subcarrierFrequency(profile, k);

    let iq = complexPolar(directAmp, -2 * Math.PI * freq * baseDelay);

    /* Static clutter: fixed delayed paths, removable by an empty-scene baseline. */
    for (let m = 0; m < profile.staticExtraMeters.length; m++) {
      const extra = profile.staticExtraMeters[m];
      const delay = (bl + extra) / SPEED_OF_LIGHT;
      const ampJitter = 0.78 + 0.44 * seededNoise(m + 3, linkId + 11);
      const fixedPhase = (seededNoise(m + 19, linkId + 5) - 0.5) * Math.PI * 0.35;
      const amp = directAmp * profile.staticAmplitudes[m] * ampJitter;
      iq = complexAdd(iq, complexPolar(amp, -2 * Math.PI * freq * delay + fixedPhase));
    }

    /* Dynamic target reflections: exact bistatic delay plus Scale-A Doppler. */
    for (let oi = 0; oi < objects.length; oi++) {
      const target = targetState(objects[oi], profile, oi, timeSec);
      const obj = target.position;
      const ref = reflectivities[oi] || 0.5;
      const txLeg = dist(txPos, obj);
      const rxLeg = dist(obj, rxPos);
      const scatter = txLeg + rxLeg;
      const scatDelay = scatter / SPEED_OF_LIGHT;
      const scatLoss = fsGain(txLeg, profile.carrierHz) * fsGain(rxLeg, profile.carrierHz);
      const scatAmp = ref * Math.sqrt(scatLoss / norm) * profile.targetReflectionGain;
      const dopplerHz = profile.mode === "leo" ? bistaticDopplerHz(profile, txPos, rxPos, obj, txVel, rxVel, target.velocity) : 0;
      iq = complexAdd(iq, complexPolar(scatAmp, -2 * Math.PI * (freq + dopplerHz) * scatDelay));
    }

    const phaseError = sfoSlope * (k - center) + phaseOffset;
    if (profile.mode === "esp32") iq = complexRotate(iq, phaseError);

    const sigma = (noiseLvl || 0) * profile.noiseScale * directAmp;
    const [nr, ni] = gaussianPair(k + 101, timeSec * 19 + linkId * 7.3);
    iq = complexAdd(iq, { re: nr * sigma, im: ni * sigma });
    values.push({
      k: k - center,
      freq,
      re: iq.re,
      im: iq.im,
      amp: Math.hypot(iq.re, iq.im),
      phase: Math.atan2(iq.im, iq.re),
      phaseError: profile.mode === "esp32" ? phaseError : 0,
      profileKey: profile.key
    });
  }
  return values;
}

/* ── Baseline subtraction ── */
function subtractCsi(live, baseline) {
  if (!baseline) return live;
  return live.map((v, i) => {
    const dre = v.re - baseline[i].re;
    const dim = v.im - baseline[i].im;
    return { ...v, re: dre, im: dim, amp: Math.hypot(dre, dim), phase: Math.atan2(dim, dre) };
  });
}

/* ── IFFT → CIR ── */
function unwrapPhases(phases) {
  const out = [];
  let offset = 0;
  for (let i = 0; i < phases.length; i++) {
    if (i > 0) {
      const d = phases[i] - phases[i - 1];
      if (d > Math.PI) offset -= 2 * Math.PI;
      else if (d < -Math.PI) offset += 2 * Math.PI;
    }
    out.push(phases[i] + offset);
  }
  return out;
}

function sanitizeCsi(csi, profileOrLink) {
  const profile = profileOrLink?.carrierHz ? profileOrLink : profileForLink(profileOrLink || { profileKey: csi[0]?.profileKey });
  if (profile.mode !== "esp32" || csi.length < 2) return csi;

  const phases = csi.every(v => Number.isFinite(v.phaseError))
    ? csi.map(v => v.phaseError)
    : unwrapPhases(csi.map(v => Math.atan2(v.im, v.re)));
  let sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  csi.forEach((v, i) => {
    const x = v.k ?? i;
    const w = Math.max(v.amp, 1e-6);
    sw += w; sx += w * x; sy += w * phases[i]; sxx += w * x * x; sxy += w * x * phases[i];
  });
  const den = Math.max(sw * sxx - sx * sx, 1e-12);
  const slope = (sw * sxy - sx * sy) / den;
  const intercept = (sy - slope * sx) / Math.max(sw, 1e-12);

  return csi.map((v, i) => {
    const x = v.k ?? i;
    const rotated = complexRotate(v, -(slope * x + intercept));
    return { ...v, re: rotated.re, im: rotated.im, amp: Math.hypot(rotated.re, rotated.im), phase: Math.atan2(rotated.im, rotated.re), phaseSlopeRemoved: slope };
  });
}

function sanitizeCsiForLink(csi, link) {
  return sanitizeCsi(csi, link);
}

function computeCir(csi) {
  const cir = [];
  const profile = getPhysicalProfile(csi[0]?.profileKey || DEFAULT_PROFILE_KEY);
  const count = csi.length;
  const pad = profile.cirPad || Math.max(CIR_PAD, count * 4);
  const spacing = count > 1 ? Math.abs(csi[1].freq - csi[0].freq) : subcarrierSpacing(profile);
  for (let n = 0; n < pad; n++) {
    let re = 0, im = 0;
    for (let k = 0; k < count; k++) {
      const ang = (2 * Math.PI * n * k) / pad;
      re += csi[k].re * Math.cos(ang) - csi[k].im * Math.sin(ang);
      im += csi[k].re * Math.sin(ang) + csi[k].im * Math.cos(ang);
    }
    cir.push({ bin: n, timeNs: (n / pad) * (1 / spacing) * 1e9, mag: Math.hypot(re, im) / count, profileKey: profile.key });
  }
  return cir;
}

/* ── CFAR peak detector ── */
function cfarDetect(cir, guardCells, threshold, maxBins) {
  const bins = Math.min(maxBins || 128, cir.length);
  const profile = getPhysicalProfile(cir[0]?.profileKey || DEFAULT_PROFILE_KEY);
  const trainCells = 8;
  const peaks = [];
  for (let i = 1; i < bins - 1; i++) {
    let sum = 0, cnt = 0;
    for (let j = Math.max(0, i - guardCells - trainCells); j < i - guardCells; j++) { sum += cir[j].mag; cnt++; }
    for (let j = i + guardCells + 1; j <= Math.min(bins - 1, i + guardCells + trainCells); j++) { sum += cir[j].mag; cnt++; }
    if (cnt < 3) continue;
    const noiseMean = sum / Math.max(cnt, 1);
    const cfarThresh = noiseMean * threshold;
    if (cir[i].mag > cfarThresh && cir[i].mag > cir[Math.max(0, i - 1)].mag && cir[i].mag >= cir[Math.min(bins - 1, i + 1)].mag) {
      const ym1 = cir[i - 1].mag;
      const y0 = cir[i].mag;
      const yp1 = cir[i + 1].mag;
      const denom = ym1 - 2 * y0 + yp1;
      const delta = denom !== 0 ? (ym1 - yp1) / (2 * denom) : 0;
      const trueBin = i + clamp(delta, -0.5, 0.5);
      
      const trueTimeNs = cir[i].timeNs + clamp(delta, -0.5, 0.5) * (cir[1].timeNs - cir[0].timeNs);
      const delayS = trueTimeNs * 1e-9;
      const bistaticRange = delayS * SPEED_OF_LIGHT;
      
      if (bistaticRange > profile.maxBistaticRangeM) continue;
      peaks.push({ bin: trueBin, timeNs: trueTimeNs, mag: y0, threshold: cfarThresh, range: bistaticRange, snr: y0 / Math.max(noiseMean, 1e-15) });
    }
  }
  return peaks;
}

/* ── Cross-link association ── */
function associateDetections(allPeaks, minLinks, toleranceNs) {
  toleranceNs = toleranceNs || 8;
  const candidates = [];
  /* Flatten all peaks with link index */
  const flat = [];
  allPeaks.forEach((peaks, li) => peaks.forEach(p => flat.push({ ...p, link: li })));
  flat.sort((a, b) => a.timeNs - b.timeNs);

  const used = new Set();
  for (let i = 0; i < flat.length; i++) {
    if (used.has(i)) continue;
    const group = [flat[i]];
    const links = new Set([flat[i].link]);
    for (let j = i + 1; j < flat.length; j++) {
      if (used.has(j)) continue;
      if (flat[j].timeNs - flat[i].timeNs > toleranceNs * 3) break;
      if (!links.has(flat[j].link) && Math.abs(flat[j].timeNs - flat[i].timeNs) < toleranceNs) {
        group.push(flat[j]);
        links.add(flat[j].link);
        used.add(j);
      }
    }
    used.add(i);
    if (links.size >= minLinks) {
      const avgRange = group.reduce((s, p) => s + p.range, 0) / group.length;
      const avgSnr = group.reduce((s, p) => s + p.snr, 0) / group.length;
      const confidence = Math.min(links.size / 3, 1);
      candidates.push({ links: [...links], peakCount: group.length, range: avgRange, snr: avgSnr, confidence, timeNs: group[0].timeNs, peaks: group });
    }
  }
  return candidates;
}

/* ── Anomaly strength per link ── */
function anomalyStrength(deltaCir, maxBins) {
  const bins = maxBins || 128;
  let sum = 0;
  for (let i = 0; i < bins; i++) sum += deltaCir[i].mag ** 2;
  return Math.sqrt(sum / bins);
}

/* ── Position solver from multi-link detections (Gauss-Newton) ── */
function solvePositionFromDetection(detection, allLinks, room) {
  /* We need at least 3 links with peaks to solve for 3D position */
  const usedLinks = detection.peaks.map(p => ({ link: allLinks[p.link], range: p.range }));
  if (usedLinks.length < 2) return null;

  /* Initial guess: centroid of all node positions involved */
  const involvedPts = [];
  usedLinks.forEach(ul => { involvedPts.push(ul.link.tx); involvedPts.push(ul.link.rx); });
  let est = detection.initialGuess || detection.seedPosition || room.initialGuess || centroid(involvedPts);
  let mu = 1e-3;

  const modelAt = (pos) => {
    const residuals = [];
    const jacobian = [];
    for (const ul of usedLinks) {
      const dTx = dist(pos, ul.link.tx);
      const dRx = dist(pos, ul.link.rx);
      const predicted = dTx + dRx;
      const measured = ul.range;
      residuals.push(predicted - measured);
      const uTx = scale(sub(pos, ul.link.tx), 1 / Math.max(dTx, 1e-6));
      const uRx = scale(sub(pos, ul.link.rx), 1 / Math.max(dRx, 1e-6));
      jacobian.push([uTx[0] + uRx[0], uTx[1] + uRx[1], uTx[2] + uRx[2]]);
    }
    return { residuals, jacobian, error: residuals.reduce((s, r) => s + r * r, 0) };
  };

  /* Gauss-Newton iterations */
  for (let iter = 0; iter < 35; iter++) {
    const residuals = [];
    const jacobian = [];
    for (const ul of usedLinks) {
      const dTx = dist(est, ul.link.tx);
      const dRx = dist(est, ul.link.rx);
      const predicted = dTx + dRx;
      /* Measured bistatic range = baseline + excess range from CFAR peak */
      const measured = ul.range;
      residuals.push(predicted - measured);
      /* Jacobian: d(predicted)/d(est) = unit(est-tx) + unit(est-rx) */
      const uTx = scale(sub(est, ul.link.tx), 1 / Math.max(dTx, 1e-6));
      const uRx = scale(sub(est, ul.link.rx), 1 / Math.max(dRx, 1e-6));
      jacobian.push([uTx[0] + uRx[0], uTx[1] + uRx[1], uTx[2] + uRx[2]]);
    }

    /* Solve J^T J δ = -J^T r via 3x3 Gaussian elimination */
    const jt = [0, 1, 2].map(c => jacobian.map(r => r[c]));
    const jtj = jt.map(r => [0, 1, 2].map(c => r.reduce((s, v, i) => s + v * jacobian[i][c], 0)));
    const jtr = jt.map(r => r.reduce((s, v, i) => s + v * residuals[i], 0));
    const error = residuals.reduce((s, r) => s + r * r, 0);
    const damped = jtj.map((row, r) => row.map((v, c) => v + (r === c ? mu : 0)));
    const step = solve3x3(damped, jtr.map(v => -v));
    if (!step) { mu *= 10; continue; }

    const candidate = [est[0] + step[0], est[1] + step[1], est[2] + step[2]];
    if (!candidate.every(Number.isFinite)) { mu *= 10; continue; }
    const next = modelAt(candidate);
    if (next.error <= error) {
      est = candidate;
      mu = Math.max(mu * 0.35, 1e-9);
    } else {
      mu = Math.min(mu * 10, 1e12);
    }
    if (Math.hypot(step[0], step[1], step[2]) < 0.001) break;
  }

  /* RMS residual */
  let rms = 0;
  for (const ul of usedLinks) {
    const r = dist(est, ul.link.tx) + dist(est, ul.link.rx) - ul.range;
    rms += r * r;
  }
  rms = Math.sqrt(rms / usedLinks.length);

  return { position: est, rms, linkCount: usedLinks.length };
}

/* ── 3x3 Gaussian elimination (augmented matrix) ── */
function det3x3(m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

function solve3x3(mat, vec) {
  if (!mat.flat().concat(vec).every(Number.isFinite)) return null;
  if (Math.abs(det3x3(mat)) < 1e-14) return null;
  const m = mat.map((row, i) => [...row, vec[i]]);
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let row = col + 1; row < 3; row++) { if (Math.abs(m[row][col]) > Math.abs(m[piv][col])) piv = row; }
    if (Math.abs(m[piv][col]) < 1e-12) return null;
    [m[col], m[piv]] = [m[piv], m[col]];
    const d = m[col][col];
    for (let k = col; k < 4; k++) m[col][k] /= d;
    for (let row = 0; row < 3; row++) {
      if (row === col) continue;
      const f = m[row][col];
      for (let k = col; k < 4; k++) m[row][k] -= f * m[col][k];
    }
  }
  return [m[0][3], m[1][3], m[2][3]];
}
