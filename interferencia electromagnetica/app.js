"use strict";

const canvas = document.querySelector("#fieldCanvas");
const ctx = canvas.getContext("2d", { alpha: false });
const detectorCanvas = document.querySelector("#detectorCanvas");
const detectorCtx = detectorCanvas.getContext("2d", { alpha: false });
const rightDetectorCanvas = document.querySelector("#rightDetectorCanvas");
const rightDetectorCtx = rightDetectorCanvas.getContext("2d", { alpha: false });
const rangeCanvas = document.querySelector("#rangeCanvas");
const rangeCtx = rangeCanvas.getContext("2d", { alpha: false });

const dx = 0.005;
const dy = dx;
const c0 = 299_792_458;
const eps0 = 8.854_187_8128e-12;
const mu0 = 1.256_637_06212e-6;
const cfl = 0.7;
const dt = (cfl * dx) / (c0 * Math.SQRT2);
const gridAspect = 360 / 224;
const boundaryCells = 42;
const pmlStrength = 0.32;
const pmlPower = 3.2;
const sourceOffsetCells = boundaryCells + 14;
const detectorBlankSteps = Math.ceil(0.35e-9 / dt);
const compactAntennaGain = 5.5;
const antennaKernel = [];

for (let oy = -2; oy <= 2; oy++) {
  for (let ox = -2; ox <= 2; ox++) {
    const weight = Math.exp(-(ox * ox + oy * oy) / (2 * 1.05 * 1.05));
    antennaKernel.push({ ox, oy, weight });
  }
}

const offscreen = document.createElement("canvas");
const offCtx = offscreen.getContext("2d", { alpha: false });

let domainWidthM = 1.8;
let NX = 360;
let NY = 224;
let N = NX * NY;
let sourceX = sourceOffsetCells;
let rightSourceX = NX - sourceOffsetCells - 1;
let detectorTraceLength = 900;
let Ez = new Float32Array(N);
let Hx = new Float32Array(N);
let Hy = new Float32Array(N);
let avgIntensity = new Float32Array(N);
let epsR = new Float32Array(N);
let sigma = new Float32Array(N);
let ca = new Float32Array(N);
let cb = new Float32Array(N);
let damping = new Float32Array(N);
let objectMask = new Int16Array(N);
let pecMask = new Uint8Array(N);
let detectorTrace = new Float32Array(detectorTraceLength);
let rightDetectorTrace = new Float32Array(detectorTraceLength);
let image = ctx.createImageData(NX, NY);
let pixels = image.data;

const materials = {
  metal: {
    label: "Metallic conductor",
    model: "pec",
    pec: true,
    color: [222, 187, 89],
    note: "Perfect electric conductor boundary"
  },
  freshWater: {
    label: "Fresh water",
    model: "debye",
    epsStatic: 78.3,
    epsInf: 4.9,
    tau: 8.27e-12,
    conductivity: 0.02,
    pec: false,
    color: [55, 150, 183],
    note: "Debye water relaxation plus ionic loss"
  },
  saltWater: {
    label: "Salt water",
    model: "debye",
    epsStatic: 74,
    epsInf: 4.9,
    tau: 8.27e-12,
    conductivity: 4.0,
    pec: false,
    color: [35, 105, 148],
    note: "Debye water relaxation with high ionic loss"
  },
  tissue: {
    label: "Water-rich tissue",
    model: "debye",
    epsStatic: 58,
    epsInf: 5.2,
    tau: 9.4e-12,
    conductivity: 0.8,
    pec: false,
    color: [194, 93, 102],
    note: "Approximate water-rich dispersive tissue"
  },
  glass: {
    label: "Glass",
    model: "constant",
    epsR: 4.2,
    conductivity: 1e-10,
    pec: false,
    color: [129, 171, 151],
    note: "Low-loss dielectric"
  },
  plastic: {
    label: "Plastic",
    model: "constant",
    epsR: 2.4,
    conductivity: 1e-12,
    pec: false,
    color: [196, 138, 63],
    note: "Low-index dielectric"
  }
};

const materialOrder = ["metal", "freshWater", "saltWater", "tissue", "glass", "plastic"];

const objects = [
  { enabled: true, material: "metal", shape: "circle", sizeCm: 28, xPct: 58, yPct: 50 },
  { enabled: false, material: "freshWater", shape: "circle", sizeCm: 22, xPct: 70, yPct: 35 },
  { enabled: false, material: "glass", shape: "square", sizeCm: 18, xPct: 72, yPct: 66 }
];

let selectedObject = 0;
let running = false;
let frequencyGHz = 1.2;
let antennaYPct = 50;
let pulseCycles = 11;
let gain = 3.0;
let viewMode = "field";
let stepCount = 0;
let needsMaterialRebuild = true;
let needsRender = true;
let frameScheduled = false;
let detectorWrite = 0;
let detectorSamplesSincePulse = detectorTraceLength;
let latestPulseStartStep = -Infinity;
let latestPulseDurationSteps = 0;
let receiverPeak = 0;
let receiverPeakStep = 0;
let activePulses = [];
let rightDetectorWrite = 0;
let rightDetectorSamplesSincePulse = detectorTraceLength;
let latestRightPulseStartStep = -Infinity;
let latestRightPulseDurationSteps = 0;
let rightReceiverPeak = 0;
let rightReceiverPeakStep = 0;
let activeRightPulses = [];

const ui = {
  triggerPulse: document.querySelector("#triggerPulse"),
  triggerRightPulse: document.querySelector("#triggerRightPulse"),
  toggleRun: document.querySelector("#toggleRun"),
  resetSim: document.querySelector("#resetSim"),
  frequency: document.querySelector("#frequency"),
  frequencyLabel: document.querySelector("#frequencyLabel"),
  antennaY: document.querySelector("#antennaY"),
  antennaYLabel: document.querySelector("#antennaYLabel"),
  sandboxWidth: document.querySelector("#sandboxWidth"),
  sandboxWidthLabel: document.querySelector("#sandboxWidthLabel"),
  pulseCycles: document.querySelector("#pulseCycles"),
  pulseCyclesLabel: document.querySelector("#pulseCyclesLabel"),
  gain: document.querySelector("#gain"),
  gainLabel: document.querySelector("#gainLabel"),
  viewMode: document.querySelector("#viewMode"),
  readoutMode: document.querySelector("#readoutMode"),
  readoutTime: document.querySelector("#readoutTime"),
  readoutEnergy: document.querySelector("#readoutEnergy"),
  objectEnabled: document.querySelector("#objectEnabled"),
  material: document.querySelector("#material"),
  shape: document.querySelector("#shape"),
  size: document.querySelector("#size"),
  sizeLabel: document.querySelector("#sizeLabel"),
  xPos: document.querySelector("#xPos"),
  xLabel: document.querySelector("#xLabel"),
  yPos: document.querySelector("#yPos"),
  yLabel: document.querySelector("#yLabel"),
  receiverStatus: document.querySelector("#receiverStatus"),
  receiverPeak: document.querySelector("#receiverPeak"),
  rightReceiverStatus: document.querySelector("#rightReceiverStatus"),
  rightReceiverPeak: document.querySelector("#rightReceiverPeak"),
  rangeDistance: document.querySelector("#rangeDistance"),
  rangeDelay: document.querySelector("#rangeDelay"),
  rightRangeDistance: document.querySelector("#rightRangeDistance"),
  rightRangeDelay: document.querySelector("#rightRangeDelay"),
  gridModelLabel: document.querySelector("#gridModelLabel"),
  materialFacts: document.querySelector("#materialFacts"),
  tabs: [...document.querySelectorAll(".object-tabs button")],
  presetSingle: document.querySelector("#presetSingle"),
  presetDouble: document.querySelector("#presetDouble"),
  presetSlit: document.querySelector("#presetSlit")
};

for (const key of materialOrder) {
  const option = document.createElement("option");
  option.value = key;
  option.textContent = materials[key].label;
  ui.material.append(option);
}

function idx(x, y) {
  return x + y * NX;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function allocateGrid(widthM) {
  domainWidthM = widthM;
  NX = Math.max(360, Math.round(domainWidthM / dx));
  NY = Math.max(224, Math.round(NX / gridAspect));
  N = NX * NY;
  sourceX = Math.min(sourceOffsetCells, NX - boundaryCells - 8);
  rightSourceX = Math.max(NX - sourceOffsetCells - 1, boundaryCells + 8);

  Ez = new Float32Array(N);
  Hx = new Float32Array(N);
  Hy = new Float32Array(N);
  avgIntensity = new Float32Array(N);
  epsR = new Float32Array(N);
  sigma = new Float32Array(N);
  ca = new Float32Array(N);
  cb = new Float32Array(N);
  damping = new Float32Array(N);
  objectMask = new Int16Array(N);
  pecMask = new Uint8Array(N);

  const diagonalM = Math.hypot(NX * dx, NY * dy);
  detectorTraceLength = Math.max(900, Math.ceil((2 * diagonalM / c0 + 12e-9) / dt));
  detectorTrace = new Float32Array(detectorTraceLength);
  rightDetectorTrace = new Float32Array(detectorTraceLength);

  image = ctx.createImageData(NX, NY);
  pixels = image.data;
  offscreen.width = NX;
  offscreen.height = NY;

  activePulses = [];
  activeRightPulses = [];
  stepCount = 0;
  computeDamping();
  needsMaterialRebuild = true;
  rebuildMaterials();
  clearDetector();
}

function materialAtFrequency(materialKey) {
  const material = materials[materialKey];
  if (material.model === "pec") {
    return { epsR: 1, sigma: 0, pec: true };
  }

  if (material.model === "debye") {
    const omegaTau = 2 * Math.PI * frequencyGHz * 1e9 * material.tau;
    const spread = material.epsStatic - material.epsInf;
    const denom = 1 + omegaTau * omegaTau;
    const epsPrime = material.epsInf + spread / denom;
    const epsLoss = (spread * omegaTau) / denom;
    return {
      epsR: epsPrime,
      sigma: material.conductivity + 2 * Math.PI * frequencyGHz * 1e9 * eps0 * epsLoss,
      pec: false
    };
  }

  return { epsR: material.epsR, sigma: material.conductivity, pec: false };
}

function clearFields() {
  Ez.fill(0);
  Hx.fill(0);
  Hy.fill(0);
  avgIntensity.fill(0);
  activePulses = [];
  activeRightPulses = [];
  stepCount = 0;
  clearDetector();
  needsRender = true;
  scheduleFrame();
}

function clearDetector() {
  detectorTrace.fill(0);
  rightDetectorTrace.fill(0);
  detectorWrite = 0;
  rightDetectorWrite = 0;
  detectorSamplesSincePulse = detectorTraceLength;
  rightDetectorSamplesSincePulse = detectorTraceLength;
  receiverPeak = 0;
  rightReceiverPeak = 0;
  receiverPeakStep = 0;
  rightReceiverPeakStep = 0;
  latestPulseStartStep = -Infinity;
  latestRightPulseStartStep = -Infinity;
  latestPulseDurationSteps = 0;
  latestRightPulseDurationSteps = 0;
}

function pulseDurationSteps(frequency, cycles) {
  return Math.max(18, Math.ceil((cycles / frequency) / dt));
}

function triggerPulse() {
  const frequency = frequencyGHz * 1e9;
  const durationSteps = pulseDurationSteps(frequency, pulseCycles);
  activePulses.push({
    startStep: stepCount,
    frequency,
    durationSteps
  });
  detectorTrace.fill(0);
  detectorWrite = 0;
  detectorSamplesSincePulse = 0;
  latestPulseStartStep = stepCount;
  latestPulseDurationSteps = durationSteps;
  receiverPeak = 0;
  receiverPeakStep = 0;
  running = true;
  needsRender = true;
  scheduleFrame();
  ui.toggleRun.textContent = "Pause";
}

function triggerRightPulse() {
  const frequency = frequencyGHz * 1e9;
  const durationSteps = pulseDurationSteps(frequency, pulseCycles);
  activeRightPulses.push({
    startStep: stepCount,
    frequency,
    durationSteps
  });
  rightDetectorTrace.fill(0);
  rightDetectorWrite = 0;
  rightDetectorSamplesSincePulse = 0;
  latestRightPulseStartStep = stepCount;
  latestRightPulseDurationSteps = durationSteps;
  rightReceiverPeak = 0;
  rightReceiverPeakStep = 0;
  running = true;
  needsRender = true;
  scheduleFrame();
  ui.toggleRun.textContent = "Pause";
}

function computeDamping() {
  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const edge = Math.min(x, y, NX - 1 - x, NY - 1 - y);
      let factor = 1;
      if (edge < boundaryCells) {
        const t = (boundaryCells - edge - 0.5) / boundaryCells;
        factor = Math.exp(-pmlStrength * Math.pow(clamp(t, 0, 1), pmlPower));
      }
      damping[idx(x, y)] = factor;
    }
  }
}

function rebuildMaterials() {
  epsR.fill(1);
  sigma.fill(0);
  objectMask.fill(-1);
  pecMask.fill(0);

  objects.forEach((object, objectIndex) => {
    if (!object.enabled) return;
    const materialProps = materialAtFrequency(object.material);
    const cx = (object.xPct / 100) * (NX - 1);
    const cy = (object.yPct / 100) * (NY - 1);
    const sizeCells = object.sizeCm / 100 / dx;
    const half = sizeCells / 2;

    for (let y = 1; y < NY - 1; y++) {
      for (let x = 1; x < NX - 1; x++) {
        const rx = x - cx;
        const ry = y - cy;
        let inside = false;

        if (object.shape === "circle") {
          inside = rx * rx + ry * ry <= half * half;
        } else if (object.shape === "square") {
          inside = Math.abs(rx) <= half && Math.abs(ry) <= half;
        } else {
          inside = Math.abs(rx) <= Math.max(2, half * 0.28) && Math.abs(ry) <= half * 1.65;
        }

        if (!inside) continue;
        const k = idx(x, y);
        epsR[k] = materialProps.epsR;
        sigma[k] = materialProps.sigma;
        objectMask[k] = objectIndex;
        pecMask[k] = materialProps.pec ? 1 : 0;
      }
    }
  });

  for (let k = 0; k < N; k++) {
    const eps = eps0 * epsR[k];
    const loss = (sigma[k] * dt) / (2 * eps);
    ca[k] = (1 - loss) / (1 + loss);
    cb[k] = dt / (eps * dx) / (1 + loss);
  }

  needsMaterialRebuild = false;
}

function sourceY() {
  return Math.round(clamp((antennaYPct / 100) * (NY - 1), boundaryCells + 3, NY - boundaryCells - 4));
}

function antennaPositionM(antennaXCell = sourceX) {
  return {
    x: antennaXCell * dx,
    y: sourceY() * dy
  };
}

function objectPositionM(object = objects[selectedObject]) {
  return {
    x: (object.xPct / 100) * (NX - 1) * dx,
    y: (object.yPct / 100) * (NY - 1) * dy
  };
}

function selectedRangeInfo(antennaXCell = sourceX) {
  const object = objects[selectedObject];
  if (!object || !object.enabled) return null;

  const antenna = antennaPositionM(antennaXCell);
  const target = objectPositionM(object);
  const distanceM = Math.hypot(target.x - antenna.x, target.y - antenna.y);
  return {
    antenna,
    target,
    distanceM,
    echoSeconds: (2 * distanceM) / c0
  };
}

function antennaFieldSample(antennaXCell = sourceX) {
  const cy = sourceY();
  let total = 0;
  let weightedField = 0;
  for (const tap of antennaKernel) {
    const x = antennaXCell + tap.ox;
    const y = cy + tap.oy;
    if (x <= 0 || x >= NX - 1 || y <= 0 || y >= NY - 1) continue;
    const k = idx(x, y);
    if (pecMask[k]) continue;
    weightedField += Ez[k] * tap.weight;
    total += tap.weight;
  }
  return total > 0 ? weightedField / total : 0;
}

function injectAntenna(value, antennaXCell = sourceX) {
  const cy = sourceY();
  let total = 0;
  for (const tap of antennaKernel) {
    const x = antennaXCell + tap.ox;
    const y = cy + tap.oy;
    if (x <= 0 || x >= NX - 1 || y <= 0 || y >= NY - 1) continue;
    if (pecMask[idx(x, y)]) continue;
    total += tap.weight;
  }

  if (total === 0) return;

  for (const tap of antennaKernel) {
    const x = antennaXCell + tap.ox;
    const y = cy + tap.oy;
    if (x <= 0 || x >= NX - 1 || y <= 0 || y >= NY - 1) continue;
    const k = idx(x, y);
    if (pecMask[k]) continue;
    Ez[k] += value * (tap.weight / total) * compactAntennaGain;
  }
}

function pulseSourceValue(pulses) {
  let value = 0;
  const remainingPulses = [];
  const barker = [1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1];

  for (const pulse of pulses) {
    const age = stepCount - pulse.startStep;
    if (age < 0) {
      remainingPulses.push(pulse);
      continue;
    }

    if (age <= pulse.durationSteps) {
      const center = pulse.durationSteps * 0.5;
      const sigmaSteps = Math.max(1, pulse.durationSteps / 5);
      const envelope = Math.exp(-0.5 * ((age - center) / sigmaSteps) ** 2);
      
      const chipDuration = pulse.durationSteps / barker.length;
      const chipIndex = Math.min(Math.floor(age / chipDuration), barker.length - 1);
      
      value += barker[chipIndex] * Math.sin(2 * Math.PI * pulse.frequency * age * dt) * envelope;
      remainingPulses.push(pulse);
    }
  }

  return { value, remainingPulses };
}

function sampleDetector(rawSignal) {
  const blankUntil = latestPulseStartStep + latestPulseDurationSteps + detectorBlankSteps;
  const gatedSignal = stepCount <= blankUntil ? 0 : rawSignal;
  detectorTrace[detectorWrite] = gatedSignal;
  detectorWrite = (detectorWrite + 1) % detectorTraceLength;

  if (detectorSamplesSincePulse < detectorTraceLength) {
    detectorSamplesSincePulse++;
    const magnitude = Math.abs(gatedSignal);
    if (stepCount > blankUntil && magnitude > receiverPeak) {
      receiverPeak = magnitude;
      receiverPeakStep = stepCount - latestPulseStartStep;
    }
  }
}

function sampleRightDetector(rawSignal) {
  const blankUntil = latestRightPulseStartStep + latestRightPulseDurationSteps + detectorBlankSteps;
  const gatedSignal = stepCount <= blankUntil ? 0 : rawSignal;
  rightDetectorTrace[rightDetectorWrite] = gatedSignal;
  rightDetectorWrite = (rightDetectorWrite + 1) % detectorTraceLength;

  if (rightDetectorSamplesSincePulse < detectorTraceLength) {
    rightDetectorSamplesSincePulse++;
    const magnitude = Math.abs(gatedSignal);
    if (stepCount > blankUntil && magnitude > rightReceiverPeak) {
      rightReceiverPeak = magnitude;
      rightReceiverPeakStep = stepCount - latestRightPulseStartStep;
    }
  }
}

function stepSimulation() {
  if (needsMaterialRebuild) rebuildMaterials();

  for (let y = 0; y < NY - 1; y++) {
    const row = y * NX;
    const nextRow = row + NX;
    for (let x = 0; x < NX; x++) {
      const k = row + x;
      Hx[k] -= (dt / (mu0 * dy)) * (Ez[nextRow + x] - Ez[k]);
    }
  }

  for (let y = 0; y < NY; y++) {
    const row = y * NX;
    for (let x = 0; x < NX - 1; x++) {
      const k = row + x;
      Hy[k] += (dt / (mu0 * dx)) * (Ez[k + 1] - Ez[k]);
    }
  }

  for (let y = 1; y < NY - 1; y++) {
    const row = y * NX;
    const prevRow = row - NX;
    for (let x = 1; x < NX - 1; x++) {
      const k = row + x;
      if (pecMask[k]) {
        Ez[k] = 0;
        continue;
      }
      const curlH = (Hy[k] - Hy[k - 1]) - (Hx[k] - Hx[prevRow + x]);
      Ez[k] = ca[k] * Ez[k] + cb[k] * curlH;
    }
  }

  sampleDetector(antennaFieldSample(sourceX));
  sampleRightDetector(antennaFieldSample(rightSourceX));
  const leftPulse = pulseSourceValue(activePulses);
  activePulses = leftPulse.remainingPulses;
  injectAntenna(1.35 * leftPulse.value, sourceX);
  const rightPulse = pulseSourceValue(activeRightPulses);
  activeRightPulses = rightPulse.remainingPulses;
  injectAntenna(1.35 * rightPulse.value, rightSourceX);

  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const k = idx(x, y);
      const d = damping[k];
      Ez[k] *= d;
      Hx[k] *= d;
      Hy[k] *= d;
      avgIntensity[k] = 0.986 * avgIntensity[k] + 0.014 * Ez[k] * Ez[k];
    }
  }

  stepCount++;
}

function colorRamp(value) {
  const v = clamp(value, -1, 1);
  if (v >= 0) {
    return [
      22 + 228 * v,
      28 + 84 * Math.sqrt(v),
      34 + 38 * (1 - v)
    ];
  }
  const a = -v;
  return [
    18 + 42 * (1 - a),
    30 + 130 * Math.sqrt(a),
    42 + 205 * a
  ];
}

function intensityRamp(value) {
  const v = clamp(value, 0, 1);
  return [
    7 + 238 * v,
    11 + 152 * Math.sqrt(v),
    15 + 22 * (1 - v)
  ];
}

function render() {
  let energy = 0;
  const renderGain = gain;

  for (let y = 0; y < NY; y++) {
    for (let x = 0; x < NX; x++) {
      const k = idx(x, y);
      const p = k * 4;
      let rgb;
      if (viewMode === "intensity") {
        rgb = intensityRamp(Math.sqrt(avgIntensity[k]) * renderGain * 0.7);
      } else {
        rgb = colorRamp(Ez[k] * renderGain);
      }

      const objectIndex = objectMask[k];
      if (objectIndex >= 0) {
        const material = materials[objects[objectIndex].material];
        const overlay = material.color;
        const mix = material.pec ? 0.52 : 0.34;
        rgb = [
          rgb[0] * (1 - mix) + overlay[0] * mix,
          rgb[1] * (1 - mix) + overlay[1] * mix,
          rgb[2] * (1 - mix) + overlay[2] * mix
        ];
      }

      pixels[p] = rgb[0];
      pixels[p + 1] = rgb[1];
      pixels[p + 2] = rgb[2];
      pixels[p + 3] = 255;

      if (x % 4 === 0 && y % 4 === 0) {
        energy += Ez[k] * Ez[k] + Hx[k] * Hx[k] * 80 + Hy[k] * Hy[k] * 80;
      }
    }
  }

  offCtx.putImageData(image, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(offscreen, 0, 0, canvas.width, canvas.height);
  drawObjectOutlines();
  updateReadout(energy / 1000);
}

function renderDetector(
  targetCanvas = detectorCanvas,
  targetCtx = detectorCtx,
  trace = detectorTrace,
  writeIndex = detectorWrite,
  latestStartStep = latestPulseStartStep,
  rangeInfo = selectedRangeInfo(sourceX),
  readoutUpdater = updateReceiverReadout
) {
  const rect = targetCanvas.getBoundingClientRect();
  const width = Math.max(260, Math.round(rect.width * window.devicePixelRatio));
  const height = Math.max(100, Math.round(rect.height * window.devicePixelRatio));
  if (targetCanvas.width !== width || targetCanvas.height !== height) {
    targetCanvas.width = width;
    targetCanvas.height = height;
  }

  targetCtx.fillStyle = "#071015";
  targetCtx.fillRect(0, 0, width, height);
  targetCtx.strokeStyle = "rgba(245, 241, 231, 0.12)";
  targetCtx.lineWidth = 1;
  targetCtx.beginPath();
  for (let i = 1; i < 4; i++) {
    const y = (height * i) / 4;
    targetCtx.moveTo(0, y);
    targetCtx.lineTo(width, y);
  }
  targetCtx.stroke();

  targetCtx.strokeStyle = "rgba(245, 241, 231, 0.42)";
  targetCtx.beginPath();
  targetCtx.moveTo(0, height / 2);
  targetCtx.lineTo(width, height / 2);
  targetCtx.stroke();

  let maxAbs = 0.012;
  for (let i = 0; i < detectorTraceLength; i++) {
    maxAbs = Math.max(maxAbs, Math.abs(trace[i]));
  }

  targetCtx.strokeStyle = "#23b7ad";
  targetCtx.lineWidth = 2;
  targetCtx.beginPath();
  for (let i = 0; i < detectorTraceLength; i++) {
    const sample = trace[(writeIndex + i) % detectorTraceLength];
    const x = (i / (detectorTraceLength - 1)) * width;
    const y = height / 2 - (sample / maxAbs) * height * 0.42;
    if (i === 0) targetCtx.moveTo(x, y);
    else targetCtx.lineTo(x, y);
  }
  targetCtx.stroke();

  if (rangeInfo && Number.isFinite(latestStartStep)) {
    const expectedEchoStep = Math.round(rangeInfo.echoSeconds / dt);
    const samplesAgo = stepCount - latestStartStep - expectedEchoStep;
    const markerIndex = detectorTraceLength - 1 - samplesAgo;
    if (markerIndex >= 0 && markerIndex < detectorTraceLength) {
      const markerX = (markerIndex / (detectorTraceLength - 1)) * width;
      targetCtx.strokeStyle = "rgba(255, 213, 92, 0.9)";
      targetCtx.lineWidth = 2;
      targetCtx.beginPath();
      targetCtx.moveTo(markerX, 0);
      targetCtx.lineTo(markerX, height);
      targetCtx.stroke();
    }
  }

  targetCtx.fillStyle = "rgba(245, 241, 231, 0.76)";
  targetCtx.font = `${Math.max(10, 11 * window.devicePixelRatio)}px sans-serif`;
  targetCtx.fillText(`scale ${maxAbs.toFixed(3)}`, 10 * window.devicePixelRatio, 18 * window.devicePixelRatio);
  readoutUpdater();
}

function renderRangeGraph() {
  const rect = rangeCanvas.getBoundingClientRect();
  const width = Math.max(260, Math.round(rect.width * window.devicePixelRatio));
  const height = Math.max(92, Math.round(rect.height * window.devicePixelRatio));
  if (rangeCanvas.width !== width || rangeCanvas.height !== height) {
    rangeCanvas.width = width;
    rangeCanvas.height = height;
  }

  rangeCtx.fillStyle = "#071015";
  rangeCtx.fillRect(0, 0, width, height);

  const pad = 14 * window.devicePixelRatio;
  const plotW = width - pad * 2;
  const plotH = height - pad * 2;
  const domainW = NX * dx;
  const domainH = NY * dy;
  const scale = Math.min(plotW / domainW, plotH / domainH);
  const ox = (width - domainW * scale) / 2;
  const oy = (height - domainH * scale) / 2;

  rangeCtx.strokeStyle = "rgba(245, 241, 231, 0.28)";
  rangeCtx.lineWidth = 1;
  rangeCtx.strokeRect(ox, oy, domainW * scale, domainH * scale);

  const leftRange = selectedRangeInfo(sourceX);
  const rightRange = selectedRangeInfo(rightSourceX);
  const leftAntenna = antennaPositionM(sourceX);
  const rightAntenna = antennaPositionM(rightSourceX);
  const leftAntennaX = ox + leftAntenna.x * scale;
  const leftAntennaY = oy + leftAntenna.y * scale;
  const rightAntennaX = ox + rightAntenna.x * scale;
  const rightAntennaY = oy + rightAntenna.y * scale;

  function drawAntennaDot(x, y, color) {
    rangeCtx.fillStyle = color;
    rangeCtx.beginPath();
    rangeCtx.arc(x, y, 4 * window.devicePixelRatio, 0, Math.PI * 2);
    rangeCtx.fill();
  }

  drawAntennaDot(leftAntennaX, leftAntennaY, "#f5f1e7");
  drawAntennaDot(rightAntennaX, rightAntennaY, "#7ed3ff");

  if (leftRange && rightRange) {
    const targetX = ox + leftRange.target.x * scale;
    const targetY = oy + leftRange.target.y * scale;
    rangeCtx.lineWidth = 2;
    rangeCtx.strokeStyle = "rgba(255, 213, 92, 0.86)";
    rangeCtx.beginPath();
    rangeCtx.moveTo(leftAntennaX, leftAntennaY);
    rangeCtx.lineTo(targetX, targetY);
    rangeCtx.stroke();

    rangeCtx.strokeStyle = "rgba(126, 211, 255, 0.86)";
    rangeCtx.beginPath();
    rangeCtx.moveTo(rightAntennaX, rightAntennaY);
    rangeCtx.lineTo(targetX, targetY);
    rangeCtx.stroke();

    const material = materials[objects[selectedObject].material];
    rangeCtx.fillStyle = `rgb(${material.color[0]}, ${material.color[1]}, ${material.color[2]})`;
    rangeCtx.beginPath();
    rangeCtx.arc(targetX, targetY, 5 * window.devicePixelRatio, 0, Math.PI * 2);
    rangeCtx.fill();

    ui.rangeDistance.textContent = `Left ${leftRange.distanceM.toFixed(2)} m`;
    ui.rangeDelay.textContent = `Left echo ${(leftRange.echoSeconds * 1e9).toFixed(2)} ns`;
    ui.rightRangeDistance.textContent = `Right ${rightRange.distanceM.toFixed(2)} m`;
    ui.rightRangeDelay.textContent = `Right echo ${(rightRange.echoSeconds * 1e9).toFixed(2)} ns`;
  } else {
    ui.rangeDistance.textContent = "Left disabled";
    ui.rangeDelay.textContent = "Left echo --";
    ui.rightRangeDistance.textContent = "Right disabled";
    ui.rightRangeDelay.textContent = "Right echo --";
  }
}

function drawObjectOutlines() {
  const sx = canvas.width / NX;
  const sy = canvas.height / NY;
  ctx.save();
  ctx.lineWidth = 2;
  objects.forEach((object, index) => {
    if (!object.enabled) return;
    const material = materials[object.material];
    const x = (object.xPct / 100) * canvas.width;
    const y = (object.yPct / 100) * canvas.height;
    const sizePx = (object.sizeCm / 100 / dx) * sx;
    ctx.strokeStyle = `rgba(${material.color[0]}, ${material.color[1]}, ${material.color[2]}, ${index === selectedObject ? 0.95 : 0.68})`;
    ctx.setLineDash(index === selectedObject ? [] : [6, 5]);
    ctx.beginPath();
    if (object.shape === "circle") {
      ctx.arc(x, y, sizePx / 2, 0, Math.PI * 2);
    } else if (object.shape === "square") {
      ctx.rect(x - sizePx / 2, y - sizePx / 2, sizePx, sizePx);
    } else {
      ctx.rect(x - sizePx * 0.14, y - sizePx * 0.825, sizePx * 0.28, sizePx * 1.65);
    }
    ctx.stroke();
  });
  ctx.setLineDash([]);
  function drawAntennaMarker(xCell, stroke, fill) {
    const antennaX = xCell * sx;
    const antennaY = sourceY() * sy;
    ctx.strokeStyle = stroke;
    ctx.fillStyle = fill;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(antennaX, antennaY, Math.max(5, 2.3 * sx), 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(antennaX, antennaY - Math.max(12, 5 * sy));
    ctx.lineTo(antennaX, antennaY + Math.max(12, 5 * sy));
    ctx.moveTo(antennaX - Math.max(12, 5 * sx), antennaY);
    ctx.lineTo(antennaX + Math.max(12, 5 * sx), antennaY);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(antennaX, antennaY, Math.max(2.2, sx), 0, Math.PI * 2);
    ctx.fill();
  }
  drawAntennaMarker(sourceX, "rgba(245, 241, 231, 0.78)", "rgba(245, 241, 231, 0.86)");
  drawAntennaMarker(rightSourceX, "rgba(126, 211, 255, 0.78)", "rgba(126, 211, 255, 0.86)");
  ctx.restore();
}

function updateReadout(energy) {
  const ns = stepCount * dt * 1e9;
  const wavelengthCm = (c0 / (frequencyGHz * 1e9)) * 100;
  ui.readoutMode.textContent =
    viewMode === "intensity" ? "Mean intensity |Ez|^2" : "Instantaneous Ez field";
  ui.readoutTime.textContent = `${ns.toFixed(2)} ns, lambda ${wavelengthCm.toFixed(1)} cm`;
  ui.readoutEnergy.textContent = `Energy ${energy.toFixed(3)}`;
}

function updateReceiverReadout() {
  if (!Number.isFinite(latestPulseStartStep)) {
    ui.receiverStatus.textContent = "Ready";
    ui.receiverPeak.textContent = "Peak 0.000";
    return;
  }

  const ageSteps = stepCount - latestPulseStartStep;
  const blankUntil = latestPulseStartStep + latestPulseDurationSteps + detectorBlankSteps;
  const ageNs = ageSteps * dt * 1e9;
  ui.receiverStatus.textContent = stepCount <= blankUntil ? "Transmit blanking" : `Listening ${ageNs.toFixed(2)} ns`;

  if (receiverPeakStep > 0) {
    const rangeCm = (c0 * receiverPeakStep * dt * 0.5) * 100;
    ui.receiverPeak.textContent = `Peak ${receiverPeak.toFixed(3)} @ ${rangeCm.toFixed(0)} cm`;
  } else {
    ui.receiverPeak.textContent = `Peak ${receiverPeak.toFixed(3)}`;
  }
}

function updateRightReceiverReadout() {
  if (!Number.isFinite(latestRightPulseStartStep)) {
    ui.rightReceiverStatus.textContent = "Ready";
    ui.rightReceiverPeak.textContent = "Peak 0.000";
    return;
  }

  const ageSteps = stepCount - latestRightPulseStartStep;
  const blankUntil = latestRightPulseStartStep + latestRightPulseDurationSteps + detectorBlankSteps;
  const ageNs = ageSteps * dt * 1e9;
  ui.rightReceiverStatus.textContent = stepCount <= blankUntil ? "Transmit blanking" : `Listening ${ageNs.toFixed(2)} ns`;

  if (rightReceiverPeakStep > 0) {
    const rangeCm = (c0 * rightReceiverPeakStep * dt * 0.5) * 100;
    ui.rightReceiverPeak.textContent = `Peak ${rightReceiverPeak.toFixed(3)} @ ${rangeCm.toFixed(0)} cm`;
  } else {
    ui.rightReceiverPeak.textContent = `Peak ${rightReceiverPeak.toFixed(3)}`;
  }
}

function syncLabels() {
  needsRender = true;
  scheduleFrame();
  ui.frequencyLabel.textContent = `${frequencyGHz.toFixed(2)} GHz`;
  ui.antennaY.value = antennaYPct;
  ui.antennaYLabel.textContent = `${antennaYPct}%`;
  ui.sandboxWidth.value = domainWidthM.toFixed(1);
  ui.sandboxWidthLabel.textContent = `${domainWidthM.toFixed(1)} m`;
  ui.pulseCycles.value = pulseCycles;
  ui.pulseCyclesLabel.textContent = `${pulseCycles} ${pulseCycles === 1 ? "cycle" : "cycles"}`;
  ui.gainLabel.textContent = `${gain.toFixed(1)}x`;
  ui.gridModelLabel.textContent =
    `Yee-grid TEz FDTD, ${NX} x ${NY} cells, ${(NX * dx).toFixed(1)} m x ${(NY * dy).toFixed(1)} m, dx = 0.5 cm, CFL 0.70.`;

  const object = objects[selectedObject];
  ui.objectEnabled.checked = object.enabled;
  ui.material.value = object.material;
  ui.shape.value = object.shape;
  ui.size.value = object.sizeCm;
  ui.sizeLabel.textContent = `${object.sizeCm} cm`;
  ui.xPos.value = object.xPct;
  ui.xLabel.textContent = `${object.xPct}%`;
  ui.yPos.value = object.yPct;
  ui.yLabel.textContent = `${object.yPct}%`;

  ui.tabs.forEach((tab, index) => {
    tab.classList.toggle("selected", index === selectedObject);
  });

  const material = materials[object.material];
  const materialProps = materialAtFrequency(object.material);
  ui.materialFacts.innerHTML = `
    <dt>epsr</dt><dd>${materialProps.pec ? "PEC" : materialProps.epsR.toPrecision(3)}</dd>
    <dt>sigma</dt><dd>${materialProps.pec ? "infinite" : `${materialProps.sigma.toPrecision(3)} S/m`}</dd>
    <dt>Boundary</dt><dd>${material.note}</dd>
  `;
}

function setObjectPatch(patch) {
  Object.assign(objects[selectedObject], patch);
  needsMaterialRebuild = true;
  syncLabels();
}

function installEvents() {
  ui.triggerPulse.addEventListener("click", () => {
    triggerPulse();
  });

  ui.triggerRightPulse.addEventListener("click", () => {
    triggerRightPulse();
  });

  ui.toggleRun.addEventListener("click", () => {
    running = !running;
    ui.toggleRun.textContent = running ? "Pause" : "Run";
    needsRender = true;
    if (running) scheduleFrame();
  });

  ui.resetSim.addEventListener("click", () => clearFields());

  ui.frequency.addEventListener("input", () => {
    frequencyGHz = Number(ui.frequency.value);
    needsMaterialRebuild = true;
    avgIntensity.fill(0);
    activePulses = [];
    activeRightPulses = [];
    clearDetector();
    syncLabels();
  });

  ui.antennaY.addEventListener("input", () => {
    antennaYPct = Number(ui.antennaY.value);
    clearDetector();
    activePulses = [];
    activeRightPulses = [];
    avgIntensity.fill(0);
    syncLabels();
  });

  ui.sandboxWidth.addEventListener("change", () => {
    allocateGrid(Number(ui.sandboxWidth.value));
    syncLabels();
  });

  ui.pulseCycles.addEventListener("input", () => {
    pulseCycles = Number(ui.pulseCycles.value);
    syncLabels();
  });

  ui.gain.addEventListener("input", () => {
    gain = Number(ui.gain.value);
    syncLabels();
  });

  ui.viewMode.addEventListener("change", () => {
    viewMode = ui.viewMode.value;
    needsRender = true;
    scheduleFrame();
  });

  ui.tabs.forEach((tab, index) => {
    tab.addEventListener("click", () => {
      selectedObject = index;
      syncLabels();
    });
  });

  ui.objectEnabled.addEventListener("change", () => {
    setObjectPatch({ enabled: ui.objectEnabled.checked });
  });

  ui.material.addEventListener("change", () => {
    setObjectPatch({ material: ui.material.value });
  });

  ui.shape.addEventListener("change", () => {
    setObjectPatch({ shape: ui.shape.value });
  });

  ui.size.addEventListener("input", () => {
    setObjectPatch({ sizeCm: Number(ui.size.value) });
  });

  ui.xPos.addEventListener("input", () => {
    setObjectPatch({ xPct: Number(ui.xPos.value) });
  });

  ui.yPos.addEventListener("input", () => {
    setObjectPatch({ yPct: Number(ui.yPos.value) });
  });

  ui.presetSingle.addEventListener("click", () => {
    objects[0] = { enabled: true, material: "metal", shape: "circle", sizeCm: 30, xPct: 58, yPct: 50 };
    objects[1] = { enabled: false, material: "freshWater", shape: "circle", sizeCm: 22, xPct: 72, yPct: 35 };
    objects[2] = { enabled: false, material: "glass", shape: "square", sizeCm: 18, xPct: 72, yPct: 66 };
    selectedObject = 0;
    needsMaterialRebuild = true;
    clearFields();
    syncLabels();
  });

  ui.presetDouble.addEventListener("click", () => {
    objects[0] = { enabled: true, material: "metal", shape: "circle", sizeCm: 22, xPct: 57, yPct: 38 };
    objects[1] = { enabled: true, material: "freshWater", shape: "circle", sizeCm: 30, xPct: 67, yPct: 62 };
    objects[2] = { enabled: true, material: "glass", shape: "square", sizeCm: 18, xPct: 78, yPct: 46 };
    selectedObject = 1;
    needsMaterialRebuild = true;
    clearFields();
    syncLabels();
  });

  ui.presetSlit.addEventListener("click", () => {
    objects[0] = { enabled: true, material: "metal", shape: "slab", sizeCm: 64, xPct: 55, yPct: 25 };
    objects[1] = { enabled: true, material: "metal", shape: "slab", sizeCm: 64, xPct: 55, yPct: 75 };
    objects[2] = { enabled: true, material: "saltWater", shape: "circle", sizeCm: 20, xPct: 76, yPct: 50 };
    selectedObject = 2;
    needsMaterialRebuild = true;
    clearFields();
    syncLabels();
  });

  canvas.addEventListener("pointerdown", moveSelectedObject);
  canvas.addEventListener("pointermove", (event) => {
    if (event.buttons === 1) moveSelectedObject(event);
  });
}

function moveSelectedObject(event) {
  const rect = canvas.getBoundingClientRect();
  const xPct = Math.round(clamp(((event.clientX - rect.left) / rect.width) * 100, 18, 88));
  const yPct = Math.round(clamp(((event.clientY - rect.top) / rect.height) * 100, 12, 88));
  setObjectPatch({ xPct, yPct });
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(480, Math.round(rect.width * window.devicePixelRatio));
  const height = Math.max(300, Math.round(rect.height * window.devicePixelRatio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    needsRender = true;
  }
}

function scheduleFrame() {
  if (frameScheduled) return;
  frameScheduled = true;
  requestAnimationFrame(frame);
}

function frame() {
  frameScheduled = false;
  resizeCanvas();
  if (running) {
    const steps = viewMode === "intensity" ? 5 : 4;
    for (let i = 0; i < steps; i++) stepSimulation();
    needsRender = true;
  }

  if (!needsRender) {
    return;
  }

  render();
  renderDetector();
  renderDetector(
    rightDetectorCanvas,
    rightDetectorCtx,
    rightDetectorTrace,
    rightDetectorWrite,
    latestRightPulseStartStep,
    selectedRangeInfo(rightSourceX),
    updateRightReceiverReadout
  );
  renderRangeGraph();
  needsRender = running;
  if (running || needsRender) scheduleFrame();
}

allocateGrid(domainWidthM);
installEvents();
syncLabels();
scheduleFrame();
