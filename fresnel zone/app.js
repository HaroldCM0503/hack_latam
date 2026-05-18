document.addEventListener("DOMContentLoaded", () => {
    // UI Elements
    const els = {
        freqSlider: document.getElementById('freqSlider'),
        freqVal: document.getElementById('freqVal'),
        velSlider: document.getElementById('velSlider'),
        velVal: document.getElementById('velVal'),
        pathBtns: document.querySelectorAll('.button-group button[data-path]'),
        ofdmToggle: document.getElementById('ofdmToggle'),
        multiPeerToggle: document.getElementById('multiPeerToggle'),
        fpsBadge: document.getElementById('fpsBadge'),
        modeBadge: document.getElementById('modeBadge'),
        spatialCanvas: document.getElementById('spatialCanvas'),
        dspCanvas: document.getElementById('dspCanvas'),
        noiseSlider: document.getElementById('noiseSlider'),
        noiseVal: document.getElementById('noiseVal'),
        crossingBadge: document.getElementById('crossingBadge'),
        speedEstBadge: document.getElementById('speedEstBadge'),
        trueSpeedBadge: document.getElementById('trueSpeedBadge'),
        spawnParticleBtn: document.getElementById('spawnParticleBtn')
    };

    // Contexts
    const ctxSpatial = els.spatialCanvas.getContext('2d', { alpha: false });
    const ctxDsp = els.dspCanvas.getContext('2d', { alpha: false });

    // Constants & Physics State
    const C = 299792458; // Speed of light (m/s)
    const deltaTime = 0.1;
    let state = {
        frequency: 23e9,
        targetSpeed: 1000,
        noiseLevel: 0.05,
        path: 'cross',
        isOFDM: false,
        isMultiPeer: false,
        time: 0,
        isDragging: false,
        timeScale: deltaTime
    };

    // Room Geometry
    const room = { width: 4000000, height: 3000 };
    const cy = room.height / 2;
    const nodes = {
        tx1: { x: 0, y: cy, color: '#f59e0b', label: 'Iridium Sat 1' },
        tx2: { x: 0, y: cy + (room.height * 0.1), color: '#d946ef', label: 'Tx2' },
        rx: { x: room.width, y: cy, color: '#10b981', label: 'Iridium Sat 2' }
    };
    let target = { x: room.width / 2, y: cy, vx: 0, vy: 0 };

    // DSP Buffers
    const bufferSize = 500;
    const dspBuffer1 = []; // Tx1 -> Rx
    const dspBuffer2 = []; // Tx2 -> Rx
    const ofdmBuffer1 = []; // Tx1 -> Rx (OFDM)
    const ofdmBuffer2 = []; // Tx2 -> Rx (OFDM)
    const spectrogramBuffer = []; // FFT Spectrogram

    const fftSize = 64; // Window size for STFT

    for (let i = 0; i < bufferSize; i++) {
        dspBuffer1.push(0);
        dspBuffer2.push(0);
        ofdmBuffer1.push(new Float32Array(52).fill(0));
        ofdmBuffer2.push(new Float32Array(52).fill(0));
        spectrogramBuffer.push(new Float32Array(fftSize / 2).fill(0));
    }

    // Resize Handling
    function resize() {
        const resizeCanvas = (canvas) => {
            const rect = canvas.parentElement.getBoundingClientRect();
            canvas.width = rect.width * (window.devicePixelRatio || 1);
            canvas.height = rect.height * (window.devicePixelRatio || 1);
        };
        resizeCanvas(els.spatialCanvas);
        resizeCanvas(els.dspCanvas);
    }
    window.addEventListener('resize', resize);
    resize(); // Initial sizing

    // Control Listeners
    els.freqSlider.addEventListener('input', (e) => {
        state.frequency = parseFloat(e.target.value) * 1e9;
        els.freqVal.textContent = parseFloat(e.target.value).toFixed(2) + ' GHz';
    });

    els.spawnParticleBtn.addEventListener('click', () => {
        state.path = 'particle';
        const crossX = room.width / 2; // cross exactly in middle
        const speed = 2000 + Math.random() * 8000; // 2km/s to 10km/s
        // Allow a fully random angle! (We just avoid perfectly horizontal < 5 degrees so it eventually crosses)
        let angle = Math.random() * Math.PI * 2;
        while (Math.abs(Math.sin(angle)) < 0.1) {
            angle = Math.random() * Math.PI * 2;
        }

        // Spawn closer to the crossing point (e.g., 25% of room height)
        const cy = room.height / 2;
        const spawnRadiusY = room.height * 0.25;
        const distToCross = Math.abs(spawnRadiusY / Math.sin(angle));

        target.x = crossX - Math.cos(angle) * distToCross;
        target.y = cy - Math.sin(angle) * distToCross;
        target.vx = Math.cos(angle) * speed;
        target.vy = Math.sin(angle) * speed;
        target.trueSpeed = speed;

        els.trueSpeedBadge.textContent = `True Speed: ${speed.toFixed(1)} m/s`;
        els.trueSpeedBadge.classList.add('active');
        els.speedEstBadge.classList.remove('active');
        els.speedEstBadge.textContent = 'Est. Speed: --';

        // Fixed time scale so different speeds actually look different visually!
        // timeScale = 0.05 means 10,000 m/s takes 2 real seconds to cross 1000m,
        // and 2,000 m/s takes 10 real seconds.
        state.timeScale = deltaTime;

        chirpState.inZone = false;
        els.pathBtns.forEach(b => b.classList.remove('active'));
    });

    els.velSlider.addEventListener('input', (e) => {
        state.targetSpeed = parseFloat(e.target.value);
        els.velVal.textContent = e.target.value + ' m/s';
    });

    els.noiseSlider.addEventListener('input', (e) => {
        state.noiseLevel = parseFloat(e.target.value);
        els.noiseVal.textContent = Math.round(state.noiseLevel * 100) + '%';
    });

    els.pathBtns.forEach(btn => {
        btn.addEventListener('click', (e) => {
            els.pathBtns.forEach(b => b.classList.remove('active'));
            e.target.classList.add('active');
            state.path = e.target.dataset.path;
        });
    });

    els.ofdmToggle.addEventListener('change', (e) => {
        state.isOFDM = e.target.checked;
        els.modeBadge.textContent = state.isOFDM ? 'OFDM (52 SC)' : 'Single-Carrier';
    });

    els.multiPeerToggle.addEventListener('change', (e) => {
        state.isMultiPeer = e.target.checked;
    });

    // Mouse Interaction for Target
    const getMousePos = (e, canvas) => {
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        const x = (e.clientX - rect.left) * dpr;
        const y = (e.clientY - rect.top) * dpr;

        const scaleX = canvas.width / room.width;
        const scaleY = canvas.height / room.height;

        return {
            x: x / scaleX,
            y: y / scaleY
        };
    };

    els.spatialCanvas.addEventListener('mousedown', (e) => {
        if (state.path === 'interactive') {
            state.isDragging = true;
            target = getMousePos(e, els.spatialCanvas);
        }
    });

    window.addEventListener('mousemove', (e) => {
        if (state.isDragging && state.path === 'interactive') {
            const p = getMousePos(e, els.spatialCanvas);
            target.x = Math.max(0, Math.min(room.width, p.x));
            target.y = Math.max(0, Math.min(room.height, p.y));
        }
    });

    window.addEventListener('mouseup', () => state.isDragging = false);

    // Physics Update
    function updateKinematics(dt) {
        state.time += dt;
        const t = state.time;
        const v = state.targetSpeed;

        if (state.path === 'particle') {
            target.x += target.vx * dt;
            target.y += target.vy * dt;
        } else if (state.path !== 'interactive') {
            state.timeScale = deltaTime;
            if (state.path === 'cross') {
                target.x = room.width / 2;
                target.y = (room.height / 2) + (room.height * 0.12) * Math.sin(t * v * 0.005);
            }
        }
    }

    // Mathematical Engine
    function calculateAmplitude(tx, rx, tgt, freqOffset = 0) {
        const f = state.frequency + freqOffset;
        const lambda = C / f;

        const d_tx = Math.hypot(tgt.x - tx.x, tgt.y - tx.y);
        const d_rx = Math.hypot(tgt.x - rx.x, tgt.y - rx.y);
        const d_total = d_tx + d_rx;

        const D = Math.hypot(rx.x - tx.x, rx.y - tx.y);
        const excess_path = d_total - D;

        // Attenuate interference heavily outside the inner Fresnel zones
        const attenuation = Math.exp(-excess_path / (lambda * 3.0));

        // Use excess_path to preserve floating point precision over 4,000 km!
        // ADDED: Math.PI phase shift. By Babinet's Principle, forward scattering by an opaque object 
        // is exactly out of phase (180 degrees or PI) with the incident wave, creating the shadow.
        const phase = (2 * Math.PI * excess_path) / lambda + Math.PI;

        // Calculate total CSI magnitude: |Direct + Scattered|
        // The Direct path is a constant 1.0. The scattered path is attenuation * exp(j * phase).
        // Using the Law of Cosines to find the magnitude of the sum of these two complex vectors:
        const totalAmplitude = Math.sqrt(1 + Math.pow(attenuation, 2) + 2 * attenuation * Math.cos(phase));

        // Add random noise
        const noise = (Math.random() * 2 - 1) * state.noiseLevel;

        // We return the variation around the baseline (1.0) so the oscilloscope stays centered at 0
        return (totalAmplitude - 1.0) + noise;
    }

    function updateDSP() {
        dspBuffer1.shift();
        dspBuffer2.shift();
        const newOfdm1 = ofdmBuffer1.shift();
        const newOfdm2 = ofdmBuffer2.shift();

        // Single Carrier calculation
        const amp1 = calculateAmplitude(nodes.tx1, nodes.rx, target);
        dspBuffer1.push(amp1);

        let amp2 = 0;
        if (state.isMultiPeer) {
            amp2 = calculateAmplitude(nodes.tx2, nodes.rx, target);
        }
        dspBuffer2.push(amp2);

        // OFDM Calculation (52 Subcarriers over 20MHz channel)
        if (state.isOFDM) {
            const spacing = 312500; // ~312.5 kHz spacing
            for (let i = 0; i < 52; i++) {
                const offset = (i - 26) * spacing;
                newOfdm1[i] = calculateAmplitude(nodes.tx1, nodes.rx, target, offset);
                if (state.isMultiPeer) {
                    newOfdm2[i] = calculateAmplitude(nodes.tx2, nodes.rx, target, offset);
                }
            }
        }

        ofdmBuffer1.push(newOfdm1);
        ofdmBuffer2.push(newOfdm2);

        // STFT Calculation for Spectrogram (using last fftSize samples)
        if (!state.isOFDM) {
            // Remove DC bias
            let mean = 0;
            for (let i = 0; i < fftSize; i++) {
                mean += dspBuffer1[bufferSize - fftSize + i];
            }
            mean /= fftSize;

            const fftMag = new Float32Array(fftSize / 2);
            for (let k = 0; k < fftSize / 2; k++) {
                let re = 0, im = 0;
                for (let n = 0; n < fftSize; n++) {
                    const val = dspBuffer1[bufferSize - fftSize + n] - mean;
                    // Apply Hanning window
                    const window = 0.5 * (1 - Math.cos(2 * Math.PI * n / (fftSize - 1)));
                    const angle = 2 * Math.PI * k * n / fftSize;
                    re += val * window * Math.cos(angle);
                    im -= val * window * Math.sin(angle);
                }
                fftMag[k] = Math.sqrt(re * re + im * im);
            }
            spectrogramBuffer.shift();
            spectrogramBuffer.push(fftMag);
        }
    }

    // Chirp Analysis
    let chirpState = {
        inZone: false,
        entryTime: 0,
        lastSpeed: 0
    };

    function analyzeChirp() {
        // Calculate moving average of absolute amplitude to find envelope
        const N = 10;
        let sum = 0;
        for (let i = bufferSize - N; i < bufferSize; i++) {
            sum += Math.abs(dspBuffer1[i]);
        }
        const envelope = sum / N;

        const threshold = 0.35 + state.noiseLevel * 0.5;

        if (envelope > threshold && !chirpState.inZone) {
            // Entered inner Fresnel zone
            chirpState.inZone = true;
            chirpState.entryTime = state.time;
            els.crossingBadge.classList.add('active');
            els.crossingBadge.textContent = 'Crossing Detected!';
        } else if (envelope < threshold * 0.8 && chirpState.inZone) {
            // Exited inner Fresnel zone
            chirpState.inZone = false;
            els.crossingBadge.classList.remove('active');
            els.crossingBadge.textContent = 'No Crossing';

            // Estimate speed
            const dt = state.time - chirpState.entryTime; // in sim time
            if (dt > 0.0001) {
                const lambda = C / state.frequency;
                const D = Math.hypot(nodes.rx.x - nodes.tx1.x, nodes.rx.y - nodes.tx1.y);
                // The Fresnel diameter at the midpoint is approx sqrt(D * lambda)
                // We'll estimate the crossing distance as the diameter.
                const distance = Math.sqrt(D * lambda);
                let speedEst = distance / dt;

                els.speedEstBadge.classList.add('active');
                els.speedEstBadge.textContent = `Est. Speed: ${speedEst.toFixed(1)} m/s`;

                setTimeout(() => {
                    els.speedEstBadge.classList.remove('active');
                }, 2000);
            }
        }
    }

    // Spatial Rendering
    function drawSpatial() {
        const c = els.spatialCanvas;
        const ctx = ctxSpatial;

        ctx.fillStyle = '#060913';
        ctx.fillRect(0, 0, c.width, c.height);

        const scaleX = c.width / room.width;
        const scaleY = c.height / room.height;

        ctx.save();

        // Draw Elliptical Fresnel Zones
        const drawFresnelZones = (tx, rx, colorStr) => {
            const lambda = C / state.frequency;
            const D = Math.hypot(rx.x - tx.x, rx.y - tx.y);
            const cx = (tx.x + rx.x) / 2;
            const cy = (tx.y + rx.y) / 2;
            // For satellites placed horizontally, angle is 0.

            for (let n = 1; n <= 8; n++) {
                const a = (D + n * lambda / 2) / 2;
                const c_dist = D / 2;
                const b2 = a * a - c_dist * c_dist;
                if (b2 <= 0) continue;
                const b = Math.sqrt(b2);

                ctx.beginPath();
                ctx.ellipse(cx * scaleX, cy * scaleY, a * scaleX, b * scaleY, 0, 0, 2 * Math.PI);
                ctx.strokeStyle = colorStr;
                ctx.lineWidth = 1.5;
                ctx.globalAlpha = Math.max(0.05, 0.6 - (n * 0.06));
                ctx.stroke();

                if (n === 1) {
                    ctx.fillStyle = colorStr;
                    ctx.globalAlpha = 1.0;
                    ctx.font = '500 12px Outfit';
                    ctx.textAlign = 'center';
                    // Label radius above the center
                    ctx.fillText(`r = ${b.toFixed(1)} m`, cx * scaleX, (cy - b) * scaleY - 10);
                }
            }
        };

        drawFresnelZones(nodes.tx1, nodes.rx, 'rgba(14, 165, 233, 1)'); // Blue
        if (state.isMultiPeer) {
            drawFresnelZones(nodes.tx2, nodes.rx, 'rgba(217, 70, 239, 1)'); // Magenta
        }

        // Draw Links
        const drawLink = (tx, rx, color) => {
            // Direct path
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
            ctx.setLineDash([5, 5]);
            ctx.beginPath();
            ctx.moveTo(tx.x * scaleX, tx.y * scaleY);
            ctx.lineTo(rx.x * scaleX, rx.y * scaleY);
            ctx.stroke();
            ctx.setLineDash([]);

            // Reflected path
            ctx.strokeStyle = color;
            ctx.globalAlpha = 0.6;
            ctx.beginPath();
            ctx.moveTo(tx.x * scaleX, tx.y * scaleY);
            ctx.lineTo(target.x * scaleX, target.y * scaleY);
            ctx.lineTo(rx.x * scaleX, rx.y * scaleY);
            ctx.stroke();
            ctx.globalAlpha = 1.0;
        };

        drawLink(nodes.tx1, nodes.rx, nodes.tx1.color);
        if (state.isMultiPeer) drawLink(nodes.tx2, nodes.rx, nodes.tx2.color);

        // Draw Nodes
        const drawNode = (p, isTarget) => {
            const x = p.x * scaleX;
            const y = p.y * scaleY;

            ctx.shadowBlur = isTarget ? 5 : 20;
            ctx.shadowColor = p.color || '#ef4444';
            ctx.fillStyle = p.color || '#ef4444';

            ctx.beginPath();
            // 2cm particle is tiny, but draw it visible
            ctx.arc(x, y, isTarget ? 4 : 9, 0, 2 * Math.PI);
            ctx.fill();

            ctx.shadowBlur = 0;
            if (p.label) {
                ctx.fillStyle = '#fff';
                ctx.font = '500 14px Outfit';
                ctx.textAlign = 'center';
                ctx.fillText(p.label, x, y - 18);
            }
        };

        drawNode(nodes.tx1);
        if (state.isMultiPeer) drawNode(nodes.tx2);
        drawNode(nodes.rx);
        if (state.path === 'particle') {
            drawNode({ x: target.x, y: target.y, label: 'Particle (2cm)' }, true);
        } else {
            drawNode({ x: target.x, y: target.y, label: 'Target' }, true);
        }



        ctx.restore();
    }

    // DSP Rendering
    function drawDSP() {
        const c = els.dspCanvas;
        const ctx = ctxDsp;

        ctx.fillStyle = '#060913';
        ctx.fillRect(0, 0, c.width, c.height);

        // Horizontal Grid Lines
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 1; i < 4; i++) {
            const y = (c.height / 4) * i;
            ctx.moveTo(0, y);
            ctx.lineTo(c.width, y);
        }
        ctx.stroke();

        const drawOscilloscope = (buffer, color, yOffset, heightScale) => {
            ctx.strokeStyle = color;
            ctx.lineWidth = 2.5;
            ctx.lineJoin = 'round';
            ctx.shadowBlur = 12;
            ctx.shadowColor = color;

            ctx.beginPath();
            for (let i = 0; i < bufferSize; i++) {
                const x = (i / bufferSize) * c.width;
                const y = yOffset - (buffer[i] * heightScale);
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
            ctx.shadowBlur = 0;
        };

        const drawWaterfall = (ofdmBuffer, yOffset, heightDivisor) => {
            const barWidth = c.width / bufferSize;
            const totalH = c.height / heightDivisor;
            const scHeight = totalH / 52;

            for (let i = 0; i < bufferSize; i += 2) { // Step 2 for perf
                const x = (i / bufferSize) * c.width;
                const amps = ofdmBuffer[i];

                for (let sc = 0; sc < 52; sc++) {
                    const y = yOffset - (totalH / 2) + sc * scHeight;
                    const val = (amps[sc] + 1) / 2; // Normalize to 0-1

                    // Colormap mapping 0->1 to deep blue -> bright cyan
                    const r = Math.floor(val * 14);
                    const g = Math.floor(val * 165);
                    const b = Math.floor(200 + val * 55);

                    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${val * 0.8 + 0.2})`;
                    ctx.fillRect(x, y, barWidth * 2.5, scHeight + 0.5);
                }
            }
        };

        if (state.isOFDM) {
            if (state.isMultiPeer) {
                drawWaterfall(ofdmBuffer1, c.height * 0.25, 2.2);
                drawWaterfall(ofdmBuffer2, c.height * 0.75, 2.2);
            } else {
                drawWaterfall(ofdmBuffer1, c.height * 0.5, 1.2);
            }
        } else {
            const hScale = state.isMultiPeer ? c.height * 0.2 : c.height * 0.35;

            if (state.isMultiPeer) {
                drawOscilloscope(dspBuffer1, '#0ea5e9', c.height * 0.25, hScale);
                drawOscilloscope(dspBuffer2, '#d946ef', c.height * 0.75, hScale);
            } else {
                // Split view: top half Oscilloscope, bottom half STFT Spectrogram
                drawOscilloscope(dspBuffer1, '#0ea5e9', c.height * 0.25, hScale * 0.5);

                // Draw Spectrogram Waterfall
                const totalH = c.height * 0.5;
                const yOffset = c.height * 0.75;
                const barWidth = c.width / bufferSize;
                const numBins = fftSize / 2;
                const binHeight = totalH / numBins;

                ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
                ctx.font = '12px Outfit';
                ctx.textAlign = 'left';
                ctx.fillText('STFT Spectrogram (Freq vs Time)', 10, c.height * 0.5 + 15);

                for (let i = 0; i < bufferSize; i += 2) {
                    const x = (i / bufferSize) * c.width;
                    const bins = spectrogramBuffer[i] || new Float32Array(numBins);

                    for (let k = 0; k < numBins; k++) {
                        const y = c.height - (k + 1) * binHeight;
                        // Normalize the magnitude. Adjust divisor based on expected peak signal.
                        const val = Math.min(bins[k] / 5.0, 1.0);

                        let r = 0, g = 0, b = 0;
                        if (val < 0.5) {
                            r = 0;
                            g = Math.floor(val * 2 * 255);
                            b = Math.floor(val * 2 * 128 + 127);
                        } else {
                            r = Math.floor((val - 0.5) * 2 * 255);
                            g = 255;
                            b = Math.floor((1.0 - val) * 255);
                        }

                        ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${val * 0.8 + 0.2})`;
                        ctx.fillRect(x, y, barWidth * 2.5, binHeight + 0.5);
                    }
                }
            }
        }
    }

    // --- AI Integration (Transit Detection) ---
    let aiState = {
        isPredicting: false,
    };

    const trainAiBtn = document.getElementById('trainAiBtn');
    const aiPredictToggle = document.getElementById('aiPredictToggle');
    const aiTransitPanel = document.getElementById('aiTransitPanel');
    const aiStatusVal = document.getElementById('aiStatusVal');
    const aiSpeedVal = document.getElementById('aiSpeedVal');
    
    if (trainAiBtn) {
        trainAiBtn.addEventListener('click', async () => {
            trainAiBtn.disabled = true;
            trainAiBtn.textContent = 'Training AI on Backend (5000 samples)...';
            
            try {
                const res = await fetch('http://localhost:5000/train', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ num_samples: 5000 })
                });
                
                if (res.ok) {
                    trainAiBtn.textContent = 'AI Trained Successfully!';
                    trainAiBtn.style.backgroundColor = '#10b981';
                } else {
                    throw new Error('Server error');
                }
            } catch (err) {
                console.error(err);
                trainAiBtn.textContent = 'Backend Error (Check Console)';
                trainAiBtn.style.backgroundColor = '#ef4444';
            }
            
            setTimeout(() => {
                trainAiBtn.disabled = false;
                trainAiBtn.textContent = 'Train AI (Backend)';
                trainAiBtn.style.backgroundColor = '#ec4899';
            }, 3000);
        });
    }
        
        if (aiPredictToggle) {
            aiPredictToggle.addEventListener('change', (e) => {
                aiState.isPredicting = e.target.checked;
                if (aiTransitPanel) {
                    aiTransitPanel.style.display = aiState.isPredicting ? 'block' : 'none';
                }
            });
            
            // Periodic inference poll (10 times a sec)
            setInterval(async () => {
                if (!aiState.isPredicting) return;
                
                // Extract the last 500 raw CSI amplitude samples
                let features = [];
                const seqLen = 500;
                for(let i = bufferSize - seqLen; i < bufferSize; i++) {
                    features.push(dspBuffer1[i] || 0.0);
                }
                
                try {
                    const res = await fetch('http://localhost:5000/predict', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ features: features })
                    });
                    if (res.ok) {
                        const data = await res.json();
                        if (data.transit) {
                            aiStatusVal.textContent = 'DETECTED';
                            aiStatusVal.style.color = '#ef4444'; // Red
                            aiSpeedVal.textContent = Math.round(data.speed * 10000) + ' m/s';
                        } else {
                            aiStatusVal.textContent = 'CLEAR';
                            aiStatusVal.style.color = '#10b981'; // Green
                            aiSpeedVal.textContent = '0 m/s';
                        }
                    }
                } catch(e) {
                    // Silently fail if server down
                }
            }, 100);
        }

    // Render Loop
    let lastTime = performance.now();
    let frameCount = 0;
    let lastFpsTime = lastTime;

    function loop(now) {
        const realDt = Math.min((now - lastTime) / 1000, 0.1);
        lastTime = now;

        const dt = realDt * state.timeScale;

        updateKinematics(dt);
        updateDSP();
        analyzeChirp();

        drawSpatial();
        drawDSP();

        frameCount++;
        if (now - lastFpsTime > 1000) {
            els.fpsBadge.textContent = `${frameCount} FPS`;
            frameCount = 0;
            lastFpsTime = now;
        }

        requestAnimationFrame(loop);
    }

    requestAnimationFrame(loop);
});
