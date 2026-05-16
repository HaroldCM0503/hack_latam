// Top-down 2D view: Tx + 3 Rx positions, gate plane, current trajectory + ghost
// trails of the last few fits. Mirrors fusion/config.py.

const TX = [0.0, 0.0];
const RX = {
  1: [-1.20, 2.40],
  2: [+1.20, 2.40],
  3: [ 0.00, 3.30],
};
const GATE_Y = 2.0;

// World-frame extents (metres) shown in the view.
const X_MIN = -3.0, X_MAX = 3.0;
const Y_MIN = -1.0, Y_MAX = 4.5;

// SVG viewport
const VB_W = 500, VB_H = 460;
const PAD = 24;

function toSvg([x, y]) {
  const sx = PAD + ((x - X_MIN) / (X_MAX - X_MIN)) * (VB_W - 2 * PAD);
  // Flip y so +y is "up" on screen
  const sy = VB_H - PAD - ((y - Y_MIN) / (Y_MAX - Y_MIN)) * (VB_H - 2 * PAD);
  return [sx, sy];
}

export default function TopDownView({ fits }) {
  const latest = fits[0] || null;
  const ghosts = fits.slice(1, 6);

  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900/40 p-4 flex flex-col">
      <div className="flex items-baseline justify-between mb-2">
        <h2 className="text-xs tracking-[0.3em] text-zinc-400">TOP-DOWN · GATE PLANE</h2>
        <span className="mono text-[10px] text-zinc-500">scale 1 m</span>
      </div>

      <svg
        viewBox={`0 0 ${VB_W} ${VB_H}`}
        className="w-full h-full"
        preserveAspectRatio="xMidYMid meet"
      >
        <Grid />
        <GateLine />

        {/* Ghost trails */}
        {ghosts.map((g, i) => (
          <Trajectory key={`g-${i}`} fit={g} alpha={0.18 - i * 0.025} faded />
        ))}

        {/* Bistatic Tx-Rx lines (faint, show the sensing geometry) */}
        {Object.entries(RX).map(([id, p]) => {
          const [tx, ty] = toSvg(TX);
          const [rx, ry] = toSvg(p);
          return (
            <line
              key={`seg-${id}`}
              x1={tx} y1={ty} x2={rx} y2={ry}
              stroke="#22c55e" strokeOpacity="0.18" strokeDasharray="3 5" strokeWidth="1"
            />
          );
        })}

        {/* Tx */}
        {(() => {
          const [sx, sy] = toSvg(TX);
          return (
            <g key="tx">
              <rect x={sx - 8} y={sy - 8} width={16} height={16} fill="#06b6d4" className="glow-cyan" />
              <circle cx={sx} cy={sy} r={22} fill="none" stroke="#06b6d4" strokeOpacity="0.22" />
              <text x={sx + 14} y={sy + 4} fill="#67e8f9" className="mono text-[11px]">
                Tx
              </text>
            </g>
          );
        })()}

        {/* Rx nodes */}
        {Object.entries(RX).map(([id, p]) => {
          const [sx, sy] = toSvg(p);
          return (
            <g key={`rx-${id}`}>
              <circle cx={sx} cy={sy} r={8} fill="#22c55e" className="glow-green" />
              <circle cx={sx} cy={sy} r={18} fill="none" stroke="#22c55e" strokeOpacity="0.25" />
              <text x={sx + 12} y={sy + 4} fill="#86efac" className="mono text-[11px]">
                Rx{id}
              </text>
            </g>
          );
        })}

        {/* Latest trajectory */}
        {latest && <Trajectory fit={latest} alpha={1} />}
      </svg>
    </section>
  );
}

function Grid() {
  const lines = [];
  for (let x = Math.ceil(X_MIN); x <= Math.floor(X_MAX); x++) {
    const [sx1, sy1] = toSvg([x, Y_MIN]);
    const [sx2, sy2] = toSvg([x, Y_MAX]);
    lines.push(
      <line key={`vx-${x}`} x1={sx1} y1={sy1} x2={sx2} y2={sy2} stroke="#27272a" strokeWidth="1" />
    );
  }
  for (let y = Math.ceil(Y_MIN); y <= Math.floor(Y_MAX); y++) {
    const [sx1, sy1] = toSvg([X_MIN, y]);
    const [sx2, sy2] = toSvg([X_MAX, y]);
    lines.push(
      <line key={`hy-${y}`} x1={sx1} y1={sy1} x2={sx2} y2={sy2} stroke="#27272a" strokeWidth="1" />
    );
  }
  return <g>{lines}</g>;
}

function GateLine() {
  const [x1, y1] = toSvg([X_MIN + 0.3, GATE_Y]);
  const [x2, y2] = toSvg([X_MAX - 0.3, GATE_Y]);
  return (
    <g>
      <line x1={x1} y1={y1} x2={x2} y2={y2} stroke="#52525b" strokeDasharray="4 4" strokeWidth="1" />
      <text x={x2 - 18} y={y1 - 6} className="mono text-[10px]" fill="#71717a">
        gate y={GATE_Y.toFixed(2)} m
      </text>
    </g>
  );
}

function Trajectory({ fit, alpha = 1, faded = false }) {
  if (!fit?.p0 || !fit?.v) return null;
  const t0 = -0.05;
  const t1 = 0.30;
  const sample = (t) => [fit.p0[0] + fit.v[0] * t, fit.p0[1] + fit.v[1] * t];

  const pts = [];
  for (let i = 0; i <= 60; i++) {
    const t = t0 + (t1 - t0) * (i / 60);
    pts.push(sample(t));
  }
  const path = pts.map(toSvg).map(([x, y], i) => `${i ? "L" : "M"}${x},${y}`).join(" ");

  // Arrowhead at the future end
  const headBase = toSvg(sample(t1 - 0.01));
  const headTip  = toSvg(sample(t1));

  // Gate crossing marker
  let cross = null;
  if (typeof fit.gate_cross_x === "number") {
    const [cx, cy] = toSvg([fit.gate_cross_x, GATE_Y]);
    cross = (
      <g>
        <circle cx={cx} cy={cy} r={6} fill="#4ade80" opacity={alpha} />
        <circle cx={cx} cy={cy} r={11} fill="none" stroke="#4ade80" strokeOpacity={alpha * 0.6} />
      </g>
    );
  }

  const stroke = faded ? "#a1a1aa" : "#22c55e";
  return (
    <g opacity={alpha} className={faded ? "" : "glow-green"}>
      <path d={path} stroke={stroke} strokeWidth={faded ? 1.2 : 2.2} fill="none" />
      {!faded && (
        <line
          x1={headBase[0]} y1={headBase[1]}
          x2={headTip[0]}  y2={headTip[1]}
          stroke={stroke} strokeWidth={3.2}
          markerEnd="url(#arrow)"
        />
      )}
      {cross}
      <defs>
        <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M0,0 L10,5 L0,10 z" fill={stroke} />
        </marker>
      </defs>
    </g>
  );
}
