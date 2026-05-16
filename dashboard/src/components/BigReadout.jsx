import { useEffect, useState } from "react";

function fmt(n, digits = 1, sign = false) {
  if (n === undefined || n === null || Number.isNaN(n)) return "––";
  const s = n.toFixed(digits);
  return sign && n >= 0 ? `+${s}` : s;
}

export default function BigReadout({ fit }) {
  // Animate a "fresh" highlight when a new fit arrives.
  const [pulseKey, setPulseKey] = useState(0);
  useEffect(() => {
    if (fit) setPulseKey((k) => k + 1);
  }, [fit?.t_recv]);

  const stale = !fit || (performance.now() / 1000 - (fit.t_recv ?? 0) > 8);

  return (
    <section
      key={pulseKey}
      className={`rounded-lg border ${stale ? "border-zinc-800 bg-zinc-900/40" : "border-green-500/40 bg-green-950/20 glow-green"} p-6`}
    >
      <div className="flex items-baseline justify-between mb-4">
        <h2 className="text-xs tracking-[0.3em] text-zinc-400">
          {stale ? "STANDBY" : "DEBRIS · DETECTED"}
        </h2>
        <span className="mono text-xs text-zinc-500">
          {fit ? `RMSE ${fmt(fit.rmse, 2)} m/s` : "––"}
        </span>
      </div>

      <div className="grid grid-cols-3 gap-6">
        <Field label="SPEED" value={fmt(fit?.speed, 2)} unit="m/s" big accent={!stale} />
        <Field label="BEARING" value={fmt(fit?.bearing, 1)} unit="deg" big accent={!stale} />
        <Field label="GATE-CROSS X" value={fmt(fit?.gate_cross_x, 2, true)} unit="m" big accent={!stale} />
      </div>

      <div className="mt-6 pt-4 border-t border-zinc-800 grid grid-cols-3 gap-6 text-xs mono text-zinc-400">
        <span>p0  &nbsp; ({fmt(fit?.p0?.[0], 2, true)}, {fmt(fit?.p0?.[1], 2, true)}) m</span>
        <span>v   &nbsp; ({fmt(fit?.v?.[0], 2, true)}, {fmt(fit?.v?.[1], 2, true)}) m/s</span>
        <span>N   &nbsp; {fit?.n_frames ?? 0} frames</span>
      </div>
    </section>
  );
}

function Field({ label, value, unit, big = false, accent = false }) {
  return (
    <div>
      <div className="text-[10px] tracking-[0.25em] text-zinc-500">{label}</div>
      <div className={`mono leading-none ${big ? "text-5xl" : "text-2xl"} ${accent ? "text-green-400" : "text-zinc-200"} mt-2`}>
        {value}
      </div>
      <div className="text-xs text-zinc-500 mt-1">{unit}</div>
    </div>
  );
}
