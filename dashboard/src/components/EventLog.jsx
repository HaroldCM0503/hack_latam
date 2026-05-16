function clock(t) {
  // t is seconds (performance.now()-style). Show as mm:ss.fff since session start.
  if (typeof t !== "number") return "––:––";
  const ts = Math.max(0, t);
  const m = Math.floor(ts / 60);
  const s = ts - m * 60;
  return `${String(m).padStart(2, "0")}:${s.toFixed(2).padStart(5, "0")}`;
}

function rmseColor(rmse) {
  if (rmse == null) return "text-zinc-500";
  if (rmse < 0.25) return "text-emerald-400";
  if (rmse < 0.5)  return "text-amber-300";
  return "text-rose-400";
}

export default function EventLog({ fits }) {
  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900/40 p-4 flex flex-col min-h-0">
      <div className="flex items-baseline justify-between mb-2">
        <h2 className="text-xs tracking-[0.3em] text-zinc-400">EVENT LOG</h2>
        <span className="mono text-[10px] text-zinc-500">{fits.length} entries</span>
      </div>

      <div className="flex-1 overflow-auto pr-1">
        <table className="w-full mono text-xs">
          <thead className="text-[10px] text-zinc-500">
            <tr className="border-b border-zinc-800">
              <th className="text-left  py-1">T</th>
              <th className="text-right py-1">SPEED</th>
              <th className="text-right py-1">BRG</th>
              <th className="text-right py-1">X@GATE</th>
              <th className="text-right py-1">RMSE</th>
              <th className="text-right py-1">N</th>
            </tr>
          </thead>
          <tbody>
            {fits.length === 0 && (
              <tr>
                <td colSpan="6" className="text-zinc-600 py-3 text-center italic">
                  awaiting first transit...
                </td>
              </tr>
            )}
            {fits.map((f, i) => (
              <tr key={i} className="border-b border-zinc-900 hover:bg-zinc-900/60">
                <td className="text-zinc-400 py-1">{clock(f.t_recv)}</td>
                <td className="text-right text-zinc-200">{f.speed?.toFixed(2)}</td>
                <td className="text-right text-zinc-200">{f.bearing?.toFixed(1)}</td>
                <td className="text-right text-zinc-300">{f.gate_cross_x?.toFixed(2)}</td>
                <td className={`text-right ${rmseColor(f.rmse)}`}>{f.rmse?.toFixed(2)}</td>
                <td className="text-right text-zinc-500">{f.n_frames}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
}
