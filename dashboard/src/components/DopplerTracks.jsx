import { useMemo } from "react";
import {
  LineChart, Line, CartesianGrid, XAxis, YAxis, Tooltip, Legend, ResponsiveContainer
} from "recharts";

const NODE_COLORS = {
  1: "#22d3ee",
  2: "#f59e0b",
};

export default function DopplerTracks({ frames }) {
  // Bucket frames by node, project onto a relative-time axis (last 4 s).
  const { series, tMin, tMax } = useMemo(() => {
    if (frames.length === 0) {
      return { series: [], tMin: 0, tMax: 4 };
    }
    const tMax = Math.max(...frames.map((f) => f.t_recv));
    const tMin = tMax - 4;
    // Build a sparse dataset: each point is { t, v_node1, v_node2 } where
    // either may be missing - recharts handles missing values fine.
    const points = frames
      .filter((f) => f.t_recv >= tMin)
      .map((f) => ({
        t: +(f.t_recv - tMax).toFixed(3),    // seconds, negative = in the past
        [`v${f.node}`]: f.v,
      }));
    return { series: points, tMin: -4, tMax: 0 };
  }, [frames]);

  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900/40 p-4 flex flex-col">
      <div className="flex items-baseline justify-between mb-2">
        <h2 className="text-xs tracking-[0.3em] text-zinc-400">DOPPLER · |v_radial|</h2>
        <span className="mono text-[10px] text-zinc-500">window 4 s</span>
      </div>

      <div className="flex-1 min-h-[200px]">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={series} margin={{ top: 8, right: 12, left: 4, bottom: 8 }}>
            <CartesianGrid stroke="#27272a" />
            <XAxis
              type="number"
              dataKey="t"
              domain={[tMin, tMax]}
              tick={{ fill: "#71717a", fontSize: 10, fontFamily: "ui-monospace" }}
              stroke="#3f3f46"
              label={{ value: "t [s]", position: "insideBottom", offset: -2, fill: "#52525b", fontSize: 10 }}
            />
            <YAxis
              domain={[0, "dataMax + 2"]}
              tick={{ fill: "#71717a", fontSize: 10, fontFamily: "ui-monospace" }}
              stroke="#3f3f46"
              width={36}
            />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              labelStyle={{ color: "#a1a1aa" }}
              formatter={(v) => (typeof v === "number" ? `${v.toFixed(2)} m/s` : v)}
            />
            <Legend wrapperStyle={{ fontSize: 11, color: "#a1a1aa" }} />
            <Line type="monotone" dataKey="v1" name="HB100·1" stroke={NODE_COLORS[1]} strokeWidth={1.6} dot={false} isAnimationActive={false} connectNulls />
            <Line type="monotone" dataKey="v2" name="HB100·2" stroke={NODE_COLORS[2]} strokeWidth={1.6} dot={false} isAnimationActive={false} connectNulls />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </section>
  );
}
