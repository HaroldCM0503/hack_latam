export default function StatusBar({ status, nFits, nFrames }) {
  const dotColor = {
    connected: "bg-emerald-400",
    demo:      "bg-amber-400",
    connecting:"bg-zinc-500",
  }[status] || "bg-zinc-500";

  const label = {
    connected:  "LINK · LIVE",
    demo:       "LINK · DEMO MODE",
    connecting: "LINK · CONNECTING...",
  }[status] || status;

  return (
    <header className="flex items-center justify-between px-6 py-3 border-b border-zinc-800 bg-zinc-900/60 scanlines">
      <div className="flex items-center gap-4">
        <span className="text-cyan-300 font-bold tracking-[0.25em] text-sm">
          ORBITAL DEBRIS &middot; LEO MONITOR
        </span>
        <span className="text-zinc-500 mono text-xs">v0.1 / hack-latam</span>
      </div>
      <div className="flex items-center gap-6 text-xs mono">
        <span className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${dotColor} animate-pulse`} />
          <span className="text-zinc-300">{label}</span>
        </span>
        <span className="text-zinc-500">FITS: <span className="text-zinc-200">{nFits}</span></span>
        <span className="text-zinc-500">FRAMES: <span className="text-zinc-200">{nFrames}</span></span>
      </div>
    </header>
  );
}
