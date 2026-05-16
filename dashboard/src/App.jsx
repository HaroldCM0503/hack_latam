// Top-level dashboard layout.
//
//  +------------------------------------------------------+
//  |  StatusBar                                           |
//  +------------------------------------------------------+
//  |  BigReadout (full width)                             |
//  +-----------------------------+------------------------+
//  |                             |                        |
//  |   TopDownView               |   DopplerTracks        |
//  |   (2D gate + trajectory)    |   (live |v_radial|)    |
//  |                             |                        |
//  +-----------------------------+------------------------+
//  |  EventLog                                            |
//  +------------------------------------------------------+
//
// Runs standalone with mock events when the WebSocket backend is unavailable.

import { useEvents } from "./hooks/useEvents";
import StatusBar      from "./components/StatusBar";
import BigReadout     from "./components/BigReadout";
import TopDownView    from "./components/TopDownView";
import EventLog       from "./components/EventLog";
import GlobeView      from "./components/GlobeView";

export default function App() {
  const { frames, fits, status } = useEvents();
  const latestFit = fits[0] || null;

  return (
    <div className="h-screen w-screen bg-zinc-950 text-zinc-100 scanlines flex justify-center overflow-hidden">
      <div className="w-full max-w-7xl h-full flex flex-col pt-4 pb-6 px-4 sm:px-8">
        <div className="flex-1 rounded-2xl border border-zinc-800 shadow-[0_0_50px_-12px_rgba(34,211,238,0.1)] bg-zinc-950/80 backdrop-blur-lg flex flex-col overflow-hidden">
          <StatusBar status={status} nFits={fits.length} nFrames={frames.length} />

          <main className="flex-1 grid gap-4 p-4 md:p-6 min-h-0"
                style={{ gridTemplateRows: "auto 1fr auto", gridTemplateColumns: "1fr" }}>
            <BigReadout fit={latestFit} />

            <div className="grid gap-4 min-h-0" style={{ gridTemplateColumns: "1fr 1fr" }}>
              <TopDownView fits={fits} />
              <GlobeView fits={fits} />
            </div>

            <div className="min-h-[160px] max-h-[260px]">
              <EventLog fits={fits} />
            </div>
          </main>
        </div>
      </div>
    </div>
  );
}
