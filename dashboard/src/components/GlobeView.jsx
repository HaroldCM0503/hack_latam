import { useEffect, useState, useMemo, useRef } from "react";
import Globe from "react-globe.gl";
import * as THREE from "three";

const PLANES = 6;
const SAT_PER_PLANE = 11;
const INCLINATION = 86.4;
const ALTITUDE = 0.122; // ~780km

// Which two satellites in the constellation play the role of our two HB100 nodes
// (i.e. the two ESP32 sensor boards in the lab). Picked from two adjacent planes
// at the same anomaly so they appear to fly side-by-side - "parallel" orbits,
// matching the in-lab gate geometry.
const NODE_A_PLANE = 0;
const NODE_B_PLANE = 1;
const NODE_SAT_IDX = 0;

const D2R = Math.PI / 180;

// ----- Tiny 3-vector helpers (globe-radius units; Earth = 1) -------------
const vAdd  = (a, b) => [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
const vSub  = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const vMul  = (a, s) => [a[0]*s,   a[1]*s,   a[2]*s];
const vDot  = (a, b) =>  a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const vCross = (a, b) => [
  a[1]*b[2] - a[2]*b[1],
  a[2]*b[0] - a[0]*b[2],
  a[0]*b[1] - a[1]*b[0],
];
const vNorm = (a) => {
  const m = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0]/m, a[1]/m, a[2]/m];
};
const latLngToCart = (lat_deg, lng_deg, r) => {
  const lat = lat_deg * D2R, lng = lng_deg * D2R;
  return [r * Math.cos(lat) * Math.cos(lng),
          r * Math.cos(lat) * Math.sin(lng),
          r * Math.sin(lat)];
};
const cartToLatLng = ([x, y, z]) => {
  const r = Math.hypot(x, y, z);
  return { lat: Math.asin(z / r) / D2R, lng: Math.atan2(y, x) / D2R };
};
// Build the debris orbit as the great circle in the plane spanned by:
//   - M_hat = radial unit vector at the encounter point (midpoint of NODE-A and NODE-B)
//   - v_hat = rock's velocity direction at the encounter, mapped LINEARLY from the
//             lab gate frame:
//               bearing 0   deg -> along the inter-satellite baseline (A -> B)
//               bearing 90  deg -> perpendicular to baseline, tangent to sphere
//               bearing 180 deg -> along -baseline (B -> A)
// This is a literal projection of the doppler-fit velocity vector from the
// lab's gate plane onto the local tangent plane of the inter-satellite gap
// in 3D - no cosine reshape, no hand-tuned inclination mapping.
//
// Returns the closed-loop orbit path AND the parametric basis (Mh, vh, r),
// so a moving debris point can be computed at any theta later:
//   p(theta) = r * (cos(theta) * Mh + sin(theta) * vh)
function debrisOrbitGeometry(fit, nodeA, nodeB, nPoints = 128) {
  if (!nodeA || !nodeB) return null;

  const bearing = typeof fit?.bearing === "number" ? fit.bearing : 90;
  const beta = bearing * D2R;

  const r = 1 + ALTITUDE;                                    // orbital shell radius
  const A = latLngToCart(nodeA.lat, nodeA.lng, r);
  const B = latLngToCart(nodeB.lat, nodeB.lng, r);
  const M = vMul(vNorm(vAdd(A, B)), r);                      // encounter point on the shell
  const Mh = vNorm(M);                                       // radial unit vector at M

  // Baseline direction A->B in 3D, then project onto the tangent plane at M
  // so it is perpendicular to the radial (i.e. lies in the local horizontal).
  let bh = vNorm(vSub(B, A));
  bh = vNorm(vSub(bh, vMul(Mh, vDot(bh, Mh))));

  // Across-track unit vector: perpendicular to baseline, also tangent to sphere.
  // Right-hand rule M_hat x b_hat -> "forward through the gate".
  const th = vNorm(vCross(Mh, bh));

  // Rock velocity DIRECTION at the encounter, linearly from the lab bearing.
  const vh = vNorm(vAdd(vMul(bh, Math.cos(beta)),
                        vMul(th, Math.sin(beta))));

  // Sample the closed-loop orbit path for rendering as a path.
  const pts = [];
  let prevLng = null;
  for (let k = 0; k <= nPoints; k++) {
    const theta = (k / nPoints) * 2 * Math.PI;
    const p = vAdd(vMul(Mh, r * Math.cos(theta)),
                   vMul(vh, r * Math.sin(theta)));
    const { lat, lng } = cartToLatLng(p);
    let lngW = lng;
    if (prevLng !== null) {
      while (lngW - prevLng >  180) lngW -= 360;
      while (lngW - prevLng < -180) lngW += 360;
    }
    prevLng = lngW;
    pts.push([lat, lngW, ALTITUDE]);
  }

  return { points: pts, Mh, vh, r };
}

// Evaluate the debris point at angle theta along an orbit basis.
function debrisPointAt(basis, theta) {
  const { Mh, vh, r } = basis;
  const p = vAdd(vMul(Mh, r * Math.cos(theta)),
                 vMul(vh, r * Math.sin(theta)));
  return cartToLatLng(p);
}

// Visual orbital period of the moving debris dots. Real LEO is ~100 min;
// we accelerate to ~12s so judges see motion in the demo window.
const DEBRIS_VIS_PERIOD_S = 12;

export default function GlobeView({ fits = [] }) {
  const globeEl = useRef();
  const [dimensions, setDimensions] = useState({ width: 400, height: 400 });
  const containerRef = useRef();

  // Resize globe to fit container
  useEffect(() => {
    const observer = new ResizeObserver((entries) => {
      if (entries[0]) {
        setDimensions({
          width: entries[0].contentRect.width,
          height: entries[0].contentRect.height
        });
      }
    });
    if (containerRef.current) {
      observer.observe(containerRef.current);
    }
    return () => observer.disconnect();
  }, []);

  // Pre-calculate the 6 orbital paths
  const pathsData = useMemo(() => {
    const paths = [];
    for (let p = 0; p < PLANES; p++) {
      const raan = p * 31.6; // Node spread
      const points = [];
      let prevLng = null;
      
      for (let u = 0; u <= 360; u += 2) {
        const uRad = (u * Math.PI) / 180;
        const iRad = (INCLINATION * Math.PI) / 180;
        
        const latRad = Math.asin(Math.sin(iRad) * Math.sin(uRad));
        const x = Math.cos(uRad);
        const y = Math.cos(iRad) * Math.sin(uRad);
        const lngOffsetRad = Math.atan2(y, x);
        
        const lat = (latRad * 180) / Math.PI;
        let lng = raan + (lngOffsetRad * 180) / Math.PI;
        
        // Unwrap longitude to prevent lines jumping across the globe
        if (prevLng !== null) {
          while (lng - prevLng > 180) lng -= 360;
          while (lng - prevLng < -180) lng += 360;
        }
        prevLng = lng;
        
        points.push([lat, lng, ALTITUDE]);
      }
      paths.push({ coordinates: points });
    }
    return paths;
  }, []);

  // Animate satellites
  const [satellites, setSatellites] = useState([]);
  const [nodes, setNodes]           = useState({ A: null, B: null });

  useEffect(() => {
    let animationFrameId;
    let time = 0;

    const animate = () => {
      time += 0.2; // Orbit speed multiplier
      const newSats = [];
      let nodeA = null, nodeB = null;

      for (let p = 0; p < PLANES; p++) {
        const raan = p * 31.6;
        for (let s = 0; s < SAT_PER_PLANE; s++) {
          const anomaly = (s * (360 / SAT_PER_PLANE)) + time;

          const uRad = (anomaly * Math.PI) / 180;
          const iRad = (INCLINATION * Math.PI) / 180;

          const latRad = Math.asin(Math.sin(iRad) * Math.sin(uRad));
          const x = Math.cos(uRad);
          const y = Math.cos(iRad) * Math.sin(uRad);
          const lngOffsetRad = Math.atan2(y, x);

          const lat = (latRad * 180) / Math.PI;
          let lng = raan + ((lngOffsetRad * 180) / Math.PI);

          // Normalize to -180 to 180 for standard coordinates
          while(lng > 180) lng -= 360;
          while(lng < -180) lng += 360;

          const isNodeA = (p === NODE_A_PLANE && s === NODE_SAT_IDX);
          const isNodeB = (p === NODE_B_PLANE && s === NODE_SAT_IDX);
          const isHighlight = isNodeA || isNodeB;

          const sat = {
            lat, lng,
            alt: ALTITUDE,
            name: isNodeA ? "HB100 NODE-A"
                : isNodeB ? "HB100 NODE-B"
                          : `IRIDIUM ${p+1}-${s+1}`,
            isHighlight,
          };
          newSats.push(sat);
          if (isNodeA) nodeA = sat;
          if (isNodeB) nodeB = sat;
        }
      }
      setSatellites(newSats);
      setNodes({ A: nodeA, B: nodeB });
      animationFrameId = requestAnimationFrame(animate);
    };
    animate();

    return () => cancelAnimationFrame(animationFrameId);
  }, []);

  // Auto-rotate the globe slowly
  useEffect(() => {
    if (globeEl.current) {
      globeEl.current.controls().autoRotate = true;
      globeEl.current.controls().autoRotateSpeed = 0.5;
      globeEl.current.pointOfView({ altitude: 2.5 });
    }
  }, []);

  // ---- Debris orbits derived from the most recent trajectory fits ----
  // Each orbit is the great circle in the plane spanned by (radial at
  // encounter, lab-velocity-direction projected into the local tangent
  // plane between NODE-A and NODE-B). The bearing maps linearly:
  //   bearing 0   deg  -> orbit plane contains the inter-satellite chord
  //   bearing 90  deg  -> orbit plane perpendicular to the chord
  // The dep list intentionally omits `nodes` so each fit "freezes" its
  // orbit at the satellite positions at the moment it arrived.
  const debrisPaths = useMemo(() => {
    const out = [];
    fits.forEach((fit, i) => {
      const geom = debrisOrbitGeometry(fit, nodes.A, nodes.B);
      if (!geom) return;
      out.push({
        coordinates: geom.points,
        // Basis + freeze-time stay attached so the moving debris dot can
        // be evaluated at any later moment without recomputing geometry.
        Mh:    geom.Mh,
        vh:    geom.vh,
        r:     geom.r,
        t_fit: fit.t_recv ?? performance.now() / 1000,
        speed: fit.speed,
        // Faint red line for the trajectory
        color:  "rgba(239,68,68,0.25)",
        stroke: 0.6,
        isDebris: true,
        rank: i,
      });
    });
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [fits]);

  // Combine the satellite-orbit rings (existing) with debris orbits (new).
  const allPaths = useMemo(() => {
    const sat = pathsData.map((p) => ({ ...p, isDebris: false }));
    return [...sat, ...debrisPaths];
  }, [pathsData, debrisPaths]);

  // ---- Moving debris points (one per frozen debris orbit) ----
  // Each fit's orbit basis (Mh, vh, r, t_fit) is captured at detection time.
  // Here we evaluate the debris position at theta(t) = 2*pi * (now - t_fit) / period.
  // theta = 0 puts the debris at the encounter midpoint of NODE-A and NODE-B
  // captured at fit-time, then it sweeps around the great-circle orbit.
  //
  // The satellite animation loop re-renders ~60 Hz, so this recompute is free.
  const now_s = performance.now() / 1000;
  const debrisPoints = debrisPaths.map((orb) => {
    const dt = now_s - orb.t_fit;
    const theta = (dt / DEBRIS_VIS_PERIOD_S) * 2 * Math.PI;
    const { lat, lng } = debrisPointAt(orb, theta);
    return {
      lat, lng,
      alt:   ALTITUDE,
      color: "#fb7185",
      size:  orb.rank === 0 ? 0.8 : Math.max(0.3, 0.6 - orb.rank * 0.15),
      label: orb.rank === 0
        ? `DEBRIS · ${orb.speed?.toFixed(1) ?? "?"} m/s`
        : `DEBRIS · prev #${orb.rank + 1}`,
      isDebrisPoint: true,
    };
  });

  const allObjects = useMemo(() => {
    return [...satellites, ...debrisPoints];
  }, [satellites, debrisPoints]);

  // ---- Inter-satellite link arc between NODE-A and NODE-B ----
  const islArc =
    nodes.A && nodes.B
      ? [{
          startLat: nodes.A.lat, startLng: nodes.A.lng,
          endLat:   nodes.B.lat, endLng:   nodes.B.lng,
          color: ["rgba(34,211,238,0.85)", "rgba(34,211,238,0.85)"],
        }]
      : [];

  return (
    <section 
      ref={containerRef} 
      className="rounded-lg border border-zinc-800 bg-zinc-900/40 flex flex-col items-center justify-center relative overflow-hidden"
    >
      <div className="absolute top-4 left-4 z-10 w-full flex justify-between pr-8 pointer-events-none">
        <h2 className="text-xs tracking-[0.3em] text-zinc-400">IRIDIUM CONSTELLATION</h2>
        <span className="mono text-[10px] text-zinc-500 text-right">live sim</span>
      </div>
      
      <div className="w-full h-full cursor-move">
        <Globe
          ref={globeEl}
          width={dimensions.width}
          height={dimensions.height}
          globeImageUrl="//unpkg.com/three-globe/example/img/earth-night.jpg"
          bumpImageUrl="//unpkg.com/three-globe/example/img/earth-topology.png"
          backgroundColor="rgba(0,0,0,0)"
          showAtmosphere={true}
          atmosphereColor="#22d3ee"
          atmosphereAltitude={0.16}

          // Render the orbital rings (faint) + debris orbits (bright red)
          pathsData={allPaths}
          pathPoints="coordinates"
          pathPointLat={(p) => p[0]}
          pathPointLng={(p) => p[1]}
          pathPointAlt={(p) => p[2]}
          pathColor={(d) => d.isDebris ? d.color : 'rgba(34, 211, 238, 0.15)'}
          pathStroke={(d) => d.isDebris ? d.stroke : 1}
          pathTransitionDuration={0}

          // Render the satellites and moving debris dots
          objectsData={allObjects}
          objectLat="lat"
          objectLng="lng"
          objectAltitude="alt"
          objectLabel="label"
          objectThreeObject={(d) => {
            if (d.isDebrisPoint) {
              return new THREE.Mesh(
                new THREE.SphereGeometry(d.size, 8, 8),
                new THREE.MeshBasicMaterial({ color: 0xfb7185 }) // red debris
              );
            }
            if (d.isHighlight) {
              const group = new THREE.Group();
              const core = new THREE.Mesh(
                new THREE.SphereGeometry(1.1, 12, 12),
                new THREE.MeshBasicMaterial({ color: 0x4ade80 })
              );
              const halo = new THREE.Mesh(
                new THREE.SphereGeometry(2.0, 12, 12),
                new THREE.MeshBasicMaterial({
                  color: 0x4ade80, transparent: true, opacity: 0.25,
                })
              );
              group.add(core);
              group.add(halo);
              return group;
            }
            return new THREE.Mesh(
              new THREE.SphereGeometry(0.5, 8, 8),
              new THREE.MeshBasicMaterial({ color: '#22d3ee' })
            );
          }}

          // Inter-satellite link between NODE-A and NODE-B
          arcsData={islArc}
          arcStartLat="startLat"
          arcStartLng="startLng"
          arcEndLat="endLat"
          arcEndLng="endLng"
          arcColor="color"
          arcStroke={0.45}
          arcAltitudeAutoScale={0.18}
          arcDashLength={0.35}
          arcDashGap={0.15}
          arcDashAnimateTime={2500}
        />
      </div>

      {/* Legend overlay */}
      <div className="absolute bottom-3 left-4 z-10 mono text-[10px] text-zinc-400 space-y-0.5 pointer-events-none">
        <div><span className="inline-block w-2 h-2 bg-cyan-400 rounded-full mr-2 align-middle" />constellation · ISL</div>
        <div><span className="inline-block w-2 h-2 bg-green-400 rounded-full mr-2 align-middle" />HB100 nodes</div>
        <div><span className="inline-block w-2 h-2 bg-rose-400 rounded-full mr-2 align-middle" />debris orbit</div>
      </div>
    </section>
  );
}
