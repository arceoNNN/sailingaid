
let watchId = null;
let polars = null; // loaded from polars.json

const BOAT_LENGTH_M = 35 * 0.3048; // 35 ft in meters

function toRad(deg) { return deg * Math.PI / 180; }
function toDeg(rad) { return rad * 180 / Math.PI; }
function norm360(deg) { return ((deg % 360) + 360) % 360; }

function bearingAndDistance(lat1, lon1, lat2, lon2) {
  const R = 6371000; // meters
  const φ1 = toRad(lat1);
  const φ2 = toRad(lat2);
  const Δφ = toRad(lat2 - lat1);
  const Δλ = toRad(lon2 - lon1);

  const a = Math.sin(Δφ / 2) ** 2 +
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d = R * c;

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) -
            Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  let brg = toDeg(Math.atan2(y, x)); // -180..180
  if (brg < 0) brg += 360;

  return { bearing: brg, distance_m: d };
}

function normSigned180(deg) {
  // normalize to (-180, 180]
  let d = ((deg + 180) % 360 + 360) % 360 - 180;
  if (d === -180) d = 180;
  return d;
}

function solveTWAFromAWA(awa_from_deg, tws, boatSpeed) {
  // Inputs:
  // - awa_from_deg: apparent wind angle (wind FROM) relative to boat heading, signed (+stbd / -port)
  // - tws: true wind speed (kt)
  // - boatSpeed: boat speed (kt). We use SOG as approximation.
  // Output:
  // - twa_from_deg: true wind angle (wind FROM) relative to boat heading, signed (+stbd / -port)
  //
  // Method: search over possible TWA_to angles to match the AWA_to direction of (W_true_to - V_boat).
  if (!isFinite(awa_from_deg) || !isFinite(tws) || !isFinite(boatSpeed) || tws <= 0) return null;

  // Convert FROM to TO in boat coordinates
  const awa_to = normSigned180(awa_from_deg + 180);
  const target = toRad(awa_to);

  const bs = Math.max(0, boatSpeed);
  const ws = tws;

  const sign = (awa_to === 0) ? 1 : Math.sign(awa_to);

  function awa_to_from_twa_to(phi) {
    // phi = TWA_to (radians, signed)
    // Apparent wind TO vector = True wind TO vector - boat velocity vector (forward)
    const x = ws * Math.cos(phi) - bs;
    const y = ws * Math.sin(phi);
    return Math.atan2(y, x); // -pi..pi
  }

  // Coarse search (1 deg)
  let bestPhi = null;
  let bestErr = 1e9;
  for (let deg = 1; deg <= 179; deg += 1) {
    const phi = toRad(sign * deg);
    const est = awa_to_from_twa_to(phi);
    let err = est - target;
    err = (err + Math.PI) % (2 * Math.PI) - Math.PI;
    const aerr = Math.abs(err);
    if (aerr < bestErr) {
      bestErr = aerr;
      bestPhi = phi;
    }
  }
  if (bestPhi == null) return null;

  // Refine around best (0.1 deg)
  const bestDeg = Math.abs(toDeg(bestPhi));
  let bestPhi2 = bestPhi;
  let bestErr2 = bestErr;
  for (let d = Math.max(0.1, bestDeg - 2); d <= Math.min(179, bestDeg + 2); d += 0.1) {
    const phi = toRad(sign * d);
    const est = awa_to_from_twa_to(phi);
    let err = est - target;
    err = (err + Math.PI) % (2 * Math.PI) - Math.PI;
    const aerr = Math.abs(err);
    if (aerr < bestErr2) {
      bestErr2 = aerr;
      bestPhi2 = phi;
    }
  }

  // Convert TWA_to to TWA_from
  const twa_to_deg = toDeg(bestPhi2);
  const twa_from_deg = normSigned180(twa_to_deg - 180);
  return twa_from_deg;
}

function smallestAngleDiff(a, b) {
  return (a - b + 540) % 360 - 180; // -180..180
}

// 1D linear interpolation helper
function interp1(x, xs, ys) {
  if (!Array.isArray(xs) || !Array.isArray(ys) || xs.length === 0 || xs.length !== ys.length) return null;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
  for (let i = 0; i < xs.length - 1; i++) {
    const x0 = xs[i], x1 = xs[i + 1];
    if (x >= x0 && x <= x1) {
      const y0 = ys[i], y1 = ys[i + 1];
      const t = (x - x0) / (x1 - x0);
      return y0 + t * (y1 - y0);
    }
  }
  return null;
}

function getTargetVMG(tws, mode) {
  if (!polars) return null;
  const xs = polars.tws;
  if (mode === 'upwind') return interp1(tws, xs, polars.beatVMG);
  if (mode === 'downwind') return interp1(tws, xs, polars.runVMG);
  return null;
}

function getGybeAngle(tws) {
  if (!polars || !Array.isArray(polars.gybeAngle)) return null;
  return interp1(tws, polars.tws, polars.gybeAngle);
}

// Target boat speed at given TWS and TWA using full polar table
function getTargetBoatSpeed(tws, twa) {
  if (!polars || !polars.angles || !polars.speeds) return null;

  const twsList = polars.tws;
  const angles = polars.angles;
  const speeds = polars.speeds;

  if (!Array.isArray(twsList) || twsList.length === 0 || !Array.isArray(angles) || angles.length === 0) return null;

  let ang = Math.abs(twa);
  if (ang > 180) ang = 360 - ang;

  // Clamp to polar table range
  const minA = angles[0];
  const maxA = angles[angles.length - 1];
  if (ang <= minA) ang = minA;
  if (ang >= maxA) ang = maxA;

  // Find bracket angles
  let a0 = angles[0], a1 = angles[angles.length - 1];
  for (let i = 0; i < angles.length - 1; i++) {
    if (ang >= angles[i] && ang <= angles[i + 1]) { a0 = angles[i]; a1 = angles[i + 1]; break; }
  }

  const arrA0 = speeds[String(a0)];
  const arrA1 = speeds[String(a1)];
  if (!arrA0 || !arrA1) return null;

  const spA0 = interp1(tws, twsList, arrA0);
  const spA1 = interp1(tws, twsList, arrA1);
  if (spA0 == null || spA1 == null) return null;

  if (a1 === a0) return spA0;

  const t = (ang - a0) / (a1 - a0);
  return spA0 + t * (spA1 - spA0);
}

// All-purpose: choose heading & speed that maximizes VMG-to-mark using the polar table
function computeOptimalToMarkGeneral(bearingToMark, windDir, tws) {
  if (!polars || !polars.angles || !polars.speeds) return null;

  let best = null;

  // Scan TWAs 0..180; getTargetBoatSpeed clamps to available polar range (e.g. 52..150)
  for (let twa = 0; twa <= 180; twa += 1) {
    const sp = getTargetBoatSpeed(tws, twa);
    if (sp == null) continue;

    // Two symmetric headings around the wind axis
    for (const side of [1, -1]) {
      const heading = norm360(windDir + side * twa);
      const diff = Math.abs(smallestAngleDiff(heading, bearingToMark));
      const vmgToMark = sp * Math.cos(toRad(diff)); // can be negative if pointing away

      if (!best || vmgToMark > best.vmgToMark) {
        best = { heading, twa, speed: sp, vmgToMark, side };
      }
    }
  }

  if (!best) return null;

  // Label for side relative to wind: +twa => starboard side, -twa => port side
  const sideLabel = (best.side === 1) ? 'starboard' : 'port';

  // Context label (optional)
  const rel = Math.abs(smallestAngleDiff(bearingToMark, windDir)); // 0..180
  let legLabel = 'reaching';
  if (rel <= 60) legLabel = 'upwind-ish';
  else if (rel >= 120) legLabel = 'downwind-ish';

  return { ...best, sideLabel, legLabel };
}

function startTracking() {
  if (!navigator.geolocation) { document.getElementById('gpsStatus').textContent = 'GPS not supported'; return; }
  document.getElementById('gpsStatus').textContent = 'Requesting GPS...';
  watchId = navigator.geolocation.watchPosition(onPosition, onError, { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 });
}

function onPosition(pos) {
  document.getElementById('gpsStatus').textContent = 'GPS OK';
  const lat = pos.coords.latitude;
  const lon = pos.coords.longitude;
  const sog = pos.coords.speed ? pos.coords.speed * 1.94384 : null; // m/s -> kn
  const cmg = pos.coords.heading; // 0..360 or null
  updateLive(lat, lon, sog, cmg);
}

function onError(err) {
  document.getElementById('gpsStatus').textContent = 'GPS error: ' + err.message;
}

function updateLive(lat, lon, sog, cmg) {
  document.getElementById('boatInfo').textContent = `Pos: ${lat.toFixed(5)}, ${lon.toFixed(5)}`;
  document.getElementById('headingInfo').textContent = `Heading (CMG): ${cmg != null ? cmg.toFixed(0) + '°' : '–'}`;
  document.getElementById('speedInfo').textContent = `Speed over ground: ${sog != null ? sog.toFixed(2) + ' kt' : '–'}`;

  const useGate = document.getElementById('useGate').checked;

  const singleLat = parseFloat(document.getElementById('markLat').value);
  const singleLon = parseFloat(document.getElementById('markLon').value);

  const gateLeftLat = parseFloat(document.getElementById('gateLeftLat').value);
  const gateLeftLon = parseFloat(document.getElementById('gateLeftLon').value);
  const gateRightLat = parseFloat(document.getElementById('gateRightLat').value);
  const gateRightLon = parseFloat(document.getElementById('gateRightLon').value);

  let activeLat = null, activeLon = null, activeLabel = '';
  let brgMark = null, dMark = null;

  if (useGate) {
    const candidates = [];
    if (isFinite(gateLeftLat) && isFinite(gateLeftLon)) {
      const bd = bearingAndDistance(lat, lon, gateLeftLat, gateLeftLon);
      candidates.push({ label: 'gate L', lat: gateLeftLat, lon: gateLeftLon, bearing: bd.bearing, dist: bd.distance_m });
    }
    if (isFinite(gateRightLat) && isFinite(gateRightLon)) {
      const bd = bearingAndDistance(lat, lon, gateRightLat, gateRightLon);
      candidates.push({ label: 'gate R', lat: gateRightLat, lon: gateRightLon, bearing: bd.bearing, dist: bd.distance_m });
    }
    if (candidates.length > 0) {
      candidates.sort((a, b) => a.dist - b.dist);
      const best = candidates[0];
      activeLat = best.lat; activeLon = best.lon; activeLabel = best.label;
      brgMark = best.bearing; dMark = best.dist;
    }
  } else {
    if (isFinite(singleLat) && isFinite(singleLon)) {
      const bd = bearingAndDistance(lat, lon, singleLat, singleLon);
      activeLat = singleLat; activeLon = singleLon;
      brgMark = bd.bearing; dMark = bd.distance_m;
    }
  }

  const markInfo = document.getElementById('markInfo');
  const vmgMarkInfo = document.getElementById('vmgMarkInfo');
  const optimalInfo = document.getElementById('optimalInfo');
  const laylineInfo = document.getElementById('laylineInfo');

  let vmgMark = null;

  if (brgMark != null && dMark != null) {
    const labelSuffix = activeLabel ? ` (${activeLabel})` : '';
    markInfo.textContent = `To mark${labelSuffix}: brg ${brgMark.toFixed(0)}°, dist ${(dMark / 1852).toFixed(2)} NM`;
    if (sog != null && cmg != null) {
      const diff = smallestAngleDiff(cmg, brgMark);
      vmgMark = sog * Math.cos(toRad(diff));
    }
  } else {
    markInfo.textContent = 'To mark: set mark or gate coordinates above.';
  }

  vmgMarkInfo.textContent = (vmgMark != null) ? `VMG to mark: ${vmgMark.toFixed(2)} kt` : 'VMG to mark: –';

  const awaVal = parseFloat(document.getElementById('awa').value);
const twsVal = parseFloat(document.getElementById('tws').value);

  // Optimal heading/speed to the mark: all-purpose (handles reaching too)
  if (isFinite(awaVal) && isFinite(twsVal) && brgMark != null) {
    const twaFrom = solveTWAFromAWA(awaVal, twsVal, (sog != null ? sog : 0));
    const windDir = (twaFrom != null && cmg != null) ? norm360(cmg + twaFrom) : null;
    const opt = computeOptimalToMarkGeneral(brgMark, windDir, twsVal);
    const gateSuffix = activeLabel ? ` (${activeLabel})` : '';
    if (opt) {
      optimalInfo.textContent =
        `Optimal to mark${gateSuffix}: heading ${opt.heading.toFixed(0)}° (${opt.sideLabel} side, TWA ${opt.twa.toFixed(0)}°, ${opt.legLabel}), target speed ${opt.speed.toFixed(2)} kt`;
    } else {
      optimalInfo.textContent = `Optimal to mark${gateSuffix}: – (need polars)`;
    }
  } else {
    optimalInfo.textContent = 'Optimal to mark: –';
  }

  // VMG vs wind + speed vs polar + laylines (same as earlier version)
  const vmgInfo = document.getElementById('vmgInfo');
  const speedPolarInfo = document.getElementById('speedPolarInfo');
  const twaInfo = document.getElementById('twaInfo');

  vmgInfo.classList.remove('perf-good', 'perf-ok', 'perf-bad');
  speedPolarInfo.classList.remove('perf-good', 'perf-ok', 'perf-bad');
  laylineInfo.textContent = 'Layline: –';

  if (!isFinite(awaVal) || !isFinite(twsVal) || sog == null || cmg == null) {
    vmgInfo.textContent = 'VMG vs wind: waiting for wind, TWS and GPS...';
    speedPolarInfo.textContent = 'Speed vs polar: waiting for wind, TWS and GPS...';
    twaInfo.textContent = 'TWA: –';
    return;
  }

  const twaFrom = solveTWAFromAWA(awaVal, twsVal, (sog != null ? sog : 0));
    const windDir = (twaFrom != null && cmg != null) ? norm360(cmg + twaFrom) : null;
  if (windDir == null) {
    vmgInfo.textContent = 'VMG vs wind: need AWA + TWS + GPS heading/speed...';
    speedPolarInfo.textContent = 'Speed vs polar: need AWA + TWS + GPS heading/speed...';
    twaInfo.textContent = 'AWA/TWA: –';
    laylineInfo.textContent = 'Layline: –';
    return;
  }

  // Current TWA (0..180)
  let rawDiff = smallestAngleDiff(cmg, windDir);
  let twa = Math.abs(rawDiff);
  if (twa > 180) twa = 360 - twa;
  twaInfo.textContent = `AWA: ${awaVal.toFixed(0)}° | TWA (est): ${rawDiff.toFixed(0)}°`;
// Upwind vs downwind for VMG vs wind display
  const absUp = Math.abs(rawDiff);
  let mode, vmg, targetVMG;

  if (absUp <= 90) {
    mode = 'upwind';
    vmg = sog * Math.cos(toRad(absUp));
    targetVMG = getTargetVMG(twsVal, 'upwind');
  } else {
    const downDir = norm360(windDir + 180);
    const diffToDown = smallestAngleDiff(cmg, downDir);
    const absDown = Math.abs(diffToDown);
    mode = 'downwind';
    vmg = sog * Math.cos(toRad(absDown));
    targetVMG = getTargetVMG(twsVal, 'downwind');
  }

  if (targetVMG && targetVMG > 0) {
    const perfVMG = (vmg / targetVMG) * 100;
    vmgInfo.textContent = `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (target ${targetVMG.toFixed(2)} kt, ${perfVMG.toFixed(0)}%)`;
    if (perfVMG >= 95) vmgInfo.classList.add('perf-good');
    else if (perfVMG >= 90) vmgInfo.classList.add('perf-ok');
    else vmgInfo.classList.add('perf-bad');
  } else {
    vmgInfo.textContent = `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (no polar VMG data for this TWS yet)`;
  }

  const targetSpeed = getTargetBoatSpeed(twsVal, twa);
  if (targetSpeed && targetSpeed > 0) {
    const perfSpd = (sog / targetSpeed) * 100;
    speedPolarInfo.textContent = `Speed vs polar: ${sog.toFixed(2)} kt (target ${targetSpeed.toFixed(2)} kt, ${perfSpd.toFixed(0)}%)`;
    if (perfSpd >= 95) speedPolarInfo.classList.add('perf-good');
    else if (perfSpd >= 90) speedPolarInfo.classList.add('perf-ok');
    else speedPolarInfo.classList.add('perf-bad');
  } else {
    speedPolarInfo.textContent = `Speed vs polar: ${sog.toFixed(2)} kt (no polar speed data for this TWS/TWA yet)`;
  }

  // Laylines (same as gate+laylines build): upwind & downwind with inside/overstood
  if (activeLat != null && activeLon != null && brgMark != null && dMark != null && polars && Array.isArray(polars.tws)) {
    const R = 6371000;
    const latRad = toRad(lat);
    const dLat = toRad(activeLat - lat);
    const dLon = toRad(activeLon - lon);
    const y_m = dLat * R; // north
    const x_m = dLon * R * Math.cos(latRad); // east

    if (mode === 'upwind' && Array.isArray(polars.beatAngle)) {
      const beatAng = interp1(twsVal, polars.tws, polars.beatAngle);
      if (beatAng != null) {
        const diffMarkWind = Math.abs(smallestAngleDiff(brgMark, windDir));
        if (diffMarkWind <= 100) {
          const onStarboard = rawDiff > 0;
          let brgLLDown = onStarboard ? norm360(windDir + beatAng + 180) : norm360(windDir - beatAng + 180);

          const brgLLRad = toRad(brgLLDown);
          const ux = Math.sin(brgLLRad);
          const uy = Math.cos(brgLLRad);

          const crossMark = x_m * uy - y_m * ux;
          const dist_m = Math.abs(crossMark);
          const bl = dist_m / BOAT_LENGTH_M;

          const sog_mps = sog * 0.514444;
          const diffLay = Math.abs(smallestAngleDiff(cmg, brgLLDown));
          const lateralSpeed = sog_mps * Math.abs(Math.sin(toRad(diffLay)));
          let ttlText = '–';
          if (lateralSpeed > 0.01) {
            const ttl_s = dist_m / lateralSpeed;
            ttlText = (ttl_s < 90) ? `${ttl_s.toFixed(0)} s` : `${Math.floor(ttl_s/60)}m ${Math.round(ttl_s%60)}s`;
          }

          const boatSign = Math.sign(-crossMark);
          const windAxisRad = toRad(windDir);
          const insideSign = Math.sign(Math.sin(windAxisRad) * uy - Math.cos(windAxisRad) * ux);

          let sideText;
          if (dist_m < 0.5 * BOAT_LENGTH_M) sideText = 'on layline';
          else if (boatSign === insideSign || insideSign === 0) sideText = `inside ${bl.toFixed(1)} BL`;
          else sideText = `overstood by ${bl.toFixed(1)} BL`;

          const tackStr = onStarboard ? 'starboard' : 'port';
          const gateSuffix = activeLabel ? ` (${activeLabel})` : '';
          const ttlPart = sideText.startsWith('overstood') ? '' : `, ${ttlText}`;
          laylineInfo.textContent = `To ${tackStr} upwind layline${gateSuffix}: ${sideText}${ttlPart}`;
        }
      }
    } else if (mode === 'downwind') {
      const windDown = norm360(windDir + 180);
      // Use gybe angle from ORC certificate when available (more correct than scanning)
      let runAng = null;
      const gy = getGybeAngle(twsVal);
      if (gy != null && isFinite(gy)) {
        runAng = gy;
      } else {
        // Fallback: scan polar angles to maximize downwind VMG
        let bestAng = null, bestV = -Infinity;
        for (let i = 0; i < polars.angles.length; i++) {
          const ang = polars.angles[i];
          const sp = getTargetBoatSpeed(twsVal, ang);
          if (sp == null) continue;
          const vmgDown = sp * Math.cos(toRad(180 - ang));
          if (vmgDown > bestV) { bestV = vmgDown; bestAng = ang; }
        }
        runAng = bestAng;
      }
      if (runAng != null) {
        const diffMarkDown = Math.abs(smallestAngleDiff(brgMark, windDown));
        if (diffMarkDown <= 100) {
          const beta = 180 - runAng;
          const diffDownNow = smallestAngleDiff(cmg, windDown);
          const onStarboardJibe = diffDownNow > 0;

          const brgLLDown = onStarboardJibe ? norm360(windDown - beta + 180) : norm360(windDown + beta + 180);

          const brgLLRad = toRad(brgLLDown);
          const ux = Math.sin(brgLLRad);
          const uy = Math.cos(brgLLRad);

          const crossMark = x_m * uy - y_m * ux;
          const dist_m = Math.abs(crossMark);
          const bl = dist_m / BOAT_LENGTH_M;

          const sog_mps = sog * 0.514444;
          const diffLay = Math.abs(smallestAngleDiff(cmg, brgLLDown));
          const lateralSpeed = sog_mps * Math.abs(Math.sin(toRad(diffLay)));
          let ttlText = '–';
          if (lateralSpeed > 0.01) {
            const ttl_s = dist_m / lateralSpeed;
            ttlText = (ttl_s < 90) ? `${ttl_s.toFixed(0)} s` : `${Math.floor(ttl_s/60)}m ${Math.round(ttl_s%60)}s`;
          }

          const boatSign = Math.sign(-crossMark);
          const windAxisRad = toRad(windDown);
          const insideSign = Math.sign(Math.sin(windAxisRad) * uy - Math.cos(windAxisRad) * ux);

          let sideText;
          if (dist_m < 0.5 * BOAT_LENGTH_M) sideText = 'on layline';
          else if (boatSign === insideSign || insideSign === 0) sideText = `inside ${bl.toFixed(1)} BL`;
          else sideText = `overstood by ${bl.toFixed(1)} BL`;

          const jibeStr = onStarboardJibe ? 'starboard' : 'port';
          const gateSuffix = activeLabel ? ` (${activeLabel})` : '';
          const ttlPart = sideText.startsWith('overstood') ? '' : `, ${ttlText}`;
          laylineInfo.textContent = `To ${jibeStr} downwind layline${gateSuffix}: ${sideText}${ttlPart}`;
        }
      }
    }
  }
}

function loadPolars() {
  fetch('polars.json').then(r => r.json()).then(data => { polars = data; console.log('Polars loaded:', polars); })
    .catch(err => console.error('Failed to load polars.json', err));
}

// Parse DMS / DM / decimal coordinate string into decimal degrees
function parseCoordString(str, isLat) {
  if (!str) return null;
  let s = str.trim().toUpperCase();
  let sign = 1;
  if (s.includes('S') || s.includes('W')) sign = -1;
  s = s.replace(/[NSEW]/g, ' ');
  s = s.replace(/[°º]/g, ' ').replace(/[′']/g, ' ').replace(/[″"]/g, ' ');
  s = s.replace(/,/g, '.');
  const parts = s.split(/\s+/).filter(p => p.length > 0);

  let deg, min = 0, sec = 0;
  if (parts.length === 1) { deg = parseFloat(parts[0]); if (!isFinite(deg)) return null; }
  else if (parts.length === 2) { deg = parseFloat(parts[0]); min = parseFloat(parts[1]); if (!isFinite(deg) || !isFinite(min)) return null; }
  else { deg = parseFloat(parts[0]); min = parseFloat(parts[1]); sec = parseFloat(parts[2]); if (!isFinite(deg) || !isFinite(min) || !isFinite(sec)) return null; }

  let dec = Math.abs(deg) + (Math.abs(min)/60) + (Math.abs(sec)/3600);
  dec *= (deg < 0 ? -1 : 1);
  dec *= sign;

  if (isLat && (dec < -90 || dec > 90)) return null;
  if (!isLat && (dec < -180 || dec > 180)) return null;
  return dec;
}

function convertCoords() {
  const latStr = document.getElementById('convLat').value;
  const lonStr = document.getElementById('convLon').value;
  const statusEl = document.getElementById('convStatus');
  statusEl.classList.remove('status-ok', 'status-error');

  const lat = parseCoordString(latStr, true);
  const lon = parseCoordString(lonStr, false);
  if (lat == null || lon == null) {
    statusEl.textContent = 'Could not parse one or both coordinates. Check format.';
    statusEl.classList.add('status-error');
    return;
  }
  document.getElementById('markLat').value = lat.toFixed(5);
  document.getElementById('markLon').value = lon.toFixed(5);
  statusEl.textContent = `Converted: lat ${lat.toFixed(5)}, lon ${lon.toFixed(5)} (decimal degrees).`;
  statusEl.classList.add('status-ok');
}

// +/- helpers for wind direction and TWS
function adjustAWA(delta) {
  const input = document.getElementById('awa');
  let val = parseFloat(input.value);
  if (!isFinite(val)) val = 0;
  val = normSigned180(val + delta);
  input.value = val.toFixed(0);
}

function adjustTWS(delta) {
  const input = document.getElementById('tws');
  let val = parseFloat(input.value);
  if (!isFinite(val)) val = 0;
  val = Math.max(0, val + delta);
  input.value = val.toFixed(1);
}

document.getElementById('startBtn').addEventListener('click', startTracking);
loadPolars();
