
let watchId = null;
let polars = null; // loaded from polars.json

const BOAT_LENGTH_M = 35 * 0.3048; // 35 ft in meters

function toRad(deg) { return deg * Math.PI / 180; }
function toDeg(rad) { return rad * 180 / Math.PI; }

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

function smallestAngleDiff(a, b) {
  let diff = (a - b + 540) % 360 - 180;
  return diff; // -180..180
}

// 1D linear interpolation helper
function interp1(x, xs, ys) {
  if (!Array.isArray(xs) || !Array.isArray(ys) || xs.length === 0 || xs.length !== ys.length) {
    return null;
  }
  if (x <= xs[0]) return ys[0];
  if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
  for (let i = 0; i < xs.length - 1; i++) {
    const x0 = xs[i];
    const x1 = xs[i + 1];
    if (x >= x0 && x <= x1) {
      const y0 = ys[i];
      const y1 = ys[i + 1];
      const t = (x - x0) / (x1 - x0);
      return y0 + t * (y1 - y0);
    }
  }
  return null;
}

function getTargetVMG(tws, mode) {
  if (!polars) return null;
  const xs = polars.tws;
  if (mode === 'upwind') {
    return interp1(tws, xs, polars.beatVMG);
  } else if (mode === 'downwind') {
    return interp1(tws, xs, polars.runVMG);
  }
  return null;
}

// Get target boat speed at given TWS and TWA using full polar table
function getTargetBoatSpeed(tws, twa) {
  if (!polars || !polars.angles || !polars.speeds) return null;

  const twsList = polars.tws;
  const angles = polars.angles;
  const speeds = polars.speeds;

  if (!Array.isArray(twsList) || twsList.length === 0 ||
      !Array.isArray(angles) || angles.length === 0) {
    return null;
  }

  // Clamp TWA to [minAngle, maxAngle]
  let ang = Math.abs(twa);
  if (ang > 180) ang = 360 - ang;
  const minA = angles[0];
  const maxA = angles[angles.length - 1];
  if (ang <= minA) ang = minA;
  if (ang >= maxA) ang = maxA;

  // Find angle bracket
  let a0 = angles[0], a1 = angles[angles.length - 1];
  for (let i = 0; i < angles.length - 1; i++) {
    if (ang >= angles[i] && ang <= angles[i + 1]) {
      a0 = angles[i];
      a1 = angles[i + 1];
      break;
    }
  }

  const arrA0 = speeds[String(a0)];
  const arrA1 = speeds[String(a1)];
  if (!arrA0 || !arrA1) return null;

  // Interpolate in TWS dimension for each bounding angle
  const spA0 = interp1(tws, twsList, arrA0);
  const spA1 = interp1(tws, twsList, arrA1);
  if (spA0 == null || spA1 == null) return null;

  if (a1 === a0) return spA0;

  // Interpolate in angle dimension
  const t = (ang - a0) / (a1 - a0);
  return spA0 + t * (spA1 - spA0);
}

// Compute best downwind TWA for given TWS using polars
function getBestDownwindAngle(tws) {
  if (!polars || !polars.angles || !polars.speeds) return null;

  const twsList = polars.tws;
  const angles = polars.angles;
  const speeds = polars.speeds;

  let bestAngle = null;
  let bestVMG = -Infinity;

  for (let i = 0; i < angles.length; i++) {
    const ang = angles[i]; // TWA
    const arr = speeds[String(ang)];
    if (!arr) continue;
    const sp = interp1(tws, twsList, arr);
    if (sp == null) continue;

    // Projection toward downwind axis (TWA = 180)
    const vmgDown = sp * Math.cos(toRad(180 - ang)); // >= 0 downwind
    if (vmgDown > bestVMG) {
      bestVMG = vmgDown;
      bestAngle = ang;
    }
  }
  return bestAngle;
}

function startTracking() {
  if (!navigator.geolocation) {
    document.getElementById('gpsStatus').textContent = 'GPS not supported';
    return;
  }

  document.getElementById('gpsStatus').textContent = 'Requesting GPS...';

  // Ask for freshest possible GPS data; real rate still depends on hardware/OS
  watchId = navigator.geolocation.watchPosition(onPosition, onError, {
    enableHighAccuracy: true,
    maximumAge: 0,
    timeout: 5000
  });
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
  const boatInfo = document.getElementById('boatInfo');
  boatInfo.textContent = `Pos: ${lat.toFixed(5)}, ${lon.toFixed(5)}`;

  const headingInfo = document.getElementById('headingInfo');
  const speedInfo = document.getElementById('speedInfo');
  const twaInfo = document.getElementById('twaInfo');

  headingInfo.textContent =
    `Heading (CMG): ${cmg != null ? cmg.toFixed(0) + '°' : '–'}`;

  speedInfo.textContent =
    `Speed over ground: ${sog != null ? sog.toFixed(2) + ' kt' : '–'}`;

  const useGate = document.getElementById('useGate').checked;

  const markInfo = document.getElementById('markInfo');
  const vmgMarkInfo = document.getElementById('vmgMarkInfo');
  const laylineInfo = document.getElementById('laylineInfo');

  let activeLat = null;
  let activeLon = null;
  let activeLabel = '';

  // Choose active mark: either single or nearest gate mark
  const singleLat = parseFloat(document.getElementById('markLat').value);
  const singleLon = parseFloat(document.getElementById('markLon').value);

  const gateLeftLat = parseFloat(document.getElementById('gateLeftLat').value);
  const gateLeftLon = parseFloat(document.getElementById('gateLeftLon').value);
  const gateRightLat = parseFloat(document.getElementById('gateRightLat').value);
  const gateRightLon = parseFloat(document.getElementById('gateRightLon').value);

  let brgMark = null;
  let dMark = null;

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
      activeLat = best.lat;
      activeLon = best.lon;
      activeLabel = best.label;
      brgMark = best.bearing;
      dMark = best.dist;
    }
  } else {
    if (isFinite(singleLat) && isFinite(singleLon)) {
      const bd = bearingAndDistance(lat, lon, singleLat, singleLon);
      activeLat = singleLat;
      activeLon = singleLon;
      activeLabel = '';
      brgMark = bd.bearing;
      dMark = bd.distance_m;
    }
  }

  let vmgMark = null;

  if (activeLat != null && activeLon != null && brgMark != null && dMark != null) {
    const labelSuffix = activeLabel ? ` (${activeLabel})` : '';
    markInfo.textContent =
      `To mark${labelSuffix}: brg ${brgMark.toFixed(0)}°, dist ${(dMark / 1852).toFixed(2)} NM`;

    if (sog != null && cmg != null) {
      const diff = smallestAngleDiff(cmg, brgMark);
      vmgMark = sog * Math.cos(toRad(diff)); // kn
    }
  } else {
    markInfo.textContent = 'To mark: set mark or gate coordinates above.';
  }

  if (vmgMark != null) {
    vmgMarkInfo.textContent = `VMG to mark: ${vmgMark.toFixed(2)} kt`;
  } else {
    vmgMarkInfo.textContent = 'VMG to mark: –';
  }

  // VMG vs wind using polars & speed vs polar
  const vmgInfo = document.getElementById('vmgInfo');
  const speedPolarInfo = document.getElementById('speedPolarInfo');

  const windDirVal = parseFloat(document.getElementById('windDir').value);
  const twsVal = parseFloat(document.getElementById('tws').value);

  vmgInfo.classList.remove('perf-good', 'perf-ok', 'perf-bad');
  speedPolarInfo.classList.remove('perf-good', 'perf-ok', 'perf-bad');
  laylineInfo.textContent = 'Layline: –';

  if (!isFinite(windDirVal) || !isFinite(twsVal) || sog == null || cmg == null) {
    vmgInfo.textContent = 'VMG vs wind: waiting for wind, TWS and GPS...';
    speedPolarInfo.textContent = 'Speed vs polar: waiting for wind, TWS and GPS...';
    twaInfo.textContent = 'TWA: –';
    return;
  }

  const windDir = ((windDirVal % 360) + 360) % 360;

  // Compute TWA (0..180)
  let rawDiff = smallestAngleDiff(cmg, windDir); // -180..180
  let twa = Math.abs(rawDiff);
  if (twa > 180) twa = 360 - twa;
  twaInfo.textContent = `TWA (approx): ${twa.toFixed(0)}°`;

  // Upwind vs downwind determination for VMG vs wind
  const absUp = Math.abs(rawDiff);

  let mode, vmg, targetVMG;

  if (absUp <= 90) {
    mode = 'upwind';
    vmg = sog * Math.cos(toRad(absUp)); // projection on upwind axis
    targetVMG = getTargetVMG(twsVal, 'upwind');
  } else {
    const downDir = (windDir + 180) % 360;
    const diffToDown = smallestAngleDiff(cmg, downDir);
    const absDown = Math.abs(diffToDown);
    mode = 'downwind';
    vmg = sog * Math.cos(toRad(absDown)); // projection on downwind axis
    targetVMG = getTargetVMG(twsVal, 'downwind');
  }

  if (targetVMG && targetVMG > 0) {
    const perfVMG = (vmg / targetVMG) * 100;
    vmgInfo.textContent =
      `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (target ${targetVMG.toFixed(2)} kt, ${perfVMG.toFixed(0)}%)`;

    if (perfVMG >= 95) {
      vmgInfo.classList.add('perf-good');
    } else if (perfVMG >= 90) {
      vmgInfo.classList.add('perf-ok');
    } else {
      vmgInfo.classList.add('perf-bad');
    }
  } else {
    vmgInfo.textContent =
      `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (no polar VMG data for this TWS yet)`;
  }

  // Speed vs polar speed at this TWS & TWA
  const targetSpeed = getTargetBoatSpeed(twsVal, twa);

  if (targetSpeed && targetSpeed > 0) {
    const perfSpd = (sog / targetSpeed) * 100;
    speedPolarInfo.textContent =
      `Speed vs polar: ${sog.toFixed(2)} kt (target ${targetSpeed.toFixed(2)} kt, ${perfSpd.toFixed(0)}%)`;

    if (perfSpd >= 95) {
      speedPolarInfo.classList.add('perf-good');
    } else if (perfSpd >= 90) {
      speedPolarInfo.classList.add('perf-ok');
    } else {
      speedPolarInfo.classList.add('perf-bad');
    }
  } else {
    speedPolarInfo.textContent =
      `Speed vs polar: ${sog != null ? sog.toFixed(2) + ' kt' : '–'} (no polar speed data for this TWS/TWA yet)`;
  }

  // Layline distance & time: works for both upwind & downwind legs
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
          let brgLLDown;
          if (onStarboard) {
            brgLLDown = (windDir + beatAng + 180) % 360;
          } else {
            brgLLDown = (windDir - beatAng + 180 + 360) % 360;
          }

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
            if (ttl_s < 90) {
              ttlText = `${ttl_s.toFixed(0)} s`;
            } else {
              const mins = Math.floor(ttl_s / 60);
              const secs = Math.round(ttl_s % 60);
              ttlText = `${mins}m ${secs}s`;
            }
          }

          const boatSign = Math.sign(-crossMark);
          const windAxisRad = toRad(windDir);
          const insideSign = Math.sign(Math.sin(windAxisRad) * uy - Math.cos(windAxisRad) * ux);

          let sideText;
          if (dist_m < 0.5 * BOAT_LENGTH_M) {
            sideText = 'on layline';
          } else if (boatSign === insideSign || insideSign === 0) {
            sideText = `inside ${bl.toFixed(1)} BL`;
          } else {
            sideText = `overstood by ${bl.toFixed(1)} BL`;
          }

          const tackStr = onStarboard ? 'starboard' : 'port';
          const gateSuffix = activeLabel ? ` (${activeLabel})` : '';
          const ttlPart = sideText.startsWith('overstood') ? '' : `, ${ttlText}`;

          laylineInfo.textContent =
            `To ${tackStr} upwind layline${gateSuffix}: ${sideText}${ttlPart}`;
        }
      }
    } else if (mode === 'downwind') {
      const windDown = (windDir + 180) % 360;
      const runAng = getBestDownwindAngle(twsVal);
      if (runAng != null) {
        const diffMarkDown = Math.abs(smallestAngleDiff(brgMark, windDown));
        if (diffMarkDown <= 100) {
          const beta = 180 - runAng;
          const diffDownNow = smallestAngleDiff(cmg, windDown);
          const onStarboardJibe = diffDownNow > 0;

          let brgLLDown;
          if (onStarboardJibe) {
            brgLLDown = (windDown - beta + 180 + 360) % 360;
          } else {
            brgLLDown = (windDown + beta + 180) % 360;
          }

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
            if (ttl_s < 90) {
              ttlText = `${ttl_s.toFixed(0)} s`;
            } else {
              const mins = Math.floor(ttl_s / 60);
              const secs = Math.round(ttl_s % 60);
              ttlText = `${mins}m ${secs}s`;
            }
          }

          const boatSign = Math.sign(-crossMark);
          const windAxisRad = toRad(windDown);
          const insideSign = Math.sign(Math.sin(windAxisRad) * uy - Math.cos(windAxisRad) * ux);

          let sideText;
          if (dist_m < 0.5 * BOAT_LENGTH_M) {
            sideText = 'on layline';
          } else if (boatSign === insideSign || insideSign === 0) {
            sideText = `inside ${bl.toFixed(1)} BL`;
          } else {
            sideText = `overstood by ${bl.toFixed(1)} BL`;
          }

          const tackStr = onStarboardJibe ? 'starboard' : 'port';
          const gateSuffix = activeLabel ? ` (${activeLabel})` : '';
          const ttlPart = sideText.startsWith('overstood') ? '' : `, ${ttlText}`;

          laylineInfo.textContent =
            `To ${tackStr} downwind layline${gateSuffix}: ${sideText}${ttlPart}`;
        }
      }
    }
  }
}

function loadPolars() {
  fetch('polars.json')
    .then(response => response.json())
    .then(data => {
      polars = data;
      console.log('Polars loaded:', polars);
    })
    .catch(err => {
      console.error('Failed to load polars.json', err);
    });
}

// Parse DMS / DM / decimal coordinate string into decimal degrees
function parseCoordString(str, isLat) {
  if (!str) return null;
  let s = str.trim().toUpperCase();

  let sign = 1;
  if (s.includes('S') || s.includes('W')) {
    sign = -1;
  }
  // Remove hemisphere letters
  s = s.replace(/[NSEW]/g, ' ');

  // Replace degree/minute/second symbols with spaces
  s = s.replace(/[°º]/g, ' ');
  s = s.replace(/[′']/g, ' ');
  s = s.replace(/[″"]/g, ' ');

  // Replace commas with dots
  s = s.replace(/,/g, '.');

  // Now split on whitespace
  const parts = s.split(/\s+/).filter(p => p.length > 0);

  let deg, min = 0, sec = 0;
  if (parts.length === 1) {
    // Decimal degrees
    deg = parseFloat(parts[0]);
    if (!isFinite(deg)) return null;
  } else if (parts.length === 2) {
    // Degrees + minutes
    deg = parseFloat(parts[0]);
    min = parseFloat(parts[1]);
    if (!isFinite(deg) || !isFinite(min)) return null;
  } else {
    // Degrees + minutes + seconds (ignore extras if any)
    deg = parseFloat(parts[0]);
    min = parseFloat(parts[1]);
    sec = parseFloat(parts[2]);
    if (!isFinite(deg) || !isFinite(min) || !isFinite(sec)) return null;
  }

  let dec = Math.abs(deg) + (Math.abs(min) / 60) + (Math.abs(sec) / 3600);
  dec *= (deg < 0 ? -1 : 1); // keep explicit negative sign if typed
  dec *= sign; // apply hemisphere

  // Basic sanity checks
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

  const markLatInput = document.getElementById('markLat');
  const markLonInput = document.getElementById('markLon');

  markLatInput.value = lat.toFixed(5);
  markLonInput.value = lon.toFixed(5);

  statusEl.textContent = `Converted: lat ${lat.toFixed(5)}, lon ${lon.toFixed(5)} (decimal degrees).`;
  statusEl.classList.add('status-ok');
}

// +/- helpers for wind direction and TWS
function adjustWindDir(delta) {
  const input = document.getElementById('windDir');
  let val = parseFloat(input.value);
  if (!isFinite(val)) val = 0;
  val += delta;
  val = ((val % 360) + 360) % 360;
  input.value = val.toFixed(0);
}

function adjustTWS(delta) {
  const input = document.getElementById('tws');
  let val = parseFloat(input.value);
  if (!isFinite(val)) val = 0;
  val += delta;
  if (val < 0) val = 0;
  input.value = val.toFixed(1);
}

document.getElementById('startBtn').addEventListener('click', startTracking);
loadPolars();
