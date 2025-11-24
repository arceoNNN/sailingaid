
let watchId = null;
let polars = null; // loaded from polars.json

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
  // Position
  const boatInfo = document.getElementById('boatInfo');
  boatInfo.textContent = `Pos: ${lat.toFixed(5)}, ${lon.toFixed(5)}`;

  // Heading & speed directly from GPS
  const headingInfo = document.getElementById('headingInfo');
  const speedInfo = document.getElementById('speedInfo');

  headingInfo.textContent =
    `Heading (CMG): ${cmg != null ? cmg.toFixed(0) + '°' : '–'}`;

  speedInfo.textContent =
    `Speed over ground: ${sog != null ? sog.toFixed(2) + ' kt' : '–'}`;

  // Mark info & VMG to mark
  const markLat = parseFloat(document.getElementById('markLat').value);
  const markLon = parseFloat(document.getElementById('markLon').value);
  const markInfo = document.getElementById('markInfo');
  const vmgMarkInfo = document.getElementById('vmgMarkInfo');

  let vmgMark = null;

  if (isFinite(markLat) && isFinite(markLon)) {
    const { bearing: brgMark, distance_m: dMark } =
      bearingAndDistance(lat, lon, markLat, markLon);

    markInfo.textContent =
      `To mark: brg ${brgMark.toFixed(0)}°, dist ${(dMark / 1852).toFixed(2)} NM`;

    if (sog != null && cmg != null) {
      const diff = smallestAngleDiff(cmg, brgMark);
      vmgMark = sog * Math.cos(toRad(diff)); // kn
    }
  } else {
    markInfo.textContent = 'To mark: set mark coordinates above.';
  }

  if (vmgMark != null) {
    vmgMarkInfo.textContent = `VMG to mark: ${vmgMark.toFixed(2)} kt`;
  } else {
    vmgMarkInfo.textContent = 'VMG to mark: –';
  }

  // VMG vs wind using polars
  const vmgInfo = document.getElementById('vmgInfo');
  const windDirVal = parseFloat(document.getElementById('windDir').value);
  const twsVal = parseFloat(document.getElementById('tws').value);

  // Reset colour classes
  vmgInfo.classList.remove('perf-good', 'perf-ok', 'perf-bad');

  if (!isFinite(windDirVal) || !isFinite(twsVal) || sog == null || cmg == null) {
    vmgInfo.textContent = 'VMG vs wind: waiting for wind, TWS and GPS...';
    return;
  }

  const windDir = ((windDirVal % 360) + 360) % 360;

  // Upwind vs downwind determination
  const diffToUp = smallestAngleDiff(cmg, windDir);
  const absUp = Math.abs(diffToUp);

  let mode, vmg, target;

  if (absUp <= 90) {
    // Upwind sector
    mode = 'upwind';
    vmg = sog * Math.cos(toRad(absUp)); // projection on upwind axis
    target = getTargetVMG(twsVal, 'upwind');
  } else {
    // Downwind sector
    const downDir = (windDir + 180) % 360;
    const diffToDown = smallestAngleDiff(cmg, downDir);
    const absDown = Math.abs(diffToDown);
    mode = 'downwind';
    vmg = sog * Math.cos(toRad(absDown)); // projection on downwind axis
    target = getTargetVMG(twsVal, 'downwind');
  }

  if (target && target > 0) {
    const perf = (vmg / target) * 100;
    vmgInfo.textContent =
      `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (target ${target.toFixed(2)} kt, ${perf.toFixed(0)}%)`;

    // Colour coding
    if (perf >= 95) {
      vmgInfo.classList.add('perf-good');
    } else if (perf >= 90) {
      vmgInfo.classList.add('perf-ok');
    } else {
      vmgInfo.classList.add('perf-bad');
    }
  } else {
    vmgInfo.textContent =
      `VMG ${mode} vs wind: ${vmg.toFixed(2)} kt (no polar data for this TWS yet)`;
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

// +/- helpers for wind direction and TWS
function adjustWindDir(delta) {
  const input = document.getElementById('windDir');
  let val = parseFloat(input.value);
  if (!isFinite(val)) val = 0;
  val += delta;
  // normalize to 0–359
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
