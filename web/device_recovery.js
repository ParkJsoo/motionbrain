/* Shared by the controller and dashboard; discovery never carries command credentials. */
(function (root) {
  'use strict';
  function localBase(value, port) {
    try {
      const u = new URL(value);
      const octets = u.hostname.split('.').map(Number);
      const privateIP = /^\d+\.\d+\.\d+\.\d+$/.test(u.hostname) && octets.every(n => n >= 0 && n <= 255) &&
        (octets[0] === 10 || (octets[0] === 172 && octets[1] >= 16 && octets[1] <= 31) || (octets[0] === 192 && octets[1] === 168));
      if (u.protocol !== 'http:' || u.username || u.password || u.search || u.hash || u.pathname !== '/' ||
          !(privateIP || /^motionbrain(?:-pi|-cam)?\.local$/.test(u.hostname))) return null;
      if (port && (u.port || '80') !== String(port)) return null;
      return u.origin;
    } catch (_) { return null; }
  }
  async function read(url) {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), 2000);
    try {
      const response = await fetch(url, {signal: abort.signal, cache: 'no-store', credentials: 'omit', redirect: 'error'});
      if (!response.ok) throw new Error('discovery HTTP ' + response.status);
      return await response.json();
    } finally { clearTimeout(timer); }
  }
  function dashboardConfig(data) {
    return data && data.service === 'motionbrain-dashboard' && data.recoveryVersion === 1 &&
      localBase(data.motionBaseUrl) && (!data.cameraUrl || localBase(data.cameraUrl));
  }
  // A registry candidate is authenticated when published, expires on the controller,
  // and is checked as a dashboard before use. No subnet scanning in the browser.
  async function discover(controllerBases, dashboardBases) {
    const candidates = [];
    for (const base of [...new Set(controllerBases)]) {
      if (root.document && root.document.hidden) return null;
      const controller = localBase(base);
      if (!controller) continue;
      try {
        const entry = await read(controller + '/vision_endpoint');
        const url = localBase(entry.dashboardUrl);
        if (entry.schema === 'motionbrain.vision_endpoint.v1' && entry.available === true && url &&
            Number.isFinite(entry.remainingMs) && entry.remainingMs > 0 && entry.remainingMs <= 180000) candidates.push({base: url, controllerBase: controller});
      } catch (_) { /* Try the next independent rendezvous. */ }
    }
    candidates.push(...dashboardBases.map(base => ({base: localBase(base)})).filter(item => item.base));
    const visited = new Set();
    for (const candidate of candidates) {
      if (root.document && root.document.hidden) return null;
      const base = candidate.base;
      if (visited.has(base)) continue;
      visited.add(base);
      try {
        const config = await read(base + '/api/discovery');
        if (dashboardConfig(config)) return {base, config, controllerBase: candidate.controllerBase};
      } catch (_) { /* Bounded candidate failure; caller schedules the next cycle. */ }
    }
    return null;
  }
  root.MotionBrainDiscovery = {localBase, read, dashboardConfig, discover};
})(globalThis);
