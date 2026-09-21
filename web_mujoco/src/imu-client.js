const POLL_MS = 20;

export function createImuClient({ onSample, onStatus }) {
  let timer = 0;
  let running = false;

  async function tick() {
    if (!running) return;
    try {
      const response = await fetch('/imu-bridge/imu', { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const sample = await response.json();
      if (sample.ok) onSample(sample);
      onStatus(sample);
    } catch (error) {
      onStatus({
        ok: false,
        connected: false,
        error: error && error.message ? error.message : String(error)
      });
    }
  }

  return {
    start() {
      if (running) return;
      running = true;
      tick();
      timer = window.setInterval(tick, POLL_MS);
    },
    stop() {
      running = false;
      window.clearInterval(timer);
      timer = 0;
    },
    isRunning() {
      return running;
    }
  };
}
