// hosts/lidar/src/native_mock.rs
//
// A hardware-free driver that satisfies the same trait contract as the
// real RPLIDAR driver in native.rs. Instead of talking to serial, it
// synthesizes a moving virtual scene: a "wall" at 3 m in front of the
// robot, plus a scaled sine wobble across sectors. This lets Aung develop
// the ekf-lidar.mec program before Aug 22 with no hardware attached.
//
// Enabled by a Cargo feature `mock`. Do NOT enable both `native` and
// `mock` at once — pick one.
//
// Usage in mech.mcfg: identical. The provider is still "lidar", the
// URI is still lidar://<instance>/scan, and it still publishes the
// scan-id + 16 sector paths per turn.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use mech_core::MResult;
use mech_runtime::{
    materialize_host_manifest, ConfigValue, HostManifestConfig, RuntimeHostFactory,
    RuntimeHostInputDriver, RuntimeHostInputSource, RuntimeHostInstallation, RuntimeIngress,
};

use crate::{
    lidar_error, lidar_host_manifest, lidar_settings_from_config, lidar_source_matches,
    new_shared_snapshot, sector_bearing_rad, LidarHostSettings, LidarResourceProvider,
    LidarSnapshot, SharedLidarSnapshot, N_SECTORS,
};

fn synth_snapshot(scan_id: u64) -> LidarSnapshot {
    // Base scene: a "wall" 3 m ahead. Sectors nearer to the front get
    // closer readings, sectors behind get farther readings. A slow
    // scan-varying sine adds motion so the EKF has something to track.
    let mut ranges = [0.0f64; N_SECTORS];
    for i in 0..N_SECTORS {
        let b = sector_bearing_rad(i);
        let front_bias = 0.5 * (1.0 + b.cos()); // 0..1, peak at forward
        let wobble = 0.10 * ((scan_id as f64) * 0.05 + (i as f64) * 0.3).sin();
        // 1.0 m near the front, up to 5.0 m behind, plus wobble
        ranges[i] = 5.0 - 4.0 * front_bias + wobble;
        if ranges[i] < 0.1 { ranges[i] = 0.1; }
    }

    // Reduced summary
    let (mut nearest_m, mut nearest_i) = (f64::INFINITY, 0usize);
    for i in 0..N_SECTORS {
        if ranges[i] < nearest_m {
            nearest_m = ranges[i];
            nearest_i = i;
        }
    }

    LidarSnapshot {
        nearest_mm: nearest_m * 1000.0,
        nearest_angle: sector_bearing_rad(nearest_i).to_degrees(),
        front_mm: ranges[0] * 1000.0, // sector 0 is centered near 0° here
        count: N_SECTORS as f64,
        scan_id: scan_id as f64,
        sector_ranges_m: ranges,
    }
}

struct WorkerLiveReset(Arc<AtomicBool>);
impl Drop for WorkerLiveReset {
    fn drop(&mut self) { self.0.store(false, Ordering::SeqCst); }
}

pub struct MockLidarInputDriver {
    instance: String,
    snapshot: SharedLidarSnapshot,
    ingress: Arc<Mutex<Option<RuntimeIngress>>>,
    live: Arc<AtomicBool>,
    interval: Duration,
    counter: Arc<AtomicU64>,
    worker: Arc<Mutex<Option<JoinHandle<()>>>>,
    stop_sender: Arc<Mutex<Option<Sender<()>>>>,
}

impl std::fmt::Debug for MockLidarInputDriver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MockLidarInputDriver")
            .field("instance", &self.instance)
            .field("live", &self.is_live())
            .finish_non_exhaustive()
    }
}

impl MockLidarInputDriver {
    pub fn new(
        instance: impl Into<String>,
        snapshot: SharedLidarSnapshot,
        interval: Duration,
    ) -> Self {
        Self {
            instance: instance.into(),
            snapshot,
            ingress: Arc::new(Mutex::new(None)),
            live: Arc::new(AtomicBool::new(false)),
            interval,
            counter: Arc::new(AtomicU64::new(0)),
            worker: Arc::new(Mutex::new(None)),
            stop_sender: Arc::new(Mutex::new(None)),
        }
    }
}

impl RuntimeHostInputDriver for MockLidarInputDriver {
    fn drives(&self, source: &RuntimeHostInputSource) -> bool {
        lidar_source_matches(&self.instance, source)
    }

    fn attach(&mut self, ingress: RuntimeIngress) -> MResult<()> {
        let mut g = self.ingress.lock()
            .map_err(|_| lidar_error("MockLidarAttach", "ingress lock poisoned"))?;
        *g = Some(ingress);
        Ok(())
    }

    fn start(&mut self) -> MResult<()> {
        if self.live.load(Ordering::SeqCst) { return Ok(()); }
        let ingress = self.ingress.lock().unwrap().clone()
            .ok_or_else(|| lidar_error("MockLidarStart", "must attach before start"))?;
        let (tx, rx) = mpsc::channel();
        *self.stop_sender.lock().unwrap() = Some(tx);
        self.live.store(true, Ordering::SeqCst);

        let live = self.live.clone();
        let snapshot = self.snapshot.clone();
        let interval = self.interval;
        let instance = self.instance.clone();
        let counter = self.counter.clone();

        let worker = thread::spawn(move || {
            let _reset = WorkerLiveReset(live.clone());
            while live.load(Ordering::SeqCst) {
                let id = counter.fetch_add(1, Ordering::SeqCst);
                let next = synth_snapshot(id);
                if let Ok(mut g) = snapshot.lock() { *g = next; }
                if let Ok(pkt) = next.into_host_input(&instance) {
                    let _ = ingress.submit(pkt);
                }
                match rx.recv_timeout(interval) {
                    Ok(()) | Err(RecvTimeoutError::Disconnected) => break,
                    Err(RecvTimeoutError::Timeout) => {}
                }
            }
        });
        *self.worker.lock().unwrap() = Some(worker);
        Ok(())
    }

    fn stop(&mut self) -> MResult<()> {
        self.live.store(false, Ordering::SeqCst);
        if let Some(tx) = self.stop_sender.lock().unwrap().take() { let _ = tx.send(()); }
        if let Some(h) = self.worker.lock().unwrap().take() { let _ = h.join(); }
        Ok(())
    }

    fn is_live(&self) -> bool { self.live.load(Ordering::SeqCst) }
}

#[derive(Debug)]
pub struct MockLidarHostFactory {
    manifest: HostManifestConfig,
}

impl MockLidarHostFactory {
    pub fn new() -> MResult<Self> {
        Ok(Self { manifest: lidar_host_manifest()? })
    }
}

impl RuntimeHostFactory for MockLidarHostFactory {
    fn provider_name(&self) -> &str { "lidar" }
    fn manifest(&self) -> &HostManifestConfig { &self.manifest }
    fn validate_settings(&self, _name: &str, settings: &ConfigValue) -> MResult<()> {
        lidar_settings_from_config(settings).map(|_| ())
    }
    fn instantiate(&self, name: &str, settings: &ConfigValue) -> MResult<RuntimeHostInstallation> {
        let settings = lidar_settings_from_config(settings)?;
        let snapshot = new_shared_snapshot(LidarSnapshot::default());
        Ok(RuntimeHostInstallation {
            interface: materialize_host_manifest(name, &self.manifest)?,
            resource_providers: vec![Box::new(LidarResourceProvider::new(name, snapshot.clone()))],
            input_drivers: vec![Box::new(MockLidarInputDriver::new(
                name,
                snapshot,
                Duration::from_millis(settings.interval_ms),
            ))],
        })
    }
}
