// hosts/lidar/src/snapshot.rs
//
// The scan is divided into N_SECTORS equal angular slices around 360°.
// For each sector we publish two scalar paths (one f64 each):
//   sector-<i>-bearing   — center angle of that sector (radians)
//   sector-<i>-range     — minimum valid range within that sector (m)
//
// Plus the original reduced-summary paths so existing programs still work.
//
// Constraint: v0.4 RuntimeHostInputValue is scalar-only, so we cannot
// publish arrays directly. The .mec program will reassemble the sector
// scalars into row vectors and broadcast the EKF over them.

use std::sync::{Arc, Mutex};

use mech_core::MResult;
use mech_runtime::{
    RuntimeHostInput, RuntimeHostInputSource, RuntimeHostInputUpdate,
    RuntimeHostInputValue,
};

pub const N_SECTORS: usize = 16;

/// Sector center bearing in radians for sector index i (0..N_SECTORS).
/// Sector i covers [i*sector_width, (i+1)*sector_width).
pub fn sector_bearing_rad(i: usize) -> f64 {
    let width = std::f64::consts::TAU / N_SECTORS as f64;
    (i as f64 + 0.5) * width
}

pub fn lidar_input_base_uri(instance: &str) -> String {
    format!("lidar://{instance}/scan")
}

/// All the path names this host publishes. The `.mec` grants block must
/// list every path the program intends to read. Generated once at startup.
pub fn scan_paths() -> Vec<String> {
    let mut paths = vec![
        "nearest-mm".to_string(),
        "nearest-angle".to_string(),
        "front-mm".to_string(),
        "count".to_string(),
        "scan-id".to_string(),
    ];
    for i in 0..N_SECTORS {
        paths.push(format!("sector-{i}-bearing"));
        paths.push(format!("sector-{i}-range"));
    }
    paths
}

pub fn lidar_source_matches(instance: &str, source: &RuntimeHostInputSource) -> bool {
    if source.base_uri() != lidar_input_base_uri(instance) {
        return false;
    }
    let p = source.path();
    scan_paths().iter().any(|s| s == p)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarSnapshot {
    // Existing reduced summary
    pub nearest_mm: f64,
    pub nearest_angle: f64,
    pub front_mm: f64,
    pub count: f64,
    pub scan_id: f64,
    // Per-sector arrays (bearings are constant, ranges update each scan)
    pub sector_ranges_m: [f64; N_SECTORS],
}

impl Default for LidarSnapshot {
    fn default() -> Self {
        Self {
            nearest_mm: 0.0,
            nearest_angle: 0.0,
            front_mm: 0.0,
            count: 0.0,
            scan_id: 0.0,
            // Default to "far away" so an EKF sees a landmark at long range
            sector_ranges_m: [10.0; N_SECTORS],
        }
    }
}

impl LidarSnapshot {
    pub fn into_host_input(self, instance: &str) -> MResult<RuntimeHostInput> {
        let base = lidar_input_base_uri(instance);
        let mut updates: Vec<RuntimeHostInputUpdate> = Vec::new();

        let push = |updates: &mut Vec<RuntimeHostInputUpdate>,
                    path: &str,
                    value: f64|
         -> MResult<()> {
            updates.push(RuntimeHostInputUpdate {
                source: RuntimeHostInputSource::new(base.clone(), path)?,
                value: RuntimeHostInputValue::F64(value),
            });
            Ok(())
        };

        push(&mut updates, "nearest-mm", self.nearest_mm)?;
        push(&mut updates, "nearest-angle", self.nearest_angle)?;
        push(&mut updates, "front-mm", self.front_mm)?;
        push(&mut updates, "count", self.count)?;
        push(&mut updates, "scan-id", self.scan_id)?;

        for i in 0..N_SECTORS {
            push(&mut updates, &format!("sector-{i}-bearing"), sector_bearing_rad(i))?;
            push(&mut updates, &format!("sector-{i}-range"), self.sector_ranges_m[i])?;
        }

        RuntimeHostInput::new(updates)
    }
}

pub type SharedLidarSnapshot = Arc<Mutex<LidarSnapshot>>;

pub fn new_shared_snapshot(snapshot: LidarSnapshot) -> SharedLidarSnapshot {
    Arc::new(Mutex::new(snapshot))
}

/// Reduce raw (angle_deg, range_mm) scan points into a `LidarSnapshot`.
/// Fills sector_ranges_m with the *minimum* valid range in each sector.
pub fn reduce_scan_points<I>(points: I, scan_id: u64) -> LidarSnapshot
where
    I: IntoIterator<Item = (f64, f64, u8)>, // (angle_deg, range_mm, quality)
{
    let mut nearest = f64::INFINITY;
    let mut nearest_angle = 0.0;
    let mut front = f64::INFINITY;
    let mut valid = 0.0;
    let mut sector_min_m = [f64::INFINITY; N_SECTORS];

    let sector_width_deg = 360.0 / N_SECTORS as f64;

    for (angle_deg, range_mm, quality) in points {
        if quality == 0 || range_mm <= 0.0 { continue; }
        valid += 1.0;

        if range_mm < nearest {
            nearest = range_mm;
            nearest_angle = angle_deg;
        }
        let ahead = angle_deg <= 10.0 || angle_deg >= 350.0;
        if ahead && range_mm < front {
            front = range_mm;
        }

        // Bin into sector (angle_deg is already 0..360)
        let mut i = (angle_deg / sector_width_deg) as usize;
        if i >= N_SECTORS { i = N_SECTORS - 1; }
        let range_m = range_mm / 1000.0;
        if range_m < sector_min_m[i] {
            sector_min_m[i] = range_m;
        }
    }

    // Any sector that saw no valid points falls back to "far"
    let mut sector_ranges_m = [10.0f64; N_SECTORS];
    for i in 0..N_SECTORS {
        if sector_min_m[i].is_finite() {
            sector_ranges_m[i] = sector_min_m[i];
        }
    }

    LidarSnapshot {
        nearest_mm: if nearest.is_finite() { nearest } else { 0.0 },
        nearest_angle,
        front_mm: if front.is_finite() { front } else { 0.0 },
        count: valid,
        scan_id: scan_id as f64,
        sector_ranges_m,
    }
}
