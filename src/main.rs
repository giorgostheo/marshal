mod streams;
mod structs;
use core::time;
use kdam::tqdm;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::time::Instant;
use streams::{cleaned, compressed, resampled};
use structs::{Coordinate, Pois, Record, TrajCollection, Trajectory};
use std::env;
use tch;

// use std::{thread, time};

static MAX_SPEED: f32 = 50.0; // knots
static RATE: i32 = 10; // seconds
static STOP_SPEED_THR: f32 = 0.5; // knots
static DISTANCE_TO_POI_THR: f32 = 1.0; // nmiles
static HISTORY_SIZE: usize = usize::MAX; // how many records should I keep in mem
static FLOCKS_DISTANCE_THRESHOLD: f32 = 0.3; // nmiles
static FLOCKS_MAX_DT_THRESHOLD: i32 = 30 * 60; // seconds
static FLOCKS_MAX_BEARING_THRESHOLD: f32 = 20.0;
static COMP_THR: f32 = 0.1;
static OPW_TR_EPSILON: f32 = 0.0003;
static OPW_EPSILON:f32 = 0.0005;
static UNIFORM_S: usize = 5;
static DEAD_REC_EPSILON: f32 = 0.0001;
static MODEL_PATH: &str = "vrf_brest_proto_jit_trace.pth";

fn run(path: &str, pois_path: &str) -> Result<(), csv::Error> {
    // let mut reader_traj = csv::Reader::from_path(env!("CRDS"))?;

    let mut reader_traj = csv::Reader::from_path(path)?;

    let pois: Pois = Pois::new_from_path(pois_path);

    let model = tch::CModule::load(MODEL_PATH).unwrap();

    // pois.pretty();
    // println!("oid\tlon\tlat\tspeed\tbearing\tstoped\ttrip\ttimestamp\tpoi_id\tgps");

    let mut traj_clean = TrajCollection {
        object: HashMap::new(),
    };
    let mut traj_resed = TrajCollection {
        object: HashMap::new(),
    };
    let mut traj_comp = TrajCollection {
        object: HashMap::new(),
    };

    let mut cnt_clean = 0.0;
    let mut cnt_pred = 0.0;
    let mut cnt_resed = 0.0;
    let mut cnt_comp = 0.0;

    for record in tqdm!(reader_traj.deserialize(),) {
        // for record in tqdm!(reader.deserialize()) {
        let record: Record = record?;

        let now = Instant::now();

        let clean_traj = cleaned(record.clone(), &traj_clean, &pois);

        traj_clean.extend_flush(clean_traj, None);

        cnt_clean += now.elapsed().as_nanos() as f64;

        // ------------

        let now = Instant::now();

        let resampled_traj = resampled(record.clone(), &traj_resed, &pois);
        traj_resed.extend_flush(resampled_traj, None);

        cnt_resed += now.elapsed().as_nanos() as f64;

        // ------------
        let now = Instant::now();

        let (compressed_traj, flush_id) = compressed(record.clone(), &traj_comp,&traj_clean, &pois);

        traj_comp.extend_flush(compressed_traj, flush_id);

        cnt_comp += now.elapsed().as_nanos() as f64;

        // ------------

        let now = Instant::now();

        traj_clean.predict_for_oid(record.oid, &model);

        cnt_pred += now.elapsed().as_nanos() as f64;
    }

    println!(
        "{} -> {},{},{},{}",
        path,
        cnt_clean / 10_000.0,
        cnt_resed / 10_000.0,
        cnt_comp / 10_000.0,
        cnt_pred / 10_000.0
    );
    Ok(())
}

fn main() {
    // Read arguments, set env variable
    let args: Vec<String> = env::args().collect();
    let cmp_algo = match args.len() {
        3 => &args[1],
        _ => panic!("$ Compression algorithm choice should be given as an argument"),
    };
    env::set_var("CMP_ALGO", cmp_algo);

    // run("mt.csv", "ports_saronikos.csv");
    eprintln!("{:?}", run("brest.csv", "ports_brest.csv"));
}

// let (processed_trajectory, flush_id) = if STREAM_ID=="1" {
//     (resampled(record.clone(), &traj, &pois),None)
// } else if STREAM_ID=="2" {
//     compressed(record.clone(), &traj, &pois)
// } else {
//     panic!("SID '{}' is not available", STREAM_ID)
// };
