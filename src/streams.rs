use crate::structs::{Coordinate, Pois, Record, TrajCollection, Trajectory};
use std::env;

// use std::{thread, time}

fn perc_diff(sp1: f32, br1: f32, sp2: f32, br2: f32) -> bool {
    let sp_diff = (sp2 - sp1).abs() / crate::MAX_SPEED;
    let br_diff = (br2 - br1).abs() / 180.0;

    sp_diff > crate::COMP_THR || br_diff > crate::COMP_THR
}

pub fn resampled(record: Record, traj_coll: &TrajCollection, pois: &Pois) -> Trajectory {
    if !traj_coll.object.contains_key(&record.oid) {
        let new_traj = Trajectory::new(
            record.oid,
            crate::HISTORY_SIZE,
            Coordinate {
                x: record.lon,
                y: record.lat,
            },
            record.t,
        );
        // new_traj.to_csv();
        return new_traj;
    }

    let oid_traj = traj_coll.object.get(&record.oid).unwrap();

    let mut new_traj = Trajectory::new_empty(record.oid, crate::HISTORY_SIZE);
    let coord = Coordinate {
        x: record.lon,
        y: record.lat,
    };

    if (record.t == oid_traj.timestamps.last().unwrap().to_owned())
        | (record.t - oid_traj.timestamps.last().unwrap() < crate::RATE)
    {
        return new_traj;
    };

    // if oid_traj.can_skip(100, coord.clone(), timestamp, oid_traj.speed.last().unwrap().clone(), oid_traj.bearing.last().unwrap().clone()){return}

    let speed_now = oid_traj.calculate_speed(&coord, &record.t);
    let bearing_now = oid_traj.calculate_bearing(&coord);

    if speed_now > crate::MAX_SPEED {
        return new_traj;
    };

    let new_coords = oid_traj.resample(crate::RATE, &record.t, speed_now, bearing_now);

    for (new_coord, timestamp) in &new_coords {
        let is_stoped = if speed_now < crate::STOP_SPEED_THR {
            1
        } else {
            0
        };
        // let poi_id = -1;
        let poi_id = if is_stoped == 1 && oid_traj.stoped.last().unwrap().to_owned() == 1 {
            oid_traj.pois.last().unwrap().to_owned()
        } else if is_stoped == 1 {
            pois.nearest(&new_coord, crate::DISTANCE_TO_POI_THR)
        } else {
            -1
        };

        let trip_id = if oid_traj.stoped.last().unwrap().to_owned() == 1 && is_stoped != 1 {
            oid_traj.trips.last().unwrap() + 1
        } else {
            oid_traj.trips.last().unwrap().to_owned()
        };

        // let gps: Vec<i32> = traj.flocks(record.oid, flocks_distance_threshold, timestamp, speed_now, bearing_now);

        // eprintln!("hey {:?}", new_traj);
        let flocked_oids = traj_coll.flocks(
            &coord,
            speed_now,
            bearing_now,
            timestamp.to_owned(),
            record.oid,
        );

        new_traj.insert_unbounded(
            new_coord.clone(),
            timestamp.to_owned(),
            speed_now,
            bearing_now,
            poi_id,
            trip_id,
            is_stoped,
            flocked_oids,
        );
    }
    // new_traj.to_csv();
    return new_traj;
}

pub fn compressed(
    record: Record,
    traj_coll: &TrajCollection,
    traj_coll_all: &TrajCollection,
    pois: &Pois,
) -> (Trajectory, Option<usize>) {
    if !traj_coll.object.contains_key(&record.oid) {
        let new_traj = Trajectory::new(
            record.oid,
            crate::HISTORY_SIZE,
            Coordinate {
                x: record.lon,
                y: record.lat,
            },
            record.t,
        );
        // new_traj.to_csv();
        return (new_traj, None);
    }

    let oid_traj = traj_coll.object.get(&record.oid).unwrap();

    let mut new_traj = Trajectory::new_empty(record.oid, crate::HISTORY_SIZE);
    let coord = Coordinate {
        x: record.lon,
        y: record.lat,
    };

    if record.t == oid_traj.timestamps.last().unwrap().to_owned() {
        return (new_traj, None);
    };

    // if oid_traj.can_skip(100, coord.clone(), timestamp, oid_traj.speed.last().unwrap().clone(), oid_traj.bearing.last().unwrap().clone()){return}

    let speed_now = oid_traj.calculate_speed(&coord, &record.t);
    let bearing_now = oid_traj.calculate_bearing(&coord);

    if speed_now > crate::MAX_SPEED {
        return (new_traj, None);
    };

    let key = "CMP_ALGO";
    let return_id = match env::var(key) {
        Ok(v) =>  match v.as_str() {
            "opw" => oid_traj.OPW(&coord, record.t),
            "opw_tr" => oid_traj.OPW_TR(&coord, record.t),
            "uniform" => oid_traj.uniform(&traj_coll_all),
            "dead_reckoning" => oid_traj.dead_reckoning(&coord, record.t),
            _ => panic!("$ {} algorithm not found",v ),
        },
        Err(e) => panic!("${} is not set ({})", key, e),
    };

    // match return_id {
    //     Some(return_id_usize) => {
    //         oid_traj.print_row(return_id_usize);
    //     },
    //     _ => ()
    // }

    let is_stoped = if speed_now < crate::STOP_SPEED_THR {
        1
    } else {
        0
    };
    // let poi_id = -1;
    let poi_id = if is_stoped == 1 && oid_traj.stoped.last().unwrap().to_owned() == 1 {
        oid_traj.pois.last().unwrap().to_owned()
    } else if is_stoped == 1 {
        pois.nearest(&coord, crate::DISTANCE_TO_POI_THR)
    } else {
        -1
    };
    let trip_id = if oid_traj.stoped.last().unwrap().to_owned() == 1 && is_stoped != 1 {
        oid_traj.trips.last().unwrap() + 1
    } else {
        oid_traj.trips.last().unwrap().to_owned()
    };

    // let gps: Vec<i32> = traj.flocks(record.oid, flocks_distance_threshold, timestamp, speed_now, bearing_now);
    // eprintln!("hey {:?}", new_traj);
    let flocked_oids = vec![];

    new_traj.insert_unbounded(
        coord.clone(),
        record.t,
        speed_now,
        bearing_now,
        poi_id,
        trip_id,
        is_stoped,
        flocked_oids,
    );

    return (new_traj, return_id);
}

pub fn cleaned(record: Record, traj_coll: &TrajCollection, pois: &Pois) -> Trajectory {
    if !traj_coll.object.contains_key(&record.oid) {
        return Trajectory::new(
            record.oid,
            crate::HISTORY_SIZE,
            Coordinate {
                x: record.lon,
                y: record.lat,
            },
            record.t,
        );
    }

    let oid_traj = traj_coll.object.get(&record.oid).unwrap();

    let mut new_traj = Trajectory::new_empty(record.oid, crate::HISTORY_SIZE);
    let coord = Coordinate {
        x: record.lon,
        y: record.lat,
    };

    if record.t == oid_traj.timestamps.last().unwrap().to_owned() {
        return new_traj;
    };

    // if oid_traj.can_skip(100, coord.clone(), timestamp, oid_traj.speed.last().unwrap().clone(), oid_traj.bearing.last().unwrap().clone()){return}

    let speed_now = oid_traj.calculate_speed(&coord, &record.t);
    let bearing_now = oid_traj.calculate_bearing(&coord);

    if speed_now > crate::MAX_SPEED {
        return new_traj;
    };

    let is_stoped = if speed_now < crate::STOP_SPEED_THR {
        1
    } else {
        0
    };

    let poi_id = if is_stoped == 1 && oid_traj.stoped.last().unwrap().to_owned() == 1 {
        oid_traj.pois.last().unwrap().to_owned()
    } else if is_stoped == 1 {
        pois.nearest(&coord, crate::DISTANCE_TO_POI_THR)
    } else {
        -1
    };

    let trip_id = if oid_traj.stoped.last().unwrap().to_owned() == 1 && is_stoped != 1 {
        oid_traj.trips.last().unwrap() + 1
    } else {
        oid_traj.trips.last().unwrap().to_owned()
    };

    // let gps: Vec<i32> = traj.flocks(record.oid, flocks_distance_threshold, timestamp, speed_now, bearing_now);
    // eprintln!("hey {:?}", new_traj);
    let flocked_oids = vec![];

    new_traj.insert_unbounded(
        coord.clone(),
        record.t,
        speed_now,
        bearing_now,
        poi_id,
        trip_id,
        is_stoped,
        flocked_oids,
    );

    return new_traj;
}
