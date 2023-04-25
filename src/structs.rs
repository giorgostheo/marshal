use itertools::izip;
use libm::atan2f;
use proj::Proj;
use serde::Deserialize;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use tch::{CModule, Tensor};

#[derive(Deserialize, Clone)]
pub struct Record {
    pub oid: i32,
    pub t: i32,
    pub lon: f32,
    pub lat: f32,
}

impl std::fmt::Display for Record {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Record {{oid: {}, t: {}, lon: {}, lat: {}}}",
            self.oid, self.t, self.lon, self.lat
        )
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Coordinate {
    pub x: f32,
    pub y: f32,
}
impl Coordinate {
    pub fn haversine(&self, coord: &Coordinate) -> f32 {
        let R = 6371000.0;
        let d_lat = (coord.y - self.y).to_radians();
        let d_lon = (coord.x - self.x).to_radians();

        let a: f32 = ((d_lat / 2.0).sin()) * ((d_lat / 2.0).sin())
            + ((d_lon / 2.0).sin())
                * ((d_lon / 2.0).sin())
                * (self.y.to_radians().cos())
                * (coord.y.to_radians().cos());
        let c: f32 = 2.0 * ((a.sqrt()).atan2((1.0 - a).sqrt()));

        R * c / 1852.0 // returns nautical miles
    }

    pub fn from_tuple(tup: (f32, f32)) -> Coordinate {
        Coordinate { x: tup.0, y: tup.1 }
    }

    pub fn bearing(&self, coord: &Coordinate) -> f32 {
        let d_lon = coord.x - self.x;
        let x = coord.y.to_radians().cos() * d_lon.to_radians().sin();
        let y = self.y.to_radians().cos() * coord.y.to_radians().sin()
            - self.y.to_radians().sin() * coord.y.to_radians().cos() * d_lon.to_radians().cos();
        // 	brgs.append(np.degrees(np.arctan2(x,y)))
        atan2f(x, y).to_degrees()
    }

    pub fn extrapolate(&self, speed: f32, bearing: f32, dt: i32) -> Coordinate {
        let distance_m_per_s = speed * 1852.0 / 3600.0 * dt as f32;

        let lon1 = self.x.to_radians();
        let lat1 = self.y.to_radians();

        let rad_bearing = bearing.to_radians();

        let delta = distance_m_per_s / 6371000.0;

        let lat2 = (lat1.sin() * delta.cos() + lat1.cos() * delta.sin() * rad_bearing.cos()).asin();

        let lon2 = lon1
            + atan2f(
                rad_bearing.sin() * delta.sin() * lat1.cos(),
                delta.cos() - lat1.sin() * lat2.sin(),
            );

        Coordinate {
            x: (lon2.to_degrees() + 540.0) % 360.0 - 180.0,
            y: lat2.to_degrees(),
        }
    }

    pub fn project(&self, from: &str, to: &str) -> Coordinate {
        let ft_to_m = Proj::new_known_crs(&from, &to, None).unwrap();
        Coordinate::from_tuple(ft_to_m.convert((self.x, self.y)).unwrap())
    }
}

#[derive(Debug, Clone)]
pub struct Trajectory {
    pub oid: i32,
    pub max_size: usize,
    pub coordinates: Vec<Coordinate>,
    pub timestamps: Vec<i32>,
    pub speed: Vec<f32>,
    pub bearing: Vec<f32>,
    pub stoped: Vec<i8>,
    pub trips: Vec<i32>,
    pub pois: Vec<i32>,
    pub gps: Vec<Vec<i32>>,
}

impl Trajectory {
    pub fn new(oid: i32, max_size: usize, coord: Coordinate, timestamp: i32) -> Trajectory {
        Trajectory {
            oid,
            max_size,
            coordinates: vec![coord.clone()],
            timestamps: vec![timestamp],
            speed: vec![-1.0],
            bearing: vec![-1.0],
            stoped: vec![-1],
            trips: vec![0],
            pois: vec![-1],
            gps: vec![vec![]],
        }
    }

    pub fn new_empty(oid: i32, max_size: usize) -> Trajectory {
        Trajectory {
            oid,
            max_size,
            coordinates: vec![],
            timestamps: vec![],
            speed: vec![],
            bearing: vec![],
            stoped: vec![],
            trips: vec![],
            pois: vec![],
            gps: vec![],
        }
    }
    pub fn insert_unbounded(
        &mut self,
        coord: Coordinate,
        ts: i32,
        sp: f32,
        br: f32,
        poi_id: i32,
        trip_id: i32,
        stoped: i8,
        gps: Vec<i32>,
    ) {
        self.speed.push(sp);
        self.bearing.push(br);
        self.coordinates.push(coord.clone());
        self.timestamps.push(ts);
        self.pois.push(poi_id);
        self.stoped.push(stoped);
        self.trips.push(trip_id);
        self.gps.push(gps);
    }

    pub fn extend(&mut self, trajectory: Trajectory) {
        self.speed.extend(trajectory.speed);
        self.bearing.extend(trajectory.bearing);
        self.coordinates.extend(trajectory.coordinates);
        self.timestamps.extend(trajectory.timestamps);
        self.pois.extend(trajectory.pois);
        self.stoped.extend(trajectory.stoped);
        self.trips.extend(trajectory.trips);
        self.gps.extend(trajectory.gps);

        let size = self.speed.len();

        if size > self.max_size {
            self.speed.drain(0..size - self.max_size);
            self.bearing.drain(0..size - self.max_size);
            self.coordinates.drain(0..size - self.max_size);
            self.timestamps.drain(0..size - self.max_size);
            self.stoped.drain(0..size - self.max_size);
            self.trips.drain(0..size - self.max_size);
            self.pois.drain(0..size - self.max_size);
            self.gps.drain(0..size - self.max_size);
        }
    }

    pub fn drop_first_n(&mut self, n: usize) {
        self.speed.drain(0..n);
        self.bearing.drain(0..n);
        self.coordinates.drain(0..n);
        self.timestamps.drain(0..n);
        self.stoped.drain(0..n);
        self.trips.drain(0..n);
        self.pois.drain(0..n);
        self.gps.drain(0..n);
    }

    pub fn to_csv(&self) {
        for i in 0..self.speed.len() {
            println!(
                "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:?}",
                self.oid,
                self.coordinates[i].x,
                self.coordinates[i].y,
                self.speed[i],
                self.bearing[i],
                self.stoped[i],
                self.trips[i],
                self.timestamps[i],
                self.pois[i],
                self.gps[i]
            )
        }
    }

    pub fn print_row(&self, i: usize) {
        println!(
            "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:?}",
            self.oid,
            self.coordinates[i].x,
            self.coordinates[i].y,
            self.speed[i],
            self.bearing[i],
            self.stoped[i],
            self.trips[i],
            self.timestamps[i],
            self.pois[i],
            self.gps[i]
        )
    }

    // pub fn append(&mut self, coord: Coordinate, timestamp: i32){
    //     let rate=60;
    //     if (timestamp==self.last_timestamp)
    //      | (timestamp-self.last_timestamp<rate)
    //         {return};

    //     // if self.can_skip(100, coord.clone(), timestamp, self.speed.last().unwrap().clone(), self.bearing.last().unwrap().clone()){return}

    //     let speed_now = self.calculate_speed(&coord, &timestamp);
    //     let bearing_now = self.calculate_bearing(&coord);

    //     // if speed_now>50.0{return};

    //     self.resample(rate, &timestamp, speed_now, bearing_now);

    //     //    self.insert_unbounded(coord, timestamp, self.speed.last().unwrap().clone(), self.bearing.last().unwrap().clone());

    // }

    pub fn calculate_speed(&self, coord: &Coordinate, timestamp: &i32) -> f32 {
        self.coordinates.last().unwrap().haversine(coord) * 3600.0
            / (timestamp - self.timestamps.last().unwrap()) as f32
    }

    pub fn calculate_bearing(&self, coord: &Coordinate) -> f32 {
        self.coordinates.last().unwrap().bearing(coord)
    }

    pub fn resample(&self, rate: i32, timestamp: &i32, sp: f32, br: f32) -> Vec<(Coordinate, i32)> {
        // eprintln!("{} - {} - {}", (timestamp-self.last_timestamp)/rate, timestamp, self.last_timestamp);
        let mut coords = vec![];
        for i in 0..(timestamp - self.timestamps.last().unwrap()) / rate {
            let new_coord = self
                .coordinates
                .last()
                .unwrap()
                .extrapolate(sp, br, rate * (i + 1));
            // self.insert_unbounded(new_coord, self.last_timestamp+rate, sp, br);
            coords.push((new_coord, self.timestamps.last().unwrap() + rate * (i + 1)))
        }
        coords
    }

    pub fn extrapolate_next(&self, dt: i32) -> Coordinate {
        self.coordinates.last().unwrap().extrapolate(
            self.speed.last().unwrap().to_owned(),
            self.bearing.last().unwrap().to_owned(),
            dt,
        )
    }

    pub fn OPW_TR(&self, coord: &Coordinate, timestamp: i32) -> Option<usize> {
        fn _calc_SED(
            pnt_s: &Coordinate,
            ts_s: i32,
            pnt_m: &Coordinate,
            ts_m: i32,
            pnt_e: &Coordinate,
            ts_e: i32,
        ) -> f32 {
            let numerator = ts_m - ts_s;
            let denominator = ts_e - ts_s;

            let time_ratio = if denominator != 0 {
                numerator / denominator
            } else {
                1
            };

            let x_value = pnt_s.x + (pnt_e.x - pnt_s.x) * time_ratio as f32;
            let y_value = pnt_s.y + (pnt_e.y - pnt_s.y) * time_ratio as f32;

            ((x_value - pnt_m.x).powi(2) + (y_value - pnt_m.y).powi(2)).sqrt()
        }

        if self.coordinates.len() < 2 {
            return None;
        }

        for (mid_id, (mid_coord, mid_ts)) in self.coordinates[1..]
            .iter()
            .zip(self.timestamps[1..].iter())
            .enumerate()
        {
            let err_sed = _calc_SED(
                self.coordinates.first().unwrap(),
                self.timestamps.first().unwrap().clone(),
                mid_coord,
                mid_ts.to_owned(),
                coord,
                timestamp.clone(),
            );
            if err_sed > crate::OPW_EPSILON {
                return Some(mid_id + 1);
            }
        }

        //     int i = originalIndex + 1;
        //     bool condOPW = true;
        //     while (i < e && condOPW){
        //         if (cacl_SED(points[originalIndex], points[i], coord) > eplision)
        //             originalIndex = i;
        //         return (originalIndex);
        //         else
        //             i++;
        //     }
        return None;
    }
}

#[derive(Debug, Clone)]
pub struct TrajCollection {
    pub object: HashMap<i32, Trajectory>,
}

impl std::fmt::Display for TrajCollection {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:?}", self.object)
    }
}

impl TrajCollection {
    // pub fn append(&mut self, record: Record){
    //     match self.object.entry(record.oid) {
    //         Entry::Vacant(e) => { e.insert(Trajectory::new(record.oid, Coordinate{x: record.lon, y: record.lat}, record.t)); },
    //         Entry::Occupied(mut e) => { e.get_mut().append(Coordinate{x: record.lon, y: record.lat}, record.t); }
    //     }
    // }

    pub fn extend_flush(&mut self, trajectory: Trajectory, n_opt: Option<usize>) {
        match self.object.entry(trajectory.oid) {
            Entry::Vacant(e) => {
                e.insert(trajectory);
            }
            Entry::Occupied(mut e) => {
                e.get_mut().extend(trajectory);
                match n_opt {
                    Some(n) => e.get_mut().drop_first_n(n),
                    _ => (),
                }
            }
        }
    }

    pub fn pretty(&self) {
        for (_, trajec) in self.object.clone().into_iter() {
            for i in 0..trajec.speed.len() {
                println!(
                    "{:.3}\t{:.3}\t{:.3}\t{:.3}\t{}\t{}\t{}",
                    trajec.coordinates[i].x,
                    trajec.coordinates[i].y,
                    trajec.speed[i],
                    trajec.bearing[i],
                    trajec.stoped[i],
                    trajec.trips[i],
                    trajec.timestamps[i]
                )
            }
        }
    }

    pub fn to_csv(&self) {
        println!("oid,lon,lat,speed,bearing,stoped,trip,timestamp,poi_id");
        for (o_id, trajec) in self.object.clone().into_iter() {
            for i in 0..trajec.speed.len() {
                println!(
                    "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    o_id,
                    trajec.coordinates[i].x,
                    trajec.coordinates[i].y,
                    trajec.speed[i],
                    trajec.bearing[i],
                    trajec.stoped[i],
                    trajec.trips[i],
                    trajec.timestamps[i],
                    trajec.pois[i]
                )
            }
        }
    }

    pub fn concat(&self, trajcol: TrajCollection) {
        todo!()
    }

    pub fn flocks(
        &self,
        coord: &Coordinate,
        speed: f32,
        bearing: f32,
        timestamp: i32,
        my_oid: i32,
    ) -> Vec<i32> {
        let mut flocked_oids = vec![];
        if speed > crate::STOP_SPEED_THR {
            for (oid, traj) in self.object.iter() {
                if oid.to_owned() == my_oid || traj.stoped.last().unwrap().to_owned() == 1 {
                    continue;
                };
                let dt = timestamp - traj.timestamps.last().unwrap();
                let db = (bearing - traj.bearing.last().unwrap()).abs();
                if dt > crate::FLOCKS_MAX_DT_THRESHOLD || db > crate::FLOCKS_MAX_BEARING_THRESHOLD {
                    continue;
                }
                // todo fix this
                let extrapolated = traj.extrapolate_next(dt);
                if coord.haversine(&extrapolated) < crate::FLOCKS_DISTANCE_THRESHOLD {
                    flocked_oids.push(oid.to_owned())
                }
            }
        }
        flocked_oids
    }

    pub fn predict_for_oid(&self, oid: i32, model: &CModule) -> Option<Coordinate> {
        // eprintln!();

        let traj = self.object.get(&oid)?;

        if traj.coordinates.len() < 13 {
            return None;
        }

        let mut xs = vec![];
        let mut ys = vec![];
        let mut ts = vec![];

        let ft_to_m = Proj::new_known_crs("EPSG:4326", "EPSG:3857", None).unwrap();

        let mut data = vec![];

        for coord in traj.coordinates[traj.coordinates.len() - 13..].iter() {
            let projected: (f32, f32) = ft_to_m.convert((coord.x, coord.y)).unwrap();
            xs.push(projected.0);
            ys.push(projected.1);
        }

        for timestamp in traj.timestamps[traj.timestamps.len() - 13..].iter() {
            ts.push(timestamp.clone());
        }

        let tmp = ts
            .iter()
            .zip(ts.iter().skip(1))
            .map(|(tsa, tsb)| (tsb - tsa) as f32 / 1800.0)
            .collect::<Vec<_>>();

        for (a, b, c, d) in izip!(
            tmp[1..11].iter(),
            tmp[2..12].iter(),
            xs.iter().skip(2).zip(xs.iter().skip(3)),
            ys.iter().skip(2).zip(ys.iter().skip(3))
        ) {
            data.push(a.to_owned() as f32);
            data.push(b.to_owned() as f32);
            data.push((c.1 - c.0 - 0.604) / 245.366);
            data.push((d.1 - d.0 - 1.619) / 232.757);
        }
        // let mut binding = data.as_mut_slice().chunks_mut(11).collect::<Vec<_>>().as_slice();
        // let data = binding.as_slice();

        let output = Vec::<f32>::from(
            model
                .forward_ts(&[
                    Tensor::of_slice(data.as_slice()).reshape(&[1, 10, 4]),
                    Tensor::of_slice(&[1]),
                ])
                .unwrap(),
        );

        let (predlondiff, predlatdiff) = (output[0] * 245.366 + 0.604, output[1] * 232.757 + 1.619);

        let predlon = xs.last().unwrap() + predlondiff;
        let predlat = ys.last().unwrap() + predlatdiff;

        let m_to_deg = Proj::new_known_crs("EPSG:3857", "EPSG:4326", None).unwrap();

        Some(Coordinate::from_tuple(
            m_to_deg.convert((predlon, predlat)).unwrap(),
        ))
    }
}

pub struct Pois {
    pub pois: Vec<Coordinate>,
}

impl Pois {
    pub fn new_from_path(path: &str) -> Pois {
        let mut reader_pois = csv::Reader::from_path(path).unwrap();

        let mut pois_list = vec![];

        for record in reader_pois.deserialize() {
            // for record in tqdm!(reader.deserialize()) {
            let record: Coordinate = record.unwrap();
            // records.push(record);
            pois_list.push(record);
        }
        Pois { pois: pois_list }
    }

    pub fn pretty(&self) {
        for poi in self.pois.iter() {
            println!("{},{}", poi.x, poi.y);
        }
    }

    pub fn nearest(&self, coord: &Coordinate, threshold: f32) -> i32 {
        let (poi_id, distance) = self
            .pois
            .iter()
            .map(|pnt| pnt.haversine(&coord))
            .enumerate()
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(index, dist)| (index, dist))
            .unwrap();
        if distance < threshold {
            poi_id as i32
        } else {
            -1
        }
    }
}
