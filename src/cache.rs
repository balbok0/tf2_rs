use derive_builder::Builder;

use crate::{error::TF2Error, transform_storage::TransformStorage, types::CompactFrameID};


pub trait TimeCacheInterface {
    fn get_data(
        &self,
        time: u64,
    ) -> Result<TransformStorage, TF2Error>;

    fn insert_data(
        &mut self,
        new_data: &TransformStorage,
    ) -> bool;

    fn clear_list(&mut self);

    fn get_parent(
        &self,
        time: u64,
    ) -> Result<Option<CompactFrameID>, TF2Error>;

    fn get_latest_timestamp(&self) -> Option<u64>;

    fn get_latest_timestamp_and_parent(&self) -> Option<(u64, CompactFrameID)>;

    fn get_oldest_timestamp(&self) -> Option<u64>;

}

#[derive(Debug, Clone, Builder, PartialEq)]
pub struct TimeCache {
    #[builder(default = "vec![]")]
    storage: Vec<TransformStorage>,
    #[builder(default = "10_000_000_000")]
    max_storage_time_ns: u64,
}

impl TimeCacheInterface for TimeCache {
    fn get_latest_timestamp(&self) -> Option<u64> {
        self.storage.first().map(|tfs| tfs.stamp)
    }

    fn get_latest_timestamp_and_parent(&self) -> Option<(u64, CompactFrameID)> {
        let latest = self.storage.first()?;
        Some((latest.stamp, latest.frame_id))
    }

    fn get_oldest_timestamp(&self) -> Option<u64> {
        self.storage.last().map(|tfs| tfs.stamp)
    }

    fn get_data(
        &self,
        time: u64,
    ) -> Result<TransformStorage, TF2Error> {

        let closest_result = self.find_closest(time)?;

        match closest_result.len() {
            0 => Err(TF2Error::Empty),
            1 => Ok(*closest_result[0]),
            2 => {
                let first = closest_result[0];
                let second = closest_result[1];
                if first.frame_id == second.frame_id {
                    Ok(TransformStorage::interpolate(first, second, time))
                } else {
                    Ok(*first)
                }
            }
            _ => panic!("Never should happen!")
        }
    }

    fn get_parent(
        &self,
        time: u64,
    ) -> Result<Option<CompactFrameID>, TF2Error> {
        let closest_frames = self.find_closest(time)?;
        Ok(closest_frames.first().map(|v| v.frame_id))
    }

    fn insert_data(
        &mut self,
        new_data: &TransformStorage,
    ) -> bool {
        // TODO: Finish me
        // From original TF2: https://github.com/ros2/geometry2/blob/ros2/tf2/src/cache.cpp#L251-L258
        // In order to minimize the number of times we iterate over this data, we:
        // (1) Prune all old data first, regardless if new_data is added,
        // (2) We use find_if to scan from newest to oldest, and stop at the first
        //     point where the timestamp is equal or older to new_data's.
        // (3) From this point, we scan with more expensive full equality checks to
        //     ensure we do not reinsert the same exact data.
        // (4) If we the data is not duplicated, then we simply insert new_data at
        //     the point found in (2).

        // (1) Prune all old data first, regardless if new_data is added,
        self.prune_list();

        // (2) We use sequential search to scan from newest to oldest, and stop at the first
        //     point where the timestamp is equal or older to new_data's.
        //     This is preferred over binary search since most of the timestamps will be inserted close to beginning.
        //     TODO: Dynamically decide which one to do based on some heuristic (new_data timestamp compared to latest and oldest?)
        let mut idx = 0;
        while idx < self.storage.len() && self.storage[idx].stamp > new_data.stamp {
            idx += 1
        }

        // (3) From this point, we scan with more expensive full equality checks to
        //     ensure we do not reinsert the same exact data.
        let mut got_match = false;
        // Traverse from idx forward to see if there is a match
        while !got_match && idx < self.storage.len() && self.storage[idx].stamp == new_data.stamp {
            if &self.storage[idx] == new_data {
                got_match = true;
                break;
            }
            idx += 1;
        }

        // Got a match. Do not insert
        if got_match {
            return false;
        }

        // (4) If we the data is not duplicated, then we simply insert new_data at
        //     the point found in (2).
        self.storage.insert(idx, *new_data);
        true
    }

    fn clear_list(&mut self) {
        self.storage.clear()
    }
}

impl TimeCache {
    pub fn new(
        storage: Vec<TransformStorage>,
        max_storage_time_ns: u64,
    ) -> Self {
        TimeCache {
            storage,
            max_storage_time_ns
        }
    }

    pub fn get_all_items(&self) -> &Vec<TransformStorage> {
        &self.storage
    }

    pub fn get_list_length(&self) -> usize {
        self.storage.len()
    }


    fn find_closest(
        &self,
        target_time: u64,
    ) -> Result<Vec<&TransformStorage>, TF2Error> {
        // If storage is 0 then return empty
        if self.storage.is_empty() {
            return Ok(Vec::new())
        }

        // If t = 0, then return latest transform
        if target_time == 0 {
            return Ok(
                vec![
                    self.storage.first().unwrap()
                ]
            );
        }

        // If length of store is 1, return only if timestamps match exactly
        if self.storage.len() == 1 {
            let cur_ts_storage = self.storage.first().unwrap();
            if target_time == cur_ts_storage.stamp {
                return Ok(vec![
                    cur_ts_storage
                ])
            } else {
                return Err(
                    TF2Error::SingleExtrapolationError(
                        target_time,
                        cur_ts_storage.stamp
                    )
                )
            }
        }

        // Check interpolated doesn't violated bounds
        let earliest_time = self.storage.last().unwrap().stamp;
        let latest_time = self.storage.first().unwrap().stamp;
        if target_time == latest_time {
            return Ok(vec![self.storage.first().unwrap()])
        } else if target_time == earliest_time {
            return Ok(vec![self.storage.last().unwrap()])
        } else if target_time < earliest_time {
            return Err(TF2Error::PastExtrapolationError(
                target_time,
                earliest_time
            ))
        } else if target_time > latest_time {
            return Err(TF2Error::FutureExtrapolationError(
                target_time,
                latest_time
            ))
        }

        // At this point, we are:
        // 1. guaranteed to have two values stored in storage.
        // 2. target time is somewhere within bounds of the
        let mut idx = 0;
        while idx < self.storage.len() && self.storage[idx].stamp > target_time {
            idx += 1
        }

        if self.storage[idx].stamp == target_time {
            Ok(vec![self.storage.get(idx).unwrap()])
        } else {
            Ok(
                vec![
                    self.storage.get(idx).unwrap(),
                    self.storage.get(idx - 1).unwrap(),
                ]
            )
        }
    }

    fn prune_list(&mut self) {
        let cut_ts = match self.get_latest_timestamp() {
            Some(dt) => {
                let acceptable_storage_time = dt as i64 - self.max_storage_time_ns as i64;
                i64::max(0, acceptable_storage_time) as u64
            },
            None => {
                // Nothing to remove, the store is empty
                return
            }
        };

        let mut idx = 0;
        while idx < self.storage.len() && self.storage[idx].stamp >= cut_ts {
            idx += 1
        }

        if idx < self.storage.len() {
            self.storage.truncate(idx)
        }
    }


}


#[cfg(test)]
mod tests {
    use chrono::TimeDelta;
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};
    use rand::{self, Rng, SeedableRng as _};

    use super::*;

    fn make_item(nanosec: i64, frame_id: u32) -> TransformStorage {
        TransformStorage::new(
            [0., 0., 0., 1.],
            [0., 0., 0.],
            nanosec as u64,
            frame_id,
            0,
        )
    }

    fn compare_stores(
        one: &Vec<TransformStorage>,
        two: &Vec<TransformStorage>,
    ) {
        // Ensure both are equal length
        assert_eq!(
            one.len(),
            two.len(),
            "Stores have different lengths: {} and {} but should be the same", one.len(), two.len()
        );

        // If both stores are 0-length we gucci
        if one.is_empty() {
            return;
        }

        // Iterate over stores in parallel. For each it should not matter what order given timestamp is in.
        let mut snapshot_start_idx = 0;
        while snapshot_start_idx < one.len() {
            let mut cur_idx = snapshot_start_idx;
            let snapshot_stamp = one[cur_idx].stamp;

            while cur_idx < one.len() {
                let one_matches = one[cur_idx].stamp == snapshot_stamp;
                let two_matches = two[cur_idx].stamp == snapshot_stamp;

                if one_matches ^ two_matches {
                    // Mismatch, so lengths of sets are different!
                    panic!(
                        "Varying lengths at timestamp {}. Both have {} elements, but next one matches(T/F) for one ({}) or two({})",
                        snapshot_stamp,
                        cur_idx - snapshot_start_idx,
                        one_matches,
                        two_matches
                    )
                }

                // Match append to size of sequence
                cur_idx += 1;

                if !one_matches && !two_matches {
                    // Neither matches: End of sequence
                    break
                }
                // Otherwise it is a match to current sequence, so iterate further
            }

            assert_eq!(
                one.clone()[snapshot_start_idx..cur_idx].sort_by(|a, b| a.partial_cmp(b).unwrap()),
                two.clone()[snapshot_start_idx..cur_idx].sort_by(|a, b| a.partial_cmp(b).unwrap()),
            );

            snapshot_start_idx = cur_idx;
        }

    }


    #[test]
    fn test_time_cache_get_all_items() {
        let max_storage_time_ns = 10;
        let mut cache = TimeCache::new(Vec::new(), max_storage_time_ns) ;

        let item_a = make_item(0, 0);
        let item_b = make_item(10, 1);
        let item_c = make_item(5, 2);
        let item_d = make_item(3, 3);
        // Same timestamp, different id.
        let item_e = make_item(8, 4);
        let item_f = make_item(8, 5);
        let item_g = make_item(8, 6);

        // Insert in order.
        cache.insert_data(&item_a);
        cache.insert_data(&item_b);
        cache.insert_data(&item_c);
        cache.insert_data(&item_d);
        cache.insert_data(&item_e);
        cache.insert_data(&item_f);
        cache.insert_data(&item_g);

        // Note that the difference between the oldest and newest timestamp is exactly equal
        // to the max storage duration.
        assert_eq!(
          cache.get_latest_timestamp().unwrap() - cache.get_oldest_timestamp().unwrap(),
          max_storage_time_ns
        );

        // Expect that storage is descending.
        let storage_expected = vec![
            item_b.clone(),
            // Same timestamps have effectively reversed insertion order.
            item_g.clone(),
            item_f.clone(),
            item_e.clone(),
            // Remaining are in descending order.
            item_c.clone(),
            item_d.clone(),
            item_a.clone()
        ];

        let storage = cache.get_all_items().clone();
        compare_stores(
            &storage,
            &storage_expected,
            // "storage: {:#?}\nstorage_expected: {:#?}\n", storage, storage_expected
        );

        // Insert repeated, in reverse. Nothing should change.
        cache.insert_data(&item_g);
        cache.insert_data(&item_f);
        cache.insert_data(&item_e);
        cache.insert_data(&item_d);
        cache.insert_data(&item_c);
        cache.insert_data(&item_b);
        cache.insert_data(&item_a);

        let storage = cache.get_all_items().clone();
        compare_stores(
            &storage,
            &storage_expected,
            // "storage_repeat: {:#?}\nstorage_expected: {:#?}\n", storage, storage_expected
        );

        // Insert newer data, and expect stale data to be pruned, even if newly inserted.
        let item_h = make_item(15, 7);
        let item_i = make_item(0, 8);  // This will be dropped.
        let item_j = make_item(5, 9);

        cache.insert_data(&item_h);
        cache.insert_data(&item_i);
        cache.insert_data(&item_j);

        let storage_expected_new = vec![
          item_h,
          item_b,
          item_g,
          item_f,
          item_e,
          item_j,
          item_c,
        ];
        // item_a, item_d, and item_i are pruned.
        let storage_new = cache.get_all_items().clone();
        compare_stores(
            &storage_new,
            &storage_expected_new,
            // "storage_new: {:#?}\nstorage_expected_new: {:#?}\n", storage_new, storage_expected_new
        );
    }

    #[test]
    fn test_time_cache_repeatability() {
        let runs = 100;
        let mut cache = TimeCacheBuilder::create_empty()
          .storage(vec![])
          .build()
          .unwrap();
        for i in 1..runs {
          let store = make_item(i, i as u32);
          cache.insert_data(&store);
        }
        assert_eq!(cache.get_list_length(), (runs - 1) as usize);
        for i in 1..runs {
          let stor = cache.get_data(i as u64).unwrap();
          assert_eq!(stor.frame_id, i as u32);
          assert_eq!(stor.stamp, i as u64);
        }
    }

    #[test]
    fn test_time_cache_repeatability_reverse_insert_order() {
        let runs = 100;
        let mut cache = TimeCacheBuilder::create_empty()
          .storage(vec![])
          .build()
          .unwrap();
        for i in (0..runs).rev() {
          let store = make_item(i, i as u32);
          cache.insert_data(&store);
        }
        assert_eq!(cache.get_list_length(), runs as usize);
        for i in 1..runs {
          let stor = cache.get_data(i as u64).unwrap();
          assert_eq!(stor.frame_id, i as u32);
          assert_eq!(stor.stamp, i as u64);
        }
    }

    #[test]
    fn test_time_cache_repeated_elements() {
        let runs = 100;
        let mut cache = TimeCacheBuilder::create_empty()
          .storage(vec![])
          .build()
          .unwrap();

        let store = make_item(0, 0);
        for _ in (0..runs).rev() {
          cache.insert_data(&store);
        }
        assert_eq!(cache.get_list_length(), 1);

    }

    #[test]
    fn test_time_cache_zero_at_front() {
        let runs = 100;

        let mut cache = TimeCacheBuilder::create_empty()
            .storage(vec![])
            .build()
            .unwrap();


        for i in 1..100 {
            cache.insert_data(&make_item(i, i as u32));
        }

        cache.insert_data(&make_item(runs, runs as u32));

        for i in 1..runs {
            let stor = cache.get_data(i as u64).unwrap();

            assert_eq!(stor.frame_id, i as CompactFrameID);
            assert_eq!(stor.stamp, i as u64);
        }

        let mut stor = cache.get_data(0).unwrap();
        assert_eq!(stor.frame_id, runs as u32);
        assert_eq!(stor.stamp, runs as u64);

        stor.frame_id = runs as u32;
        stor.stamp += 1;
        cache.insert_data(&stor);

        // Make sure we get a different value now that a new values is added at the front
        let stor = cache.get_data(0).unwrap();
        assert_eq!(stor.frame_id, runs as CompactFrameID);
        assert_eq!(stor.stamp, (runs + 1) as u64);
    }

    #[test]
    fn test_time_cache_cartesian_interpolation() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(32);

        let runs = 100;
        let epsilon = 2e-6;

        let mut cache = TimeCacheBuilder::create_empty()
            .build()
            .unwrap();
        let mut x_values = vec![0., 0.];
        let mut y_values = vec![0., 0.];
        let mut z_values = vec![0., 0.];

        let offset = 200;

        for _i in 1..runs  {
          for step in 0..2usize {

            x_values[step] = 10.0 * rng.gen::<f64>();
            y_values[step] = 10.0 * rng.gen::<f64>();
            z_values[step] = 10.0 * rng.gen::<f64>();

            let mut stor = make_item(step as i64 * 100 + offset, 2);
            stor.translation = Vector3::new(x_values[step], y_values[step], z_values[step]);
            cache.insert_data(&stor);
          }

          for pos in 0..100 {
            let stor = cache.get_data((offset + pos) as u64).unwrap();
            let x_out = stor.translation[0];
            let y_out = stor.translation[1];
            let z_out = stor.translation[2];

            approx::assert_relative_eq!(x_values[0] + (x_values[1] - x_values[0]) * (pos as f64) / 100.0, x_out, epsilon = epsilon);
            approx::assert_relative_eq!(y_values[0] + (y_values[1] - y_values[0]) * (pos as f64) / 100.0, y_out, epsilon = epsilon);
            approx::assert_relative_eq!(z_values[0] + (z_values[1] - z_values[0]) * (pos as f64) / 100.0, z_out, epsilon = epsilon);
          }
          cache.clear_list();
        }
    }


    #[test]
    fn test_time_cache_reparenting_interpolation_protection() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(32);

        let runs = 100;
        let epsilon = 1e-6;

        let mut cache = TimeCacheBuilder::create_empty()
            .build()
            .unwrap();
        let mut x_values = vec![0., 0.];
        let mut y_values = vec![0., 0.];
        let mut z_values = vec![0., 0.];

        let offset = 555;

        for i in 1..runs  {
          for step in 0..2usize {

            x_values[step] = 10.0 * rng.gen::<f64>();
            y_values[step] = 10.0 * rng.gen::<f64>();
            z_values[step] = 10.0 * rng.gen::<f64>();

            let mut stor = make_item(
                step as i64 * 100 + offset,
                step as u32 + 4
            );
            stor.translation = Vector3::new(x_values[step], y_values[step], z_values[step]);
            cache.insert_data(&stor);
          }

          for pos in 0..100 {
            let stor = cache.get_data((offset + pos) as u64).unwrap();
            let x_out = stor.translation[0];
            let y_out = stor.translation[1];
            let z_out = stor.translation[2];

            approx::assert_relative_eq!(x_values[0], x_out, epsilon = epsilon);
            approx::assert_relative_eq!(y_values[0], y_out, epsilon = epsilon);
            approx::assert_relative_eq!(z_values[0], z_out, epsilon = epsilon);
          }
          cache.clear_list();
          assert!(cache.get_all_items().is_empty());
        }
    }

    #[test]
    fn test_time_cache_angular_interpolation() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(32);

        let runs = 100;
        let epsilon = 1e-6;

        let mut cache = TimeCacheBuilder::create_empty()
            .build()
            .unwrap();
        let mut yaw_values = vec![0., 0.];
        let mut pitch_values = vec![0., 0.];
        let mut roll_values = vec![0., 0.];

        let offset = 200;

        for _i in 1..runs  {
            let mut quats = vec![];
            for step in 0..2usize {

              yaw_values[step] = 0.1 * rng.gen::<f64>();
              pitch_values[step] = 0.1 * rng.gen::<f64>();
              roll_values[step] = 0.1 * rng.gen::<f64>();

              let mut stor = make_item(step as i64 * 100 + offset, 2);
              quats.push(UnitQuaternion::from_euler_angles(yaw_values[step], pitch_values[step], roll_values[step]));
              let last_quat = quats.last().unwrap();
              stor.rotation = last_quat.clone();
              cache.insert_data(&stor);
            }

            for pos in 0..100 {
              let stor = cache.get_data((offset + pos) as u64).unwrap();

              let ground_truth = quats[0].slerp(&quats[1], pos as f64 / 100.0);

              let stor_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
                stor.rotation[3],
                stor.rotation[0],
                stor.rotation[1],
                stor.rotation[2],
              ));

              approx::assert_relative_eq!(0.0, ground_truth.angle_to(&stor_rotation), epsilon = epsilon);
            }
            cache.clear_list();
        }
    }

    #[test]
    fn test_duplicate_entries() {
        let mut cache = TimeCacheBuilder::create_empty().build().unwrap();

        let stor = make_item(1, 3);

        cache.insert_data(&stor);
        assert_eq!(cache.get_list_length(), 1);

        cache.insert_data(&stor);
        // Exact repeated element, should not grow in length.
        assert_eq!(cache.get_list_length(), 1);

        let stor_out = cache.get_data(1);

        assert!(stor_out.is_ok());
    }
}