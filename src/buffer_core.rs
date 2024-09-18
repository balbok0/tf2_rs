use std::{collections::{HashMap, HashSet}, u64};

use parking_lot::RwLock;

use crate::{cache::{TimeCache, TimeCacheInterface}, error::TF2Error, static_cache::StaticCache, transform_accumulator::TransformAccumulator, transform_storage::TransformStorage, types::CompactFrameID};

const MAX_GRAPH_DEPTH: usize = 1000usize;

// NOTE: For now use r2r. Will make it into a trait later
pub trait TransformStamped {
    fn stamp(&self) -> u64;
    fn frame_id(&self) -> &str;
    fn child_frame_id(&self) -> &str;
    fn translation(&self) -> &[f64; 3];
    fn rotation(&self) -> &[f64; 4];
}


pub struct BufferCore {
    frames_: RwLock<Vec<Box<dyn TimeCacheInterface>>>,
    frame_to_id: HashMap<String, CompactFrameID>,
    id_to_frame: Vec<String>,
    frame_authority: HashMap<CompactFrameID, String>,
    cache_time_ns: u64,
}

impl BufferCore {
    pub fn new(cache_time_ns: u64) -> Self {
        let mut frame_to_id = HashMap::new();
        frame_to_id.insert("NO_PARENT".to_string(), 0);
        Self {
            frames_: RwLock::new(vec![Box::new(StaticCache::new(TransformStorage::new(
                [0., 0., 0., 1.], [0., 0., 0.], 0, 0, 0
            )))]),
            frame_to_id,
            id_to_frame: vec!["NO_PARENT".to_string()],
            frame_authority: HashMap::new(),
            cache_time_ns: cache_time_ns,
        }

    }

    pub fn set_transform(
        &mut self,
        transform: &dyn TransformStamped,
        authority: &str,
        is_static: bool,
    ) -> Result<(), TF2Error> {
        let child_frame_id = transform.child_frame_id().trim_start_matches('/');
        let frame_id = transform.frame_id().trim_start_matches('/');

        if child_frame_id == frame_id {
            return Err(TF2Error::MatchingFrameIDs(authority.to_string(), child_frame_id.to_string()));
        }

        if child_frame_id.is_empty() {
            return Err(TF2Error::EmptyFrameID(authority.to_string(), "child_frame_id"));
        }

        if frame_id.is_empty() {
            return Err(TF2Error::EmptyFrameID(authority.to_string(), "frame_id"));
        }

        let frame_num = self.get_or_insert_new_frame(frame_id, is_static);
        let child_frame_num = self.get_or_insert_new_frame(child_frame_id, is_static);

        {
            let mut frames_lock = self.frames_.write();
            // unwrap is ok, because 2 lines above insert the store
            let frame = frames_lock.get_mut(child_frame_num as usize).unwrap();

            frame.insert_data(&TransformStorage::new(
                *transform.rotation(),
                *transform.translation(),
                transform.stamp(),
                frame_num,
                child_frame_num,
            ));
            self.frame_authority.insert(child_frame_num, authority.to_string());
        }

        Ok(())
    }

    pub fn get_or_insert_new_frame(&mut self, frame_id: &str, is_static: bool) -> CompactFrameID {
        let frame_id = frame_id.trim_start_matches('/');

        if let Some(result) = self.frame_to_id.get(frame_id) {
            *result
        } else {
            let mut frames = self.frames_.write();
            let new_id = frames.len() as CompactFrameID;

            if is_static {
                frames.push(Box::new(StaticCache::new(TransformStorage::identity(u64::MAX, 0, new_id))));
            } else {
                frames.push(Box::new(TimeCache::new(Vec::new(), self.cache_time_ns)));
            }
            self.frame_to_id.insert(frame_id.to_string(), new_id);
            self.id_to_frame.push(frame_id.to_string());

            new_id as CompactFrameID
        }
    }

    pub fn clear(&mut self) {
        let mut tfs = self.frames_.write();

        let mut idx = 1;
        while idx < tfs.len() {
            tfs[idx].clear_list();
            idx += 1;
        }
    }

    fn lookup_frame_number(&self, frame_id: &str) -> Option<CompactFrameID> {
        self.frame_to_id.get(frame_id).copied()
    }

    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: u64,
    ) -> Result<TransformStorage, TF2Error> {

        // Same frame, return identity
        let target_id = self.lookup_frame_number(target_frame).ok_or_else(
            || TF2Error::UnknownFrameID(target_frame.to_string())
        )?;
        if target_frame == source_frame {
            let stamp = if time == 0 {
                self.frames_.read().get(target_id as usize).unwrap().get_latest_timestamp().unwrap_or(0)
            } else {
                time
            };

            return Ok(TransformStorage::identity(stamp, target_id, target_id))
        }

        // Get frame ids
        let source_id = self.lookup_frame_number(source_frame).ok_or_else(
            || TF2Error::UnknownFrameID(source_frame.to_string())
        )?;

        self.walk_to_top_parent(time, target_id, source_id)
    }

    pub fn lookup_transform_full(
        &self,
        target_frame: &str,
        target_time: u64,
        source_frame: &str,
        source_time: u64,
        fixed_frame: &str,
    ) -> Result<TransformStorage, TF2Error> {
        let fixed_to_target = self.lookup_transform(target_frame, fixed_frame, target_time)?;
        let source_to_fixed = self.lookup_transform(fixed_frame, source_frame, source_time)?;

        let mut result = source_to_fixed * fixed_to_target;
        // Expected timestamp is that of a target
        result.stamp = target_time;

        Ok(result)
    }

    pub fn can_transform(&self, target_frame: &str, source_frame: &str, fixed_frame: &str, time: u64) -> bool {
        // Get ids
        let (target_id, source_id, fixed_id) = {
            // Acquire lock
            let _lock = self.frames_.read();

            // Get frame ids
            let target_id = if let Some(target_id) = self.frame_to_id.get(target_frame) {
                *target_id
            } else {
                return false;
            };
            let source_id = if let Some(source_id) = self.frame_to_id.get(source_frame) {
                *source_id
            } else {
                return false;
            };
            let fixed_id = if let Some(fixed_id) = self.frame_to_id.get(fixed_frame) {
                *fixed_id
            } else {
                return false;
            };

            // Check for time bounds
            (target_id, source_id, fixed_id)
        };

        for (t_id, s_id) in [
            (target_id, fixed_id),
            (fixed_id, source_id),
        ] {
            if let Some(time_bounds) = self.get_common_time_bounds(t_id, s_id) {
                // If the time is outside of the time bounds then we cannot transform
                if time_bounds.0 > time || time > time_bounds.1 {
                    return false;
                }
            } else {
                return false;
            }
        }

        true
    }

    pub fn get_all_frame_names(&self) -> Vec<String> {
        let _lock = self.frames_.read();
        self.frame_to_id.keys().cloned().collect()
    }

    pub fn get_cache_length(&self) -> u64 {
        self.cache_time_ns
    }

    pub fn get_latest_authority_for_frame(&self, frame: &str) -> Option<&str> {
        let _lock = self.frames_.read(); // Acquire lock
        Some(self.frame_authority.get(self.frame_to_id.get(frame)?)?.as_str())
    }

    fn walk_to_top_parent(
        &self,
        time: u64,
        target_id: CompactFrameID,
        source_id: CompactFrameID,
    ) -> Result<TransformStorage, TF2Error> {
        if source_id == target_id {
            return Ok(TransformStorage::identity(time, target_id, source_id));
        }

        let time = if time == 0 {
            self.get_common_time_bounds(target_id, source_id).ok_or_else(|| TF2Error::UnknownRelationBetweenFrames(target_id, source_id))?.1
        } else {
            time
        };

        let mut tf_acc = TransformAccumulator::new();

        let mut frame_id = source_id;
        let parent_frame = self.get_closest_shared_ancestor(target_id, source_id).ok_or_else(|| TF2Error::UnknownRelationBetweenFrames(target_id, source_id))?;
        let frames_lock = self.frames_.read();

        while frame_id != parent_frame {
            let frame_cache = frames_lock.get(frame_id as usize).ok_or_else(|| TF2Error::UnknownFrameID(frame_id.to_string()))?;

            let tf_stor = frame_cache.get_data(time)?;

            tf_acc.accum(true, tf_stor);

            frame_id = tf_stor.frame_id;
        }

        let mut frame_id = target_id;
        while frame_id != parent_frame {
            let frame_cache = frames_lock.get(frame_id as usize).ok_or_else(|| TF2Error::UnknownFrameID(frame_id.to_string()))?;

            let tf_stor = frame_cache.get_data(time)?;

            tf_acc.accum(false, tf_stor);

            frame_id = tf_stor.frame_id;
        }

        let (final_translation, final_rotation) = if parent_frame == target_id {
            tf_acc.finalize(crate::transform_accumulator::WalkEnding::TargetParentOfSource)
        } else if parent_frame == source_id {
            tf_acc.finalize(crate::transform_accumulator::WalkEnding::SourceParentOfTarget)
        } else {
            tf_acc.finalize(crate::transform_accumulator::WalkEnding::FullPath)
        };

        let final_tf = TransformStorage::new_from_nalgebra(
            final_rotation,
            final_translation,
            time,
            target_id,
            source_id,
        );

        Ok(final_tf)
    }

    fn get_common_time_bounds(
        &self,
        target_id: CompactFrameID,
        source_id: CompactFrameID,
    ) -> Option<(u64, u64)> {
        if target_id == 0 || source_id == 0 {
            return None;
        }

        // Both frames are the same
        if target_id == source_id {
            let frames_lock = self.frames_.read();
            let target_cache = frames_lock.get(target_id as usize)?;
            return target_cache.get_latest_timestamp().and_then(|x| {
                Some((target_cache.get_oldest_timestamp()?, x))
            });
        }
        let frames_lock = self.frames_.read();

        // Walk from source -> root
        let mut source_to_root_frame_idxs_stamps: HashMap<u32, (u64, u64, u32)> = HashMap::new();

        let mut frame_id = source_id;
        let mut loop_depth = 0;
        let mut path_latest_time = u64::MAX;
        let mut path_oldest_time = 0;
        while frame_id != 0 {
            let frame_cache = frames_lock.get(frame_id as usize)?;
            let (frame_latest_time, frame_parent) = match frame_cache.get_latest_timestamp_and_parent() {
                Some(x) => x,
                // Top most frame of this tree. Point at the root
                None => (path_latest_time, 0),
            };
            // None case is handled above
            let frame_oldest_time = frame_cache.get_oldest_timestamp().unwrap_or(0);

            if frame_id == target_id {
                // Break. Frame found
                return Some((
                    path_oldest_time.max(
                        source_to_root_frame_idxs_stamps
                         .values()
                             .map(|x| x.0 as u64)
                             .max()
                             .unwrap_or(path_oldest_time),
                    ),
                    path_latest_time.min(
                        source_to_root_frame_idxs_stamps
                        .values()
                            .map(|x| x.1 as u64)
                            .min()
                            .unwrap_or(path_latest_time)
                    )
                ));
            }

            // Insert timestamp to get to the current frame
            source_to_root_frame_idxs_stamps.insert(frame_id, (path_oldest_time, path_latest_time, frame_parent));
            // Update timestamp to get to the parent frame
            path_latest_time = path_latest_time.min(frame_latest_time);
            path_oldest_time = path_oldest_time.max(frame_oldest_time);

            // Update frame id for next iteration
            frame_id = frame_parent;
            loop_depth += 1;

            // Disallow really large depth
            if loop_depth > MAX_GRAPH_DEPTH {
                // TODO: Probably should return Result then
                return None;
            }
        }

        // Walk from target to root parent
        let mut frame_id = target_id;
        let mut loop_depth = 0;
        let mut path_latest_time = u64::MAX;
        let mut path_oldest_time = 0;
        while frame_id != 0 {
            if let Some((source_path_oldest_time, source_path_latest_time, _frame_parent)) = source_to_root_frame_idxs_stamps.get(&frame_id) {
                return Some((path_oldest_time.max(*source_path_oldest_time), path_latest_time.min(*source_path_latest_time)));
            }

            let frame_cache = frames_lock.get(frame_id as usize)?;
            let (frame_latest_time, frame_parent) = match frame_cache.get_latest_timestamp_and_parent() {
                Some(x) => x,
                // Top most frame of this tree. Point at the root
                None => (path_latest_time, 0),
            };
            let frame_oldest_time = frame_cache.get_oldest_timestamp().unwrap_or(0);
            path_latest_time = path_latest_time.min(frame_latest_time);
            path_oldest_time = path_oldest_time.max(frame_oldest_time);

            // Update frame id for next iteration
            frame_id = frame_parent;
            loop_depth += 1;

            // Disallow really large depth
            if loop_depth > MAX_GRAPH_DEPTH {
                // TODO: Probably should return Result then
                return None;
            }
        }

        // No common frame found
        None
    }

    fn get_closest_shared_ancestor(
        &self,
        target_id: CompactFrameID,
        source_id: CompactFrameID,
    ) -> Option<CompactFrameID> {
        if target_id == 0 || source_id == 0 {
            return None;
        }

        // Both frames are the same
        if target_id == source_id {
            return Some(source_id);
        }
        let frames_lock = self.frames_.read();

        // Walk from source -> root
        let mut source_to_root_frame_idxs_stamps = HashSet::new();

        let mut frame_id = source_id;
        let mut loop_depth = 0;
        while frame_id != 0 {
            let frame_cache = frames_lock.get(frame_id as usize)?;
            let frame_parent = match frame_cache.get_latest_timestamp_and_parent() {
                Some(x) => x.1,
                // Top most frame of this tree. Point at the root
                None => 0,
            };

            if frame_id == target_id {
                // Break. Frame found
                return Some(frame_id);
            }

            // Update frame id for next iteration
            source_to_root_frame_idxs_stamps.insert(frame_id);
            frame_id = frame_parent;
            loop_depth += 1;

            // Disallow really large depth
            if loop_depth > MAX_GRAPH_DEPTH {
                // TODO: Probably should return Result then
                return None;
            }
        }

        // Walk from target to root parent
        let mut frame_id = target_id;
        let mut loop_depth = 0;
        while frame_id != 0 {
            if source_to_root_frame_idxs_stamps.contains(&frame_id) {
                return Some(frame_id);
            }

            let frame_cache = frames_lock.get(frame_id as usize)?;
            let frame_parent = match frame_cache.get_latest_timestamp_and_parent() {
                Some(x) => x.1,
                // Top most frame of this tree. Point at the root
                None => 0,
            };

            // Update frame id for next iteration
            frame_id = frame_parent;
            loop_depth += 1;

            // Disallow really large depth
            if loop_depth > MAX_GRAPH_DEPTH {
                // TODO: Probably should return Result then
                return None;
            }
        }

        // No common frame found
        None
    }
}

// Write tests for this module here.
#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use approx::assert_relative_eq;
    use nalgebra::{Quaternion, UnitQuaternion};
    use serde::de::Deserialize;

    use super::*;

    fn fake_buffer_core<'a>() -> BufferCore {
        let bc = BufferCore::new(10);

        bc
    }

    #[test]
    fn test_new() {
        for i in [1, 50, u64::MAX] {
            let buffer_core = BufferCore::new(i);

            // Assert assignment of var is correct
            assert_eq!(buffer_core.cache_time_ns, i);

            // Verify integrity
            assert_eq!(buffer_core.frame_to_id.len(), buffer_core.id_to_frame.len());
            assert_eq!(buffer_core.frames_.read().len(), buffer_core.id_to_frame.len());
        }
    }

    // Mock implementation of transform stamped
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct MockTransformStamped<'a>  {
        stamp: u64,
        frame_id: &'a str,
        child_frame_id: &'a str,
        translation: [f64; 3],
        rotation: [f64; 4],
    }

    impl<'a> TransformStamped for MockTransformStamped<'a> {
        fn stamp(&self) -> u64 {
            self.stamp
        }

        fn frame_id(&self) -> &str {
            &self.frame_id
        }

        fn child_frame_id(&self) -> &str {
            &self.child_frame_id
        }

        fn translation(&self) -> &[f64; 3] {
            &self.translation
        }

        fn rotation(&self) -> &[f64; 4] {
            &self.rotation
        }
    }

    impl<'a> MockTransformStamped<'a> {
        fn identity(stamp: u64, frame_id: &'a str, child_frame_id: &'a str) -> MockTransformStamped<'a> {
            return MockTransformStamped {
                stamp: stamp,
                frame_id: frame_id,
                child_frame_id: child_frame_id,
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0]
             }
        }
    }

    #[test]
    fn test_get_or_insert_new_frame() {
        let mut bc = fake_buffer_core();
        let mut frame_count = 1; // Init root frame w/ no parents

        // First insertion
        bc.get_or_insert_new_frame("frame1", false);
        assert_eq!(bc.frame_to_id.len(), frame_count + 1);
        assert!(bc.frame_to_id.contains_key("frame1"));
        frame_count += 1;

        // Re-insert
        bc.get_or_insert_new_frame("frame1", false);
        assert_eq!(bc.frame_to_id.len(), frame_count);

        // Re-insert w/ starting "/""
        bc.get_or_insert_new_frame("/frame1", false);
        assert_eq!(bc.frame_to_id.len(), frame_count);

        // New insertion
        bc.get_or_insert_new_frame("frame2", true);
        assert_eq!(bc.frame_to_id.len(), frame_count + 1);
        assert!(bc.frame_to_id.contains_key("frame2"));
        frame_count += 1;

        // Re-insert first frame
        bc.get_or_insert_new_frame("frame1", false);
        assert_eq!(bc.frame_to_id.len(), frame_count);

        // Insert a bunch of new frames
        for i in 10..15 {
            let key = format!("frame{}", i);
            bc.get_or_insert_new_frame(key.as_str(), false);
            assert_eq!(bc.frame_to_id.len(), frame_count + 1);
            assert!(bc.frame_to_id.contains_key(key.as_str()));
            frame_count += 1;
        }
        // Insert a bunch of static frames
        for i in 10..15 {
            let key = format!("frame-static-{}", i);
            bc.get_or_insert_new_frame(key.as_str(), true);
            assert_eq!(bc.frame_to_id.len(), frame_count + 1);
            assert!(bc.frame_to_id.contains_key(key.as_str()));
            frame_count += 1;
        }

        // Colliding static. Since frame already exists, is_static is ignored
        bc.get_or_insert_new_frame("frame1", true);
        assert_eq!(bc.frame_to_id.len(), frame_count);


        // Empty string
        bc.get_or_insert_new_frame("", true);
        assert!(bc.frame_to_id.contains_key(""));
        frame_count += 1;

        // Just a slash
        bc.get_or_insert_new_frame("/", true);
        assert_eq!(bc.frame_to_id.len(), frame_count);
    }


    #[test]
    fn test_set_transform() {
        let mut bc = fake_buffer_core();
        let mut num_frames = 1;

        // Simple insertion
        let transform = MockTransformStamped::identity(0, "parent", "child");
        let result = bc.set_transform(&transform, "", false);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        // Verify that child frame is present in the map
        assert!(bc.frame_to_id.contains_key("child"));
        assert_eq!(bc.frame_to_id.len(), num_frames + 2);
        num_frames += 2;

        // Repeat insertion
        let result = bc.set_transform(&transform, "", false);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        // No new frame added
        assert_eq!(bc.frame_to_id.len(), num_frames);

        // Insert a new child frame
        let transform = MockTransformStamped::identity(0, "parent", "child2");
        let result = bc.set_transform(&transform, "", false);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        assert!(bc.frame_to_id.contains_key("child2"));
        assert_eq!(bc.frame_to_id.len(), num_frames + 1);
        num_frames += 1;

        // Reinsert first frame
        let transform = MockTransformStamped::identity(0, "parent", "child");
        let result = bc.set_transform(&transform, "", false);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        assert_eq!(bc.frame_to_id.len(), num_frames);

        // Verify that static doesn't matter on re-insert
        let transform = MockTransformStamped::identity(0, "parent", "child");
        let result = bc.set_transform(&transform, "", true);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        assert_eq!(bc.frame_to_id.len(), num_frames);

        // Add a completely separate parent
        let transform = MockTransformStamped::identity(0, "parent-2", "child-2");
        let result = bc.set_transform(&transform, "", true);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        // Verify that child frame is present in the map
        assert!(bc.frame_to_id.contains_key("child-2"));
        assert_eq!(bc.frame_to_id.len(), num_frames + 2);
        num_frames += 2;


        // Insert a top level frame as parent
        let transform = MockTransformStamped::identity(0, "NO_PARENT", "parent");
        let result = bc.set_transform(&transform, "", true);
        assert!(!result.is_err(), "Set transform failed with {result:?}");
        assert_eq!(bc.frame_to_id.len(), num_frames);


        // Empty frames
        let transform = MockTransformStamped::identity(0, "", "parent");
        let result = bc.set_transform(&transform, "a", true);
        assert_eq!(result, Err(TF2Error::EmptyFrameID("a".to_string(), "frame_id")));

        let transform = MockTransformStamped::identity(0, "parent", "");
        let result = bc.set_transform(&transform, "a", true);
        assert_eq!(result, Err(TF2Error::EmptyFrameID("a".to_string(), "child_frame_id")));

        // Matching frame name
        let transform = MockTransformStamped::identity(0, "parent", "/parent");
        let result = bc.set_transform(&transform, "a", true);
        assert_eq!(result, Err(TF2Error::MatchingFrameIDs("a".to_string(), "parent".to_string())));
    }

    #[test]
    fn test_get_common_time_bounds() {
        let mut bc = BufferCore::new(10);
        let transform1 = MockTransformStamped::identity(1, "a", "b1");
        bc.set_transform(&transform1, "a", false).unwrap();

        let transform2 = MockTransformStamped::identity(1, "a", "c1");
        bc.set_transform(&transform2, "a", false).unwrap();

        // Common parent
        let result = bc.get_common_time_bounds(bc.frame_to_id["c1"], bc.frame_to_id["b1"]);
        assert_eq!(result, Some((1, 1)));

        // Slightly longer chain
        for i in 2..5 {
            let frame_id = format!("c{}", i - 1);
            let child_frame_id = format!("c{}", i);
            let transform2 = MockTransformStamped::identity(i, &frame_id, &child_frame_id);
            bc.set_transform(&transform2, "a", false).unwrap();
        }

        let result = bc.get_common_time_bounds(bc.frame_to_id["c4"], bc.frame_to_id["b1"]);
        assert_eq!(result, Some((4, 1)));

        // Same frame
        let result = bc.get_common_time_bounds(bc.frame_to_id["c4"], bc.frame_to_id["c4"]);
        assert_eq!(result, Some((4, 4)));

        // Target is direct parent of source. Child -> timestamp is defined for t=4
        let result = bc.get_common_time_bounds(bc.frame_to_id["c3"], bc.frame_to_id["c4"]);
        assert_eq!(result, Some((4, 4)));

        // Target is direct child of source
        let result = bc.get_common_time_bounds(bc.frame_to_id["c4"], bc.frame_to_id["c3"]);
        assert_eq!(result, Some((4, 4)));

        // Target is a parent of source. Non-direct
        let result = bc.get_common_time_bounds(bc.frame_to_id["c1"], bc.frame_to_id["c4"]);
        assert_eq!(result, Some((4, 2)));

        // Target is a child of source. Non-direct
        let result = bc.get_common_time_bounds(bc.frame_to_id["c4"], bc.frame_to_id["c1"]);
        assert_eq!(result, Some((4, 2)));


        // Make a really large tree.
        for i in 5..MAX_GRAPH_DEPTH + 2 {
            let frame_id = format!("c{}", i - 1);
            let child_frame_id = format!("c{}", i);
            let transform2 = MockTransformStamped::identity(i as u64, &frame_id, &child_frame_id);
            bc.set_transform(&transform2, "a", false).unwrap();
            let transform2 = MockTransformStamped::identity(0, &frame_id, &child_frame_id);
            bc.set_transform(&transform2, "a", false).unwrap();
        }

        // Check with more realistic time bounds
        let result = bc.get_common_time_bounds(bc.frame_to_id["c10"], bc.frame_to_id["c25"]);
        assert_eq!(result, Some((0, 11)));

        // Hit the limit of the tree
        let result = bc.get_common_time_bounds(bc.frame_to_id["a"], bc.frame_to_id[format!("c{}", MAX_GRAPH_DEPTH + 1).as_str()]);
        assert_eq!(result, None);

        // And hit it the other way
        let result = bc.get_common_time_bounds( bc.frame_to_id[format!("c{}", MAX_GRAPH_DEPTH + 1).as_str()], bc.frame_to_id["a"]);
        assert_eq!(result, None);

        // Disjointed trees
        let transform2 = MockTransformStamped::identity(1, "a2", "d1");
        bc.set_transform(&transform2, "a", false).unwrap();
        let result = bc.get_common_time_bounds(bc.frame_to_id["d1"], bc.frame_to_id["b1"]);
        assert_eq!(result, None);

        // Walk to the root
        let result = bc.get_common_time_bounds(0, bc.frame_to_id["b1"]);
        assert_eq!(result, None);
        let result = bc.get_common_time_bounds(bc.frame_to_id["b1"], 0);
        assert_eq!(result, None);
    }

    #[test]
    fn test_lookup_transform() {
        let mut bc = BufferCore::new(10);

        let transform1 = MockTransformStamped::identity(1, "a", "b1");
        bc.set_transform(&transform1, "a", false).unwrap();

        let transform2 = MockTransformStamped::identity(1, "a", "c1");
        bc.set_transform(&transform2, "a", false).unwrap();

        // Common parent
        let result = bc.lookup_transform("c1", "b1", 0);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c1"], bc.frame_to_id["b1"])));

        // Slightly longer chain
        for i in 2..5 {
            let frame_id = format!("c{}", i - 1);
            let child_frame_id = format!("c{}", i);
            let transform2 = MockTransformStamped::identity(1, &frame_id, &child_frame_id);
            bc.set_transform(&transform2, "a", false).unwrap();
        }

        let result = bc.lookup_transform("c4", "b1", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c4"], bc.frame_to_id["b1"])));

        // Same frame
        let result = bc.lookup_transform("c4", "c4", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c4"], bc.frame_to_id["c4"])));

        // Target is direct child of source
        let result = bc.lookup_transform("c4", "c3", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c4"], bc.frame_to_id["c3"])));

        // Target is direct parent of source.
        let result = bc.lookup_transform("c3", "c4", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c3"], bc.frame_to_id["c4"])));

        // Target is a parent of source. Non-direct
        let result = bc.lookup_transform("c1", "c4", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c1"], bc.frame_to_id["c4"])));

        // Target is a child of source. Non-direct
        let result = bc.lookup_transform("c4", "c1", 1);
        assert_eq!(result, Ok(TransformStorage::identity(1, bc.frame_to_id["c4"], bc.frame_to_id["c1"])));
    }

    #[test]
    fn test_lookup_transform_with_ref() {
        let mut bc = BufferCore::new(u64::MAX);

        let mut d = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        d.push("tests/auxiliary_files/buffer_core");

        // First read transforms and create the tree
        let mut tf_file = d.clone();
        tf_file.push("lookup_transform_with_ref_transforms.yaml");
        let f = std::fs::File::open(tf_file).unwrap();
        for document in serde_yaml::Deserializer::from_reader(f) {
            let data = serde_yaml::Value::deserialize(document).unwrap();

            let frame_id = data.get("frame_id").unwrap().as_str().unwrap();
            let child_frame_id = data.get("child_frame_id").unwrap().as_str().unwrap();
            let is_static = data.get("is_static").unwrap().as_bool().unwrap();
            let stamp = data.get("timestamp").unwrap().as_u64().unwrap();
            let rotation: Vec<f64> = data.get("rotation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let translation: Vec<f64> = data.get("translation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();

            let tf = MockTransformStamped {
                stamp,
                frame_id,
                child_frame_id,
                translation: translation.try_into().unwrap(),
                rotation: rotation.try_into().unwrap()
            };
            bc.set_transform(&tf, "", is_static).unwrap();
        }

        // Then load inputs and outputs
        let mut inputs_file = d.clone();
        inputs_file.push("lookup_transform_with_ref_inputs.yaml");
        let inputs_f = std::fs::File::open(inputs_file).unwrap();
        let mut outputs_file = d.clone();
        outputs_file.push("lookup_transform_with_ref_outputs.yaml");
        let outputs_f = std::fs::File::open(outputs_file).unwrap();
        for (input_document, output_document) in serde_yaml::Deserializer::from_reader(inputs_f).zip(serde_yaml::Deserializer::from_reader(outputs_f)) {
            let input_data = serde_yaml::Value::deserialize(input_document).unwrap();
            let output_data = serde_yaml::Value::deserialize(output_document).unwrap();

            // Read input
            let frame_id = input_data.get("frame_id").unwrap().as_str().unwrap();
            let child_frame_id = input_data.get("child_frame_id").unwrap().as_str().unwrap();
            let time = input_data.get("time").unwrap().as_u64().unwrap();

            // Read output
            let expected_frame_id = output_data.get("frame_id").unwrap().as_str().unwrap();
            let expected_child_frame_id = output_data.get("child_frame_id").unwrap().as_str().unwrap();
            let expected_stamp = output_data.get("timestamp").unwrap().as_u64().unwrap();
            let expected_rotation: Vec<f64> = output_data.get("rotation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let actual_transform = bc.lookup_transform(frame_id, child_frame_id, time);

            assert!(actual_transform.is_ok(), "Transform errored with {actual_transform:?}");
            let actual_transform = actual_transform.unwrap();
            assert_eq!(bc.id_to_frame[actual_transform.frame_id as usize], expected_frame_id);
            assert_eq!(bc.id_to_frame[actual_transform.child_frame_id as usize], expected_child_frame_id);
            assert!(vec![expected_stamp, time].contains(&actual_transform.stamp));
            assert_relative_eq!(actual_transform.rotation, UnitQuaternion::from_quaternion(Quaternion::new(
                expected_rotation[3],
                expected_rotation[0],
                expected_rotation[1],
                expected_rotation[2],
            )), epsilon = 1e-6);
        }
    }


    #[test]
    fn test_lookup_transform_full_with_ref() {
        let mut bc = BufferCore::new(u64::MAX);

        let mut d = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        d.push("tests/auxiliary_files/buffer_core");

        // First read transforms and create the tree
        let mut tf_file = d.clone();
        tf_file.push("lookup_transform_full_with_ref_transforms.yaml");
        let f = std::fs::File::open(tf_file).unwrap();
        for document in serde_yaml::Deserializer::from_reader(f) {
            let data = serde_yaml::Value::deserialize(document).unwrap();

            let frame_id = data.get("frame_id").unwrap().as_str().unwrap();
            let child_frame_id = data.get("child_frame_id").unwrap().as_str().unwrap();
            let is_static = data.get("is_static").unwrap().as_bool().unwrap();
            let stamp = data.get("timestamp").unwrap().as_u64().unwrap();
            let rotation: Vec<f64> = data.get("rotation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let translation: Vec<f64> = data.get("translation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();

            let tf = MockTransformStamped {
                stamp,
                frame_id,
                child_frame_id,
                translation: translation.try_into().unwrap(),
                rotation: rotation.try_into().unwrap()
            };
            bc.set_transform(&tf, "", is_static).unwrap();
        }

        // Then load inputs and outputs
        let mut inputs_file = d.clone();
        inputs_file.push("lookup_transform_full_with_ref_inputs.yaml");
        let inputs_f = std::fs::File::open(inputs_file).unwrap();
        let mut outputs_file = d.clone();
        outputs_file.push("lookup_transform_full_with_ref_outputs.yaml");
        let outputs_f = std::fs::File::open(outputs_file).unwrap();
        for (input_document, output_document) in serde_yaml::Deserializer::from_reader(inputs_f).zip(serde_yaml::Deserializer::from_reader(outputs_f)) {
            let input_data = serde_yaml::Value::deserialize(input_document).unwrap();
            let output_data = serde_yaml::Value::deserialize(output_document).unwrap();

            // Read input
            let target_frame = input_data.get("target_frame").unwrap().as_str().unwrap();
            let target_time = input_data.get("target_time").unwrap().as_u64().unwrap();
            let source_frame = input_data.get("source_frame").unwrap().as_str().unwrap();
            let source_time = input_data.get("source_time").unwrap().as_u64().unwrap();
            let fixed_frame = input_data.get("fixed_frame").unwrap().as_str().unwrap();

            // Read output
            let expected_frame_id = output_data.get("frame_id").unwrap().as_str().unwrap();
            let expected_child_frame_id = output_data.get("child_frame_id").unwrap().as_str().unwrap();
            let expected_stamp = output_data.get("timestamp").unwrap().as_u64().unwrap();
            let expected_rotation: Vec<f64> = output_data.get("rotation").unwrap().as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let actual_transform = bc.lookup_transform_full(
                target_frame, target_time,
                source_frame, source_time, fixed_frame);

            assert!(actual_transform.is_ok(), "Transform errored with {actual_transform:?}");
            let actual_transform = actual_transform.unwrap();
            assert_eq!(bc.id_to_frame[actual_transform.frame_id as usize], expected_frame_id);
            assert_eq!(bc.id_to_frame[actual_transform.child_frame_id as usize], expected_child_frame_id);
            // Assert timestamps are sufficiently close. This is due to rounding errors I think?
            assert!((expected_stamp as i128 - actual_transform.stamp as i128) < 200);
            assert_relative_eq!(actual_transform.rotation, UnitQuaternion::from_quaternion(Quaternion::new(
                expected_rotation[3],
                expected_rotation[0],
                expected_rotation[1],
                expected_rotation[2],
            )), epsilon = 1e-6);
        }
    }
}
