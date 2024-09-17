use crate::{cache::TimeCacheInterface, transform_storage::TransformStorage, types::CompactFrameID};


#[derive(Debug, Clone, Copy)]
pub struct StaticCache {
    storage: TransformStorage,
}

impl TimeCacheInterface for StaticCache {
    fn get_data(
        &self,
        time: u64,
    ) -> Result<TransformStorage, crate::error::TF2Error> {
        let mut data_out = self.storage;
        data_out.stamp = time;
        Ok(data_out)
    }

    fn insert_data(
        &mut self,
        new_data: &TransformStorage,
    ) -> bool {
        self.storage = *new_data;
        true
    }

    fn clear_list(&mut self) {}

    fn get_parent(
        &self,
        _time: u64,
    ) -> Result<Option<CompactFrameID>, crate::error::TF2Error> {
        Ok(Some(self.storage.frame_id))
    }

    fn get_latest_timestamp(&self) -> Option<u64> {
        None
    }

    fn get_latest_timestamp_and_parent(&self) -> Option<(u64, CompactFrameID)> {
        Some((0, self.storage.frame_id))
    }

    fn get_oldest_timestamp(&self) -> Option<u64> {
        None
    }

    fn len(&self) -> usize {
        1
    }
}

impl StaticCache {
    pub fn new(
        storage: TransformStorage,
    ) -> Self {
        Self {
            storage
        }
    }


    pub fn get_list_length(&self) -> usize {
        1
    }

    pub fn get_latest_time_and_parent(&self) -> (Option<u64>, CompactFrameID) {
        (self.get_latest_timestamp(), self.storage.frame_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_item(nanosec: u64, frame_id: u32) -> TransformStorage {
        TransformStorage::new(
            [0., 0., 0., 1.],
            [0., 0., 0.],
            nanosec,
            frame_id,
            0,
        )
    }

    #[test]
    fn test_static_cache_repeatability() {
        let runs = 100;

        let dummy_transform_store = make_item(0, 0);
        let mut cache = StaticCache::new(dummy_transform_store);

        for i in 1..runs {
            cache.insert_data(&make_item(i, i as u32));

            let stor = cache.get_data(i as u64).unwrap();
            assert_eq!(stor.frame_id, i as u32);
            assert_eq!(stor.stamp, i as u64);
        }
    }

    #[test]
    fn test_static_cache_duplicate_entries() {
        let stor = make_item(1, 3);
        let mut cache = StaticCache::new(stor);

        cache.insert_data(&stor);

        let stor_out = cache.get_data(1).unwrap();

        assert_eq!(stor_out.frame_id, stor.frame_id);
        assert_eq!(stor_out.stamp, stor.stamp);
    }
}