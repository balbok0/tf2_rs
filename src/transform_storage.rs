use chrono::{DateTime, Utc};
use nalgebra::{UnitQuaternion, Vector3};

use crate::types::CompactFrameID;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct TransformStorage {
    pub(crate) rotation: UnitQuaternion<f64>,
    pub(crate) translation: Vector3<f64>,
    pub(crate) stamp: DateTime<Utc>,
    pub(crate) frame_id: CompactFrameID,
    pub(crate) child_frame_id: CompactFrameID,
}

impl PartialOrd for TransformStorage {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self.stamp.partial_cmp(&other.stamp) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.frame_id.partial_cmp(&other.frame_id) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.child_frame_id.partial_cmp(&other.child_frame_id) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.translation.partial_cmp(&other.translation) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        self.rotation.magnitude().partial_cmp(&other.rotation.magnitude())
    }
}

impl TransformStorage {
    pub fn new(
        rotation: UnitQuaternion<f64>,
        translation: Vector3<f64>,
        stamp: DateTime<Utc>,
        frame_id: CompactFrameID,
        child_frame_id: CompactFrameID,
    ) -> Self {
        Self {
            rotation,
            translation,
            stamp,
            frame_id,
            child_frame_id,
        }
    }

    pub(crate) fn empty(&self) {

    }
}
