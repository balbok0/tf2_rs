use chrono::{DateTime, Utc};
use nalgebra::{Quaternion, UnitQuaternion};

use crate::types::CompactFrameID;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformStorage {
    pub(crate) rotation: [f64; 4],
    pub(crate) translation: [f64; 3],
    pub(crate) stamp: u64,
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
        self.rotation.partial_cmp(&other.rotation)
    }
}

impl TransformStorage {
    pub fn new(
        rotation: [f64; 4],
        translation: [f64; 3],
        stamp: u64,
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

    pub fn interpolate(
        first: &Self,
        second: &Self,
        time: u64
    ) -> TransformStorage {
        if first.stamp == second.stamp {
            return *first;
        }
        let second_ratio = (time as f64 - first.stamp as f64) / (second.stamp as f64 - first.stamp as f64);
        let first_ratio = 1. - second_ratio;

        let translation = [
            (first_ratio * first.translation[0]) + (second_ratio * second.translation[0]),
            (first_ratio * first.translation[1]) + (second_ratio * second.translation[1]),
            (first_ratio * first.translation[2]) + (second_ratio * second.translation[2]),
        ];
        let first_quat = UnitQuaternion::from_quaternion(Quaternion::new(
            first.rotation[3],
            first.rotation[0],
            first.rotation[1],
            first.rotation[2],
        ));
        let second_quat = UnitQuaternion::from_quaternion(Quaternion::new(
            second.rotation[3],
            second.rotation[0],
            second.rotation[1],
            second.rotation[2],
        ));

        let rotation = first_quat.slerp(&second_quat, second_ratio);


        TransformStorage {
            rotation: [
                rotation[0],
                rotation[1],
                rotation[2],
                rotation[3],
            ],
            translation,
            stamp: time,
            frame_id: first.frame_id,
            child_frame_id: second.frame_id,
        }
    }
}
