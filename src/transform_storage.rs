use std::ops;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::types::CompactFrameID;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformStorage {
    pub(crate) rotation: UnitQuaternion<f64>,
    pub(crate) translation: Vector3<f64>,
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
        self.rotation.magnitude().partial_cmp(&other.rotation.magnitude())
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
        Self::new_from_nalgebra(
            UnitQuaternion::from_quaternion(Quaternion::new(
                rotation[3],
                rotation[0],
                rotation[1],
                rotation[2],
            )),
            Vector3::new(translation[0], translation[1], translation[2]),
            stamp,
            frame_id,
            child_frame_id
        )
    }

    pub fn identity(
        stamp: u64,
        frame_id: CompactFrameID,
        child_frame_id: CompactFrameID,
    ) -> Self {
        Self::new_from_nalgebra(
            UnitQuaternion::identity(),
            Vector3::zeros(),
            stamp,
            frame_id,
            child_frame_id
        )
    }

    pub fn new_from_nalgebra(
        rotation: UnitQuaternion<f64>,
        translation: Vector3<f64>,
        stamp: u64,
        frame_id: CompactFrameID,
        child_frame_id: CompactFrameID,
    ) -> Self {
        TransformStorage {
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

        // let translation = [
        //     (first_ratio * first.translation[0]) + (second_ratio * second.translation[0]),
        //     (first_ratio * first.translation[1]) + (second_ratio * second.translation[1]),
        //     (first_ratio * first.translation[2]) + (second_ratio * second.translation[2]),
        // ];
        let translation = first_ratio * first.translation + second_ratio * second.translation;
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
            rotation,
            translation,
            stamp: time,
            frame_id: first.frame_id,
            child_frame_id: second.frame_id,
        }
    }

    pub fn rotate_pos(&self, pos: &[f64; 3]) -> [f64; 3] {
        let v3 = self.rotate_vec(&Vector3::new(pos[0], pos[1], pos[2]));
        [v3.x, v3.y, v3.z]
    }

    pub fn rotate_vec(&self, pos: &Vector3<f64>) -> Vector3<f64> {
        let quat = UnitQuaternion::from_quaternion(Quaternion::new(
            self.rotation[3],
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
        ));
        quat.transform_vector(pos)
    }
}

impl ops::Mul<TransformStorage> for TransformStorage {
    type Output = TransformStorage;

    fn mul(self, rhs: TransformStorage) -> Self::Output {
        // Ensure common frame
        if self.frame_id == rhs.child_frame_id {
            TransformStorage {
                rotation: self.rotation * rhs.rotation,
                translation: self.translation + rhs.translation,
                stamp: self.stamp,
                frame_id: self.frame_id,
                child_frame_id: self.child_frame_id,
            }
        } else if self.child_frame_id == rhs.frame_id {
            rhs.mul(self)
        } else {
            panic!(
                "Could not find common frame. self contains {} -> {} and other contains {} -> {}",
                self.child_frame_id,
                self.frame_id,
                rhs.child_frame_id,
                rhs.frame_id,
            )
        }
    }
}