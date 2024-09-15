use nalgebra::{AbstractRotation, UnitQuaternion, Vector3};

use crate::{error::TF2Error, transform_storage::TransformStorage, types::CompactFrameID};


#[derive(Debug, Clone, Copy)]
pub(crate) enum WalkEnding {
    Identity,
    TargetParentOfSource,
    SourceParentOfTarget,
    FullPath,
}


pub(crate) struct TransformAccumulator {
    pub(crate) source_to_top_quat: UnitQuaternion<f64>,
    pub(crate) source_to_top_vec: Vector3<f64>,
    pub(crate) target_to_top_quat: UnitQuaternion<f64>,
    pub(crate) target_to_top_vec: Vector3<f64>,

    time: u64,
    st: TransformStorage,
}

impl TransformAccumulator {
    // pub(crate) fn gather(
    //     &self,
    //     cache: &dyn TimeCacheInterface,
    //     time: u64,
    // ) -> Result<Option<CompactFrameID>, TF2Error> {
    //     cache.get_data(time).map(|r| r.map(|v| v.frame_id))
    // }

    pub(crate) fn accum(
        &mut self,
        source: bool
    ) {
        if source {
            self.source_to_top_vec = self.st.rotate_vec(&self.source_to_top_vec) + self.st.translation;
            self.source_to_top_quat = self.st.rotation * self.source_to_top_quat;
        } else {
            self.target_to_top_vec = self.st.rotate_vec(&self.target_to_top_vec) + self.st.translation;
            self.target_to_top_quat = self.st.rotation * self.target_to_top_quat;
        }
    }

    pub(crate) fn finalize(
        &self,
        end: WalkEnding,
    ) -> Option<(Vector3<f64>, UnitQuaternion<f64>)> {
        match end {
            WalkEnding::Identity => {
                Some((Vector3::zeros(), UnitQuaternion::identity()))
            },
            WalkEnding::TargetParentOfSource => {
                Some((self.source_to_top_vec, self.source_to_top_quat))
            },
            WalkEnding::SourceParentOfTarget => {
                let inv_target_quat = self.target_to_top_quat.inverse();
                let inv_target_vec = inv_target_quat.transform_vector(&-self.target_to_top_vec);
                Some((inv_target_vec, inv_target_quat))
            },
            WalkEnding::FullPath => {
                let inv_target_quat = self.target_to_top_quat.inverse();
                let inv_target_vec = inv_target_quat.transform_vector(&-self.target_to_top_vec);

                let result_vec = inv_target_quat.transform_vector(&self.source_to_top_vec) + inv_target_vec;
                let result_quat = inv_target_quat * self.source_to_top_quat;

                Some((result_vec, result_quat))
            }
        }
    }
}


#[cfg(test)]
mod tests {
    // #[test]
    // fn test_accum
}