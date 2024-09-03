use std::{collections::HashMap, sync::Mutex};

use chrono::{DateTime, Duration, Utc};
use nalgebra::Transform3;

use crate::{cache::TimeCache, error::TF2Error, types::CompactFrameID};

// NOTE: For now use r2r. Will make it into a trait later
trait TransformStamped {
    fn stamp(&self) -> u64;
    fn frame_id(&self) -> &str;
    fn child_frame_id(&self) -> &str;
    fn translation(&self) -> &[f64; 3];
    fn rotation(&self) -> &[f64; 4];
}


struct TransformableRequest<'a>
{
  time: DateTime<Utc>,
  request_handle: u64,
  cb_handle: u32,
  target_id: CompactFrameID,
  source_id: CompactFrameID,
  target_string: &'a str,
  source_string: &'a str,
}

pub struct BufferCore<'a> {
    frames_: Mutex<Vec<TimeCache>>,

    frame_to_id: HashMap<&'a str, CompactFrameID>,

    id_to_frame: Vec<&'a str>,

    frame_authority: HashMap<CompactFrameID, &'a str>,

    cache_time: Duration,

    transformable_callbacks_: Mutex<HashMap<u32, ()>>,
    transformable_requests: Mutex<Vec<TransformableRequest<'a>>>,

    using_dedicated_thread: bool,
}

impl<'a> BufferCore<'a> {
    fn set_transform(
        _transform: &dyn TransformStamped,
        _authority: &str,
        _is_static: bool,
    ) -> Result<(), TF2Error> {

        Ok(())
    }
}
