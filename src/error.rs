use thiserror::Error;

use crate::types::CompactFrameID;

#[derive(Error, Debug, PartialEq, Eq)]
pub enum TF2Error {
    #[error("unknown data store error")]
    Unknown,
    #[error("Empty container")]
    Empty,

    // Frame id issues
    #[error("Ignoring transform w/ authority `{0}` with frame_id and child_frame_id `{1}` because they are the same.")]
    MatchingFrameIDs(String, String),
    #[error("Ignoring transform w/ authority `{0}` with empty `{1}`.")]
    EmptyFrameID(String, &'static str),
    #[error("Unknown frame id `{0}`")]
    UnknownFrameID(String),
    #[error("No known relation between found between frame ids `{0}` and `{1}`. The cause might be disjointed trees in the cache, or more then 1 parent per frame (i.e. not a Tree).")]
    UnknownRelationBetweenFrames(CompactFrameID, CompactFrameID),

    // Extrapolation errors
    #[error("Lookup would require extrapolation at time `{0}`, but only time `{1}` is in the buffer")]
    SingleExtrapolationError(u64, u64),
    #[error("Lookup would require extrapolation into the future. Requested time `{0}` but the latest data is at time `{1}`")]
    FutureExtrapolationError(u64, u64),
    #[error("Lookup would require extrapolation into the past. Requested time `{0}` but the earliest data is at time `{1}`")]
    PastExtrapolationError(u64, u64),

    // Buffer specific errors
    #[error("When trying to access TransformStores, got an exception from Lock. This is probably due to issue that ocurred before.")]
    PoisonedCache,
}