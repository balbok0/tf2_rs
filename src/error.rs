use thiserror::Error;

#[derive(Error, Debug)]
pub enum TF2Error {
    #[error("unknown data store error")]
    Unknown,
    #[error("Empty container")]
    Empty,
    #[error("Lookup would require extrapolation at time `{0}`, but only time `{1}` is in the buffer")]
    SingleExtrapolationError(f64, f64),
    #[error("Lookup would require extrapolation into the future. Requested time `{0}` but the latest data is at time `{1}`")]
    FutureExtrapolationError(f64, f64),
    #[error("Lookup would require extrapolation into the past. Requested time `{0}` but the earliest data is at time `{1}`")]
    PastExtrapolationError(f64, f64),
}