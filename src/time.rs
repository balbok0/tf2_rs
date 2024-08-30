use chrono::{DateTime, Duration, Utc};

pub fn duration_from_sec(t_sec: f64) -> Duration {
    Duration::nanoseconds((t_sec * 1e9) as i64)
}

pub fn time_from_sec(t_sec: f64) -> DateTime<Utc> {
    DateTime::from_timestamp_nanos((t_sec * 1e9) as i64)
}

pub fn duration_to_sec(input: &Duration) -> f64 {
    input.num_seconds() as f64 + (input.subsec_nanos() as f64 / 1e9)
}

pub fn time_to_sec(input: &DateTime<Utc>) -> f64 {
    input.timestamp() as f64 + (input.timestamp_subsec_nanos() as f64 / 1e9)
}


#[cfg(test)]
mod tests{
    use core::f64;

    use chrono::{DateTime, Duration};

    use approx::assert_relative_eq;

    use super::*;


    #[test]
    fn test_duration_from_sec() {
        // Vanilla case
        assert_eq!(
            duration_from_sec(1.123456789f64),
            Duration::new(
                1,
                123456789
            ).unwrap()
        );

        // Zero
        assert_eq!(
            duration_from_sec(0.0),
            Duration::new(
                0,
                0
            ).unwrap()
        );

        // Verify conversion works for max value of f64
        assert_eq!(
            duration_from_sec(f64::MAX),
            Duration::new(
                9223372036,
                854775807
            ).unwrap()
        );

        // ... and the min value of f64
        assert_eq!(
            duration_from_sec(f64::MIN),
            Duration::new(
                -9223372037,
                145224192
            ).unwrap()
        );
    }

    #[test]
    fn test_time_from_sec() {
        // Vanilla case
        assert_eq!(
            time_from_sec(1.123456789f64),
            DateTime::from_timestamp(
                1,
                123456789
            ).unwrap()
        );

        // Zero
        assert_eq!(
            time_from_sec(0.0),
            DateTime::from_timestamp(
                0,
                0
            ).unwrap()
        );

        // Verify conversion works for max value of f64
        assert_eq!(
            time_from_sec(f64::MAX),
            DateTime::from_timestamp(
                9223372036,
                854775807
            ).unwrap()
        );

        // ... and the min value of f64
        assert_eq!(
            time_from_sec(f64::MIN),
            DateTime::from_timestamp(
                -9223372037,
                145224192
            ).unwrap()
        );

        // Leap seconds
        assert_eq!(
            time_from_sec(9_223_372_036.854_775_807),
            DateTime::from_timestamp(
                9_223_372_036,
                854_775_807
            ).unwrap()
        );
    }


    #[test]
    fn test_duration_to_sec() {
        // Vanilla case
        assert_eq!(
            duration_to_sec(
            &Duration::new(
                1,
                123456789
            ).unwrap()),
            1.123456789f64,
        );

        // Vanilla case
        assert_eq!(
            duration_to_sec(
            &Duration::new(
                -1,
                123456789
            ).unwrap()),
            -0.876543211,
        );

        // Zero
        assert_eq!(
            duration_to_sec(&Duration::new(
                0,
                0
            ).unwrap()),
            0.0,
        );

        // Large value
        assert_eq!(
            duration_to_sec(&Duration::new(
                9223372036,
                854775807
            ).unwrap()),
            9223372036.854775807,
        );

        // Large negative value
        assert_relative_eq!(
            duration_to_sec(&Duration::new(
                -923702,
                854775807
            ).unwrap()),
            -923701.145224193,
        );
    }

    #[test]
    fn test_time_to_sec() {
        // Vanilla case
        assert_relative_eq!(
            time_to_sec(
                &DateTime::from_timestamp(
                    1,
                    123456789
                ).unwrap()
            ),
            1.123456789f64,
        );

        // Zero
        assert_relative_eq!(
            time_to_sec(
                &DateTime::from_timestamp(
                    0,
                    0
                ).unwrap()
            ),
            0.0,
        );

        // Verify conversion works for max value of f64
        assert_relative_eq!(
            time_to_sec(
                &DateTime::from_timestamp(
                    9223372036,
                    854775807
                ).unwrap()
            ),
            9223372036.854775807
        );

        // ... and the min value of f64
        assert_relative_eq!(
            time_to_sec(
                &DateTime::from_timestamp(
                    -9223372037,
                    145224192
                ).unwrap()
            ),
            -9223372036.854775808
        );

        // Leap seconds
        assert_relative_eq!(
            time_to_sec(
                &DateTime::from_timestamp(
                    9223372036,
                    854775807
                ).unwrap()
            ),
            9223372036.854775807,
        );
    }

}