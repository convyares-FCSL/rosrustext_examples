/// Result of the stats computation.
/// Pure data type: no ROS dependencies.
#[derive(Debug, Clone, PartialEq)]
pub struct StatsResult {
    pub sum: f64,
    pub average: f64,
    pub status: String,
}

/// Compute sum + average from a slice of f64.
/// This is pure business logic: deterministic, testable, and transport-agnostic.
pub fn compute(data: &[f64]) -> StatsResult {
    if data.is_empty() {
        return StatsResult {
            sum: 0.0,
            average: 0.0,
            status: String::from("Warning: No data provided. Returning 0."),
        };
    }

    let sum: f64 = data.iter().sum();
    let average: f64 = sum / data.len() as f64;

    StatsResult {
        sum,
        average,
        status: String::from("Success"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_approx_eq(a: f64, b: f64, eps: f64) {
        assert!(
            (a - b).abs() <= eps,
            "assert_approx_eq failed: a={a}, b={b}, |a-b|={}, eps={eps}",
            (a - b).abs()
        );
    }

    #[test]
    fn test_compute_normal_values() {
        let data = vec![1.0, 2.0, 3.0];
        let result = compute(&data);
        assert_approx_eq(result.sum, 6.0, 1e-12);
        assert_approx_eq(result.average, 2.0, 1e-12);
        assert_eq!(result.status, "Success");
    }

    #[test]
    fn test_compute_empty_list() {
        let data: Vec<f64> = vec![];
        let result = compute(&data);
        assert_approx_eq(result.sum, 0.0, 1e-12);
        assert_approx_eq(result.average, 0.0, 1e-12);
        assert_eq!(result.status, "Warning: No data provided. Returning 0.");
    }

    #[test]
    fn test_compute_floats() {
        let data = vec![1.5, 2.5];
        let result = compute(&data);
        assert_approx_eq(result.sum, 4.0, 1e-12);
        assert_approx_eq(result.average, 2.0, 1e-12);
        assert_eq!(result.status, "Success");
    }

    #[test]
    fn test_compute_single_element() {
        let data = vec![42.0];
        let result = compute(&data);
        assert_approx_eq(result.sum, 42.0, 1e-12);
        assert_approx_eq(result.average, 42.0, 1e-12);
        assert_eq!(result.status, "Success");
    }
}
