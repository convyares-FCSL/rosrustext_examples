pub struct StatsResult {
    pub sum: f64,
    pub average: f64,
    pub status: String,
}

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

    StatsResult {sum, average, status: String::from("Success")}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_normal_values() {
        let data = vec![1.0, 2.0, 3.0];
        let result = compute(&data);
        assert_eq!(result.sum, 6.0);
        assert_eq!(result.average, 2.0);
        assert_eq!(result.status, "Success");
    }

    #[test]
    fn test_compute_empty_list() {
        let data = vec![];
        let result = compute(&data);
        assert_eq!(result.sum, 0.0);
        assert_eq!(result.average, 0.0);
        assert_eq!(result.status, "Warning: No data provided. Returning 0.");
    }

    #[test]
    fn test_compute_floats() {
        let data = vec![1.5, 2.5];
        let result = compute(&data);
        assert_eq!(result.sum, 4.0);
        assert_eq!(result.average, 2.0);
    }

    #[test]
    fn test_compute_single_element() {
        let data = vec![42.0];
        let result = compute(&data);
        assert_eq!(result.sum, 42.0);
        assert_eq!(result.average, 42.0);
        assert_eq!(result.status, "Success");
    }
}