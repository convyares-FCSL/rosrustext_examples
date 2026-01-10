class StatsLogic:
    """
    Pure business logic for statistical calculations.
    Decoupled from ROS 2 middleware to enable easy unit testing.
    """

    @staticmethod
    def compute(data: list[float]) -> tuple[float, float, str]:
        """
        Computes sum and average.
        Returns: (sum, average, status_message)
        """
        if not data:
            return 0.0, 0.0, "Warning: No data provided. Returning 0."

        total = sum(data)
        average = total / len(data)
        
        return total, average, "Success"