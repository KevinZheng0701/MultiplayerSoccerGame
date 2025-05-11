class ArtificialFaultObserver:
    """
    Observes and logs occurrences of artificial faults such as network delays, wind forces, and terrain disturbances.
    """

    def __init__(self):
        self.faults_detected = {
            "network": 0,
            "wind": 0,
            "terrain": 0,
        }  # Initialize fault counters for each type of artificial fault

    def detect_network_delay(self, arrival_time, curr_time, tolerance=1):
        """
        Detects if a network delay exceeds a given tolerance threshold.

        Args:
            arrival_time (float): The reported arrival time of a network message.
            curr_time (float): The current system or game time.
            tolerance (float): Maximum acceptable delay before it's considered a fault (default is 1 second).

        Returns:
            bool: True if a network delay fault is detected and False otherwise.
        """
        if arrival_time > curr_time + tolerance:
            self.faults_detected["network"] += 1
            self.print_fault("network")
            return True
        else:
            return False

    def detect_wind_forces(self, wind_forces, tolerance):
        """
        Determines whether the current wind forces exceed given tolerance levels.

        Args:
            wind_forces (list[float]): The wind force vector [x_force, y_force].
            tolerance (list[float]): The minimum force threshold [x_tol, y_tol] for detection in each direction.

        Returns:
            bool: True if wind is considered a fault and False otherwise.
        """
        x_force, y_force = wind_forces
        x_tolerance, y_tolerance = tolerance
        if abs(x_force) > x_tolerance or abs(y_force) > y_tolerance:
            self.faults_detected["wind"] += 1
            self.print_fault("wind")
            return True
        else:
            return False

    def detect_terrain_fault(self, terrain_forces, tolerance):
        """
        Determines if the given terrain force vector exceeds any of the specified tolerance thresholds.

        Args:
            terrain_forces (list[float]): The 3D terrain force vector [fx, fy, fz].
            tolerance (list[float]): The minimum threshold per component [x_tol, y_tol, z_tol].

        Returns:
            bool: True if any component of the terrain force exceeds its corresponding threshold.
        """
        fx, fy, _ = terrain_forces
        x_tolerance, y_tolerance = tolerance
        if abs(fx) > x_tolerance or abs(fy) > y_tolerance:
            self.faults_detected["terrain"] += 1
            self.print_fault("terrain")
            return True
        else:
            return False

    def print_fault(self, fault_type):
        """
        Prints the number of times a specific type of fault has been detected.

        Args:
            fault_type (str): The name or key of the fault type to report.
        """
        print(
            f"[{fault_type.capitalize()} Fault] Detected (Total: {self.faults_detected[fault_type]})"
        )
