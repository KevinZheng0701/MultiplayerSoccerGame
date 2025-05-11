import random

# 6 fixed terrain zones with 3 on red side (x < 0) and 3 on blue side (x > 0)
TERRIAN_ZONES = [
    {"x_min": -2.6, "x_max": -2.3, "y_min": 0.5, "y_max": 0.8},
    {"x_min": -1.5, "x_max": -1.2, "y_min": -1.8, "y_max": -1.5},
    {"x_min": -0.6, "x_max": -0.3, "y_min": 0.2, "y_max": 0.5},
    {"x_min": 0.3, "x_max": 0.6, "y_min": -0.5, "y_max": -0.2},
    {"x_min": 1.4, "x_max": 1.7, "y_min": 1.0, "y_max": 1.3},
    {"x_min": 2.1, "x_max": 2.4, "y_min": -1.6, "y_max": -1.3},
]
TERRIAN_COOLDOWN = 3  # Cooldown duration(seconds) before a robot can be affected by terrain forces again


class ArtificialFaultInjector:
    """
    Simulates environmental disturbances and network irregularities during a match,
    such as random terrain forces, wind effects, and artificial network delays.
    """

    def __init__(self, chance=0.25, lambd=1):
        self.fault_chance = chance  # Probability that a fault will occur when triggered
        self.lambd = lambd  # Lambda for exponential distribution: higher λ = more frequent events, lower λ = more spread out
        self.last_terrain_fault_time = (
            {}
        )  # Tracks the last time each robot was affected by terrain fault
        self.last_wind_time = 0  # # Timestamp of the last wind event
        self.next_wind_interval = random.uniform(
            60, 90
        )  # Time (in seconds) until the next wind event

    def inject_artificial_network_delay(self, duration=-1):
        """
        Injects an artificial network delay to simulate communication latency.
        Args:
            duration (float): If greater or equal to 0, this fixed delay is returned directly. Else a randomized delay logic is used.
        Returns:
            float: The delay in seconds to apply.
                - Returns 0 with probability `fault_chance`.
                - Otherwise, returns a delay sampled from an exponential distribution.
        """
        # Use fixed delay if explicitly provided
        if duration >= 0:
            return duration
        # With some probability, no delay is injected
        if random.random() > self.fault_chance:
            return 0
        # Otherwise, sample a delay from the exponential distribution
        return random.expovariate(self.lambd)

    def inject_wind_forces(self, curr_time, max_wind_strength):
        """
        Generates and returns a random wind force vector if the cooldown period has elapsed since the last wind event.

        Args:
            curr_time (float): The current time in seconds, used to determine if the cooldown has passed.
            max_wind_strength (float): The maximum strength (magnitude) for the wind force.

        Returns:
            list: A list of two float values representing the wind force vector [x_force, y_force].
                - Each component is a random value uniformly sampled between -max_wind_strength and +max_wind_strength.
                - If the cooldown has not yet expired, returns [None, None] indicating no wind should be applied.
        """
        if curr_time - self.last_wind_time >= self.next_wind_interval:
            # Update wind timing
            self.last_wind_time = curr_time
            self.next_wind_interval = random.uniform(60, 90)
            # Random wind direction
            return [
                random.uniform(-max_wind_strength, max_wind_strength),  # x-direction
                random.uniform(-max_wind_strength, max_wind_strength),  # y-direction
            ]
        return [None, None]  # No force is generated

    def get_terrain_patches(self):
        """
        Returns a list of Webots patch strings that visually represent the fixed terrain zones on the field.

        Returns:
            list[str]: A list of Webots-compatible `Solid` node strings. Each represents a translucent green patch over a predefined terrain zone.
        """
        # Add green translucent patches once
        patch_string_template = """
        Solid {{
        translation {x} {y} 0.01
        rotation 0 0 1 0
        children [
            Shape {{
            appearance Appearance {{
                material Material {{
                diffuseColor 0 1 0
                transparency 0.6
                }}
            }}
            geometry Box {{
                size {w} {h} 0.01
            }}
            }}
        ]
        name "VisualTerrainPatch"
        contactMaterial "default"
        }}
        """
        patches = []
        for zone in TERRIAN_ZONES:
            x_center = (zone["x_min"] + zone["x_max"]) / 2
            y_center = (zone["y_min"] + zone["y_max"]) / 2
            width = zone["x_max"] - zone["x_min"]
            height = zone["y_max"] - zone["y_min"]
            patch_string = patch_string_template.format(
                x=x_center, y=y_center, w=width, h=height
            )
            patches.append(patch_string)
        return patches

    def inject_terrain_forces(self, player_id, curr_time, x_position, y_position):
        """
        Applies a randomized terrain force to a player if they are inside a defined terrain zone and eligible based on cooldown and chance.

        Args:
            player_id (str): Unique identifier for the player or robot.
            curr_time (float): The current time in seconds, used to check cooldowns.
            x_position (float): The current x-coordinate of the player.
            y_position (float): The current y-coordinate of the player.

        Returns:
            tuple:
                - force (list[float] or None): A 3D force vector [fx, fy, fz] applied to the player if triggered.
                - offset (list[float] or None): A 3D offset vector indicating where the force is applied relative to the robot's position.
                If no terrain force is triggered, both values return as (None, None).
        """
        last_time = self.last_terrain_fault_time.get(player_id, 0)
        for zone in TERRIAN_ZONES:
            # Check if the player is within the current terrain zone
            if (
                zone["x_min"] <= x_position <= zone["x_max"]
                and zone["y_min"] <= y_position <= zone["y_max"]
                and curr_time - last_time >= TERRIAN_COOLDOWN  # Cooldown check
                and random.random() < self.fault_chance  # Chance-based trigger
            ):
                # Generate random 3D force vector to simulate disturbance
                force = [
                    random.uniform(-150, 150),  # x-direction force
                    random.uniform(-150, 150),  # y-direction force
                    random.uniform(-50, -70),  # downward z-force
                ]
                # Randomized point above the robot where the force is applied
                offset = [random.uniform(-0.03, 0.03), 0, 0.25]
                # Record the time this fault was applied to the player
                self.last_terrain_fault_time[player_id] = curr_time
                return force, offset
        return None, None  # No terrain force applied
