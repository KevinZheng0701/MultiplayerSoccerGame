import socket
import threading
import math
from nao_robot import Nao

GOALIE_X_POSITION = 4.5  # X-position for the goalie line
MIDFIELDER_DISTANCE = (
    1  # Y-axis offset used by midfielders to spread out relative to the ball
)


class SoccerRobot(Nao):
    """
    SoccerRobot inherits from the Nao robot class and uses motion files for movement execution.
    It communicates with the game server via TCP sockets to receive state updates and send actions.
    The robot understands its environment by tracking ball position, team composition, player states,
    and dynamically adapts its behavior based on its assigned role (e.g., goalie, striker, midfielder).
    """

    def __init__(self):
        super().__init__()
        # Team and game state
        self.my_team = set()
        self.opponent_team = set()
        self.ball_position = [0, 0, 0]
        self.player_states = {}  # Store player positions and role status

        # Identity and communication
        self.lock = threading.Lock()
        self.sock = None
        self.team_number = None
        self.player_id = None

        # Robot behavior state
        self.state = None
        self.role = None
        self.target_position = [0, 0]  # The position the robot is heading towards
        self.target_rotation = [0, 0]  # The rotation the robot is targeting

        # Timing and movement tracking
        self.start_time = 0
        self.setup_time = math.inf  # The delay needed for physics calculations
        self.last_state = None
        self.last_position = [0, 0]
        self.last_rotation = 0

        # Fall recovery to ensure robot is standing up right
        self.recovery_mode = False

    # ───────────────────────────────────────
    # 📡 Server Connection & Communication
    # ───────────────────────────────────────

    def connect_to_server(self, host, port):
        """
        Establishes a TCP connection to the game server and starts a listener thread.

        Args:
            host (str): The IP address or hostname of the game server.
            port (int): The port number to connect to.
        """
        print("🔄 Attempting to connect to server...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            client_thread = threading.Thread(target=self.listen_for_server)
            client_thread.start()
            print("✅ Successfully connected to server.")
        except Exception as error:
            print(f"⚠️ Connection to server failed: {error}")

    def listen_for_server(self):
        """
        Continuously listens for messages from the game server in a background thread.
        """
        try:
            buffer = ""  # Buffer to store the message
            while True:
                data = self.sock.recv(1024)
                if not data:
                    print("⚠️ Disconnected from server.")
                    break
                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    message, buffer = buffer.split("\n", 1)
                    self.handle_message(message)
        except Exception as error:
            print(f"⚠️ Communication error: {error}")
        finally:
            print(f"Client {self.player_id} closing connection with server.")
            self.sock.close()

    def send_message(self, message, delay_tolerance=0.5):
        """
        Sends a message to the server, including the current timestamp and delay tolerance.

        Args:
            message (str): Message content (without timing data).
            delay_tolerance (float): Max delay allowed in seconds for the message.
        """
        self.sock.sendall(
            f"{message}|{self.getTime()}|{delay_tolerance}\n".encode("utf-8")
        )

    # ───────────────────────────────────────
    # 🧠 Robot Decision Making
    # ───────────────────────────────────────

    def determine_action(self):
        """
        Determines the next action for the robot based on its current role and game state.
        """
        if not self.is_motion_over():
            return
        # If the robot has fallen then ensure it is getting up first
        if self.has_fallen():
            self.play_standup_motion()
            self.state = "Recovering"
            return
        if self.state == "Recovering" or self.state == "Kicking":
            self.state = None
        if self.recovery_mode:
            # Once in recovery mode and stable, send acknowledge to server
            self.setup_time = math.inf
            message = f"ACK|{self.player_id}"
            self.send_message(message)
            self.recovery_mode = False
            return
        match self.role:
            case "Goalie":
                self.determine_goalie_action()
            case "Striker":
                self.determine_striker_action()
            case "Midfielder":
                self.determine_midfielder_action()
            case _:
                print(f"⚠️ Role not recognized: {self.role}")
        if self.state == "Moving":
            self.move_to_position(self.target_position)
        elif self.state == "Turning":
            self.turn_to_direction(self.target_rotation, moveAfterTurn=True)
        elif self.state == "Sliding":
            self.side_step_to_position(self.target_position[1])
        elif self.state == "Backing":
            self.back_up(self.target_position[0])

    def determine_goalie_action(self, threshold=0.1):
        """
        Determines the goalie's behavior based on ball position and proximity to goal.

        Args:
            threshold (float): Minimum y-distance from the ball before adjusting alignment.

        The goalie will:
        - Face the opposing goal.
        - Back up if behind the ball.
        - Slide along the goal line to align with the ball.
        - Approach and kick the ball if it's close to the goal.
        """
        # If the ball is far from goal the goalie should just stay along the x axis of the goal
        position = self.get_position()
        ball_x_position, ball_y_position, _ = self.ball_position
        if abs(ball_x_position - position[0]) < 4:
            # Goalie should be facing towards the other team goal
            target_direction = [1, 0] if self.team_number == 1 else [-1, 0]
            if self.turn_to_direction(target_direction, 20):
                return

            # If the robot is in front of the ball, the robot should move back
            goalie_x_axis = (
                -GOALIE_X_POSITION if self.team_number == 1 else GOALIE_X_POSITION
            )
            if not self.is_ball_ahead(ball_x_position, position[0], 0.1):
                self.target_position[0] = goalie_x_axis
                self.state = "Backing"
            # Move the robot in front of the ball
            elif abs(ball_y_position - position[1]) > threshold:
                self.slide_to_y_position(ball_y_position, -1, 1)
            # Move the robot towards the ball
            elif (
                self.get_distance(
                    [ball_x_position, ball_y_position], [goalie_x_axis, position[1]]
                )
                < 1
            ):
                self.go_to(ball_x_position, ball_y_position, threshold)
                # Kick the ball if the robot stopped moving
                if not self.state:
                    self.state = "Kicking"
                    self.play_kick_ball()

    def determine_striker_action(
        self, threshold=0.2, alignment_threshold=0.8, offset=0.35
    ):
        """
        Determines the striker's behavior based on position relative to the ball and goal.

        Args:
            threshold (float): Distance considered "close enough" to engage with the ball.
            alignment_threshold (float): Threshold for dot product to check alignment.
            offset (float): Distance to stay behind the ball when realigning.

        The striker will:
        - Move directly to the ball if far away.
        - Kick if aligned and behind the ball.
        - Reposition if in front of the ball.
        - Otherwise, move behind the ball to align for a better shot.
        """
        # Player with role Striker should head towards the ball if it is far away
        if self.distance_to_ball() > threshold * 3:
            self.go_to(self.ball_position[0], self.ball_position[1])
            return

        # Find the direction from the ball to the goal and the ball to the robot
        target_goal = GOALIE_X_POSITION if self.team_number == 1 else -GOALIE_X_POSITION
        x_position, y_position, _ = self.get_position()
        ball_to_goal_direction = self.normalize_vector(
            [target_goal - self.ball_position[0], -self.ball_position[1]]
        )
        ball_to_robot_direction = self.normalize_vector(
            [x_position - self.ball_position[0], y_position - self.ball_position[1]]
        )
        dot_product = (
            ball_to_goal_direction[0] * ball_to_robot_direction[0]
            + ball_to_goal_direction[1] * ball_to_robot_direction[1]
        )

        # If the dot product is close to -1 then it means the robot is behind the ball and reasonably aligned
        if dot_product < -alignment_threshold:
            self.go_to(self.ball_position[0], self.ball_position[1])
            # Kick the ball if the robot stopped moving
            if not self.state:
                self.state = "Kicking"
                self.play_kick_ball()
        elif dot_product > alignment_threshold:
            # If the dot product is close to 1 then it means the robot is in front of the ball and must reposition
            point_one, point_two = self.get_rotated_points(
                ball_to_goal_direction, self.ball_position, 120
            )
            dist_one = self.get_distance(point_one, [x_position, y_position])
            dist_two = self.get_distance(point_two, [x_position, y_position])
            best_target = point_one if dist_one < dist_two else point_two
            self.go_to(best_target[0], best_target[1])
        else:
            # The robot is not along the target direction so it should move to a position behind the ball
            x_target = self.ball_position[0] - ball_to_goal_direction[0] * offset
            y_target = self.ball_position[1] - ball_to_goal_direction[1] * offset
            self.go_to(x_target, y_target)

    def determine_midfielder_action(self):
        """
        Determines the midfielder's behavior based on team composition and ball position.

        The midfielder will:
        - Skip action if team role assignments are inconsistent.
        - Dynamically position itself to support the striker based on field sections.
        - Choose the optimal y-offset relative to the ball (±MIDFIELDER_DISTANCE) to spread the field.
        """
        sections = {"negative": -MIDFIELDER_DISTANCE, "positive": MIDFIELDER_DISTANCE}
        striker_count = 0
        midfielder_position = None
        # Extract the position of the striker and the other midfielder in the team
        for player_id in self.my_team:
            player_role = self.player_states[player_id][0]
            position = self.player_states[player_id][2]
            if player_role == "Striker":
                striker_count += 1
            elif player_role == "Midfielder":
                midfielder_position = position
        # Changes in the roles might have delay which can cause inconsistency so actions should be skipped until states are consistent
        if striker_count != 1:
            print(striker_count)
            return

        x_position, y_position, _ = self.get_position()
        ball_x_position, ball_y_position, _ = self.ball_position
        # Section allocation depends on both midfielders when there's 4 players on each team
        if len(self.my_team) == 4:
            for offset in sections.values():
                target_y_position = ball_y_position + offset
                robot_distance = self.get_distance(
                    [ball_x_position, target_y_position], [x_position, y_position]
                )
                midfielder_distance = self.get_distance(
                    [ball_x_position, target_y_position], midfielder_position
                )
                if robot_distance < midfielder_distance:
                    self.go_to(ball_x_position, target_y_position)
        else:
            # With only one midfielder, the player should only consider the closet support position
            offset = min(
                sections.items(),
                key=lambda item: abs(y_position - (ball_y_position + item[1])),
            )[1]
            self.go_to(ball_x_position, ball_y_position + offset)

    # ───────────────────────────────────────
    # 🚶 Robot Motion Control
    # ───────────────────────────────────────

    def has_fallen(self, force_threshold=5):
        """
        Determines whether the robot has fallen using foot pressure sensor values.

        Args:
            force_threshold (float): Minimum total force (in Newtons) required to consider the robot upright.

        Returns:
            bool: True if the robot is considered to have fallen, False otherwise.
        """
        if not self.is_motion_over() or not self.is_setup_time_over():
            return False

        fsv = []  # Force sensor values
        fsv.append(self.fsr[0].getValues())  # Left foot
        fsv.append(self.fsr[1].getValues())  # Right foot

        # Compute total force on each foot
        newtonsLeft = sum(
            [
                fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1],  # Left Front Left
                fsv[0][2] / 3.4
                + 1.5 * fsv[0][0]
                - 1.15 * fsv[0][1],  # Left Front Right
                fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1],  # Left Rear Right
                fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1],  # Left Rear Left
            ]
        )

        newtonsRight = sum(
            [
                fsv[1][2] / 3.4
                + 1.5 * fsv[1][0]
                + 1.15 * fsv[1][1],  # Right Front Left
                fsv[1][2] / 3.4
                + 1.5 * fsv[1][0]
                - 1.15 * fsv[1][1],  # Right Front Right
                fsv[1][2] / 3.4
                - 1.5 * fsv[1][0]
                - 1.15 * fsv[1][1],  # Right Rear Right
                fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1],  # Right Rear Left
            ]
        )

        total_force = newtonsLeft + newtonsRight
        if total_force < force_threshold:
            print(f"⚠️ Robot has fallen! Total force: {total_force:.2f}N")
            return True
        return False

    def start_turn(self, direction):
        """
        Initiates a turning action toward a specified direction.

        Args:
            direction (list[float]): Normalized direction vector to turn toward.
        """
        self.target_rotation = direction
        self.state = "Turning"

    def set_target_position(self, x_position, y_position):
        """
        Sets the robot's target (x, y) position for navigation or movement logic.

        Args:
            x_position (float): Target x-coordinate.
            y_position (float): Target y-coordinate.
        """
        self.target_position = [float(x_position), float(y_position)]

    def stop_turn_and_start_moving(self):
        """
        Stops the robot's current turning motion and transitions to the 'Moving' state.
        """
        self.stop_motion()
        self.state = "Moving"

    def go_to(self, x_position, y_position, threshold=0.15):
        """
        Directs the robot to move toward a specified (x, y) target position.

        Args:
            x_position (float): Target x-coordinate on the field.
            y_position (float): Target y-coordinate on the field.
            threshold (float): Distance within which the robot is considered to have arrived.
        """
        self.set_target_position(x_position, y_position)
        position = self.get_position()
        # If the robot reached the target position then stop moving
        distance = self.get_distance(position, self.target_position)
        if distance < threshold:
            self.state = None
            return

        # Turn before moving
        x_current, y_current = position[0], position[1]
        direction = self.normalize_vector(
            [x_position - x_current, y_position - y_current]
        )
        if self.turn_to_direction(direction):
            self.start_turn(direction)
        else:
            self.state = "Moving"

    def move_to_position(self, target_position, threshold=0.15):
        """
        Moves the robot toward a specified target position.

        Args:
            target_position (list[float]): The (x, y) coordinates to move toward.
            threshold (float): Distance within which the robot is considered to have arrived.

        Returns:
            bool: True if the target was reached, False otherwise.
        """
        x_position, y_position, _ = self.get_position()
        # If the robot reached the target position then stop moving
        distance = self.get_distance([x_position, y_position], target_position)
        if distance < threshold:
            self.state = None
            return True

        # Before moving again, ensure robot direction is correct
        direction = self.normalize_vector(
            [target_position[0] - x_position, target_position[1] - y_position]
        )
        if self.turn_to_direction(direction, moveAfterTurn=True):
            self.start_turn(direction)
            return False
        # Take small steps when it is close else large steps
        if distance < threshold * 3:
            self.start_motion(self.smallForwards)
        else:
            self.start_motion(self.largeForwards)
        return False

    def turn_to_direction(self, direction, threshold=35, moveAfterTurn=False):
        """
        Rotates the robot to face a given direction vector.

        Args:
            direction (list[float]): Normalized 2D vector indicating the desired direction.
            threshold (float): Angle in degrees below which turning is considered complete.
            moveAfterTurn (bool): If True, initiates movement immediately after turning.

        Returns:
            bool: True if turning was initiated, False if already aligned.
        """
        angle_difference = self.get_turn_angle(direction)
        # Positive angle indicates a left turn and negative angle indicates a right turn
        if abs(angle_difference) < math.radians(threshold):
            if moveAfterTurn:
                self.stop_turn_and_start_moving()
            else:
                self.state = None
            return False  # Return false for no turning needed

        self.start_motion(self.turnLeft40 if angle_difference > 0 else self.turnRight40)
        return True

    def slide_to_y_position(
        self, y_position, lower_y_threshold=-math.inf, upper_y_threshold=math.inf
    ):
        """
        Sets the robot's target y-position using a sliding action.

        Args:
            y_position (float): Desired y-coordinate to slide to.
            lower_y_threshold (float): Minimum allowed y-position.
            upper_y_threshold (float): Maximum allowed y-position.
        """
        clamped_pos = max(lower_y_threshold, min(upper_y_threshold, y_position))
        self.target_position[1] = clamped_pos
        self.state = "Sliding"

    def side_step_to_position(self, y_position, threshold=0.1):
        """
        Moves the robot sideways (left/right) to reach a target y-position using side-step motions.

        Args:
            y_position (float): Target y-coordinate to move toward.
            threshold (float): Acceptable distance to consider the robot has arrived.
        """
        y_current = self.get_position()[1]
        difference = y_position - y_current
        # If the robot reached the target position then stop moving
        if abs(difference) < threshold:
            self.state = None
            return

        # Move the robot
        if self.team_number == 1:
            motion = self.sideStepLeft if difference > 0 else self.sideStepRight
        else:
            motion = self.sideStepRight if difference > 0 else self.sideStepLeft
        self.start_motion(motion)

    def back_up(self, target, threshold=0.1):
        """
        Makes the robot move backward along the x-axis toward a target position.

        Args:
            target (float): Target x-coordinate to back up to.
            threshold (float): Acceptable distance to consider the robot has arrived.
        """
        x_position = self.get_position()[0]
        if abs(target - x_position) < threshold:
            self.state = None
            return
        self.start_motion(self.backwards)

    # ───────────────────────────────────────
    # 🛰️ Game State Synchronization
    # ───────────────────────────────────────

    def handle_message(self, message):
        """
        Handles incoming messages from the server and updates robot state accordingly.

        Args:
            message (str): A message string from the server, delimited by '|'.
        """
        message_parts = message.split("|")
        message_type = message_parts[0]

        match message_type:
            case "STATE":
                # Message details
                player_id = message_parts[1]
                state = message_parts[2]
                x_position, y_position = float(message_parts[3]), float(
                    message_parts[4]
                )
                angle = float(message_parts[5])

                # Update the position and rotation of the players
                if player_id == self.player_id:
                    print("🔄 Reset robot position")
                    self.state = None  # Resume normal behavior
                else:
                    self.update_state(player_id, state)
                    self.update_position(player_id, x_position, y_position)
                    self.update_rotation(player_id, angle)
            case "BALL":
                x_position, y_position, z_position = (
                    float(message_parts[1]),
                    float(message_parts[2]),
                    float(message_parts[3]),
                )
                self.update_ball_position(x_position, y_position, z_position)
            case "ROLE":
                player_id = message_parts[1]
                role = message_parts[2]
                if player_id == self.player_id:
                    with self.lock:
                        self.role = role
                    print(f"📢 Assigned Role: {self.role}")
                else:
                    with self.lock:
                        self.player_states[player_id][0] = role
            case "REVERT":
                self.create_delay(
                    float(message_parts[1])
                )  # Revert comes as a penalty for network delay
            case "RESET":
                self.recovery_mode = True
            case "INFO":
                player_id = message_parts[1]
                # Set player information
                if not self.player_id:
                    self.player_id = player_id
                    robot_name = self.getName()
                    robot_number = robot_name.split("_")[1]
                    self.team_number = int(robot_number) % 2 + 1
                    message = f"ROBOT|{self.player_id}|{robot_name}|{self.team_number}"
                    self.send_message(message)
                    print(
                        f"🆔 Player ID: {self.player_id} is in team {self.team_number}"
                    )
                else:
                    # Set up player default state
                    team_number = int(message_parts[2])
                    with self.lock:
                        self.player_states[player_id] = [None, "None", [0, 0], 0]
                        if team_number == self.team_number:
                            self.my_team.add(player_id)
                            print(f"📢 New teammate: {player_id}")
                        else:
                            self.opponent_team.add(player_id)
                            print(f"📢 New opponent: {player_id}")
            case "START":
                self.create_delay(float(message_parts[1]))
            case _:
                print(f"❓ Unknown message type: {message}")

    def send_player_state(self, force=False):
        """
        Sends the robot's current position, rotation, and action state to the game server.

        Args:
            force (bool): If True, forces a state update regardless of movement or rotation changes.
        """
        position, angle = self.get_position(), self.get_rotation()[2]
        if (
            force
            or self.get_distance(position, self.last_position) > 0.25
            or abs(self.calculate_angle_difference(angle, self.last_rotation))
            > math.radians(10)
            or self.state != self.last_state
        ):
            state = self.state if self.state else "Idle"
            message = f"STATE|{self.player_id}|{state}|{position[0]:.3f}|{position[1]:.3f}|{angle:.3f}"
            self.send_message(message)
            self.last_position = [position[0], position[1]]
            self.last_rotation = angle
            self.last_state = self.state

    def update_state(self, player_id, state):
        """
        Updates the current action/state of the specified player.

        Args:
            player_id (str): ID of the player to update.
            state (str): New state (e.g., "Moving", "Idle", "Kicking").
        """
        with self.lock:
            self.player_states[player_id][1] = state

    def update_position(self, player, x, y):
        """
        Updates the (x, y) position of a player.

        Args:
            player (str): ID of the player.
            x (float): New x-coordinate.
            y (float): New y-coordinate.
        """
        with self.lock:
            self.player_states[player][2] = [x, y]

    def update_rotation(self, player, angle):
        """
        Updates the rotation angle of a player.

        Args:
            player (str): ID of the player.
            angle (float): New rotation angle in radians.
        """
        with self.lock:
            self.player_states[player][3] = angle

    def update_ball_position(self, x, y, z):
        """
        Updates the (x, y, z) position of the ball.

        Args:
            x (float): New x-coordinate of the ball.
            y (float): New y-coordinate of the ball.
            z (float): New z-coordinate of the ball.
        """
        with self.lock:
            self.ball_position[0] = x
            self.ball_position[1] = y
            self.ball_position[2] = z

    def create_delay(self, duration):
        """
        Adds a delay to the robot's motion or physics update system.

        If no delay is currently active, starts a new one. Otherwise,
        stacks the new delay onto the remaining active delay duration.

        Args:
            float: Time in seconds to delay.
        """
        with self.lock:
            if self.is_setup_time_over() or self.setup_time == math.inf:
                # No active delay, start a new one
                self.start_time = self.getTime()
                self.setup_time = duration
            else:
                # Stack the new delay onto the current remaining time
                self.setup_time += duration

    # ───────────────────────────────────────
    # 🧮 Math & Geometry Utilities
    # ───────────────────────────────────────

    def get_distance(self, point_one, point_two):
        """
        Calculates Euclidean distance between two points in the xy-plane.

        Args:
            point_one (list[float]): Coordinates of the first point [x1, y1].
            point_two (list[float]): Coordinates of the second point [x2, y2].

        Returns:
            float: The distance between the two points.
        """
        return math.sqrt(
            (point_two[0] - point_one[0]) ** 2 + (point_two[1] - point_one[1]) ** 2
        )

    def normalize_vector(self, vector):
        """
        Returns the normalized version of a 2D or 3D vector.

        Args:
            vector (list[float]): A list of vector components.

        Returns:
            list[float]: A unit vector in the same direction, or the original if norm is 0.
        """
        norm = math.sqrt(sum(val**2 for val in vector))
        return [v / norm for v in vector] if norm != 0 else vector

    def get_turn_angle(self, direction):
        """
        Calculates the angle the robot needs to turn to face the given direction.

        Args:
            direction (list[float]): Normalized direction vector.

        Returns:
            float: Angular difference (in radians) between current orientation and target.
        """
        current_angle = self.get_rotation()[2]
        target_angle = math.atan2(direction[1], direction[0])
        return self.calculate_angle_difference(current_angle, target_angle)

    def calculate_angle_difference(self, angle1, angle2):
        """
        Returns the smallest signed difference between two angles in the range [-π, π].

        Args:
            angle1 (float): Current angle.
            angle2 (float): Target angle.

        Returns:
            float: The angular difference angle2 - angle1.
        """
        return (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi

    def find_rotated_vector(self, vector, angle):
        """
        Rotates a 2D vector by a given angle.

        Args:
            vector (list[float]): A 2D vector [x, y].
            angle (float): Rotation angle in radians.

        Returns:
            list[float]: The rotated vector.
        """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x, y = vector
        return [x * cos_a - y * sin_a, x * sin_a + y * cos_a]

    def get_rotated_points(self, direction, origin_point, angle_degree, distance=0.5):
        """
        Returns two points rotated left and right from a direction vector, originating from a base point.

        Args:
            direction (list[float]): Unit direction vector to rotate from.
            origin_point (list[float]): [x, y] origin of the rotation.
            angle_degree (float): Angle in degrees to rotate left and right.
            distance (float): Distance from the origin to the rotated point.

        Returns:
            list[list[float]]: Two 2D points [left_point, right_point] at the specified angle and distance.
        """
        # Rotate direction vector by ±angle
        radians = math.radians(angle_degree)
        rotated_left = self.find_rotated_vector(direction, radians)
        rotated_right = self.find_rotated_vector(direction, -radians)

        # Scale by distance and offset from origin
        left_point = [
            origin_point[0] + rotated_left[0] * distance,
            origin_point[1] + rotated_left[1] * distance,
        ]
        right_point = [
            origin_point[0] + rotated_right[0] * distance,
            origin_point[1] + rotated_right[1] * distance,
        ]
        return [left_point, right_point]

    def distance_to_ball(self):
        """
        Calculates the Euclidean distance from the robot to the ball.

        Returns:
            float: Distance between the robot's position and the ball's position.
        """
        position = self.get_position()
        return self.get_distance(position, self.ball_position)

    # ───────────────────────────────────────
    # 🧰 Helper Functions
    # ───────────────────────────────────────

    def is_ball_ahead(self, ball_x, robot_x, min_distance=0.15):
        """
        Determines whether the ball is ahead of the robot on the field.

        Args:
            ball_x (float): x-position of the ball.
            robot_x (float): x-position of the robot.
            min_distance (float): Minimum distance ahead to count as "ahead".

        Returns:
            bool: True if the ball is ahead of the robot, False otherwise.
        """
        return (
            ball_x - robot_x > min_distance
            if self.team_number == 1
            else robot_x - ball_x > min_distance
        )

    def is_setup_time_over(self):
        """
        Checks if the current setup delay period has elapsed.

        Returns:
            bool: True if the delay has ended, False if still active.
        """
        return self.getTime() >= self.start_time + self.setup_time

    def is_motion_over(self):
        """
        Checks if the currently playing motion has finished.

        Returns:
            bool: True if no motion is playing or the motion has ended.
        """
        return not self.currentlyPlaying or self.currentlyPlaying.isOver()


host = "127.0.0.1"
port = 5555

soccer_robot = SoccerRobot()
timeStep = soccer_robot.timeStep

soccer_robot.step(timeStep)  # Wait for the game server to be up

client_thread = threading.Thread(
    target=soccer_robot.connect_to_server,
    args=(
        host,
        port,
    ),
)
client_thread.start()

# Webots main loop
while soccer_robot.step(timeStep) != -1:
    if soccer_robot.is_setup_time_over():
        soccer_robot.determine_action()
        soccer_robot.send_player_state()
