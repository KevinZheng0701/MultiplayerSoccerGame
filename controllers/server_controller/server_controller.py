from controller import Supervisor
import socket
import threading
import uuid
import math
from collections import defaultdict
from RobotTeam import Team
from ClientMessageQueue import ClientMessageQueue
from Observer import ArtificialFaultObserver
from Injector import ArtificialFaultInjector

GOALIE_X_POSITION = 4.5  # The starting x position of the goalie robot
ROBOT_X_POSITION = 2.5  # The starting x position of the robots
ROBOT_Z_POSITION = 0.333  # The starting z position of the robots represents the height

FIELD_X_BOUND = 4.5  # The largest x value the ball can be within
FIELD_Y_BOUND = 3.0  # The largest y value the ball can be within

# Defines the bounds on the goal area
LOWER_GOAL_AREA_X = 4.5  # The lower bound on the x value of the goal area
UPPER_GOAL_AREA_X = 5.0  # The upper bound on the x value of the goal area
LOWER_GOAL_AREA_Y = -1.2  # The lower bound on the y value of the goal area
UPPER_GOAL_AREA_Y = 1.2  # The upper bound on the y value of the goal area

OFFSET_DISTANCE = 0.3  # The distance in front of robot

INIT_BALL_POS = [0, 0, 0.07]  # The starting position of the ball


class GameServer(Supervisor):
    def __init__(self, players_limit=6):
        super().__init__()  # Initialize as a supervisor so it has access to objects in the world
        if players_limit < 2:
            print("⚠️ Minimum player size is 2.")
            players_limit = 2
        elif players_limit > 8:
            print("⚠️ Maximum player size is 8.")
            players_limit = 8

        # Game states
        self.players_limit = players_limit
        self.client_lock = threading.Lock()
        self.clients = {}  # Store player id to their connections
        self.team1 = Team(1, players_limit // 2)
        self.team2 = Team(2, players_limit // 2)
        self.player_states = {}  # Store player game stats
        self.players = {}  # Store the reference to the robots
        self.ball = self.getFromDef("BALL")  # Get the soccer ball in the world
        self.last_ball_position = INIT_BALL_POS
        self.game_started = False
        self.ack_count = 0
        self.delayed_messages = defaultdict(
            ClientMessageQueue
        )  # List of delayed messages
        self.add_terrain_patches()

    def start_server(self, host, port):
        """Starts the server and wait for connections"""
        print("🔄 Starting server...")
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((host, port))
            server.listen(self.players_limit)
            print(f"✅ Server started on {host}:{port}")
        except ConnectionError as error:
            print(f"⚠️ Error binding server: {error}")
            return
        while True:
            try:
                connection, address = server.accept()
                self.handle_client_connection(connection, address)
                thread = threading.Thread(
                    target=self.listen_for_client, args=(connection,)
                )
                thread.start()
            except Exception as e:
                print(f"⚠️ Error while accepting connection: {e}")
                break

    def listen_for_client(self, connection):
        """Listens for incoming messages from clients"""
        try:
            buffer = ""  # Buffer to store the message
            while True:
                data = connection.recv(1024)
                if not data:
                    print("⚠️ Client disconnected from server.")
                    break
                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    message, buffer = buffer.split("\n", 1)
                    self.handle_message(message)
        except Exception as error:
            print(f"⚠️ Client error: {error}")
        finally:
            self.remove_client(connection)

    def handle_message(self, message, from_delay_queue=False):
        """Handles messages from the clients"""
        message_parts = message.split("|")
        message_type = message_parts[0]
        sender = message_parts[1]

        match message_type:
            case "STATE":
                # Only inject artificial fault if the message is not from the queue
                if not from_delay_queue:
                    send_time = float(message_parts[-2])
                    tolerance = float(message_parts[-1])
                    if self.handle_client_delay(message, sender, send_time, tolerance):
                        return

                state = message_parts[2]
                x_position, y_position = float(message_parts[3]), float(
                    message_parts[4]
                )
                angle = float(message_parts[5])

                # Update the state of the player
                if sender in self.player_states:
                    self.update_position(sender, x_position, y_position)
                    self.update_rotation(sender, angle)
                    self.update_state(sender, state)
                    self.broadcast(
                        f"{message_type}|{sender}|{state}|{x_position}|{y_position}|{angle}\n",
                        sender,
                    )  # Send the updated state to other clients
                else:
                    print(f"⚠️ Unknown player {sender} received. Ignoring POS update.")
            case "ACK":
                self.ack_count += 1
                if self.ack_count == self.players_limit:
                    self.reset_game()
                    self.ack_count = 0
            case "ROBOT":
                self.players[sender] = self.getFromDef(
                    message_parts[2]
                )  # Add the robot reference
                team_number = message_parts[3]

                # Send previously connected robots to the new client
                team1 = self.team1.get_players()
                team2 = self.team2.get_players()
                connection = self.clients[sender]
                for id in team1:
                    if id != sender:
                        connection.sendall(f"INFO|{id}|1\n".encode("utf-8"))
                for id in team2:
                    if id != sender:
                        connection.sendall(f"INFO|{id}|2\n".encode("utf-8"))

                # Add the player to its team and send info to other clients
                self.broadcast(f"INFO|{sender}|{team_number}\n", sender)
                if team_number == "1":
                    self.team1.add_player(sender)
                else:
                    self.team2.add_player(sender)

                # If all clients joined, then start the game by first assigning initial roles
                if len(self.players) == self.players_limit:
                    self.assign_initial_team_states(self.team1)
                    self.assign_initial_team_states(self.team2)
                    self.send_initial_states()
                    self.start_game()
            case _:
                print(f"❓ Unknown message type: {message}")

    def handle_client_connection(self, connection, address):
        """Handles new client connections and assigns the client a team and a unqiue identifer"""
        player_id = str(uuid.uuid4())
        print(f"🆔 New client connection: {player_id}, {address}.")

        # Add player to the clients list
        with self.client_lock:
            self.clients[player_id] = connection

        # Send id to the connected robot
        connection.sendall(f"INFO|{player_id}\n".encode("utf-8"))
        print(f"📢 Clients connected: {len(self.clients)}")

    def send_initial_states(self):
        """Broadcasts the initial state of the game(players and ball)"""
        for player, details in self.player_states.items():
            role = details[0]
            state = details[1]
            x_position, y_position = details[2]
            angle = details[3]
            state_message = (
                f"STATE|{player}|{state}|{x_position}|{y_position}|{angle}\n"
            )
            self.broadcast(state_message, player)
            role_message = f"ROLE|{player}|{role}\n"
            self.broadcast(role_message)
        self.send_ball_position(True)

    def assign_initial_team_states(self, team):
        """Assigns starting states to players in a team"""
        players = team.get_players()
        team_number = team.get_team_number()
        x_position, angle = self.get_initial_x_position_and_rotation(team_number)
        y_position = self.calculate_player_y_position(0)

        # In the order of role, current action, xy coordinate, rotation, etc
        self.player_states[players[0]] = [
            "Striker",
            None,
            [x_position, y_position],
            angle,
        ]
        self.apply_player_state_in_simulation(players[0])

        if len(players) >= 2:
            goalie_x_position = (
                -GOALIE_X_POSITION if team_number == 1 else GOALIE_X_POSITION
            )
            self.player_states[players[1]] = [
                "Goalie",
                None,
                [goalie_x_position, y_position if team_number == 1 else -y_position],
                angle,
            ]
            self.apply_player_state_in_simulation(players[1])

            for i in range(2, len(players)):
                y_position = self.calculate_player_y_position(i)
                self.player_states[players[i]] = [
                    "Midfielder",
                    None,
                    [x_position, y_position if team_number == 1 else -y_position],
                    angle,
                ]
                self.apply_player_state_in_simulation(players[i])

    def get_initial_x_position_and_rotation(self, team_number):
        """Returns the starting x position and rotation angle of the player based on team"""
        x = ROBOT_X_POSITION
        angle = 0
        if team_number == 1:
            x *= -1
        else:
            angle = math.pi
        return (x, angle)

    def calculate_player_y_position(self, player_index):
        """Finds the y position based on the index of the player"""
        y = player_index // 2
        return (
            y if player_index % 2 == 0 else -y
        )  # If the index is even then the position will be on the right else it will be on the left

    def apply_player_state_in_simulation(self, player_id):
        """Updates the position and rotation of the robot"""
        player_state = self.player_states[player_id]
        robot = self.players[player_id]

        # Update the rotation
        rotation_field = robot.getField("rotation")
        rotation_field.setSFRotation([0, 0, 1, player_state[3]])

        # Update the position
        translation_field = robot.getField("translation")
        translation_field.setSFVec3f(
            [player_state[2][0], player_state[2][1], ROBOT_Z_POSITION]
        )

    def start_game(self):
        """Sends a start message to the players"""
        self.game_started = True
        self.broadcast(f"START|1\n")

    def is_player_near_ball(self, player_id, threshold=0.5):
        """Checks if the player is close enough to the ball"""
        distance = self.player_distance_to_ball(player_id)
        return distance <= threshold

    def player_distance_to_ball(self, player_id):
        """Calculates the Euclidean distance from a player to the ball"""
        player_position = self.player_states[player_id][2]
        x_position, y_position = player_position[0], player_position[1]
        ball_position = self.ball.getPosition()
        return self.get_distance([x_position, y_position], ball_position)

    def send_ball_position(self, force=False):
        """Sends the current ball position to the clients"""
        ball_position = self.ball.getPosition()
        # If there is substanal change in the ball position or a force send
        if force or self.get_distance(ball_position, self.last_ball_position) > 0.1:
            self.last_ball_position = ball_position
            self.broadcast(
                f"BALL|{ball_position[0]:.2f}|{ball_position[1]:.2f}|{ball_position[2]:.2f}\n"
            )

    def get_distance(self, point_one, point_two):
        """Returns the Euclidean distance between two points in xy-plane"""
        return math.sqrt(
            (point_two[0] - point_one[0]) ** 2 + (point_two[1] - point_one[1]) ** 2
        )

    def update_state(self, player_id, state):
        """Updates the state of the player"""
        self.player_states[player_id][1] = state

    def update_position(self, player_id, x, y):
        """Updates the position of the player"""
        self.player_states[player_id][2] = [x, y]

    def update_rotation(self, player_id, angle):
        """Updates the rotation of the player"""
        self.player_states[player_id][3] = angle

    def remove_client(self, connection):
        """Removes disconnected clients"""
        with self.client_lock:
            for player_id, conn in self.clients.items():
                if conn == connection:
                    del self.clients[player_id]
                    self.team1.remove_player(player_id)
                    self.team2.remove_player(player_id)
                    print(f"❌ Removing player {player_id}")
                    break
        connection.close()

    def broadcast(self, message, sender=None):
        """Sends message to every client besides the sender"""
        for id, conn in self.clients.items():
            if id != sender:
                conn.sendall(message.encode("utf-8"))

    def update_roles_based_on_proximity(self, team):
        """Reassign striker role to closest player to the ball per team with others becoming the midfielders"""
        players = team.get_players()
        closet_player = None
        min_distance = math.inf

        # Find the closet player to the ball
        for player in players:
            role = self.player_states[player][0]
            if role != "Goalie":
                distance = self.player_distance_to_ball(player)
                if distance < min_distance:
                    min_distance = distance
                    closet_player = player

        # Reassign role if necessary
        for player in players:
            role = self.player_states[player][0]
            new_role = None
            if role == "Goalie":
                continue
            if (
                closet_player == player and role == "Striker"
            ):  # No need to change role since striker is already the closet player
                return
            elif closet_player == player:
                new_role = "Striker"
                print(f"⚽ {player} is now the closest and becomes Striker.")
            elif role != "Midfielder":
                new_role = "Midfielder"
                print(f"📥 {player} is no longer closest and becomes Midfielder.")
            if new_role:
                self.player_states[player][0] = new_role
                self.broadcast(f"ROLE|{player}|{new_role}\n")

    def check_robot_events(self):
        """Check if robot role should be updated"""
        ball_position = self.ball.getPosition()
        # Only update roles if large position changes are detected with the ball
        if self.get_distance(ball_position, self.last_ball_position) > 0.1:
            game_server.update_roles_based_on_proximity(game_server.team1)
            game_server.update_roles_based_on_proximity(game_server.team2)

    def check_ball_events(self):
        """Checks if the ball is out of bounds or a goal is scored or if its position change"""
        ball_x_position, ball_y_position, _ = self.ball.getPosition()

        # Check if ball is in the goal area
        if self.is_in_goal(ball_x_position, ball_y_position):
            scoring_team = "1" if ball_x_position > 0 else "2"
            self.handle_goal(scoring_team)
            return

        # Check if the ball went out of bounds
        if self.is_out_of_bounds(ball_x_position, ball_y_position):
            self.handle_out_of_bounds()
            return

        self.send_ball_position()

    def check_queue_messages(self):
        """
        Process all delayed messages for each client if they are ready,
        and notify the client of any penalty delay incurred.
        """
        curr_time = self.getTime()
        # Process through all client message queues
        for player, message_queue in self.delayed_messages.items():
            if message_queue.is_queue_empty():
                continue
            earliest_sent_time = message_queue.get_earliest_sent_time()
            message = ""

            # Process all ready messages
            while message_queue.can_message_be_processed(curr_time):
                message = message_queue.get_top_message()

            if message:
                # Process the last state of the client
                self.handle_message(message, True)
                # Notify the client of the delay for potential correction
                penalty = (
                    curr_time - earliest_sent_time
                )  # The time the message was delayed
                connection = self.clients[player]
                connection.sendall(f"REVERT|{penalty}\n".encode("utf-8"))

    def is_out_of_bounds(self, x_position, y_position):
        """Check if a point is out of the bounds of the field"""
        return abs(x_position) > FIELD_X_BOUND or abs(y_position) > FIELD_Y_BOUND

    def is_in_goal(self, x_position, y_position):
        """Check if a point is within the bounds of the goal area"""
        return (
            LOWER_GOAL_AREA_X < abs(x_position) < UPPER_GOAL_AREA_X
            and LOWER_GOAL_AREA_Y < abs(y_position) < UPPER_GOAL_AREA_Y
        )

    def handle_out_of_bounds(self):
        """Handles out-of-bounds by placing the ball near the closest non-goalie player
        who is not inside the goal area and is still within the bounds of the field"""
        print("⚠️ Ball went out of bounds")
        min_distance = math.inf
        closest_valid_player = None

        for player, state in self.player_states.items():
            role = state[0]
            x_position, y_position = state[2]

            # Skip goalies
            if role == "Goalie":
                continue

            # Check if player is not inside the goal zone and not out of bounds
            in_goal_zone = self.is_in_goal(x_position, y_position)
            in_bounds = not self.is_out_of_bounds(x_position, y_position)
            if not in_goal_zone and in_bounds:
                distance = self.player_distance_to_ball(player)
                if distance < min_distance:
                    min_distance = distance
                    closest_valid_player = player

        # Reposition if a valid player exist
        if closest_valid_player:
            robot = self.players[closest_valid_player]
            rotation = robot.getField("rotation").getSFRotation()
            axis, angle = rotation[:3], rotation[3]

            if abs(axis[2]) > 0.9:
                yaw = angle if axis[2] > 0 else -angle
            else:
                yaw = 0

            # Offset the ball in front of the player based on their facing direction
            x, y = self.player_states[closest_valid_player][2]
            dx = OFFSET_DISTANCE * math.cos(yaw)
            dy = OFFSET_DISTANCE * math.sin(yaw)

            # Clamp spawn near field bounds, avoid corners
            x_bound = FIELD_X_BOUND + 2 * OFFSET_DISTANCE
            y_bound = FIELD_Y_BOUND - OFFSET_DISTANCE
            new_ball_x = max(min(x + dx, x_bound), -x_bound)
            new_ball_y = max(min(y + dy, y_bound), -y_bound)
            self.ball.getField("translation").setSFVec3f([new_ball_x, new_ball_y, 0.07])
            self.send_ball_position()
            print(
                f"📦 Ball reset near Player {closest_valid_player} at ({new_ball_x:.2f}, {new_ball_y:.2f})"
            )
        else:
            # Reset the game if all robots are out of bounds or in goal area
            # Inform robots to start recovery motion
            self.broadcast("RESET\n")
            self.game_started = False

            # Move robots to the sideline
            self.move_robots_to_sideline()
            print("📦 Game reset because there is no valid player around!")

    def handle_goal(self, scoring_team):
        """Update game state after a goal is scored"""
        print(f"🥅 Goal scored by Team {scoring_team}!")
        # Update the team's score
        if scoring_team == "1":
            self.team1.add_score()
            self.update_score("1", self.team1.get_score())
        else:
            self.team2.add_score()
            self.update_score("2", self.team2.get_score())

        # Inform robots to start recovery motion
        self.broadcast("RESET\n")
        self.game_started = False

        # Move robots to the sideline
        self.move_robots_to_sideline()

    def reset_game(self):
        """Reset the position and rotation of the ball and the robots"""
        print("🔄 Resetting ball and players to initial positions")

        # Reset ball position
        self.ball.getField("translation").setSFVec3f(INIT_BALL_POS)
        self.last_ball_position = INIT_BALL_POS

        # Reset all player states and inform clients
        self.assign_initial_team_states(self.team1)
        self.assign_initial_team_states(self.team2)
        self.send_initial_states()
        self.start_game()

    def update_score(self, team_number, score):
        """Update the score of the team"""
        if team_number == "1":
            self.setLabel(
                1, f"Red Team: {score}", 0, 0.01, 0.15, 0xFF0000, 0.0, "Arial"
            )
        else:
            self.setLabel(
                2, f"Blue Team: {score}", 0, 0.93, 0.15, 0x0000FF, 0.0, "Arial"
            )

    def move_robots_to_sideline(self):
        print("🚧 Moving robots to sideline for recovery")
        spacing = 1.0  # Spacing between players on sideline
        x_start = -ROBOT_X_POSITION
        y_sideline = FIELD_Y_BOUND
        for i, player_id in enumerate(self.players):
            x = x_start + spacing * i
            self.update_position(player_id, x, y_sideline)
            self.update_rotation(player_id, -math.pi / 2)
            self.apply_player_state_in_simulation(player_id)

    def handle_client_delay(self, message, client, send_time, tolerance):
        """
        Handles network delay logic for a client message.

        Simulates artificial delay, evaluates whether the message is delayed
        due to network latency or existing pipeline delay, and queues it if needed.

        Args:
            message (str): The incoming message from the client.
            client (str): The client identifier.
            send_time (float): The time the message was originally sent.
            tolerance (float): The allowable delay threshold.
        Returns:
            bool: True if the message was delayed and queued, False if it can be processed immediately.
        """
        # Inject artificial delay
        curr_time = self.getTime()
        artificial_delay = injector.inject_artificial_network_delay()
        simulated_arrival_time = send_time + artificial_delay
        client_queue = self.delayed_messages[client]

        # If any message is still queued, this one must be delayed behind it
        if not client_queue.is_queue_empty():
            client_queue.add_message(simulated_arrival_time, send_time, message)
            return True

        # Otherwise, check if this message is late on its own
        delay_detected = observer.detect_network_delay(
            simulated_arrival_time, curr_time, tolerance
        )
        if delay_detected:
            # Add the message to the queue for processing later on
            client_queue.add_message(simulated_arrival_time, send_time, message)
            return True
        return False

    def add_terrain_patches(self):
        """Create fixed terrain zones to simulate roughness in real soccer patches"""
        patches = injector.get_terrain_patches()
        root = self.getRoot()
        for patch in patches:
            root.getField("children").importMFNodeFromString(-1, patch)
        print("🎨 6 terrain patches visualized.")

    def apply_terrain_faults(self):
        """Add some force on robots that are on top of the patches"""
        current_time = float(self.getTime())
        for player_id, robot in self.players.items():
            x_position, y_position, _ = robot.getPosition()
            force, offset = injector.inject_terrain_forces(
                player_id, current_time, x_position, y_position
            )
            if force and observer.detect_terrain_fault(force, [50, 50]):
                robot.addForceWithOffset(force, offset, False)
                print(
                    f"🌀 Terrain fault triggered on {player_id} at {x_position}, {y_position} with forces: {force}"
                )

    def apply_wind_force(self):
        """Apply a random wind force once the cooldown is over"""
        current_time = self.getTime()
        x_force, y_force = injector.inject_wind_forces(current_time, 0.2)
        if x_force and observer.detect_wind_forces([x_force, y_force], [0.025, 0.025]):
            # Apply to all robots and the ball
            for robot in self.players.values():
                robot.addForce((x_force, y_force, 0), False)
            self.ball.addForce((x_force, y_force, 0), False)
            print(f"💨 Wind force triggered in the direction: {x_force}, {y_force}")


host = "127.0.0.1"
port = 5555

# Fault injection and detection
observer = ArtificialFaultObserver()
injector = ArtificialFaultInjector(lambd=1)

# Start the server in a separate thread
game_server = GameServer(6)
server_thread = threading.Thread(
    target=game_server.start_server,
    args=(
        host,
        port,
    ),
    daemon=True,
)
server_thread.start()

timestep = int(game_server.getBasicTimeStep())

# Create the score text
game_server.setLabel(1, "Red Team: 0", 0, 0, 0.15, 0xFF0000, 0.0, "Arial")
game_server.setLabel(2, "Blue Team: 0", 0, 0.93, 0.15, 0x0000FF, 0.0, "Arial")

# Webots main loop
while game_server.step(timestep) != 1:
    if game_server.game_started:
        game_server.check_queue_messages()
        game_server.check_robot_events()
        game_server.check_ball_events()
        game_server.apply_terrain_faults()
        game_server.apply_wind_force()
