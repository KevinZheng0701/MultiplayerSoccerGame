from controller import Supervisor
import socket
import threading
import uuid
import math
import time

GOALIE_X_POSITION = 4.5  # The starting x position of the goalie robot
ROBOT_X_POSITION = 2.5 # The starting x position of the robots
ROBOT_Z_POSITION = 0.333 # The starting z position of the robots represents the height

class Team:
    def __init__(self, team_number = 0, capacity = 0):
        self.players = set()
        self.team_lock = threading.Lock()
        self.capacity = capacity
        self.team_number = team_number
        self.score = 0

    def __len__(self):
        """Return the number of players in the team"""
        return len(self.players)
    
    def add_player(self, player):
        """Add a player to the team"""
        with self.team_lock:
            if len(self.players) < self.capacity:
                self.players.add(player)

    def remove_player(self, player):
        """Remove a player from the team"""
        with self.team_lock:
            self.players.discard(player)

    def get_players(self):
        """Get all players from the team"""
        return list(self.players)
    
    def get_team_strategy(self, state, ball):
        """Decide the role for each player"""
        return None
    
    def get_team_number(self):
        """Get the team number"""
        return self.team_number
    
    def has(self, player_id):
        """Returns true if player is in the team else false"""
        return player_id in self.players
    
    def add_score(self):
        """Add one to the score"""
        self.score += 1

    def get_score(self):
        """Get the current score of the team"""
        return self.score
    
class GameServer(Supervisor):
    def __init__(self, players_limit = 6):
        if players_limit < 2:
            print("⚠️ Minimum player size is 2.")
            players_limit = 2
        elif players_limit > 8:
            print("⚠️ Minimum player size is 8.")
            players_limit = 8
        super().__init__() # Initialize as a supervisor so it has access to objects in the world
        self.players_limit = players_limit
        self.client_lock = threading.Lock()
        self.clients = {} # Store player id to their connections
        self.team1 = Team(1, players_limit // 2)
        self.team2 = Team(2, players_limit // 2)
        self.player_states = {}  # Store player game stats
        self.players = {} # Store the reference to the robots
        self.ball = self.getFromDef("BALL") # Get the soccer ball in the world
        self.last_ball_position = [0, 0, 0]
        self.game_started = False
        self.last_roles = {}
        self.last_role_update_time = 0
        self.role_update_interval = 0.5


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

    def handle_message(self, message):
        """Handles messages from the clients"""
        message_parts = message.split("|")
        message_type = message_parts[0]
        sender = message_parts[1]
        match message_type:
            case "POS":
                player_id = message_parts[1]
                x_position, y_position = float(message_parts[2]), float(message_parts[3])
                angle = float(message_parts[4])
                # 🛡️ Update only if we know about this player
                if player_id in self.player_states:
                    self.update_position(player_id, x_position, y_position)
                    self.update_rotation(player_id, angle)
                else:
                    print(f"⚠️ Unknown player {player_id} received. Ignoring POS update.")
            case "ACK":
                print("Acknowledgement.")
                if sender in self.player_states:
                    self.player_states[sender][1] = 'ACK'
            case "MOVE":
                print("Moving.")
                #position = message_parts[1]
                #broadcast(f"MOVE|{sender}|{position}", sender)
            case "GOAL":
                print("Scored a goal.")
                #broadcast(f"GOAL|{sender}")
            case "KICK":
                print("Ball kick.")
                #broadcast(f"KICK|{sender}")
            case "GET":
                print("Requesting information.")
                #sender.sendall("Data".encode("utf-8"))
            case "ROBOT":
                self.players[sender] = self.getFromDef(message_parts[2]) # Add the robot reference
                team_number = message_parts[3]

                # Send previously connected robots to the new client
                team1 = self.team1.get_players()
                team2 = self.team2.get_players()
                connection = self.clients[sender]
                for id in team1:
                    if id != sender:
                        connection.sendall(f'INFO|{id}|1\n'.encode("utf-8"))
                for id in team2:
                    if id != sender:
                        connection.sendall(f'INFO|{id}|2\n'.encode("utf-8"))
                
                # Add the player to its team and send info to other clients
                self.broadcast(f'INFO|{sender}|{team_number}\n', sender)
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
                print(message)
                print("Unknown Type.")

    def handle_client_connection(self, connection, address):
        """Handles new client connections and assigns the client a team and a unqiue identifer"""
        player_id = str(uuid.uuid4())
        print(f"🆔 New client connection: {player_id}, {address}.")

        # Add player to the clients list
        with self.client_lock:
            self.clients[player_id] = connection

        # Send id to the connected robot
        connection.sendall(f'INFO|{player_id}\n'.encode("utf-8"))
        print(f"📢 Clients connected: {len(self.clients)}")

    def send_initial_states(self):
        """Broadcasts the initial state of the game(players and ball)"""
        for player, details in self.player_states.items():
            role = details[0]
            x_position, y_position = details[2]
            angle = details[3]
            state = details[4] if len(details) > 4 else "Idle"
            state_message = f'POS|{player}|{x_position}|{y_position}|{angle}|{state}\n'
            self.broadcast(state_message, player)
            role_message = f'ROLE|{player}|{role}\n'
            self.broadcast(role_message)
        self.send_ball_position(True)

    def assign_initial_team_states(self, team):
        """Assigns starting states to players in a team"""
        players = team.get_players()
        team_number = team.get_team_number()
        x_position, angle = self.get_initial_x_position_and_rotation(team_number)
        y_position = self.calculate_player_y_position(0)

        # In the order of role, current action, xy coordinate, rotation, etc
        self.player_states[players[0]] = ["Striker", None, [x_position, y_position], angle]
        self.apply_player_state_in_simulation(players[0])

        if len(players) >= 2:
            goalie_x_position = -GOALIE_X_POSITION if team_number == 1 else GOALIE_X_POSITION
            self.player_states[players[1]] = ["Goalie",  None, [goalie_x_position, y_position if team_number == 1 else -y_position], angle]
            self.apply_player_state_in_simulation(players[1])
            
            for i in range(2, len(players)):
                y_position = self.calculate_player_y_position(i)
                self.player_states[players[i]] = ["Midfielder", None, [x_position, y_position if team_number == 1 else -y_position], angle]
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
        return y if player_index % 2 == 0 else -y # If the index is even then the position will be on the right else it will be on the left

    def apply_player_state_in_simulation(self, player_id):
        """Updates the position and rotation of the robot"""
        player_state = self.player_states[player_id]
        robot = self.players[player_id]

        # Update the rotation
        rotation_field = robot.getField("rotation")
        rotation_field.setSFRotation([0, 0, 1, player_state[3]])

        # Update the position
        translation_field = robot.getField("translation")
        translation_field.setSFVec3f([player_state[2][0], player_state[2][1], ROBOT_Z_POSITION])

    def start_game(self):
        """Sends a start message to the players"""
        self.broadcast(f'START|1\n')
        self.game_started = True

    def is_player_near_ball(self, player_id, threshold = 0.5):
        """Checks if the player is close enough to the ball"""
        distance = self.player_distance_to_ball(player_id)
        return distance <= threshold
   
    def player_distance_to_ball(self, player_id):
        """Calculates the Euclidean distance from a player to the ball"""
        player_state = self.player_states[player_id]
        x_position, y_position = player_state[2], player_state[3]
        ball_position = self.ball.getPosition()
        return self.get_distance([x_position, y_position], ball_position)

    def send_ball_position(self, force = False):
        """Sends the current ball position to the clients"""
        ball_position = self.ball.getPosition()
        # If there is substanal change in the ball position or a force send
        if force or self.get_distance(ball_position, self.last_ball_position) > 0.1:
            self.last_ball_position = ball_position
            self.broadcast(f'BALL|{ball_position[0]:.2f}|{ball_position[1]:.2f}|{ball_position[2]:.2f}\n')

    def get_distance(self, point_one, point_two):
        """Returns the Euclidean distance between two points in xy-plane"""
        return math.sqrt((point_two[0] - point_one[0]) ** 2 + (point_two[1] - point_one[1]) ** 2)

    def update_position(self, player_id, x, y):
        """Updates the position of the player"""
        self.player_states[player_id][2] = [x, y]

    def update_rotation(self, player_id, angle):
        """Updates the rotation of the player"""
        self.player_states[player_id][3] = angle

    def update_state(self, player_id, state):
        """Updates the state of the player"""
        if player_id in self.player_states:
            self.player_states[player_id][4] = state
        else:
            print(f"⚠️ Tried to update state for unknown player {player_id}")

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

    def broadcast(self, message, sender = None):
        """Sends message to every client besides the sender"""
        for id, conn in self.clients.items():
            if id != sender:
                conn.sendall(message.encode("utf-8"))
    
    def update_roles_based_on_proximity(self):
        """Update roles: assign Striker to closest player to ball per team; others become Midfielders."""
        print("🔄 Checking for role updates based on proximity...", flush=True)
        team_roles = {1: [], 2: []}

        # Get current ball position
        ball_position = self.ball.getPosition()

        # Group players by team (excluding goalies)
        for player_id, state in self.player_states.items():
            role = state[0]
            team = 1 if self.team1.has(player_id) else 2
            if role != "Goalie":
                position = state[2]
                distance = self.get_distance(position, ball_position)
                team_roles[team].append((player_id, distance))
                print(f"🧮 {player_id} (Team {team}) is {distance:.2f} units from the ball", flush=True)

        # For each team, assign closest as striker, rest as midfielders
        for team, players in team_roles.items():
            if not players:
                continue

            players.sort(key=lambda x: x[1])  # sort by distance to ball
            closest_player_id = players[0][0]

            for player_id, _ in players:
                new_role = "Striker" if player_id == closest_player_id else "Midfielder"
                current_role = self.player_states[player_id][0]
                print(f"🔍 {player_id} current role: {current_role}, new role: {new_role}", flush=True)

                if current_role != new_role:
                    print(f"🔁 Changing {player_id} to {new_role}", flush=True)
                    self.player_states[player_id][0] = new_role
                    self.broadcast(f"ROLE|{player_id}|{new_role}\n")
                    last = self.last_roles.get(player_id)
                    if last != new_role:
                        if new_role == "Striker":
                            print(f"⚽ {player_id} is now the closest and becomes Striker.", flush=True)
                        else:
                            print(f"📥 {player_id} is no longer closest and becomes Midfielder.", flush=True)
                        self.last_roles[player_id] = new_role
    
    def check_ball_events(self):
        """Checks if the ball is out of bounds or scored a goal."""
        ball_position = self.ball.getPosition()
        ball_x, ball_y = ball_position[0], ball_position[1]

        # Refined boundaries based on field layout
        sideline_y_limit = 2.85     # sidelines
        field_x_limit = 5.3         # total x span
        goal_area_x_min = 4.45
        goal_area_x_max = 5.0
        goal_area_y_min = -1.35
        goal_area_y_max = 1.35

        # Skip out-of-bounds only if ball is deep inside goal, but don't block goal scoring
        if abs(ball_x) <= goal_area_x_min and goal_area_y_min <= ball_y <= goal_area_y_max:
            return  # Ball is sitting inside the goal box, do nothing

        # Behind net but NOT within scoring area = out of bounds
        if abs(ball_x) > goal_area_x_min and not (goal_area_y_min <= ball_y <= goal_area_y_max):
            print("⚠️ Ball went out of bounds (behind net, outside scoring area)!")
            time.sleep(1)  # Give time for position updates to arrive
            self.handle_out_of_bounds()
            return

        # Ball crossed sideline = out of bounds
        if abs(ball_y) > sideline_y_limit or abs(ball_x) > field_x_limit:
            print("⚠️ Ball went out of bounds (sideline or endline)!")
            time.sleep(1)  # Give time for position updates to arrive
            self.handle_out_of_bounds()
            return

        # Ball is fully past the goal line and within goal bounds = score
        if abs(ball_x) > goal_area_x_min and goal_area_y_min <= ball_y <= goal_area_y_max:
            scoring_team = 2 if ball_x < 0 else 1
            team_color = "Red" if scoring_team == 1 else "Blue"
            print(f"🥅 Goal scored by Team {scoring_team} ({team_color})!")
            self.handle_goal(scoring_team)

    def handle_out_of_bounds(self):
        """Handles out-of-bounds by placing the ball near the closest non-goalie player 
        who is NOT inside the goal area and is still within field bounds."""
        ball_position = self.ball.getPosition()
        min_dist = float('inf')
        fallback_player = None
        closest_valid_player = None

         # field and goal area limits
        field_x_limit = 5.3
        field_y_limit = 2.85
        goal_x_min = 4.45
        goal_x_max = 5.0
        goal_y_min = -1.35
        goal_y_max = 1.35

        for player_id, state in self.player_states.items():
            role = state[0]
            player_x, player_y = state[2]

            # Skip goalies
            if role == "Goalie":
                continue

            # Save a fallback just in case
            if fallback_player is None:
                fallback_player = player_id

            # Check if player is NOT inside the goal zone
            in_goal_zone = goal_x_min <= abs(player_x) <= goal_x_max and goal_y_min <= player_y <= goal_y_max
            in_bounds = abs(player_x) <= field_x_limit and abs(player_y) <= field_y_limit

            # Ignore players inside goal zone
            if not in_goal_zone and in_bounds:
                distance = self.get_distance([player_x, player_y], ball_position)
                if distance < min_dist:
                    min_dist = distance
                    closest_valid_player = player_id

        # Use fallback if all players were in the goal
        chosen_player = closest_valid_player if closest_valid_player else fallback_player

        if chosen_player:
            robot = self.players[chosen_player]
            rotation = robot.getField("rotation").getSFRotation()
            axis, angle = rotation[:3], rotation[3]

            if abs(axis[2]) > 0.9:
                yaw = angle if axis[2] > 0 else -angle
            else:
                yaw = 0

            player_pos = self.player_states[chosen_player][2]
            x, y = player_pos

            # Offset the ball in front of the player based on their facing direction
            offset_distance = 0.4  # distance in front of robot
            dx = offset_distance * math.cos(yaw)
            dy = offset_distance * math.sin(yaw)

            # Clamp spawn near field bounds, avoid corners
            new_ball_x = max(min(x + dx, 5.1), -5.1)
            new_ball_y = max(min(y + dy, 2.8), -2.8)

            self.ball.getField("translation").setSFVec3f([new_ball_x, new_ball_y, 0.07])
            print(f"📦 Ball reset near Player {chosen_player} at ({new_ball_x:.2f}, {new_ball_y:.2f})")

    def handle_goal(self, scoring_team):
        print(f"🥅 Handling goal scored by Team {scoring_team}")

        # Update the team's score
        if scoring_team == 1:
            self.team1.add_score()
            self.update_score("1", self.team1.get_score())
        else:
            self.team2.add_score()
            self.update_score("2", self.team2.get_score())

        # Inform robots to start recovery motion
        self.broadcast("GOAL|RECOVER\n")
        for player_id in self.player_states:
            self.player_states[player_id][1] = 'WAITING'

        # TELEPORT robots to sideline
        self.teleport_robots_to_sideline()

        # Let Webots physics engine stabilize robots at sideline
        stabilization_steps = int(1.5 / (self.getBasicTimeStep() / 1000.0))  # 1.5 seconds worth of steps
        print(f"⏳ Stabilizing robots at sideline with {stabilization_steps} steps...")
        for _ in range(stabilization_steps):
            self.step(int(self.getBasicTimeStep()))

        # THEN wait for acknowledgments
        self.await_robot_acknowledgments()

        # Reset ball and player starting positions
        self.reset_positions_after_goal()

        # Send second recover command
        self.broadcast("GOAL|RECOVER_AFTER_RESET\n")
        for player_id in self.player_states:
            self.player_states[player_id][1] = 'WAITING'

        # Wait for ACKs again
        self.await_robot_acknowledgments()

        # Resume play
        self.broadcast("GOAL|RESET\n")
        for player_id in self.player_states:
            self.player_states[player_id][1] = None

    def await_robot_acknowledgments(self, timeout=15):
        print("⏳ Awaiting acknowledgments from all robots...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(state[1] == 'ACK' for state in self.player_states.values()):
                print("✅ All robots acknowledged recovery.")
                return
            time.sleep(0.05)
        print("⚠️ Timeout waiting for robot acknowledgments.")

    def reset_positions_after_goal(self):
        print("🔄 Resetting ball and players to initial positions.")

        # Reset ball position
        self.ball.getField("translation").setSFVec3f([0, 0, 0.07])
        self.last_ball_position = [0, 0, 0.07]
        self.send_ball_position(force=True)

        # Reset all player states and inform clients
        self.assign_initial_team_states(self.team1)
        self.assign_initial_team_states(self.team2)

        stabilization_steps = int(1.0 / (self.getBasicTimeStep() / 1000.0))  # 1 second worth of steps
        print(f"⏳ Stabilizing robots after reset with {stabilization_steps} steps...")
        for _ in range(stabilization_steps):
            self.step(int(self.getBasicTimeStep()))

        self.send_initial_states()

        # Reset player ACK states
        for state in self.player_states.values():
            state[1] = None

    def update_score(self, team_number, score):
        """Update the score of the team"""
        if team_number == "1":
            self.setLabel(
                0,
                f'Red Team: {score}',
                0, 0.01,
                0.15,
                0xFF0000,
                0.0,
                "Arial"
            )
        else:
            self.setLabel(
            1,
            f'Blue Team: {score}',
            0.725, 0.01,
            0.15,
            0x0000FF,
            0.0,
            "Arial"
        )
            
    def teleport_robots_to_sideline(self):
        print("🚧 Teleporting robots to sideline for recovery.")
        y_sideline = -2.85
        spacing = 1.0  # spacing between players on sideline
        x_start = -2.5
        for i, player_id in enumerate(self.players):
            robot = self.players[player_id]
            x = x_start + spacing * i
            translation_field = robot.getField("translation")
            translation_field.setSFVec3f([x, y_sideline, ROBOT_Z_POSITION])

            rotation_field = robot.getField("rotation")
            rotation_field.setSFRotation([0, 0, 1, 0])  # facing forward


host = "127.0.0.1"
port = 5555

# Start the server in a separate thread
game_server = GameServer(6)
server_thread = threading.Thread(target=game_server.start_server, args=(host, port,), daemon=True)
server_thread.start()

game_server.last_role_update_time = 0
game_server.role_update_interval = 0.5  # seconds

timestep = int(game_server.getBasicTimeStep())

# Create the score text
game_server.setLabel(
    0,
    "Red Team: 0",
    0, 0.01,
    0.15,
    0xFF0000,
    0.0,
    "Arial"
)

game_server.setLabel(
    1,
    "Blue Team: 0",
    0.725, 0.01,
    0.15,
    0x0000FF,
    0.0,
    "Arial"
)

# Webots main loop
while game_server.step(timestep) != 1:
    if game_server.game_started:
        game_server.send_ball_position()
        game_server.check_ball_events()


        current_time = game_server.getTime()

        if current_time - game_server.last_role_update_time > game_server.role_update_interval:
            game_server.update_roles_based_on_proximity()
            game_server.last_role_update_time = current_time
