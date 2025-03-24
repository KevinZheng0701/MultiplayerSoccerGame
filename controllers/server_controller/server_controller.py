from controller import Supervisor
import socket
import threading
import uuid
import time
import math

class Team:
    def __init__(self, capacity = 0):
        self.players = set()
        self.team_lock = threading.Lock()
        self.capacity = capacity

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

class GameServer(Supervisor):
    def __init__(self, players_limit=6):
        super().__init__() # Initialize as a supervisor so it has access to objects in the world
        self.players_limit = players_limit
        self.client_lock = threading.Lock()
        self.clients = {} # Store player id to their connections
        self.team1 = Team(players_limit // 2)
        self.team2 = Team(players_limit // 2)
        self.player_states = {}  # Store player game stats
        self.ball = self.getFromDef("BALL") # Get the soccer ball in the world 
        self.players = {} # Store the reference to the robots

    def start_server(self):
        """Start the server and wait for connections"""
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
        """Listen for incoming messages from clients"""
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
        """
        try:
            while True:
                data = connection.recv(1024).decode().strip()
                if not data:
                    break
                print(f"📡 Received: {data}")

                message_parts = data.split("|")
                message_type = message_parts[0]

                if message_type == "ROLE_ACK":
                    print(f"✅ {message_parts[1]} acknowledged role.")

                elif message_type == "MOVE":
                    player_id = message_parts[1]
                    new_position = json.loads(message_parts[2])  # Expecting [x, y]
                    self.player_positions[player_id] = new_position
                    print(f"🚶 Player {player_id} moved to {new_position}")
                    self.assign_roles()

                elif message_type == "KICK":
                    player_id = message_parts[1]

                    # ✅ TODO: Ensure player is close enough to the ball to kick
                    if self.is_near_ball(player_id):
                        print(f"⚽ Player {player_id} attempted a kick.")

                        # ✅ TODO: Apply physics-based ball movement
                        self.apply_ball_kick(player_id)

                        # ✅ TODO: Send updated game state to all clients
                        self.broadcast_game_state()
                    else:
                        print(f"❌ Player {player_id} too far from ball to kick.")

                elif message_type == "GOAL":
                    print("🥅 Goal scored!")
        """

    def handle_message(self, message):
        """Handle messages from the clients"""
        message_parts = message.split("|")
        message_type = message_parts[0]
        sender = message_parts[1]
        match message_type:
            case "POS":
                # Update the position of the client and broadcast it to every other clients
                x_position, y_position = message_parts[2], message_parts[3]
                x_rotation, y_rotation, z_rotation = message_parts[4], message_parts[5], message_parts[6]
                self.update_position(sender, x_position, y_position)
                self.update_rotation(sender, x_rotation, y_rotation, z_rotation)
                message = f'POS|{sender}|{x_position}|{y_position}|{x_rotation}|{y_rotation}|{z_rotation}\n'
                self.broadcast(message)
            case "ACK":
                print("Acknowledgement.")
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
            self.players[player_id] = self.getFromDef("Robot_" + str(len(self.clients))) # Add the robot reference
        # Assign client a team
        if len(self.team1) <= len(self.team2):
            team_number = 1
            self.team1.add_player(player_id)
        else:
            team_number = 2
            self.team2.add_player(player_id)

        # Send client information
        message = f"INFO|{team_number}|{player_id}|\n"
        connection.sendall(message.encode("utf-8"))
        print(f"📢 Clients connected: {len(self.clients)}")
        
        # If all clients joined, then start the game by first assigning initial roles
        with self.client_lock:
            if len(self.clients) == self.players_limit:
                self.send_intial_states()

    def send_intial_states(self):
        '''One member from each team will be a designated goalie, one will have no roles, and the rest will be midfielder'''
        self.assign_initial_team_states(self.team1)
        self.assign_initial_team_states(self.team2)
        # Send the role and starting information to all the clients
        for player, details in self.player_states.items():
            role = details[0]
            x_position, y_position = details[2], details[3]
            x_rotation, y_rotation, z_rotation = details[4], details[5], details[6]
            conn = self.clients[player]
            conn.sendall(f'ROLE|{role}\n'.encode('utf-8'))
            message = f'POS|{player}|{x_position}|{y_position}|{x_rotation}|{y_rotation}|{z_rotation}\n'
            self.broadcast(message)
        # Send ball position to clients
        self.send_ball_position()

    def assign_initial_team_states(self, team):
        """Assign starting states to players in a team"""
        players = team.get_players()
         # In the order of role, current action, xy coordinate, rotation, etc
        if len(players) == 1:
            self.player_states[players[0]] = ["None", None, 0, 0, 0, 0, 0]
        else:
            self.player_states[players[0]] = ["Goalie", None, 0, 0, 0, 0, 0]
            self.player_states[players[1]] = ["None", None, 0, 0, 0, 0, 0]
        for i in range(2, len(players)):
            self.player_states[players[i]] = ["Midfielder", None, 0, 0, 0, 0, 0]

    def is_near_ball(self, player_id, threshold=0.5):
        """Checks if the player is close enough to the ball to kick."""
        player_x, player_y = self.player_states[player_id]
        ball_x, ball_y = self.ball_position
        distance = ((player_x - ball_x) ** 2 + (player_y - ball_y) ** 2) ** 0.5
        return distance <= threshold  # Allow kicking if within `threshold` meters

    def apply_ball_kick(self, player_id):
        """Moves the ball in the direction of the player's kick."""
        if player_id in self.player_positions:
            player_x, player_y = self.player_positions[player_id]

            # Simple physics: move ball in the same direction as player
            direction_x = self.ball_position[0] - player_x
            direction_y = self.ball_position[1] - player_y
            speed = 1.0  # Adjustable kick speed

            # Normalize direction vector and apply movement
            magnitude = (direction_x**2 + direction_y**2) ** 0.5
            if magnitude > 0:
                direction_x /= magnitude
                direction_y /= magnitude

            # Update ball position
            self.ball_position[0] += direction_x * speed
            self.ball_position[1] += direction_y * speed

            print(f"⚽ Ball moved to {self.ball_position}")

    def player_distance_to_ball(self, player):
        """Calculates the Euclidean distance from a player to the ball."""
        player = self.player_states[player]
        x_position, y_position = player[2], player[3]
        return math.sqrt((self.ball_position[0] - x_position) ** 2 + (self.ball_position[2] - y_position) ** 2)

    def get_current_strategy(self):
        """Determines the current team strategy based on game conditions."""

        # Count how many players are near the ball
        ball_x, ball_y = self.ball_position
        players_near_ball = 0
        possession_team = None  # Track which team is closest to the ball

        for player_id, position in self.player_positions.items():
            x, y = position
            distance_to_ball = ((x - ball_x) ** 2 + (y - ball_y) ** 2) ** 0.5

            # If a player is close enough to the ball, count them
            if distance_to_ball < 1.0:  # Distance threshold (adjust if needed)
                players_near_ball += 1
                if player_id in self.team1:
                    possession_team = 1
                elif player_id in self.team2:
                    possession_team = 2

        # Strategy Decision Logic
        if possession_team == 1:
            if ball_x < 0:  # Ball is in defensive half
                return "defensive"
            else:  # Ball is in attacking half
                return "aggressive"
        elif possession_team == 2:
            if ball_x > 0:  # Ball is in defensive half
                return "defensive"
            else:  # Ball is in attacking half
                return "aggressive"

        # Default to "neutral" if no team is currently in possession
        return "neutral"

    def process_action_queue(self):
        """Processes queued critical actions with acknowledgment to ensure synchronization."""
        while self.action_queue:
            action_type, player_id = self.action_queue.pop(0)

            # Broadcast action request to all clients
            for conn in self.clients.values():
                conn.sendall(
                    f"VERIFY_ACTION|{action_type}|{player_id}\n".encode("utf-8")
                )

            # Wait for acknowledgment from majority (not all)
            if self.await_acknowledgment(
                player_id, action_type, required_ack_ratio=0.5
            ):
                print(f"✅ {action_type} confirmed by majority.")
            else:
                print(f"❌ {action_type} failed, reverting.")
                for conn in self.clients.values():
                    conn.sendall(
                        f"REVERT_ACTION|{action_type}|{player_id}\n".encode("utf-8")
                    )

    def await_acknowledgment(
        self, player_id, action_type, required_ack_ratio=0.5, timeout=1
    ):
        """Ensures a critical action is acknowledged by a majority of clients before proceeding."""
        acknowledgments = set()
        start_time = time.time()
        required_acks = max(1, int(len(self.clients) * required_ack_ratio))

        while time.time() - start_time < timeout:
            for conn in self.clients.values():
                try:
                    conn.settimeout(0.1)  # Prevents blocking indefinitely
                    ack = conn.recv(1024).decode().strip()

                    if ack == f"{action_type}_ACK|{player_id}":
                        acknowledgments.add((player_id, action_type))
                        print(
                            f"✅ Acknowledgment received for {action_type} from {player_id}"
                        )

                except socket.timeout:
                    continue  # Avoid blocking
                except Exception as e:
                    print(f"⚠️ Error receiving acknowledgment: {e}")
                    continue

            if len(acknowledgments) >= required_acks:
                print(
                    f"✅ {action_type} confirmed by majority ({len(acknowledgments)}/{required_acks})"
                )
                return True  # Majority confirmed

        print(
            f"❌ {action_type} failed: only {len(acknowledgments)}/{required_acks} acknowledgments received."
        )
        return False  # Timeout reached

    def send_ball_position(self):
        """Send the current ball position to the clients"""
        ball_position = self.ball.getPosition()
        self.broadcast(f'BALL|{ball_position[0]}|{ball_position[1]}|{ball_position[2]}\n')

    def update_position(self, player, x, y):
        """Update the position of the player"""
        self.players[player][2] = x
        self.players[player][3] = y

    def update_rotation(self, player, x, y, z):
        """Update the rotation of the player"""
        self.players[player][4] = x
        self.players[player][5] = y
        self.players[player][6] = z

    def update_ball_position(self, x, y, z):
        """Update the position of the ball"""
        self.ball[0] = x
        self.ball[1] = y
        self.ball[2] = z

    def remove_client(self, connection):
        """Removes disconnected clients"""
        with self.client_lock:
            for player_id, conn in self.clients.items():
                if conn == connection:
                    del self.clients[player_id]
                    with self.team_lock:
                        self.team1.discard(player_id)
                        self.team2.discard(player_id)
                    print(f"❌ Removing player {player_id}")
                    break
        connection.close()

    def broadcast(self, message, sender = None):
        """Sends message to every client besides the sender"""
        for id, conn in self.clients.items():
            if id != sender:
                conn.sendall(message.encode("utf-8"))

host = "127.0.0.1"
port = 5555

# Start the server in a separate thread
game_server = GameServer(6)
server_thread = threading.Thread(target=game_server.start_server, daemon=True)
server_thread.start()

timestep = int(game_server.getBasicTimeStep())

# Webots main loop
while game_server.step(timestep) != 1:
    pass
