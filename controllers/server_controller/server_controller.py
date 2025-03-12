from controller import Robot
import socket
import threading
import uuid
import json
import time

host = "127.0.0.1"
port = 5555

class GameServer:
    def __init__(self, players_limit=6):
        self.client_lock = threading.Lock()
        self.team_lock = threading.Lock()
        self.players_limit = players_limit
        self.clients = {}
        self.team1 = set()
        self.team2 = set()
        self.roles = ["Striker", "Midfielder", "Defender"]  # Assign these after goalies
        self.ball_position = [0, 0]  
        self.player_positions = {}   # Store player positions

    def start_server(self):
        """Start the server and wait for connections."""
        print("Starting server...")
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((host, port))
            server.listen(self.players_limit)
            print(f"Server started on {host}:{port}")
        except ConnectionError as error:
            print(f"Error binding server: {error}")
            return
        
        threading.Thread(target=self.send_game_updates, daemon=True).start()

        while True:
            try:
                connection, address = server.accept()
                self.handle_client_connection(connection, address)
                thread = threading.Thread(target=self.handle_client_communication, args=(connection,))
                thread.start()
            except Exception as e:
                print(f"Error while accepting connection: {e}")
                print(f"Error details: {e}")
                break

    def handle_client_communication(self, connection):
        """Handles incoming messages from clients."""
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

        except Exception as e:
            print(f"⚠️ Client error: {e}")

        finally:
            self.remove_client(connection)

    def is_near_ball(self, player_id, threshold=0.5):
        """Checks if the player is close enough to the ball to kick."""
        if player_id in self.player_positions:
            player_x, player_y = self.player_positions[player_id]
            ball_x, ball_y = self.ball_position
            distance = ((player_x - ball_x) ** 2 + (player_y - ball_y) ** 2) ** 0.5
            return distance <= threshold  # Allow kicking if within `threshold` meters
        return False

    def apply_ball_kick(self, player_id):
        """Moves the ball in the direction of the player's kick."""
        if player_id in self.player_positions:
            player_x, player_y = self.player_positions[player_id]

            # Simple physics: move ball in the same direction as player
            direction_x = self.ball_position[0] - player_x
            direction_y = self.ball_position[1] - player_y
            speed = 1.0  # Adjustable kick speed

            # Normalize direction vector and apply movement
            magnitude = (direction_x ** 2 + direction_y ** 2) ** 0.5
            if magnitude > 0:
                direction_x /= magnitude
                direction_y /= magnitude
            
            # Update ball position
            self.ball_position[0] += direction_x * speed
            self.ball_position[1] += direction_y * speed

            print(f"⚽ Ball moved to {self.ball_position}")
        
    def handle_client_connection(self, connection, address):
        """Handles new client connections and assigns a team."""
        player_id = str(uuid.uuid4())
        print(f"New client connection: {player_id}, {address}.")

        with self.client_lock:
            self.clients[player_id] = connection
        with self.team_lock:
            if len(self.team1) <= len(self.team2):
                team_number = 1
                self.team1.add(player_id)
            else:
                team_number = 2
                self.team2.add(player_id)

        # Send setup information (No spawn position assigned)
        setup_message = f"SETUP|{team_number}|{player_id}|[]\n"
        connection.sendall(setup_message.encode("utf-8"))
        print(f"📡 Sent setup message to {player_id}: {setup_message.strip()}")

        print(f"Clients connected: {len(self.clients)}")

        # Roles should be assigned **after** clients send their positions.
        self.assign_roles()
    

    def assign_roles(self):
        """Dynamically assigns roles ensuring a balanced game even in 1v1 testing."""
        if len(self.player_positions) < 2:  # Ensure at least 2 players before assigning roles
            print("⚠️ Waiting for at least 2 players before assigning roles.")
            return

        assigned_roles = {}

        # Get the two players
        players = list(self.player_positions.keys())

        # Assign the first player as Goalie, second as Striker
        assigned_roles[players[0]] = "Goalie"
        assigned_roles[players[1]] = "Striker"

        # Store assigned roles
        self.roles = assigned_roles

        # Send roles to clients
        for player_id, role in assigned_roles.items():
            self.send_role_update(player_id, role)

        print(f"✅ Roles Assigned: {assigned_roles}")


    def distance_to_ball(self, position):
        """Calculates the Euclidean distance from a player to the ball."""
        return ((self.ball_position[0] - position[0]) ** 2 + (self.ball_position[1] - position[1]) ** 2) ** 0.5

    def send_role_update(self, player_id, role):
        """Sends the assigned role to a player."""
        if player_id in self.clients:
            role_message = f"ROLE|{role}\n"
            self.clients[player_id].sendall(role_message.encode("utf-8"))
            print(f"✅ Sent Role: {role} to Player {player_id}")

    def send_game_updates(self):
        """Continuously send properly formatted game state updates to clients."""
        while True:
            time.sleep(1)  

            game_state = {
                "ball_position": self.ball_position,
                "players": self.player_positions,
                "strategy": self.get_current_strategy()
            }

            # Ensure message is properly JSON-encoded and newline-separated
            message = f"GAME_STATE|{json.dumps(game_state, separators=(',', ':'))}\n"

            with self.client_lock:
                for conn in self.clients.values():
                    try:
                        conn.sendall(message.encode("utf-8"))
                    except BrokenPipeError:
                        continue

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
              conn.sendall(f"VERIFY_ACTION|{action_type}|{player_id}\n".encode("utf-8"))

          # Wait for acknowledgment from majority (not all)
          if self.await_acknowledgment(player_id, action_type, required_ack_ratio=0.5):
              print(f"✅ {action_type} confirmed by majority.")
          else:
              print(f"❌ {action_type} failed, reverting.")
              for conn in self.clients.values():
                  conn.sendall(f"REVERT_ACTION|{action_type}|{player_id}\n".encode("utf-8"))


    def await_acknowledgment(self, player_id, action_type, required_ack_ratio=0.5, timeout=1):
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
                        print(f"✅ Acknowledgment received for {action_type} from {player_id}")

                except socket.timeout:
                    continue  # Avoid blocking
                except Exception as e:
                    print(f"⚠️ Error receiving acknowledgment: {e}")
                    continue

            if len(acknowledgments) >= required_acks:
                print(f"✅ {action_type} confirmed by majority ({len(acknowledgments)}/{required_acks})")
                return True  # Majority confirmed

        print(f"❌ {action_type} failed: only {len(acknowledgments)}/{required_acks} acknowledgments received.")
        return False  # Timeout reached

    def remove_client(self, connection):
        """Removes disconnected clients."""
        with self.client_lock:
            for player_id, conn in list(self.clients.items()):
                if conn == connection:
                    print(f"❌ Removing player {player_id}")
                    del self.clients[player_id]
                    with self.team_lock:
                        self.team1.discard(player_id)
                        self.team2.discard(player_id)
                    break
        connection.close()

# Start the server in a separate thread
game_server = GameServer(6)
server_thread = threading.Thread(target=game_server.start_server, daemon=True)
server_thread.start()

# Webots main loop
robot = Robot()
timestep = int(robot.getBasicTimeStep())
while robot.step(timestep) != 1:
    pass
