from controller import Robot
import socket
import threading
import uuid

host = "127.0.0.1"
port = 5555

class GameServer:
  def __init__(self, players_limit=0):
    self.client_lock = threading.Lock()
    self.team_lock = threading.Lock()
    self.players_limit = players_limit
    self.clients = {}
    self.team1 = set()
    self.team2 = set()
  
  # Start the server and awaits for connections
  def start_server(self):
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
    while True:
      try:
        connection, address = server.accept()
        self.handle_client_connection(connection, address)
        thread = threading.Thread(target=self.handle_client_communication, args=(connection, address))
        thread.start()
      except Exception as e:
        print(f"Error while accepting connection: {e}")
        break

  # Handles connection of clients with the server
  def handle_client_connection(self, connection, address):
    # Generate a unique id for client 
    player_id = str(uuid.uuid4())
    print(f"New client connection: {player_id}, {address}.")
    # Add player to a team
    with self.client_lock:
      self.clients[player_id] = connection
    with self.team_lock:
      if len(self.team1) <= len(self.team2):
        team_number = 1
        self.team1.add(player_id)
      else:
        team_number = 2
        self.team2.add(player_id)
    # Send player id and team number to the client and broadcast this to other clients
    connection.sendall(f"SETUP|{team_number}|{player_id}".encode("utf-8"))
    self.broadcast(f"ADD|{team_number}|{player_id}", player_id)
    print(f"Clients connected: {len(self.clients)}")
    if len(self.clients) == self.players_limit:
      self.handle_inital_roles()

  # Handles communication with clients
  def handle_client_communication(self, connection, address):
    try:
      while True:
        data = connection.recv(1024)
        if not data:
          print(f"Client {address} disconnected.")
          break
        message = data.decode("utf-8")
        print(f"Received from {address}: {message}")
        self.handle_message(message, connection)
    except ConnectionResetError:
      print(f"Client {address} forcibly closed the connection.")
    except Exception as e:
      print(f"Error with {address}: {e}")
    finally:
      player_id = ""
      with self.client_lock:
        for id, conn in self.clients.items():
          if conn == connection:
            player_id = id
            del self.clients[player_id]
            with self.team_lock:
              self.team1.discard(player_id)
              self.team2.discard(player_id)
            break 
      print(f"Server closing connection with {player_id}")
      connection.close()

  # Handle messages from clients
  def handle_message(self, message, sender):
    message_parts = message.split("|")
    message_type = message_parts[0]
    sender_id = message_parts[1]
    match message_type:
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

  # Sends message to every client besides the sender
  def broadcast(self, message, sender=None):
    for id, conn in self.clients.items():
      if id != sender:
        conn.sendall(message.encode("utf-8"))

  # Assign initial roles
  def handle_inital_roles(self):
      goalie_one = self.team1[0]
      self.clients[goalie_one].sendall(f"ROLE|GOALIE".encode("utf-8"))
      goalie_two = self.team2[0]
      self.clients[goalie_two].sendall(f"ROLE|GOALIE".encode("utf-8"))
      #start_game()

game_server = GameServer(6) # Create a server up to 6 clients
server_thread = threading.Thread(target=game_server.start_server, daemon=True)
server_thread.start()

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Webots main loop
while robot.step(timestep) != 1:
  pass
