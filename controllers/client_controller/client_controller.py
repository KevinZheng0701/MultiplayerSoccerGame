from controller import Robot
import socket
import threading
import time

host = "127.0.0.1"
port = 5555

class ClientController:
  def __init__(self):
    self.players = {}
    self.team_number = 0
    self.player_id = 0
    self.role = ""

  # Start the connection between client and server
  def client_setup(self):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      client.connect((host, port))
      client_thread = threading.Thread(target=self.handle_server_communcation, args=(client,))
      client_thread.start()
    except (ConnectionRefusedError, OSError) as error:
      print(f"Connection error {error}")
    except Exception as error:
      print(f"Error: {error}")

  # Handles messages from the server
  def handle_server_communcation(self, connection):
    try:
      while True:
        data = connection.recv(1024)
        if not data:
          print("Disconnected from server.")
          break
        message = data.decode("utf-8")
        print(f"Message from server: {message}")
        self.handle_message(message)
    except Exception as e:
      print(f"Communication error: {e}")
    finally:
      print(f"Client {self.player_id} closing connection with server.")
      connection.close()

  # Handles messages from the server
  def handle_message(self, message):
    message_parts = message.split("|")
    message_type = message_parts[0]
    match message_type:
      case "SETUP":
        self.team_number = message_parts[1]
        self.player_id = message_parts[2]
        print(f"Assigned to team {self.team_number} with Player ID: {self.player_id}")
      case "ADD":
        self.team = message_parts[1]
        self.id = message_parts[2]
        self.players[id] = self.team
      case "ACK":
        print("Acknowledgement.")
      case "MOVE":
        print("Moving.")      
      case "GOAL":
        print("Scored a goal.")
      case "KICK":
        print("Ball kick.")
      case "GET":
        print("Requesting information.")
      case "ROLE":
        print("Getting role")
        self.role = message_parts[1]
      case _:
        print(message)
        print("Unknown Type.")

time.sleep(0.5) # Wait for the game server to be up
robot_client = ClientController()
client_thread = threading.Thread(target=robot_client.client_setup)
client_thread.start()

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Webots main loop
while robot.step(timestep) != 1:
  pass