from controller import Robot
import socket
import threading
import time

host = "127.0.0.1"
port = 5555

players = {}
team_number = 0
player_id = 0

# Start the connection between client and server
def client_setup():
  time.sleep(1) # Wait for the server to be up
  client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  try:
    client.connect((host, port))
    client_thread = threading.Thread(target=handle_server_communcation, args=(client,))
    client_thread.start()
  except (ConnectionRefusedError, OSError) as error:
    print(f"Connection error {error}")
  except Exception as error:
    print(f"Error: {error}")

# Handles messages from the server
def handle_server_communcation(connection):
  try:
    while True:
      data = connection.recv(1024)
      if not data:
        print("Disconnected from server.")
        break
      message = data.decode("utf-8")
      print(f"Message from server: {message}")
      handle_message(message)
  except Exception as e:
    print(f"Communication error: {e}")
  finally:
    print(f"Client {player_id} closing connection with server.")
    connection.close()

# Handles messages from the server
def handle_message(message):
  message_parts = message.split("|")
  message_type = message_parts[0]
  match message_type:
    case "SETUP":
      team_number = message_parts[1]
      player_id = message_parts[2]
      print(f"Assigned to team {team_number} with Player ID: {player_id}")
    case "ADD":
      team = message_parts[1]
      id = message_parts[2]
      players[id] = team
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
    case _:
      print(message)
      print("Unknown Type.")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

client_thread = threading.Thread(target=client_setup)
client_thread.start()

# Webots main loop
while robot.step(timestep) != 1:
  pass