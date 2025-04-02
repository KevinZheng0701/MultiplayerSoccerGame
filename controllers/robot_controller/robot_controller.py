from controller import Robot, Motion
import socket
import threading
import time
import math
from collections import deque

class Nao(Robot):
    PHALANX_MAX = 8

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.load_motion_files()
        self.find_and_enable_devices()

    def load_motion_files(self):
        """Load motion files for the robot"""
        self.handWave = Motion("../../motions/HandWave.motion")
        self.largeForwards = Motion("../../motions/Forwards50.motion")
        self.smallForwards = Motion("../../motions/Forwards.motion")
        self.backwards = Motion("../../motions/Backwards.motion")
        self.shoot = Motion("../../motions/Shoot.motion")
        self.sideStepLeft = Motion("../../motions/SideStepLeft.motion")
        self.sideStepRight = Motion("../../motions/SideStepRight.motion")
        self.standupFromFront = Motion("../../motions/StandUpFromFront.motion")
        self.standupFromBack = Motion("../../motions/StandUpFromBack.motion")
        self.turnLeft40 = Motion("../../motions/TurnLeft40.motion")
        self.turnLeft60 = Motion("../../motions/TurnLeft60.motion")
        self.turnRight40 = Motion("../../motions/TurnRight40.motion")
        self.turnRight60 = Motion("../../motions/TurnRight60.motion")
        self.turn180 = Motion("../../motions/TurnLeft180.motion")
        self.taiChi = Motion("../../motions/TaiChi.motion")

    def start_motion(self, motion):
        """Starts a robot motion"""
        # Interrupt current motion
        if self.currentlyPlaying and self.currentlyPlaying != motion:
            self.currentlyPlaying.stop()
        # Start new motion
        motion.play()
        self.currentlyPlaying = motion

    def play_standup_motion(self):
        """Play the standup motion"""
        self.start_motion(self.standupFromBack)

    def get_acceleration(self):
        """Get the current acceleration"""
        return self.accelerometer.getValues()

    def print_acceleration(self):
        """Prints the current acceleration of the robot"""
        # The accelerometer axes are oriented as on the real robot however the sign of the returned values may be opposite
        acc = self.get_acceleration()
        print("----------accelerometer----------")
        print("acceleration: [ x y z ] = [%f %f %f]" % (acc[0], acc[1], acc[2]))

    def get_velocity(self):
        """Get the current velocity"""
        return self.gyro.getValues()

    def print_gyro(self):
        """Prints the current velocity of the robot"""
        # The gyro axes are oriented as on the real robot however the sign of the returned values may be opposite
        vel = self.get_velocity()
        print("----------gyro----------")
        print(
            "angular velocity: [ x y ] = [%f %f]" % (vel[0], vel[1])
        )  # Z value is meaningless due to the orientation of the Gyro

    def get_position(self):
        """Get the position of the robot"""
        return self.gps.getValues()

    def print_gps(self):
        """Prints the current position of the robot"""
        pos = self.get_position()
        print("----------gps----------")
        print("position: [ x y z ] = [%f %f %f]" % (pos[0], pos[1], pos[2]))

    def get_rotation(self):
        """Get the rotation of the robot"""
        return self.inertialUnit.getRollPitchYaw()

    def print_inertial_unit(self):
        """Prints the current rotation of the robot"""
        # The InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
        rpy = self.get_rotation()
        print("----------inertial unit----------")
        print("roll/pitch/yaw: [%f %f %f]" % (rpy[0], rpy[1], rpy[2]))

    def print_foot_sensors(self):
        """Prints the forces on the foot sensors"""
        fsv = []  # Force sensor values
        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())
        left, right = [], []
        newtonsLeft, newtonsRight = 0, 0
        # The coefficients were calibrated against the real robot so as to obtain realistic sensor values.
        left.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Front Left
        left.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Front Right
        left.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Rear Right
        left.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Rear Left
        right.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Front Left
        right.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Front Right
        right.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Rear Right
        right.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Rear Left

        for i in range(0, len(left)):
            left[i] = max(min(left[i], 25), 0)
            right[i] = max(min(right[i], 25), 0)
            newtonsLeft += left[i]
            newtonsRight += right[i]

        print("----------foot sensors----------")
        print("+ left ---- right +")
        print("+-------+ +-------+")
        print(
            "|"
            + str(round(left[0], 1))
            + "  "
            + str(round(left[1], 1))
            + "| |"
            + str(round(right[0], 1))
            + "  "
            + str(round(right[1], 1))
            + "|  front"
        )
        print("| ----- | | ----- |")
        print(
            "|"
            + str(round(left[3], 1))
            + "  "
            + str(round(left[2], 1))
            + "| |"
            + str(round(right[3], 1))
            + "  "
            + str(round(right[2], 1))
            + "|  back"
        )
        print("+-------+ +-------+")
        print(
            "total: %f Newtons, %f kilograms"
            % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81))
        )

    def print_foot_bumpers(self):
        """Prints the forces on the foot bumpers"""
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()

        print("----------foot bumpers----------")
        print("+ left ------ right +")
        print("+--------+ +--------+")
        print("|" + str(ll) + "  " + str(lr) + "| |" + str(rl) + "  " + str(rr) + "|")
        print("|        | |        |")
        print("|        | |        |")
        print("+--------+ +--------+")

    def print_ultrasound_sensors(self):
        """Prints the value of the ultrasound sensors"""
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print("-----ultrasound sensors-----")
        print("left: %f m, right %f m" % (dist[0], dist[1]))

    def print_camera_image(self, camera):
        """Prints the view of the camera"""
        scaled = 2  # Defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # Read rgb pixel values from the camera
        image = camera.getImage()

        print("----------camera image (gray levels)---------")
        print(
            "original resolution: %d x %d, scaled to %d x %f"
            % (width, height, width / scaled, height / scaled)
        )

        for y in range(0, height // scaled):
            line = ""
            for x in range(0, width // scaled):
                gray = (
                    camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255
                )  # Rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def set_all_leds_color(self, rgb):
        """Set the colors of the leds"""
        # These leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)
        # Ear leds are single color (blue) and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def set_hands_angle(self, angle):
        """Controls the hands of the robot"""
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]
            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def find_and_enable_devices(self):
        """Start and find the parts of the robot"""
        # Get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # Camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # Accelerometer
        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(4 * self.timeStep)

        # Gyro
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(4 * self.timeStep)

        # Gps
        self.gps = self.getDevice("gps")
        self.gps.enable(4 * self.timeStep)

        # Inertial unit
        self.inertialUnit = self.getDevice("inertial unit")
        self.inertialUnit.enable(self.timeStep)

        # Ultrasound sensors
        self.us = []
        usNames = ["Sonar/Left", "Sonar/Right"]
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # Foot sensors
        self.fsr = []
        fsrNames = ["LFsr", "RFsr"]
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # Foot bumpers
        self.lfootlbumper = self.getDevice("LFoot/Bumper/Left")
        self.lfootrbumper = self.getDevice("LFoot/Bumper/Right")
        self.rfootlbumper = self.getDevice("RFoot/Bumper/Left")
        self.rfootrbumper = self.getDevice("RFoot/Bumper/Right")
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # There are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice("ChestBoard/Led"))
        self.leds.append(self.getDevice("RFoot/Led"))
        self.leds.append(self.getDevice("LFoot/Led"))
        self.leds.append(self.getDevice("Face/Led/Right"))
        self.leds.append(self.getDevice("Face/Led/Left"))
        self.leds.append(self.getDevice("Ears/Led/Right"))
        self.leds.append(self.getDevice("Ears/Led/Left"))

        # Get phalanx motor tags for RHand/LHand containing 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))
            # Assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # Shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

    def has_fallen(self, threshold = 1):
        """Checks if the robot has fallen based on pitch angle"""
        rpy = self.get_rotation()  # Get roll, pitch, yaw
        pitch_angle = rpy[1]  # Pitch is the second value in roll-pitch-yaw
        if abs(pitch_angle) > threshold:  # Robot has fallen forward or backward
            return True
        return False
    
    def is_standup_motion_in_action(self):
        """Checks if the robot is in process of standing up"""
        if self.currentlyPlaying == self.standup and not self.currentlyPlaying.isOver():
            return True
        return False

    def move_to_position(self, position, step_size = 0.5):
        """Move the robot to a xyz position"""
        # TODO: Use a target position to move the robot, so movement is concurrent and doesn't block the thread execution
        x_target, y_target = position
        current_position = self.get_position()
        x_current, y_current = current_position[0], current_position[2]

        print(f"🚶 Moving from ({x_current}, {y_current}) → ({x_target}, {y_target})")
        while (x_current - x_target) ** 2 + (y_current - y_target) ** 2 > 0.1:
            dx = x_target - x_current
            dy = y_target - y_current

            # Normalize movement direction
            move_x = step_size if dx > 0 else -step_size if dx < 0 else 0
            move_y = step_size if dy > 0 else -step_size if dy < 0 else 0

            # Move in the correct direction
            if move_x > 0:
                self.start_motion(self.forwards)
            elif move_x < 0:
                self.start_motion(self.backwards)

            if move_y > 0:
                self.start_motion(self.sideStepRight)
            elif move_y < 0:
                self.start_motion(self.sideStepLeft)

            # Wait for movement to complete
            while self.currentlyPlaying and not self.currentlyPlaying.isOver():
                self.step(self.timeStep)

            # Update current position
            self.step(self.timeStep * 5)
            current_position = self.gps.getValues()
            x_current, y_current = current_position[0], current_position[2]

        print(f"✅ Reached target position: ({x_target}, {y_target})")

    def kick_ball(self):
        if hasattr(self, "kick_motion"):
            self.kick_motion.play()
            print("⚽ Kicking the ball!")
            self.sock.sendall(
                f"KICK|{self.client_id}\n".encode("utf-8")
            )  # ✅ Notify server
        else:
            print("⚠️ Kick motion not loaded.")

    def run(self):
        print("🤖 Robot is ready and waiting for server commands.")
        while self.step(self.timeStep) != -1:
            pass  # The robot will now respond only to MOVE and KICK commands.

    def is_in_goal_area(self, position):
        goal_area_x = [-0.5, 0.5]  # Define the goal area (x-axis)
        goal_area_y = [-0.5, 0.5]  # Define the goal area (y-axis)
        return (
            goal_area_x[0] <= position[0] <= goal_area_x[1]
            and goal_area_y[0] <= position[1] <= goal_area_y[1]
        )

    def move_as_goalie(self, ball_position):
        if self.has_fallen():
            print("⚠️ Goalie has fallen! Standing up.")
            self.start_motion(self.standup)
            while self.currentlyPlaying and not self.currentlyPlaying.isOver():
                self.step(self.timeStep)
            print("✅ Goalie has recovered!")

        goal_x = -4.5  # Goalie's x-position should remain near the goal line
        goal_area_y = [-0.5, 0.5]  # Goalkeeper's allowed movement range

        # Get current position
        current_position = self.gps.getValues()
        x_current, y_current = current_position[0], current_position[2]

        # Limit movement: Move only side-to-side within the goal area
        target_x = goal_x  # Always keep the goalie near the goal
        target_y = max(goal_area_y[0], min(goal_area_y[1], ball_position[1]))

        # Move with smaller steps to avoid falling
        self.move_to_position([target_x, target_y], step_size=0.1)

    def move_as_striker(self, ball_position):
        self.move_to_position(ball_position)  # Move toward the ball

        # If close to the ball, attempt to kick
        current_position = self.gps.getValues()
        x_current, y_current = current_position[0], current_position[2]
        if (
            abs(x_current - ball_position[0]) < 0.2
            and abs(y_current - ball_position[1]) < 0.2
        ):
            self.kick_ball()

    def move_as_defender(self, ball_position):
        # Move toward the ball if it is in the defensive half
        if ball_position[0] < 0:  # Example: defensive half is x < 0
            self.move_to_position(ball_position)
        else:
            # Stay in a defensive position
            self.move_to_position([-1.0, 0])  # Example: stay at x = -1.0, y = 0

class SoccerRobot:
    def __init__(self):
        self.player_team = {}
        self.ball_position = [0, 0, 0]
        self.player_states = {}  # Store player positions and role status
        self.team_number = 0
        self.player_id = 0
        self.role = None
        self.sock = None
        self.robot = Nao()
        self.action_queue = deque() # Priority queue for handling actions
        self.target_positon = [0, 0] # The position the robot is heading towards
        self.target_rotation = [0, 0] # The rotation the robot is targeting
        self.state = None
        self.paused = True

    def connect_to_server(self, host, port):
        """Establishes a connection to the game server and sends its GPS position"""
        print("🔄 Attempting to connect to server...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            print("✅ Successfully connected to server.")
            client_thread = threading.Thread(target=self.listen_for_server)
            client_thread.start()
        except Exception as error:
            print(f"⚠️ Connection to server failed: {error}")

    def listen_for_server(self):
        """Listen to messages from the server"""
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

    def handle_message(self, message):
        """Handles messages from the server"""
        message_parts = message.split("|")
        message_type = message_parts[0]
        match message_type:
            case "POS":
                player_id = message_parts[1]
                x_position, y_position = message_parts[2], message_parts[3]
                x_rotation, y_rotation, z_rotation, angle = message_parts[4], message_parts[5], message_parts[6], message_parts[7]
                if player_id != self.player_id:
                    # Update the position and rotation of the players
                    if player_id not in self.player_states:
                        self.player_states[player_id] = [None, [x_position, y_position], [x_rotation, y_rotation, z_rotation, angle]]
                    else:
                        self.update_position(player_id, x_position, y_position)
                        self.update_rotation(player_id, x_rotation, y_rotation, z_rotation, angle)
                else:
                    current_position = self.robot.get_position()
                    print(current_position)
            case "BALL":
                x_position, y_position, z_position = message_parts[1], message_parts[2], message_parts[3]
                self.update_ball_position(x_position, y_position, z_position)
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
                self.role = message_parts[1]
                print(f"📢 Assigned Role: {self.role}")
            case "ADD":
                team = message_parts[1]
                id = message_parts[2]
                self.player_team[id] = team
                print(f"📢 New teammate: {id}")
            case "START":
                self.paused = False
            case "INFO":
                self.team_number = message_parts[1]
                self.player_id = message_parts[2]
                print(
                    f"🆔 Assigned to team {self.team_number} with Player ID: {self.player_id}"
                )
            case _:
                print(message)
                print("❓ Unknown message type")

    def move_based_on_role(self, ball_position):
        """Moves the robot based on its role and corrects sidestepping issues"""
        current_position = self.robot.get_position()
        x_current, y_current = current_position[0], current_position[1]
        
        # Ensure the robot is standing up
        if self.robot.has_fallen():
            print("⚠️ Robot has fallen! Attempting to stand up.")
            self.robot.play_standup_motion()
            print("✅ Robot has recovered!")

        if self.role == "Striker":
            print(f"⚽ Striker moving towards ball at {ball_position}")
            self.robot.move_to_position(ball_position)  # Move to ball's x, y position

            # If close to ball, attempt a kick
            if (x_current - ball_position[0]) ** 2 + (
                y_current - ball_position[1]
            ) ** 2 < 0.25:
                self.robot.kick_ball()

        elif self.role == "Goalie":
            print(f"🥅 Goalie adjusting to ball position {ball_position}")
            self.robot.move_as_goalie(ball_position)

        elif self.role == "Defender":
            defensive_position = [ball_position[0] - 0.5, ball_position[1]]
            self.robot.move_to_position(defensive_position)

    def determine_action(self):
        """Determine what to do based on role and current game state"""
        # If the robot has fallen then ensure it is getting up first
        if self.robot.has_fallen():
            self.robot.play_standup_motion()
            return
        elif self.robot.is_standup_motion_in_action():
            return
        match self.role:
            case "Goalie":
                pass
            case "Striker":
                pass
            case "Midfielder":
                pass
            case "Defender":
                pass
            case _:
                pass
        
    def go_to(self, x_position, y_position, threshold = 0.25):
        """Function to tell the robot to go a certain position"""
        self.target_position = [x_position, y_position]
        position = self.get_position()
        # If the robot reached the target position then stop moving
        distance = self.get_distance(position, self.target_position)
        if distance < threshold:
            print(f'At position {x_position}, {y_position}.')
            self.state = None
            return
        # Turn before moving
        x_current, y_current = position[0], position[1]
        direction = self.normalize_vector([x_position - x_current, y_position - y_current])
        if self.turn_to_direction(direction):
            self.start_turn(direction)

    def move_to_position(self, position, threshold = 0.2):
        """Move the robot to the target position"""
        curr_position = self.robot.get_position()
        x_current, y_current = curr_position[0], curr_position[1]
        # If the robot reached the target position then stop moving
        distance = self.get_distance([x_current, y_current], position)

        if distance < threshold:
            print(f'At position {x_current}, {y_current}.')
            self.stop_motion()
            self.state = None
            return
        
        if not self.currentlyPlaying or self.currentlyPlaying.isOver():
            # Before moving again, ensure robot direction is correct
            direction = self.normalize_vector([position[0] - x_current, position[1] - y_current])
            if self.turn_to_direction(direction, 5):
                self.start_turn(direction)
                return
            # Take small steps when it is close else large steps
            if distance < threshold * 2: 
                self.robot.start_motion(self.robot.smallForwards)
            else: 
                self.robot.start_motion(self.robot.largeForwards)

    def turn_to_direction(self, direction, threshold = 1):
        """Turn the robot towards the target direction"""
        # Find the angle to turn to within [-pi, pi]
        current_angle = self.robot.get_rotation()[2]
        target_angle = math.atan2(direction[1], direction[0])
        angle_difference = self.calculate_angle_difference(current_angle, target_angle)

        # Positive angle indicates a left turn and negative angle indicates a right turn
        print(f"Current yaw: {math.degrees(current_angle):.2f}°, Target yaw: {math.degrees(target_angle):.2f}°, Yaw diff: {abs(math.degrees(angle_difference)):.2f}°")
        if abs(angle_difference) < math.radians(threshold):
            if self.state == "Turning":
                self.stop_turn_and_start_moving()
            return False # Return false for no turning needed
        
        # If the motion is not playing, then play it
        if not self.currentlyPlaying or self.currentlyPlaying.isOver():
            # Larger angle differences use larger steps
            if abs(angle_difference) > 160:
                self.robot.start_motion(self.robot.turn180)
            else:
                self.robot.start_motion(self.robot.turnLeft40 if angle_difference > 0 else self.robot.turnRight40)
        return True

    def normalize_vector(self, vector):
        """Normalizes a vector"""
        norm = math.sqrt(sum(val ** 2 for val in vector))
        return [v / norm for v in vector] if norm != 0 else vector
    
    def get_distance(self, point_one, point_two):
        """Returns the Euclidean distance between two points in xy-plane"""
        return math.sqrt((point_two[0] - point_one[0]) ** 2 + (point_two[1] - point_one[1]) ** 2)
    
    def calculate_angle_difference(self, angle1, angle2):
        """Returns the angular difference in radians between two angles"""
        return (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi
    
    def stop_motion(self):
        """Stops the currently playing motion and resets the motion state"""
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        self.currentlyPlaying = False

    def stop_turn_and_start_moving(self):
        """Stops the current turning motion and transitions the robot to the moving state"""
        self.stop_motion()
        self.state = "Moving"
        self.robot.start_motion(self.robot.smallForwards)

    def start_turn(self, direction):
        """Starts and set up the turning"""
        self.target_rotation = direction
        self.state = "Turning"

    def set_target_position(self, target):
        """Set a target position"""
        self.target_positon = target

    def distance_to_ball(self):
        """Calculates the Euclidean distance from the player to the ball"""
        player_state = self.player_states[self.player_id]
        x_position, y_position = player_state[2], player_state[3]
        return math.sqrt((x_position - self.ball_position[0]) ** 2 + (y_position - self.ball_position[1]) ** 2)

    def update_position(self, player, x, y):
        """Update the position of the player"""
        self.player_states[player][1] = [x, y]

    def update_rotation(self, player, x, y, z, angle):
        """Update the rotation of the player"""
        self.player_states[player][2] = [x, y, z, angle]

    def update_ball_position(self, x, y, z):
        """Update the position of the ball"""
        self.ball_position[0] = x
        self.ball_position[1] = y
        self.ball_position[2] = z

host = "127.0.0.1"
port = 5555

time.sleep(0.5)  # Wait for the game server to be up

soccer_robot = SoccerRobot()
timeStep = soccer_robot.robot.timeStep

client_thread = threading.Thread(target=soccer_robot.connect_to_server, args=((host, port)))
client_thread.start()

# Webots main loop
while soccer_robot.robot.step(timeStep) != -1:
    if not soccer_robot.paused:
        soccer_robot.determine_action()