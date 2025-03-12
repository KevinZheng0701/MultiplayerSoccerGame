from controller import Robot, Motion
import socket
import threading
import json
import time



class Nao(Robot):
    PHALANX_MAX = 8

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.taiChi = Motion('../../motions/TaiChi.motion')
        self.standup = Motion('../../motions/StandUpFromFront.motion')

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        left = []
        right = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

        for i in range(0, len(left)):
            left[i] = max(min(left[i], 25), 0)
            right[i] = max(min(right[i], 25), 0)
            newtonsLeft += left[i]
            newtonsRight += right[i]

        print('----------foot sensors----------')
        print('+ left ---- right +')
        print('+-------+ +-------+')
        print('|' + str(round(left[0], 1)) +
              '  ' + str(round(left[1], 1)) +
              '| |' + str(round(right[0], 1)) +
              '  ' + str(round(right[1], 1)) +
              '|  front')
        print('| ----- | | ----- |')
        print('|' + str(round(left[3], 1)) +
              '  ' + str(round(left[2], 1)) +
              '| |' + str(round(right[3], 1)) +
              '  ' + str(round(right[2], 1)) +
              '|  back')
        print('+-------+ +-------+')
        print('total: %f Newtons, %f kilograms'
              % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81)))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print('----------foot bumpers----------')
        print('+ left ------ right +')
        print('+--------+ +--------+')
        print('|' + str(ll) + '  ' + str(lr) + '| |' + str(rl) + '  ' + str(rr) + '|')
        print('|        | |        |')
        print('|        | |        |')
        print('+--------+ +--------+')

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
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

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.sock = None  # Store socket connection
        self.role = None   # Store assigned role

        self.connect_to_server()
        self.findAndEnableDevices()
        self.loadMotionFiles()

    def connect_to_server(self):
        """Establishes a connection to the game server and sends its real GPS position."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            print("🔄 Attempting to connect to server...")
            self.sock.connect(("127.0.0.1", 5555))
            print("✅ Successfully connected to server.")

            # Enable and read GPS sensor
            self.gps = self.getDevice("gps")  # Match the Webots DEF name
            self.gps.enable(int(self.getBasicTimeStep()))  # Enable GPS updates
            self.step(int(self.getBasicTimeStep()) * 5)  # Wait for GPS to update

            gps_position = self.gps.getValues()  # Read GPS data
            spawn_position = [gps_position[0], gps_position[2]]  # Ignore height (y-axis)

            # Receive setup message
            setup_msg = self.sock.recv(1024).decode().strip()
            print(f"📡 Received setup message: {setup_msg}")

            parts = setup_msg.split("|")
            if len(parts) < 4:  # Prevents index errors
                print(f"⚠️ Invalid SETUP message received: {setup_msg}")
                return

            self.team_number = int(parts[1])
            self.client_id = parts[2]

            print(f"🆔 Assigned ID: {self.client_id}, Team: {self.team_number}")
            print(f"📡 Actual Spawn Position from GPS: {spawn_position}")

            # Send real position to the server
            self.sock.sendall(f"MOVE|{self.client_id}|{json.dumps(spawn_position)}\n".encode("utf-8"))
            print(f"📡 Sent Initial Position: {spawn_position}")

            # Start role & command listeners
            threading.Thread(target=self.listen_for_role_assignment, daemon=True).start()
            threading.Thread(target=self.listen_for_server_commands, daemon=True).start()

        except Exception as e:
            print(f"⚠️ Connection to server failed: {e}")

    def listen_for_role_assignment(self):
        """Waits for the server to assign a role and acknowledges it."""
        try:
            self.sock.settimeout(10)  # Avoids blocking forever
            while True:
                print("🛰️ Listening for role assignment...")
                try:
                    data = self.sock.recv(1024).decode().strip()
                    if not data:
                        break

                    if data.startswith("ROLE|"):
                        self.role = data.split("|")[1]
                        print(f"📢 Assigned Role: {self.role}")

                        # ✅ Send acknowledgment immediately
                        self.sock.sendall(f"ROLE_ACK|{self.client_id}\n".encode("utf-8"))
                        print(f"✅ Role Acknowledged: {self.role}")

                        return  # Exit after acknowledgment

                except socket.timeout:
                    print("⚠️ Role assignment timed out.")
                    return

        except Exception as e:
            print(f"⚠️ Error receiving role: {e}")

    def listen_for_server_commands(self):
        """Continuously listens for movement and action commands from the server."""
        try:
            buffer = ""  # Store incomplete messages

            while True:
                data = self.sock.recv(1024).decode().strip()
                if not data:
                    break

                buffer += data  # Append new data to buffer
                messages = buffer.split("\n")  # Split messages by newline
                buffer = messages.pop() if not data.endswith("\n") else ""  # Keep last incomplete message

                for message in messages:
                    message_parts = message.split("|")
                    if message_parts[0] == "GAME_STATE":
                        try:
                            game_state = json.loads(message_parts[1])
                            ball_position = game_state["ball_position"]
                            print(f"📡 Ball position: {ball_position}")
                            self.move_based_on_role(ball_position)
                        except Exception as e:
                            print(f"⚠️ Error parsing game state: {e}")
        except Exception as e:  # ✅ Ensure there's an except block
            print(f"Error: {e}")

    def has_fallen(self):
        """Checks if the robot has fallen based on pitch angle."""
        rpy = self.inertialUnit.getRollPitchYaw()  # Get roll, pitch, yaw

        pitch_angle = rpy[1]  # Pitch is the second value in roll-pitch-yaw
        threshold = 1.0  # Adjust this threshold if needed

        if abs(pitch_angle) > threshold:  # Robot has fallen forward or backward
            return True
        return False


    def move_based_on_role(self, ball_position):
        """Moves the robot based on its role and corrects sidestepping issues."""

        current_position = self.gps.getValues()
        x_current, y_current = current_position[0], current_position[2]

        if self.has_fallen():
            print("⚠️ Robot has fallen! Attempting to stand up.")
            self.startMotion(self.standup)  # Play stand-up motion
            while self.currentlyPlaying and not self.currentlyPlaying.isOver():
                self.step(self.timeStep)  # Wait until the motion is complete
            print("✅ Robot has recovered!")

        if self.role == "Striker":
            print(f"⚽ Striker moving towards ball at {ball_position}")
            self.move_to_position(ball_position)  # Move to ball's x, y position

            # If close to ball, attempt a kick
            if abs(x_current - ball_position[0]) < 0.2 and abs(y_current - ball_position[1]) < 0.2:
                self.kick_ball()

        elif self.role == "Goalie":
            print(f"🥅 Goalie adjusting to ball position {ball_position}")
            self.move_as_goalie(ball_position)

        elif self.role == "Defender":
            defensive_position = [ball_position[0] - 0.5, ball_position[1]]
            self.move_to_position(defensive_position)

    def move_to_position(self, position, step_size=0.5):
        """Moves the robot toward a given (x, y) position, ensuring correct direction."""

        x_target, y_target = position
        current_position = self.gps.getValues()
        x_current, y_current = current_position[0], current_position[2]

        print(f"🚶 Moving from ({x_current}, {y_current}) → ({x_target}, {y_target})")

        while abs(x_current - x_target) > 0.1 or abs(y_current - y_target) > 0.1:
            dx = x_target - x_current
            dy = y_target - y_current

            # Normalize movement direction
            move_x = step_size if dx > 0 else -step_size if dx < 0 else 0
            move_y = step_size if dy > 0 else -step_size if dy < 0 else 0

            # Move in the correct direction
            if move_x > 0:
                self.startMotion(self.forwards)
            elif move_x < 0:
                self.startMotion(self.backwards)

            if move_y > 0:
                self.startMotion(self.sideStepRight)
            elif move_y < 0:
                self.startMotion(self.sideStepLeft)

            # Wait for movement to complete
            while self.currentlyPlaying and not self.currentlyPlaying.isOver():
                self.step(self.timeStep)

            # Update current position
            self.step(self.timeStep * 5)
            current_position = self.gps.getValues()
            x_current, y_current = current_position[0], current_position[2]

        print(f"✅ Reached target position: ({x_target}, {y_target})")

        # ✅ Send updated position to the server
        self.sock.sendall(f"MOVE|{self.client_id}|{json.dumps([x_target, y_target])}\n".encode("utf-8"))

    def kick_ball(self):
        """Executes a kick and informs the server."""
        if hasattr(self, "kick_motion"):
            self.kick_motion.play()
            print("⚽ Kicking the ball!")
            self.sock.sendall(f"KICK|{self.client_id}\n".encode("utf-8"))  # ✅ Notify server
        else:
            print("⚠️ Kick motion not loaded.")

    def run(self):
        """Main loop for the robot."""
        print("🤖 Robot is ready and waiting for server commands.")
        while self.step(self.timeStep) != -1:
            pass  # The robot will now respond only to MOVE and KICK commands.

    def is_in_goal_area(self, position):
        """Checks if the given position is within the goal area."""
        goal_area_x = [-0.5, 0.5]  # Define the goal area (x-axis)
        goal_area_y = [-0.5, 0.5]  # Define the goal area (y-axis)
        return (goal_area_x[0] <= position[0] <= goal_area_x[1] and
                goal_area_y[0] <= position[1] <= goal_area_y[1])

    def move_as_goalie(self, ball_position):
        """Goalie movement logic: stay in goal area and block shots with limited movement."""
        
        if self.has_fallen():
            print("⚠️ Goalie has fallen! Standing up.")
            self.startMotion(self.standup)
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
        """Striker movement logic: move toward the ball and attempt to score."""
        self.move_to_position(ball_position)  # Move toward the ball

        # If close to the ball, attempt to kick
        current_position = self.gps.getValues()
        x_current, y_current = current_position[0], current_position[2]
        if abs(x_current - ball_position[0]) < 0.2 and abs(y_current - ball_position[1]) < 0.2:
            self.kick_ball()

    def move_as_defender(self, ball_position):
        """Defender movement logic: block opponents and clear the ball."""
        # Move toward the ball if it is in the defensive half
        if ball_position[0] < 0:  # Example: defensive half is x < 0
            self.move_to_position(ball_position)
        else:
            # Stay in a defensive position
            self.move_to_position([-1.0, 0])  # Example: stay at x = -1.0, y = 0


# create the Robot instance and run main loop
robot = Nao()
robot.run()