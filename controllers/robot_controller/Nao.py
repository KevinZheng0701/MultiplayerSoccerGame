from controller import Robot, Motion


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

    def stop_motion(self):
        """Stops the currently playing motion and resets the motion state"""
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        self.currentlyPlaying = False

    def play_standup_motion(self):
        """Play the standup motion"""
        self.start_motion(self.standupFromBack)

    def play_kick_ball(self):
        """Play the kick motion"""
        self.start_motion(self.shoot)

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
