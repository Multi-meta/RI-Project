# Webots Sample Controllers Audit

This document audits the Webots sample device-demo controllers located in `RI-Project/controllers/`. Each section covers one sample folder: what the demo shows, the key Webots Python API calls it uses, any sample world it references, and how IntelliBot (our warehouse robot project) reuses the pattern. Notes are factual and based only on the file contents. Four folders (`imu`, `motor2`, `motor3`, `vacuum_gripper`) ship only a C controller and no `.py` file; their C sources are summarized briefly. None of the 26 samples reference a specific `.wbt` world file in their comments or Makefiles — worlds live outside the controller folders (typically in the Webots `projects/` tree), so the "world" lines below reflect that.

## lidar
- **Demonstrates:** Simulation of a lidar-equipped differential-drive robot that drives while avoiding collisions using two ultrasonic sensors; the lidar device is enabled with point-cloud rendering but its range data is not actually read in the loop.
- **Key API calls:** `getDevice('lidar')`, `lidar.enable(self.timeStep)`, `lidar.enablePointCloud()`, `us0.getValue()`, `left_motor.setPosition(float('inf'))`, `left_motor.setVelocity(...)`, `self.step(self.timeStep)`
- **World:** Not referenced in the file (the matching Webots sample world is the e-puck lidar demo, but no `.wbt` is named in code).
- **IntelliBot reuse:** The `enable()/enablePointCloud()` pattern is exactly how our warehouse robot's lidar is activated for obstacle detection and mapping.

## camera
- **Demonstrates:** A camera-equipped robot that spins, analyzes the center of its image for a red/green/blue color blob, stops, prints the color, and saves a snapshot to the user's home directory.
- **Key API calls:** `camera.enable(timestep)`, `camera.getWidth()`, `camera.getHeight()`, `camera.getImage()`, `Camera.imageGetRed(image, width, i, j)` / `imageGetGreen` / `imageGetBlue`, `camera.saveImage(filename, 100)`, plus `getBasicTimeStep()` and wheel-motor velocity control.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Confirms the BGRA image pipeline and `saveImage` for evidence capture; our detector replaces the pixel-summing blob test with OpenCV HSV.

## camera_recognition
- **Demonstrates:** A robot with a recognition-enabled camera that rotates in place and prints all recognized objects with their 3D position, orientation, size, image coordinates, model names, ids, and colors.
- **Key API calls:** `camera.enable(timestep)`, `camera.recognitionEnable(timestep)`, `camera.getRecognitionNumberOfObjects()`, `camera.getRecognitionObjects()`, and per-object `object.getPosition()`, `object.getOrientation()`, `object.getSize()`, `object.getPositionOnImage()`, `object.getSizeOnImage()`, `object.getNumberOfColors()`, `object.getColors()`, `object.getModel()`, `object.getId()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Ground-truth object list used ONLY to validate our own OpenCV detector (Phase 4.10); the detector itself remains our own code.

## camera_segmentation
- **Demonstrates:** Same rotating camera robot as camera_recognition, but additionally enables recognition segmentation and renders the segmented image onto an in-world Display device each step.
- **Key API calls:** `camera.recognitionEnable(timestep)`, `camera.enableRecognitionSegmentation()`, `camera.isRecognitionSegmentationEnabled()`, `camera.getRecognitionSamplingPeriod()`, `camera.getRecognitionSegmentationImage()`, `camera.getWidth()/getHeight()`, and Display calls `display.imageNew(data, Display.BGRA, width, height)`, `display.imagePaste(...)`, `display.imageDelete(...)`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Segmentation masks could give ground-truth IoU for our HSV mask (P2).

## gps
- **Demonstrates:** A wall-avoiding robot whose GPS position can be read either from the GPS device or from a supervisor broadcasting position via Emitter/Receiver, switched with keyboard keys G/S/V.
- **Key API calls:** `gps.enable(timestep)`, `gps.getValues()`, `gps.getSpeedVector()`, `receiver.enable(timestep)`, `receiver.getQueueLength()`, `receiver.nextPacket()`, `receiver.getFloats()`, `getKeyboard()`, `keyboard.enable(timestep)`, `keyboard.getKey()`, plus distance sensors `ds0/ds1.getValue()` and wheel velocity control.
- **World:** Not referenced in the file (docstring points to the companion `gps_supervisor` controller).
- **IntelliBot reuse:** `gps.getValues()` gives the operating pose estimate (x, y) for control and validation.

## gps_supervisor
- **Demonstrates:** The supervisor half of the GPS demo: a Supervisor playing "satellite" that reads the robot's translation field each step and broadcasts it over an Emitter.
- **Key API calls:** `Supervisor` class, `getDevice('emitter')`, `emitter.send(...)`, `getFromDef('GPS_ROBOT')`, `getField('translation')`, `field.getSFVec3f()`.
- **World:** Not referenced in the file (pairs with the `gps` controller above).
- **IntelliBot reuse:** Supervisor ground-truth pattern reused for the world/config consistency check and package-position validation.

## inertial_unit
- **Demonstrates:** A three-axis gimbal (yaw/pitch/roll motors) that drives to random orientations while an InertialUnit reports roll/pitch/yaw until each target is reached.
- **Key API calls:** `inertial_unit.enable(timestep)`, `inertial_unit.getRollPitchYaw()`, `motor.setPosition(yaw/pitch/roll)` on three hinge motors, `self.step(timestep)`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** `getRollPitchYaw()[2]` is our yaw/heading source (GPS+IMU pose policy).

## imu
- **Note:** Only `imu.c` exists — no Python version.
- **Demonstrates (from the C file):** Compares a ground-truth InertialUnit against (a) an absolute attitude estimate from Accelerometer + Compass and (b) a drift-prone relative estimate from Gyro quaternion integration, while a gimbal moves to random orientations.
- **Key C API calls:** `wb_robot_get_device`, `wb_accelerometer_enable/get_values`, `wb_gyro_enable/get_values`, `wb_compass_enable/get_values`, `wb_inertial_unit_enable` / `wb_inertial_unit_get_roll_pitch_yaw`, `wb_motor_set_position`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Background reference for attitude estimation; we use the InertialUnit directly.

## position_sensor
- **Demonstrates:** An inverted-pendulum robot balanced by a PID controller that reads a hinge-joint position sensor and adjusts wheel speed.
- **Key API calls:** `position_sensor.enable(timestep)`, `position_sensor.getValue()`, wheel motors `setPosition(float('inf'))` / `setVelocity(...)`, `getBasicTimeStep()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** `enable()` + `getValue()` is exactly our wheel-encoder reading pattern for odometry.

## encoders
- **Demonstrates:** Wheel-encoder closed-loop control on a differential-drive robot: it reads encoder values, drives wheels toward random integer goals, then re-zeros encoders using offsets.
- **Key API calls:** `left_position_sensor.enable(timestep)`, `left_position_sensor.getValue()`, encoder offset reset, `left_motor.setPosition(float('inf'))`, `left_motor.setVelocity(...)`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Direct template for wheel-encoder odometry (cumulative angles into `Odometry.update`).

## motor
- **Demonstrates:** Simple motor position-control demo that steps a motor by pi/4 every 50 cycles while printing torque feedback and battery level.
- **Key API calls:** `motor.enableTorqueFeedback(timestep)`, `motor.setPosition(target)`, `motor.getTorqueFeedback()`, `robot.batterySensorEnable(timestep)`, `robot.batterySensorGetValue()`, `getBasicTimeStep()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Not needed directly (we use velocity mode); confirms motor API shape.

## motor2
- **Note:** Only `motor2.c` exists — no Python version.
- **Demonstrates (from the C file):** Simple position control of the two motors of a Hinge2Joint, stepping each to setpoints then oscillating them sinusoidally.
- **Key C API calls:** `wb_robot_get_device("motor 1"/"motor 2")`, `wb_motor_set_position`, `wb_robot_step`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Not needed (Hinge2Joint steering); C calls map 1:1 to Python names.

## motor3
- **Note:** Only `motor3.c` exists — no Python version.
- **Demonstrates (from the C file):** Position control of the three motors of a BallJoint, driving each axis then combining sinusoidal motion.
- **Key C API calls:** `wb_robot_get_device("motor 1"/"motor 2"/"motor 3")`, `wb_motor_set_position`, `wb_robot_step`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Not needed.

## distance_sensor
- **Demonstrates:** Classic 8-sensor braitenberg-style obstacle avoidance: IR distance sensors are mixed through a fixed matrix into left/right wheel speeds.
- **Key API calls:** `getDevice('ds' + str(i))`, `ps[i].enable(timestep)`, `ps[i].getValue()`, wheel motors `setPosition(float('inf'))` / `setVelocity(...)`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** P2/final: short-range sensors for the final approach to a package.

## sample_supervisor
- **Demonstrates:** General supervisor demo: enumerates world nodes, reads WorldInfo gravity, moves a PointLight, spawns a Sphere from a string, and animates it in a circle, using on-screen labels.
- **Key API calls:** `Supervisor` class, `getRoot()`, `getField('children')`, `field.getCount()`, `field.getMFNode(i)`, `node.getTypeName()`, `node.getField('gravity')`, `field.getSFFloat()`, `node.getField('location')` / `field.setSFVec3f(location)`, `field.importMFNodeFromString(-1, 'Pose { ... }')`, `self.setLabel(...)`, `self.getTime()`, `self.step(2000)`.
- **World:** Not referenced in the file (it operates on whatever world it runs in via the root children field).
- **IntelliBot reuse:** Core reference for supervisor tooling — world/config consistency check now; dynamic obstacle spawn + package attach in the final eval.

## display
- **Demonstrates:** A maze-wandering robot with two Display devices: one shows a random emoticon from a PNG sprite sheet, the other shows the live camera image with a yellow-blob rectangle and label drawn on top.
- **Key API calls:** `camera_display.attachCamera(camera)`, `display.imageLoad('emoticons.png')`, `display.imagePaste(...)`, `display.imageDelete(...)`, `display.setColor(0xFFFF00)`, `display.setFont('Palatino Linotype', 16, True)`, `display.setAlpha(0.0)`, `display.fillRectangle(...)`, `display.drawRectangle(...)`, `display.drawText(...)`, `display.getWidth()/getHeight()`, plus `camera.getImage()`, `Camera.imageGetRed/Green/Blue`.
- **World:** Not referenced in the file (requires an `emoticons.png` shipped alongside the controller, which is present in the folder).
- **IntelliBot reuse:** P2/final in-sim dashboard (map, path, state, detections).

## display_supervisor
- **Demonstrates:** A supervisor that tracks the robot's absolute translation and paints its trajectory as fading blue dots on a ground Display, saving a screenshot every 50 steps.
- **Key API calls:** `Supervisor`, `getFromDef('MYBOT')`, `getField('translation')`, `translation_field.getSFVec3f()`, `ground_display.getWidth()/getHeight()`, `setColor`, `fillRectangle`, `drawLine`, `drawText`, `fillOval`, `setOpacity`, `imageCopy(0, 0, width, height)`, `imageSave(to_store, filename)`, `imageDelete`.
- **World:** Not referenced in the file (expects a robot defined `DEF MYBOT`).
- **IntelliBot reuse:** Same get-translation-and-plot pattern for a live map with trajectory trail (final eval dashboard).

## pen
- **Demonstrates:** A wall-avoiding robot with a Pen device that draws colored trails on the floor; the pen is toggled with keyboard keys X/Y and ink color/intensity are randomized.
- **Key API calls:** `getDevice('pen')`, `pen.write(True/False)`, `pen.setInkColor(color, intensity)`, `getKeyboard()` / `keyboard.enable(timestep)` / `keyboard.getKey()`, `motor.getMaxVelocity()`, distance sensors `ds0/ds1.getValue()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** P2: pen trail visualizes the robot's actual driven path during the demo.

## connector
- **Demonstrates:** Two robots cooperating to "hop" over each other by docking their upper arms via Connector devices, a state machine over CONNECTING/PASSING_OVER/NEXT_HOP.
- **Key API calls:** `connector.enablePresence(timestep)`, `connector.getPresence()`, `connector.lock()`, `connector.unlock()`, upper motor `setPosition(...)`, `upper_position_sensor.enable(timestep)` / `getValue()`, `self.getName()` to differentiate robots.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** FINAL EVAL: connector lock/unlock is one candidate "magnetic attach" pickup mechanism.

## vacuum_gripper
- **Note:** Only `vacuum_gripper.c` exists — no Python version.
- **Demonstrates (from the C file):** A robotic arm with a VacuumGripper turns the vacuum on, approaches an object until presence is detected, then moves through ten target pick-and-place orientations, turning the vacuum off to release.
- **Key C API calls:** `wb_robot_get_device("vacuum gripper")`, `wb_vacuum_gripper_turn_on/turn_off`, `wb_vacuum_gripper_enable_presence` / `disable_presence`, `wb_vacuum_gripper_get_presence`, plus `wb_motor_set_position` on roll/pitch/yaw/gripper motors and `wb_inertial_unit_get_roll_pitch_yaw` for arrival checks.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** FINAL EVAL alternative pickup mechanism (option B); the presence-check loop is the pick-confirmation pattern.

## emitter_receiver
- **Demonstrates:** One controller serving two robots (distinguished by `getName()`): the emitter broadcasts "Hello!" each step on channel 1 while the receiver polls its queue and prints the message, both while avoiding walls.
- **Key API calls:** `emitter.getChannel()` / `emitter.setChannel(1)` / `emitter.send("Hello!")`, `receiver.enable(timestep)` / `getQueueLength()` / `getString()` / `nextPacket()`, plus `self.getName()` and distance-sensor driving.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Optional final-eval messaging (second robot / external dashboard).

## bumper
- **Demonstrates:** A bumper touch sensor robot that backs up and turns whenever the bumper is triggered, using a counter-based movement state machine.
- **Key API calls:** `getDevice('bumper')`, `bumper.enable(timestep)`, `bumper.getValue()`, wheel motors `setPosition(float('inf'))` / `setVelocity(...)`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** FINAL EVAL: TouchSensor collision counter for the "Collisions: N" mission-report line.

## led
- **Demonstrates:** A robot that drives with wall avoidance while toggling three LED types: a multi-color discrete LED (random colors), a monochromatic gradual LED (breathing glow), and an RGB gradual LED (random 24-bit colors).
- **Key API calls:** `getDevice('led0'/'led1'/'led2')`, `led.set(color_or_intensity)` (0 = off, 0..255 gradual, 0xRRGGBB for RGB), plus distance sensors `ds0/ds1.getValue()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** P2: LED status color per FSM state using `led.set()`.

## compass
- **Demonstrates:** A robot whose needle Display-arrow constantly points north: it reads the compass north vector, converts it to an angle, and rotates an arrow device accordingly while driving with ultrasonic avoidance.
- **Key API calls:** `compass.enable(timestep)`, `compass.getValues()`, `atan2(north[1], north[0])`, `arrow.setPosition(angle)`, ultrasonic `us0/us1.getValue()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Skimmed only — the InertialUnit covers our heading needs.

## gyro
- **Demonstrates:** A three-axis Gyro mounted on three orthogonal motorized axes; each motor spins in turn and the gyro's x/y/z rates are printed, showing each axis climbing to the motors' 10 rad/s max.
- **Key API calls:** `gyro.enable(timestep)`, `gyro.getValues()` (returns 3 angular rates), `motor_x/y/z.setPosition(...)`, `getBasicTimeStep()`-style fixed `timeStep = 8`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Skimmed only — gyro rates could refine odometry in future work.

## accelerometer
- **Demonstrates:** A spinning robot that uses accelerometer feedback (gravity direction) to switch on whichever bottom LED faces "down," effectively showing orientation from acceleration.
- **Key API calls:** `accelerometer.enable(timestep)`, `accelerometer.getValues()` (3-axis), `front_led/back_led/left_led/right_led.set(True/False)`, `getBasicTimeStep()`.
- **World:** Not referenced in the file.
- **IntelliBot reuse:** Skimmed only — tilt/impact detection could support collision detection in the final eval.
