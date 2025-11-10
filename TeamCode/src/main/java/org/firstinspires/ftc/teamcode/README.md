# TeamCode OpModes Directory

This directory contains custom OpModes (robot programs) for your FTC team. OpModes are programs that run during either the autonomous or driver-controlled (teleop) periods of an FTC match.

## 📁 Files in This Directory

### 🏁 Competition-Ready OpModes

#### **StarterBotTeleop.java** ✅ ENABLED
**Type:** TeleOp (Driver-Controlled)
**Hardware:** 2-motor tank drive + launcher system + feeders
**Purpose:** Official StarterBot driver control program for the DECODE (2025-2026) season

**Features:**
- 2-motor differential/tank drive (left_drive, right_drive)
- High-speed launcher motor with velocity control
- Two continuous rotation servos for feeding projectiles
- State machine for launch sequence management
- BRAKE mode for precise stopping
- RUN_USING_ENCODER for consistent speed control
- Custom PIDF coefficients for launcher optimization
- Arcade-style controls (left stick forward/back, right stick rotate)

**Controls:**
- Left Stick Y: Forward/Backward
- Right Stick X: Rotation
- Y Button: Spin up launcher
- B Button: Stop launcher
- Right Bumper: Fire projectile

**Hardware Requirements:**
- Motors: `left_drive`, `right_drive`, `launcher`
- Servos: `left_feeder`, `right_feeder`

---

#### **StarterBotAuto.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 2-motor tank drive + launcher system + feeders
**Purpose:** Autonomous routine for StarterBot

**What It Does:**
- Starts against the goal
- Launches all three projectiles
- Drives away from starting line
- Uses state machine for sequential actions

**Hardware Requirements:** Same as StarterBotTeleop

---

### 🎓 Sample/Learning OpModes

#### **BasicOmniOpMode_Linear.java** ✅ ENABLED
**Type:** TeleOp (Enhanced 2-Motor Drive)
**Hardware:** 2-motor tank drive
**Purpose:** Production-ready, enhanced 2-motor drive with advanced features

**Features:**
- ✨ **NEW: Competition-hardened with code review fixes**
- Try-catch error handling for graceful failures
- Null checks and hardware validation
- Joystick deadzone (5%) to prevent drift
- Rate-limited telemetry (100ms updates)
- Encoder availability auto-detection with fallback
- 3-speed control system:
  - **Slow Mode (30%)** - Left Bumper
  - **Normal Mode (80%)** - Default
  - **Turbo Mode (100%)** - Right Bumper
- BRAKE mode for precise control
- RUN_USING_ENCODER for velocity control
- Enhanced telemetry with motor velocities
- Arcade-style controls

**Controls:**
- Left Stick Y: Forward/Backward
- Right Stick X: Rotation
- Left Bumper: Slow mode (precision)
- Right Bumper: Turbo mode (speed)

**Hardware Requirements:**
- Motors: `left_drive`, `right_drive`

**Note:** Works with mecanum wheels but provides tank drive only (no strafing). Upgrade to 4 motors for full omnidirectional movement.

**Code Quality:** Grade A - Production-ready for competition ✓

---

#### **BasicOpMode_Linear.java** 🔒 DISABLED
**Type:** TeleOp (Basic Sample)
**Hardware:** 2-motor tank drive
**Purpose:** Minimal skeleton example for learning LinearOpMode structure

**What It Does:**
- Basic 2-motor tank drive
- Simple joystick mapping (left stick = left motor, right stick = right motor)
- Demonstrates LinearOpMode basics

**Best For:** Learning the fundamental structure of an OpMode

---

#### **RobotTeleopPOV_Linear.java** 🔒 DISABLED
**Type:** TeleOp (POV Style)
**Hardware:** 2-motor drive + arm motor + claw servos
**Purpose:** Demonstrates POV (Point of View) control style with manipulator

**What It Does:**
- POV arcade drive (left stick forward/back, right stick rotate)
- Arm control using Y/A buttons (raise/lower)
- Claw control using bumpers (open/close slowly)
- Includes manipulator example

**Best For:** Learning how to control arms and claws

---

### 🤖 Autonomous Sample OpModes

#### **RobotAutoDriveByTime_Linear.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 4-motor drive
**Purpose:** Simple time-based autonomous driving

**What It Does:**
- Drives forward for a set time
- Turns for a set time
- Drives backward for a set time
- Uses `sleep()` for timing

**Best For:** Simplest autonomous introduction, no encoders needed

---

#### **RobotAutoDriveByEncoder_Linear.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 4-motor drive with encoders
**Purpose:** Precise encoder-based autonomous driving

**What It Does:**
- Drives specific distances using encoder counts
- Calculates inches/centimeters from wheel diameter
- More accurate than time-based driving
- Includes functions for driving straight and turning

**Best For:** Learning encoder-based autonomous programming

---

#### **RobotAutoDriveToAprilTagTank.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 2-motor tank drive + camera
**Purpose:** Vision-based autonomous using AprilTags

**What It Does:**
- Uses camera to detect AprilTags
- Drives to a specific AprilTag location
- Adjusts position based on tag detection
- Demonstrates vision processing

**Best For:** Learning AprilTag detection and vision-based navigation

**Hardware Requirements:**
- Camera configured in Robot Configuration
- Motors: `left_drive`, `right_drive`

---

### 📷 Vision Sample OpModes

#### **ConceptAprilTagEasy.java** 🔒 DISABLED
**Type:** TeleOp (Vision Demonstration)
**Hardware:** Camera (any FTC-approved camera)
**Purpose:** Simple AprilTag detection example for DECODE season

**What It Does:**
- Detects AprilTags in camera view
- Displays tag ID, position, and orientation
- Shows how to use the AprilTag library
- Demonstrates basic vision processing

**Best For:** Learning AprilTag detection basics

**Note:** Uses DECODE (2025-2026) season AprilTag library

---

## 🎮 Control Styles Explained

### Arcade Drive
- **Left Stick Y:** Forward/Backward
- **Right Stick X:** Rotation
- One stick moves, one stick turns
- Used by: StarterBotTeleop, BasicOmniOpMode_Linear

### Tank Drive
- **Left Stick Y:** Left side motors
- **Right Stick Y:** Right side motors
- Direct control of each side
- Used by: BasicOpMode_Linear

### POV (Point of View) Drive
- **Left Stick:** Forward/Backward
- **Right Stick:** Rotation
- Similar to arcade but with POV perspective
- Used by: RobotTeleopPOV_Linear

---

## 🔧 Hardware Configuration Guide

Before using any OpMode, configure your robot hardware in the **Robot Configuration** menu on the Driver Station:

### StarterBot Configuration
```
Motors:
  - left_drive (goBILDA motor, port ?)
  - right_drive (goBILDA motor, port ?)
  - launcher (goBILDA high-speed motor, port ?)

Servos:
  - left_feeder (Continuous Rotation Servo, port ?)
  - right_feeder (Continuous Rotation Servo, port ?)
```

### Enhanced 2-Motor Drive Configuration
```
Motors:
  - left_drive (motor, port ?)
  - right_drive (motor, port ?)
```

### 4-Motor Drive Configuration
```
Motors:
  - left_front_drive
  - left_back_drive
  - right_front_drive
  - right_back_drive
```

---

## 📋 Quick Start Guide

### For Drivers (Teleop)
1. **Competition:** Use `StarterBotTeleop` for full robot control
2. **Testing/Practice:** Use `Enhanced 2-Motor Drive` for just the drivetrain
3. **Learning:** Start with `Basic: Linear OpMode` samples

### For Programmers
1. **Start Here:** Copy `BasicOpMode_Linear.java` as a template
2. **Add Features:** Study `BasicOmniOpMode_Linear.java` for enhanced features
3. **Competition Ready:** Base your code on `StarterBotTeleop.java`

### For Autonomous
1. **Beginner:** Start with `RobotAutoDriveByTime_Linear` (simplest)
2. **Intermediate:** Use `RobotAutoDriveByEncoder_Linear` (accurate)
3. **Advanced:** Try `RobotAutoDriveToAprilTagTank` (vision-based)

---

## ✅ Enabled vs Disabled OpModes

**Enabled** (appears in Driver Station menu):
- ✅ StarterBotTeleop
- ✅ BasicOmniOpMode_Linear (Enhanced 2-Motor Drive)

**Disabled** (commented with `@Disabled`, hidden from menu):
- 🔒 All other files

To **enable** a disabled OpMode:
1. Open the file in Android Studio
2. Find the line: `@Disabled`
3. Comment it out: `//@Disabled` or remove it
4. Rebuild and deploy to Robot Controller

To **disable** an OpMode:
1. Add `@Disabled` annotation above `@TeleOp` or `@Autonomous`

---

## 🏆 Recommended OpModes for Competition

### Driver-Controlled Period
**Primary:** `StarterBotTeleop` - Full robot control with launcher
**Backup:** `Enhanced 2-Motor Drive` - Drivetrain only (no launcher)

### Autonomous Period
**Primary:** `StarterBotAuto` - Launches projectiles and moves
**Custom:** Build your own based on the samples

---

## 📚 Additional Resources

- **Official FTC Docs:** https://ftc-docs.firstinspires.org/
- **Game Manual 0:** https://gm0.org/ (community guide)
- **JavaDoc Reference:** https://javadoc.io/doc/org.firstinspires.ftc
- **Sample OpModes:** `FtcRobotController/src/main/java/.../samples/`

---

## 🛠️ Development Tips

1. **Always test motor directions** - Push left stick forward, robot should go forward
2. **Use telemetry** - Display debug info on Driver Station
3. **Start simple** - Get basic movement working before adding complexity
4. **Use encoders** - RUN_USING_ENCODER provides better control
5. **Enable BRAKE mode** - Stops robot faster and more precisely
6. **Add error handling** - Check for null hardware and configuration errors
7. **Rate-limit telemetry** - Update every 100ms, not every loop

---

## 🐛 Common Issues

**OpMode doesn't appear in Driver Station:**
- Check that `@Disabled` is removed or commented out
- Verify the OpMode compiled successfully
- Rebuild and reinstall the app

**Robot doesn't move:**
- Check motor configuration names match code
- Verify motors are plugged in
- Test motor directions individually

**Motors run backward:**
- Flip motor direction: `FORWARD` ↔ `REVERSE`
- See motor direction section in each file

**Controller drift:**
- Use deadzone (built into Enhanced 2-Motor Drive)
- Recalibrate gamepad in Driver Station

**Encoders not working:**
- Check encoder cables are plugged in adjacent to motor
- Verify motor polarity is consistent
- Use Enhanced 2-Motor Drive (auto-detects encoders)

---

## 📝 License

All OpModes in this directory are based on FTC SDK samples and are subject to the FIRST Tech Challenge software license. See individual file headers for specific copyright information.

---

**Last Updated:** 2025-01-09
**Season:** DECODE (2025-2026)
**SDK Version:** 11.0
