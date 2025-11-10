# FTC OpMode Types - Comprehensive Guide

**Last Updated:** 2025-01-09
**FTC SDK Version:** 11.0
**Season:** DECODE (2025-2026)

---

## Table of Contents

1. [Overview](#overview)
2. [The Two Base OpMode Types](#the-two-base-opmode-types)
3. [LinearOpMode Deep Dive](#linearopmode-deep-dive)
4. [OpMode (Iterative) Deep Dive](#opmode-iterative-deep-dive)
5. [Comparison Table](#comparison-table)
6. [OpMode Annotations](#opmode-annotations)
7. [Choosing the Right Type](#choosing-the-right-type)
8. [Common Patterns](#common-patterns)
9. [Common Mistakes](#common-mistakes)
10. [Related Concepts](#related-concepts)
11. [Examples from TeamCode](#examples-from-teamcode)
12. [Best Practices](#best-practices)
13. [FAQ](#faq)
14. [Resources](#resources)

---

## Overview

In FIRST Tech Challenge (FTC), an **OpMode** is a program that runs during either the autonomous or driver-controlled (teleop) period of a match. There are **exactly 2 base OpMode types** to choose from:

1. **LinearOpMode** - Sequential, top-to-bottom programming
2. **OpMode** - Iterative, event-driven programming

Everything else (annotations, naming conventions, etc.) is built on top of these two fundamental types.

---

## The Two Base OpMode Types

### Quick Comparison

| Feature | LinearOpMode | OpMode |
|---------|-------------|---------|
| **Programming Model** | Sequential | Event-driven |
| **Complexity** | 🟢 Simple | 🟡 Moderate |
| **Best For** | Autonomous, Beginners | Complex Teleop |
| **Main Method** | `runOpMode()` | `init()`, `loop()`, `stop()` |
| **Can Block** | ✅ Yes (`sleep()`, `while`) | ❌ No |
| **Loop Control** | Manual | Automatic (~50 Hz) |

### Inheritance Hierarchy

```
java.lang.Object
    ↓
com.qualcomm.robotcore.eventloop.opmode.OpMode
    ├─ Your iterative OpModes (extend OpMode)
    │   └─ Example: StarterBotTeleop
    │
    └─ com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
        └─ Your linear OpModes (extend LinearOpMode)
            └─ Example: BasicOmniOpMode_Linear
```

**Important:** `LinearOpMode` is actually a **subclass** of `OpMode`, but provides a completely different programming interface.

---

## LinearOpMode Deep Dive

### What Is It?

LinearOpMode provides a **sequential, procedural** programming model where your code executes from top to bottom, like reading a book. It's the most intuitive for beginners and perfect for autonomous routines.

### Basic Structure

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="My Linear OpMode", group="Tutorial")
public class MyLinearOpMode extends LinearOpMode {

    // Declare hardware
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void runOpMode() {
        // 1. INITIALIZATION PHASE
        // This runs when you press INIT
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Show initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 2. WAIT FOR START
        // Code execution STOPS here until START is pressed
        waitForStart();

        // 3. MAIN EXECUTION PHASE
        // This loop runs repeatedly until STOP is pressed
        while (opModeIsActive()) {
            // Read gamepad inputs
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            // Set motor powers
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Update telemetry
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }

        // 4. CLEANUP (optional)
        // This runs after STOP is pressed
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
```

### Lifecycle Diagram

```
Press INIT
    ↓
runOpMode() called
    ↓
[Your initialization code executes]
    ↓
waitForStart() reached
    ↓
⏸️  PAUSED - Waiting for START button
    ↓
Press START
    ↓
[Code after waitForStart() executes]
    ↓
while (opModeIsActive()) loop begins
    ↓
[Loop body executes repeatedly]
    ↓
Press STOP
    ↓
opModeIsActive() returns false
    ↓
Exit loop
    ↓
[Optional cleanup code]
    ↓
runOpMode() returns
    ↓
OpMode ends
```

### Key Methods

#### `waitForStart()`
- **Purpose:** Blocks execution until START button is pressed
- **Use:** Separates initialization from execution
- **Returns:** `void`
- **Note:** Code before this runs during INIT, code after runs during match

```java
// Code here runs during INIT
telemetry.addData("Status", "Ready to start");
telemetry.update();

waitForStart();  // ⏸️ Waits here

// Code here runs after START is pressed
telemetry.addData("Status", "Running!");
```

#### `opModeIsActive()`
- **Purpose:** Checks if OpMode is still running
- **Returns:** `true` if running, `false` if STOP was pressed
- **Use:** Condition for main loop to continue
- **Critical:** Always use this in your while loop condition

```java
while (opModeIsActive()) {
    // Loop continues while running
    // Exits immediately when STOP is pressed
}
```

#### `sleep(long milliseconds)`
- **Purpose:** Pauses execution for specified time
- **Use:** Timing in autonomous routines
- **Note:** Blocks execution (OK in LinearOpMode!)

```java
motor.setPower(1.0);
sleep(2000);  // Wait 2 seconds
motor.setPower(0);
```

#### `idle()`
- **Purpose:** Yields control briefly to system
- **Use:** In long loops to prevent watchdog timeouts
- **Best Practice:** Call in tight loops without sleep

```java
while (opModeIsActive() && motor.isBusy()) {
    idle();  // Let system do housekeeping
}
```

### When to Use LinearOpMode

✅ **Perfect for:**
- Autonomous routines (sequential tasks)
- Learning FTC programming (most intuitive)
- Simple teleop programs
- Testing hardware (quick scripts)
- Programs with clear step-by-step logic

❌ **Not ideal for:**
- Complex teleop with multiple concurrent systems
- When you need guaranteed loop timing
- Programs requiring maximum responsiveness

### LinearOpMode Examples

#### Example 1: Simple Autonomous

```java
@Autonomous(name="Drive Square", group="Tutorial")
public class DriveSquare extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;

    @Override
    public void runOpMode() {
        // Initialize
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        waitForStart();

        // Drive in a square
        for (int i = 0; i < 4; i++) {
            driveForward(1000);  // Drive forward 1 second
            turnRight(500);      // Turn right 0.5 seconds
        }
    }

    private void driveForward(long milliseconds) {
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(milliseconds);
        stopMotors();
    }

    private void turnRight(long milliseconds) {
        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        sleep(milliseconds);
        stopMotors();
    }

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
```

#### Example 2: Encoder-Based Autonomous

```java
@Autonomous(name="Drive Distance", group="Tutorial")
public class DriveDistance extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;
    private static final double COUNTS_PER_INCH = 50; // Adjust for your robot

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Drive 24 inches forward
        driveDistance(24, 0.5);

        // Turn 90 degrees
        turnDegrees(90, 0.3);

        // Drive 12 inches forward
        driveDistance(12, 0.5);
    }

    private void driveDistance(double inches, double power) {
        int targetPosition = (int)(inches * COUNTS_PER_INCH);

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + targetPosition);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + targetPosition);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            telemetry.addData("Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnDegrees(double degrees, double power) {
        // Implementation depends on your robot's dimensions
        // This is a simplified example
        int targetPosition = (int)(degrees * 10); // Adjust multiplier

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + targetPosition);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - targetPosition);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            idle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
```

---

## OpMode (Iterative) Deep Dive

### What Is It?

OpMode provides an **event-driven, state machine** programming model where the FTC system repeatedly calls your methods at a fixed rate (~50 Hz). This gives you fine-grained control over initialization and execution phases.

### Basic Structure

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="My OpMode", group="Tutorial")
public class MyOpMode extends OpMode {

    // Declare hardware
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    // State variables
    private int loopCount = 0;

    /**
     * Called ONCE when INIT is pressed
     */
    @Override
    public void init() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Show status
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Called REPEATEDLY while waiting for START
     * Runs at ~50 Hz
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
    }

    /**
     * Called ONCE when START is pressed
     */
    @Override
    public void start() {
        loopCount = 0;
        telemetry.addData("Status", "Started!");
    }

    /**
     * Called REPEATEDLY during match
     * Runs at ~50 Hz (every 20ms)
     */
    @Override
    public void loop() {
        loopCount++;

        // Read gamepad inputs
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Update telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Loop Count", loopCount);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
    }

    /**
     * Called ONCE when STOP is pressed
     */
    @Override
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addData("Status", "Stopped");
    }
}
```

### Lifecycle Diagram

```
Press INIT
    ↓
init() called ONCE
    ↓
init_loop() called REPEATEDLY (~50 Hz)
    ↓  ↓  ↓  (loops while waiting)
    ↓  ↓  ↓
    ↓  ↓  ↓
Press START
    ↓
start() called ONCE
    ↓
loop() called REPEATEDLY (~50 Hz)
    ↓  ↓  ↓  (loops during match)
    ↓  ↓  ↓
    ↓  ↓  ↓
Press STOP
    ↓
stop() called ONCE
    ↓
OpMode ends
```

### Key Methods

#### `init()`
- **Called:** Once when INIT button is pressed
- **Purpose:** Initialize hardware, set default states
- **Duration:** Should complete quickly (< 1 second)
- **Cannot:** Block or loop indefinitely

```java
@Override
public void init() {
    motor = hardwareMap.get(DcMotor.class, "motor");
    servo = hardwareMap.get(Servo.class, "servo");

    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    servo.setPosition(0.5);

    telemetry.addData("Status", "Ready");
}
```

#### `init_loop()`
- **Called:** Repeatedly (~50 Hz) while waiting for START
- **Purpose:** Update telemetry, check sensors, show status
- **Duration:** Must complete quickly (< 20ms per call)
- **Cannot:** Block or use sleep()

```java
@Override
public void init_loop() {
    telemetry.addData("Encoder", motor.getCurrentPosition());
    telemetry.addData("Waiting", "Press START to begin");
}
```

#### `start()`
- **Called:** Once when START button is pressed
- **Purpose:** Reset timers, initialize runtime variables
- **Duration:** Should complete quickly
- **Typical use:** Start timers, reset counters

```java
@Override
public void start() {
    runtime.reset();
    stateTimer.reset();
    currentState = State.IDLE;
}
```

#### `loop()`
- **Called:** Repeatedly (~50 Hz / every 20ms) during match
- **Purpose:** Main control logic, read inputs, update outputs
- **Duration:** **MUST** complete in < 20ms
- **Cannot:** Block, use sleep(), or long while loops

```java
@Override
public void loop() {
    // This MUST complete quickly!

    // Read inputs
    double power = gamepad1.left_stick_y;

    // Process
    motor.setPower(power);

    // Update telemetry
    telemetry.addData("Power", power);

    // Total time: < 20ms
}
```

#### `stop()`
- **Called:** Once when STOP button is pressed
- **Purpose:** Cleanup, stop motors, save data
- **Duration:** Should complete quickly

```java
@Override
public void stop() {
    motor.setPower(0);
    servo.setPosition(0);
    telemetry.addData("Status", "Stopped");
}
```

### When to Use OpMode (Iterative)

✅ **Perfect for:**
- Complex teleop with multiple systems
- State machines (drive + arm + intake + launcher)
- Maximum responsiveness requirements
- Professional/competition code
- Multiple concurrent operations

❌ **Not ideal for:**
- Simple sequential autonomous
- Learning FTC programming (steeper curve)
- Code requiring sleep() or blocking waits

### OpMode State Machine Pattern

The most common pattern with OpMode is using enums for state machines:

```java
@TeleOp(name="State Machine Example", group="Tutorial")
public class StateMachineExample extends OpMode {

    // Define states
    private enum ArmState {
        DOWN,
        MOVING_UP,
        UP,
        MOVING_DOWN
    }

    private enum IntakeState {
        STOPPED,
        COLLECTING,
        EJECTING
    }

    private DcMotor armMotor, leftDrive, rightDrive;
    private CRServo intakeServo;

    private ArmState armState = ArmState.DOWN;
    private IntakeState intakeState = IntakeState.STOPPED;

    private ElapsedTime armTimer = new ElapsedTime();

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intakeServo = hardwareMap.get(CRServo.class, "intake");
    }

    @Override
    public void loop() {
        // Drive is ALWAYS responsive (runs every loop)
        driveTrain();

        // Arm state machine (non-blocking)
        armStateMachine();

        // Intake state machine (non-blocking)
        intakeStateMachine();

        // Telemetry
        telemetry.addData("Arm State", armState);
        telemetry.addData("Intake State", intakeState);
    }

    private void driveTrain() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        leftDrive.setPower(forward + turn);
        rightDrive.setPower(forward - turn);
    }

    private void armStateMachine() {
        switch (armState) {
            case DOWN:
                armMotor.setPower(0);
                if (gamepad1.y) {
                    armState = ArmState.MOVING_UP;
                    armTimer.reset();
                }
                break;

            case MOVING_UP:
                armMotor.setPower(0.5);
                if (armTimer.seconds() > 2.0) {
                    armState = ArmState.UP;
                }
                break;

            case UP:
                armMotor.setPower(0.1);  // Hold position
                if (gamepad1.a) {
                    armState = ArmState.MOVING_DOWN;
                    armTimer.reset();
                }
                break;

            case MOVING_DOWN:
                armMotor.setPower(-0.5);
                if (armTimer.seconds() > 2.0) {
                    armState = ArmState.DOWN;
                }
                break;
        }
    }

    private void intakeStateMachine() {
        switch (intakeState) {
            case STOPPED:
                intakeServo.setPower(0);
                if (gamepad1.left_bumper) {
                    intakeState = IntakeState.COLLECTING;
                } else if (gamepad1.right_bumper) {
                    intakeState = IntakeState.EJECTING;
                }
                break;

            case COLLECTING:
                intakeServo.setPower(1.0);
                if (!gamepad1.left_bumper) {
                    intakeState = IntakeState.STOPPED;
                }
                break;

            case EJECTING:
                intakeServo.setPower(-1.0);
                if (!gamepad1.right_bumper) {
                    intakeState = IntakeState.STOPPED;
                }
                break;
        }
    }
}
```

---

## Comparison Table

### Detailed Feature Comparison

| Feature | LinearOpMode | OpMode (Iterative) |
|---------|-------------|-------------------|
| **Base Class** | `LinearOpMode` | `OpMode` |
| **Programming Model** | Sequential (procedural) | Event-driven (state machine) |
| **Main Method** | `runOpMode()` | `init()`, `init_loop()`, `start()`, `loop()`, `stop()` |
| **Execution Flow** | Top-to-bottom | Method callbacks |
| **Loop Control** | Manual (`while (opModeIsActive())`) | Automatic (FTC manages) |
| **Loop Rate** | Variable (depends on code) | Fixed ~50 Hz (20ms) |
| **Can Use `sleep()`** | ✅ Yes | ❌ No (blocks loop) |
| **Can Use Blocking `while`** | ✅ Yes | ❌ No (blocks loop) |
| **Init Phase** | Code before `waitForStart()` | `init()` method |
| **Start Wait** | `waitForStart()` method | Automatic between `init_loop()` and `start()` |
| **Main Phase** | Code in `while (opModeIsActive())` | `loop()` method |
| **Learning Curve** | 🟢 Easy | 🟡 Moderate |
| **Code Verbosity** | Less code | More code (multiple methods) |
| **Best For Autonomous** | ✅ Excellent (sequential tasks) | ⚠️ Possible (needs state machines) |
| **Best For Teleop** | ✅ Good (simple) | ✅ Excellent (complex) |
| **Multi-System Control** | ⚠️ Harder (nested logic) | ✅ Easy (separate state machines) |
| **Guaranteed Timing** | ❌ No | ✅ Yes (~50 Hz) |
| **Code Responsiveness** | Depends on implementation | ✅ Always responsive |
| **State Management** | Manual (variables, if/else) | Natural (enums, switch) |
| **Example Files** | 7 in TeamCode | 2 in TeamCode |

### Performance Comparison

| Metric | LinearOpMode | OpMode |
|--------|-------------|---------|
| **Typical Loop Time** | 10-100ms (varies) | 20ms (fixed) |
| **Worst Case Latency** | Can be seconds (if using sleep) | 20ms max |
| **Best For Responsiveness** | ⚠️ Variable | ✅ Guaranteed |
| **CPU Usage** | Lower (can sleep) | Higher (always looping) |
| **Memory Usage** | Similar | Similar |

### Use Case Recommendations

| Use Case | Recommended Type | Reason |
|----------|-----------------|---------|
| **Simple Autonomous** | LinearOpMode | Sequential steps are natural |
| **Encoder-Based Auto** | LinearOpMode | Easy to wait for encoder targets |
| **Vision-Based Auto** | LinearOpMode | Easy to wait for detection |
| **Simple Teleop** | LinearOpMode | Quick to implement |
| **Complex Teleop** | OpMode | Multiple systems easier to manage |
| **State Machine Heavy** | OpMode | Native support for states |
| **Learning/Teaching** | LinearOpMode | More intuitive |
| **Competition Code** | OpMode | Maximum reliability and responsiveness |

---

## OpMode Annotations

Annotations are **metadata labels** that tell the Driver Station how to categorize your OpMode. They are **not** different OpMode types!

### @TeleOp

Marks an OpMode as driver-controlled (teleop).

```java
@TeleOp(name="My Teleop", group="Competition")
public class MyTeleop extends LinearOpMode {
    // ...
}
```

**Parameters:**
- `name` (required): Display name in Driver Station menu
- `group` (optional): Category for organization (default: "default")

**Driver Station:** Shows in "TeleOp" section

### @Autonomous

Marks an OpMode as autonomous.

```java
@Autonomous(name="Red Alliance Auto", group="Autonomous")
public class RedAuto extends LinearOpMode {
    // ...
}
```

**Parameters:**
- `name` (required): Display name in Driver Station menu
- `group` (optional): Category for organization
- `preselectTeleOp` (optional): TeleOp to auto-select after this autonomous

**Driver Station:** Shows in "Autonomous" section

**Advanced Example:**
```java
@Autonomous(
    name="Blue Alliance Auto",
    group="Competition",
    preselectTeleOp="Blue Alliance Teleop"
)
public class BlueAuto extends LinearOpMode {
    // After this auto completes, Driver Station will
    // automatically select "Blue Alliance Teleop"
}
```

### @Disabled

Hides an OpMode from the Driver Station menu.

```java
@Disabled  // This OpMode won't appear in menu
@TeleOp(name="Work In Progress")
public class WorkInProgress extends LinearOpMode {
    // ...
}
```

**Use Cases:**
- Work-in-progress code
- Deprecated OpModes
- Sample code you don't want to run
- Testing/debugging code

**To Enable:** Comment out or remove the `@Disabled` line:
```java
//@Disabled  // Now it will appear!
@TeleOp(name="Ready To Test")
```

### Annotation Combinations

You **must** use either `@TeleOp` or `@Autonomous`, and optionally `@Disabled`:

```java
// Valid combinations:
@TeleOp(name="...")                    // ✅ Teleop, enabled
@Autonomous(name="...")                 // ✅ Auto, enabled
@Disabled @TeleOp(name="...")          // ✅ Teleop, disabled
@Disabled @Autonomous(name="...")      // ✅ Auto, disabled

// Invalid combinations:
@TeleOp @Autonomous                     // ❌ Can't be both
// (no annotation)                      // ❌ Must have one
```

---

## Choosing the Right Type

### Decision Tree

```
Do you need to write autonomous code?
├─ Yes → Do you need complex state management?
│        ├─ No → LinearOpMode ✅
│        └─ Yes → OpMode (but LinearOpMode probably works)
│
└─ No (writing teleop) → Do you have multiple subsystems?
         ├─ No (simple drive only) → LinearOpMode ✅
         └─ Yes (drive + arm + intake + launcher)
                  ├─ Are you a beginner? → Start with LinearOpMode
                  └─ Need max performance? → OpMode ✅
```

### Specific Scenarios

#### Scenario 1: First Time Programming FTC
**Recommendation:** LinearOpMode
**Why:** Simpler to understand, more intuitive flow

```java
@TeleOp(name="First OpMode")
public class FirstOpMode extends LinearOpMode {
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
```

#### Scenario 2: Autonomous - Drive Forward, Turn, Drive Back
**Recommendation:** LinearOpMode
**Why:** Sequential steps are natural

```java
@Autonomous(name="Simple Auto")
public class SimpleAuto extends LinearOpMode {
    public void runOpMode() {
        initHardware();
        waitForStart();

        driveForward(24);   // 24 inches
        turnRight(90);      // 90 degrees
        driveForward(12);   // 12 inches
    }
}
```

#### Scenario 3: Competition Teleop with Drive, Arm, Intake, Launcher
**Recommendation:** OpMode
**Why:** Multiple state machines are easier to manage

```java
@TeleOp(name="Competition Teleop")
public class CompetitionTeleop extends OpMode {

    enum ArmState { DOWN, MOVING, UP }
    enum IntakeState { STOPPED, COLLECTING, EJECTING }
    enum LauncherState { IDLE, SPINNING, LAUNCHING }

    public void loop() {
        driveStateMachine();    // Always responsive
        armStateMachine();      // Independent
        intakeStateMachine();   // Independent
        launcherStateMachine(); // Independent
    }
}
```

#### Scenario 4: Vision-Based Autonomous
**Recommendation:** LinearOpMode
**Why:** Easier to wait for vision detection

```java
@Autonomous(name="Vision Auto")
public class VisionAuto extends LinearOpMode {
    public void runOpMode() {
        initVision();
        waitForStart();

        // Drive until AprilTag detected
        while (opModeIsActive() && !tagDetected()) {
            driveForward();
            sleep(100);
        }
        stopDriving();

        // Align to tag
        alignToTag();

        // Continue autonomous
        // ...
    }
}
```

---

## Common Patterns

### Pattern 1: LinearOpMode with Helper Functions

```java
@Autonomous(name="Autonomous Pattern")
public class AutonomousPattern extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;
    private static final double DRIVE_POWER = 0.6;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Main autonomous sequence
        driveForwardTime(2000);    // 2 seconds
        turnRightTime(1000);       // 1 second
        driveForwardTime(1500);    // 1.5 seconds
        stop Motors();
    }

    private void initHardware() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void driveForwardTime(long milliseconds) {
        leftMotor.setPower(DRIVE_POWER);
        rightMotor.setPower(DRIVE_POWER);
        sleep(milliseconds);
        stopMotors();
    }

    private void turnRightTime(long milliseconds) {
        leftMotor.setPower(DRIVE_POWER);
        rightMotor.setPower(-DRIVE_POWER);
        sleep(milliseconds);
        stopMotors();
    }

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
```

### Pattern 2: OpMode with Multiple State Machines

```java
@TeleOp(name="Multi-System Teleop")
public class MultiSystemTeleop extends OpMode {

    // Hardware
    private DcMotor leftDrive, rightDrive, armMotor, launcherMotor;
    private Servo claw;

    // States
    private enum ArmState { DOWN, MOVING_UP, UP, MOVING_DOWN }
    private enum ClawState { OPEN, CLOSING, CLOSED, OPENING }
    private enum LauncherState { IDLE, SPINNING_UP, READY, LAUNCHING }

    private ArmState armState = ArmState.DOWN;
    private ClawState clawState = ClawState.OPEN;
    private LauncherState launcherState = LauncherState.IDLE;

    // Timers
    private ElapsedTime armTimer = new ElapsedTime();
    private ElapsedTime clawTimer = new ElapsedTime();
    private ElapsedTime launcherTimer = new ElapsedTime();

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        claw = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop() {
        // All systems run independently every loop
        driveControl();           // Always responsive
        armStateMachine();        // Independent state machine
        clawStateMachine();       // Independent state machine
        launcherStateMachine();   // Independent state machine

        updateTelemetry();
    }

    private void driveControl() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        leftDrive.setPower(forward + turn);
        rightDrive.setPower(forward - turn);
    }

    private void armStateMachine() {
        switch (armState) {
            case DOWN:
                armMotor.setPower(0);
                if (gamepad1.y) {
                    armState = ArmState.MOVING_UP;
                    armTimer.reset();
                }
                break;
            case MOVING_UP:
                armMotor.setPower(0.5);
                if (armTimer.seconds() > 1.5) {
                    armState = ArmState.UP;
                }
                break;
            case UP:
                armMotor.setPower(0.1);  // Hold
                if (gamepad1.a) {
                    armState = ArmState.MOVING_DOWN;
                    armTimer.reset();
                }
                break;
            case MOVING_DOWN:
                armMotor.setPower(-0.5);
                if (armTimer.seconds() > 1.5) {
                    armState = ArmState.DOWN;
                }
                break;
        }
    }

    private void clawStateMachine() {
        switch (clawState) {
            case OPEN:
                claw.setPosition(0.0);
                if (gamepad1.right_bumper) {
                    clawState = ClawState.CLOSING;
                    clawTimer.reset();
                }
                break;
            case CLOSING:
                claw.setPosition(1.0);
                if (clawTimer.seconds() > 0.5) {
                    clawState = ClawState.CLOSED;
                }
                break;
            case CLOSED:
                if (gamepad1.left_bumper) {
                    clawState = ClawState.OPENING;
                    clawTimer.reset();
                }
                break;
            case OPENING:
                claw.setPosition(0.0);
                if (clawTimer.seconds() > 0.5) {
                    clawState = ClawState.OPEN;
                }
                break;
        }
    }

    private void launcherStateMachine() {
        switch (launcherState) {
            case IDLE:
                launcherMotor.setPower(0);
                if (gamepad1.x) {
                    launcherState = LauncherState.SPINNING_UP;
                    launcherTimer.reset();
                }
                break;
            case SPINNING_UP:
                launcherMotor.setPower(1.0);
                if (launcherTimer.seconds() > 2.0) {
                    launcherState = LauncherState.READY;
                }
                break;
            case READY:
                launcherMotor.setPower(1.0);
                if (gamepad1.b) {
                    launcherState = LauncherState.LAUNCHING;
                    launcherTimer.reset();
                }
                if (!gamepad1.x) {
                    launcherState = LauncherState.IDLE;
                }
                break;
            case LAUNCHING:
                // Feed mechanism would run here
                if (launcherTimer.seconds() > 0.5) {
                    launcherState = LauncherState.READY;
                }
                break;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Arm State", armState);
        telemetry.addData("Claw State", clawState);
        telemetry.addData("Launcher State", launcherState);
    }
}
```

### Pattern 3: LinearOpMode with ElapsedTime (Non-Blocking)

```java
@TeleOp(name="Non-Blocking LinearOpMode")
public class NonBlockingLinear extends LinearOpMode {

    private DcMotor motor;
    private Servo servo;

    private ElapsedTime motorTimer = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

    private enum MotorState { STOPPED, RUNNING, COOLDOWN }
    private MotorState motorState = MotorState.STOPPED;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        motorTimer.reset();
        servoTimer.reset();

        while (opModeIsActive()) {
            // Non-blocking motor control with timer
            switch (motorState) {
                case STOPPED:
                    motor.setPower(0);
                    if (gamepad1.a) {
                        motorState = MotorState.RUNNING;
                        motorTimer.reset();
                    }
                    break;
                case RUNNING:
                    motor.setPower(1.0);
                    if (motorTimer.seconds() > 2.0) {
                        motorState = MotorState.COOLDOWN;
                        motorTimer.reset();
                    }
                    break;
                case COOLDOWN:
                    motor.setPower(0);
                    if (motorTimer.seconds() > 1.0) {
                        motorState = MotorState.STOPPED;
                    }
                    break;
            }

            // Non-blocking servo control
            if (gamepad1.x && servoTimer.seconds() > 0.5) {
                servo.setPosition(0.0);
                servoTimer.reset();
            } else if (gamepad1.b && servoTimer.seconds() > 0.5) {
                servo.setPosition(1.0);
                servoTimer.reset();
            }

            telemetry.addData("Motor State", motorState);
            telemetry.addData("Motor Timer", motorTimer.seconds());
            telemetry.update();
        }
    }
}
```

---

## Common Mistakes

### Mistake 1: Using sleep() in OpMode.loop()

❌ **WRONG:**
```java
@TeleOp(name="Bad OpMode")
public class BadOpMode extends OpMode {
    @Override
    public void loop() {
        motor.setPower(1.0);
        sleep(2000);  // ❌ BLOCKS EVERYTHING FOR 2 SECONDS!
        motor.setPower(0);
        // Robot becomes completely unresponsive
    }
}
```

✅ **CORRECT:**
```java
@TeleOp(name="Good OpMode")
public class GoodOpMode extends OpMode {

    private ElapsedTime timer = new ElapsedTime();
    private boolean motorRunning = false;

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (!motorRunning && gamepad1.a) {
            motor.setPower(1.0);
            motorRunning = true;
            timer.reset();
        }

        if (motorRunning && timer.seconds() > 2.0) {
            motor.setPower(0);
            motorRunning = false;
        }

        // Robot stays responsive!
    }
}
```

### Mistake 2: Forgetting opModeIsActive() in LinearOpMode

❌ **WRONG:**
```java
@Autonomous(name="Bad Auto")
public class BadAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (true) {  // ❌ NEVER STOPS!
            motor.setPower(1.0);
            sleep(1000);
        }
        // Pressing STOP doesn't work!
    }
}
```

✅ **CORRECT:**
```java
@Autonomous(name="Good Auto")
public class GoodAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {  // ✅ Checks if still running
            motor.setPower(1.0);
            sleep(1000);
        }
        // Pressing STOP exits loop immediately
    }
}
```

### Mistake 3: Long Blocking Operations in OpMode.init()

❌ **WRONG:**
```java
@TeleOp(name="Slow Init")
public class SlowInit extends OpMode {
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        // ❌ This blocks initialization for 10 seconds!
        for (int i = 0; i < 10; i++) {
            telemetry.addData("Calibrating", i);
            telemetry.update();
            sleep(1000);  // Can't use sleep() anyway!
        }
    }
}
```

✅ **CORRECT:**
```java
@TeleOp(name="Good Init")
public class GoodInit extends OpMode {

    private int calibrationStep = 0;
    private ElapsedTime calibrationTimer = new ElapsedTime();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        calibrationTimer.reset();
    }

    @Override
    public void init_loop() {
        // Non-blocking calibration in init_loop
        if (calibrationTimer.seconds() > 1.0 && calibrationStep < 10) {
            calibrationStep++;
            calibrationTimer.reset();
        }

        telemetry.addData("Calibrating", calibrationStep + "/10");

        if (calibrationStep >= 10) {
            telemetry.addData("Status", "Ready!");
        }
    }
}
```

### Mistake 4: Not Handling Hardware Exceptions

❌ **WRONG:**
```java
@TeleOp(name="No Error Handling")
public class NoErrorHandling extends LinearOpMode {
    @Override
    public void runOpMode() {
        // ❌ Crashes if motor not configured!
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(1.0);
        }
    }
}
```

✅ **CORRECT:**
```java
@TeleOp(name="With Error Handling")
public class WithErrorHandling extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor = null;

        try {
            motor = hardwareMap.get(DcMotor.class, "motor");
        } catch (IllegalArgumentException e) {
            telemetry.addLine("ERROR: Motor 'motor' not found!");
            telemetry.addLine("Check robot configuration.");
            telemetry.update();
            sleep(5000);
            return;  // Exit gracefully
        }

        if (motor == null) {
            telemetry.addLine("ERROR: Motor initialization failed!");
            telemetry.update();
            return;
        }

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
```

### Mistake 5: Telemetry Spam in OpMode.loop()

❌ **WRONG:**
```java
@TeleOp(name="Telemetry Spam")
public class TelemetrySpam extends OpMode {
    @Override
    public void loop() {
        // ❌ Updates telemetry 50 times per second!
        // Slows down loop and floods Driver Station
        telemetry.addData("Status", "Running");
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", gamepad1.right_stick_y);
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        // ... 20 more lines ...
        telemetry.update();  // Every 20ms = 50 updates/sec!
    }
}
```

✅ **CORRECT:**
```java
@TeleOp(name="Throttled Telemetry")
public class ThrottledTelemetry extends OpMode {

    private ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_UPDATE_RATE = 0.1;  // 100ms

    @Override
    public void loop() {
        // Control logic runs every loop (~20ms)
        motor.setPower(gamepad1.left_stick_y);

        // Telemetry updates only 10 times per second
        if (telemetryTimer.seconds() > TELEMETRY_UPDATE_RATE) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.update();
            telemetryTimer.reset();
        }
    }
}
```

---

## Related Concepts

### 1. OpModeManager
**What it is:** Internal FTC system that manages OpMode lifecycle
**You don't extend it:** It's framework code
**What it does:** Calls your `init()`, `loop()`, `stop()` methods

### 2. HardwareMap
**What it is:** Interface to access robot hardware
**How to use:** `hardwareMap.get(ClassName.class, "name")`
**Available in:** Both LinearOpMode and OpMode

```java
DcMotor motor = hardwareMap.get(DcMotor.class, "motor_name");
Servo servo = hardwareMap.get(Servo.class, "servo_name");
```

### 3. Gamepad
**What it is:** Controller input interface
**Available:** `gamepad1` and `gamepad2`
**Properties:** Sticks, buttons, triggers, d-pad

```java
double power = -gamepad1.left_stick_y;  // Joystick
boolean pressed = gamepad1.a;            // Button
boolean bumper = gamepad1.left_bumper;   // Bumper
```

### 4. Telemetry
**What it is:** Display output to Driver Station
**How to use:** `telemetry.addData()` then `telemetry.update()`

```java
telemetry.addData("Label", "Value");
telemetry.addData("Motor Power", motor.getPower());
telemetry.update();  // Send to Driver Station
```

### 5. ElapsedTime
**What it is:** High-resolution timer
**Use:** Timing in autonomous, rate limiting

```java
ElapsedTime timer = new ElapsedTime();
timer.reset();  // Start timer
double seconds = timer.seconds();  // Get elapsed time
```

---

## Examples from TeamCode

### LinearOpMode Examples (7 files)

1. **BasicOmniOpMode_Linear.java** ✅
   - Type: LinearOpMode + TeleOp
   - Purpose: Enhanced 2-motor drive with speed control
   - Features: Error handling, deadzone, rate-limited telemetry

2. **BasicOpMode_Linear.java** 🔒
   - Type: LinearOpMode + TeleOp
   - Purpose: Minimal skeleton example
   - Best for: Learning basics

3. **RobotTeleopPOV_Linear.java** 🔒
   - Type: LinearOpMode + TeleOp
   - Purpose: POV drive with arm and claw
   - Best for: Learning manipulators

4. **RobotAutoDriveByTime_Linear.java** 🔒
   - Type: LinearOpMode + Autonomous
   - Purpose: Time-based autonomous
   - Best for: Simplest autonomous

5. **RobotAutoDriveByEncoder_Linear.java** 🔒
   - Type: LinearOpMode + Autonomous
   - Purpose: Encoder-based autonomous
   - Best for: Accurate autonomous

6. **RobotAutoDriveToAprilTagTank.java** 🔒
   - Type: LinearOpMode + Autonomous
   - Purpose: Vision-based autonomous
   - Best for: AprilTag navigation

7. **ConceptAprilTagEasy.java** 🔒
   - Type: LinearOpMode + TeleOp
   - Purpose: AprilTag detection demo
   - Best for: Learning vision processing

### OpMode Examples (2 files)

1. **StarterBotTeleop.java** ✅
   - Type: OpMode + TeleOp
   - Purpose: Competition robot with launcher
   - Features: State machine, velocity control, PIDF tuning

2. **StarterBotAuto.java** 🔒
   - Type: OpMode + Autonomous
   - Purpose: Launch and drive autonomous
   - Features: State machine for sequential actions

---

## Best Practices

### For LinearOpMode

1. **Always use `opModeIsActive()` in loops**
   ```java
   while (opModeIsActive()) {  // ✅
       // Your code
   }
   ```

2. **Use helper functions for repeated actions**
   ```java
   driveForward(24);
   turnRight(90);
   driveForward(12);
   ```

3. **Add error handling for hardware**
   ```java
   try {
       motor = hardwareMap.get(DcMotor.class, "motor");
   } catch (IllegalArgumentException e) {
       telemetry.addLine("Motor not found!");
       return;
   }
   ```

4. **Use `idle()` in long wait loops**
   ```java
   while (opModeIsActive() && motor.isBusy()) {
       idle();  // Let system do housekeeping
   }
   ```

5. **Show initialization status**
   ```java
   telemetry.addData("Status", "Initialized");
   telemetry.addData("Motors", "Ready");
   telemetry.update();
   waitForStart();
   ```

### For OpMode

1. **Keep `loop()` execution time < 20ms**
   ```java
   @Override
   public void loop() {
       // Must complete in < 20ms
       processInputs();      // Fast
       updateStateMachines(); // Fast
       updateTelemetry();    // Rate-limited
   }
   ```

2. **Use enums for state machines**
   ```java
   private enum State {
       IDLE, MOVING, COMPLETE
   }
   private State currentState = State.IDLE;
   ```

3. **Initialize in `init()`, not `loop()`**
   ```java
   @Override
   public void init() {
       motor = hardwareMap.get(DcMotor.class, "motor");
       // Not in loop()!
   }
   ```

4. **Use `init_loop()` for status updates**
   ```java
   @Override
   public void init_loop() {
       telemetry.addData("Encoder", motor.getCurrentPosition());
   }
   ```

5. **Rate-limit telemetry**
   ```java
   if (telemetryTimer.seconds() > 0.1) {
       telemetry.update();
       telemetryTimer.reset();
   }
   ```

### For Both Types

1. **Use descriptive names**
   ```java
   @TeleOp(name="Blue Alliance Main Teleop", group="Competition")
   ```

2. **Add telemetry for debugging**
   ```java
   telemetry.addData("Motor Power", motor.getPower());
   telemetry.addData("Encoder", motor.getCurrentPosition());
   ```

3. **Set motor directions correctly**
   ```java
   leftMotor.setDirection(DcMotor.Direction.REVERSE);
   rightMotor.setDirection(DcMotor.Direction.FORWARD);
   ```

4. **Use BRAKE mode for precise control**
   ```java
   motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   ```

5. **Document your code**
   ```java
   // Drive forward 24 inches at 50% power
   driveDistance(24, 0.5);
   ```

---

## FAQ

### Q: Which OpMode type should I use?
**A:** Start with LinearOpMode for learning and simple autonomous. Use OpMode for complex teleop with multiple systems.

### Q: Can I mix both types in one program?
**A:** No, you must extend either `LinearOpMode` OR `OpMode`, not both. But you can have multiple OpMode files of different types in your project.

### Q: Can I use sleep() in OpMode.loop()?
**A:** No! It blocks everything. Use `ElapsedTime` and state machines instead.

### Q: Why is my LinearOpMode teleop laggy?
**A:** Probably using `sleep()` or long loops. Remove blocking code or switch to OpMode for better responsiveness.

### Q: How do I convert LinearOpMode to OpMode?
**A:** Replace `waitForStart()` and `while(opModeIsActive())` with state machines in `loop()`. See examples above.

### Q: What's the loop rate for each type?
**A:** LinearOpMode varies (depends on your code). OpMode is fixed at ~50 Hz (20ms per loop).

### Q: Can I use @TeleOp with @Autonomous?
**A:** No, use one or the other. An OpMode is either teleop OR autonomous, not both.

### Q: What does @Disabled do?
**A:** Hides the OpMode from the Driver Station menu. Remove it to make the OpMode visible.

### Q: Is LinearOpMode slower than OpMode?
**A:** Not inherently, but it CAN be if you use `sleep()` or inefficient loops. Well-written LinearOpMode can be just as fast.

### Q: Can I have multiple `loop()` methods?
**A:** No, only one `loop()` per OpMode. Use helper functions or state machines to organize code.

### Q: Why doesn't my OpMode appear in Driver Station?
**A:** Check that:
   1. You have `@TeleOp` or `@Autonomous` annotation
   2. You don't have `@Disabled`
   3. The app successfully compiled and installed
   4. The OpMode class is in the `teamcode` package

### Q: Can I stop an OpMode from code?
**A:** Yes, in LinearOpMode use `return` or set loop condition to false. In OpMode, there's no direct way, but you can call `requestOpModeStop()`.

---

## Resources

### Official Documentation
- **FTC Docs:** https://ftc-docs.firstinspires.org/
- **JavaDoc - OpMode:** https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/eventloop/opmode/OpMode.html
- **JavaDoc - LinearOpMode:** https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.html

### Community Resources
- **Game Manual 0:** https://gm0.org/ (Comprehensive community guide)
- **FTC Community Forum:** https://ftc-community.firstinspires.org/
- **Discord:** FTC Discord community

### Sample Code
- **SDK Samples:** `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`
- **TeamCode Samples:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

### Learning Path
1. Start with `BasicOpMode_Linear.java` (simplest example)
2. Try `RobotTeleopPOV_Linear.java` (add manipulators)
3. Learn autonomous with `RobotAutoDriveByTime_Linear.java`
4. Graduate to `StarterBotTeleop.java` (OpMode with state machines)

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2025-01-09 | 1.0 | Initial comprehensive guide |

---

**Document maintained by:** FTC Team Development
**Last reviewed:** 2025-01-09
**Next review:** Start of next season (2026)
