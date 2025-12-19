// The Keep Version 2.0
package org.firstinspires.ftc.teamcode.theKeep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Motors;
import org.firstinspires.ftc.teamcode.hardware.PedroPathing;
import org.firstinspires.ftc.teamcode.hardware.Vision;
@TeleOp(name="The Keep TeleOp", group="The Keep")
public class TheKeepTeleOp extends OpMode {

    // creates new variables to store our new instances of the hardware classes - Jason
    private Vision vision;
    private PedroPathing pathing;
    private Motors motors;

    @Override
    public void init() {

        // Creates a new instance of the hardware classes - Jason
        vision = new Vision();
        pathing = new PedroPathing();
        motors = new Motors();

        // Call the hardware init methods - Jason
        vision.initAprilTag(hardwareMap);
        pathing.initFollower(hardwareMap);
        motors.initMotors(hardwareMap);

        // Reports the status - Jason
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void start() {

        // Turns Pedro Path into TeleOp drive mode - Jason
        pathing.follower.startTeleopDrive();

    } // This runs once when the play button is pressed
    @Override
    public void loop() {

        // Updates the follower - Jason
        pathing.follower.update();

        // This tells the follower to activate manual drive mode if it is not following a path -Jason
        if (!pathing.automatedDrive) {
            pathing.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    TheKeepAuto.robotCentric // Robot Centric
            );
        }
        // Turns the bots heading to face the alliance goal - Jason
        if (gamepad1.crossWasPressed()) {
            pathing.follower.followPath(pathing.turnToGoal.get());
            pathing.automatedDrive = true;
        }

        // The tringle button will be changed in the future but for now is used as an emergency stop - Jason
        if (pathing.automatedDrive && (gamepad1.triangleWasPressed() || !pathing.follower.isBusy())) {
            pathing.follower.startTeleopDrive();
            pathing.automatedDrive = false;
        } // Switches to TeleOp drive if the follower is done - Jason



        /* These lines check to make sure the fidget tech is not at the maximum or minimum range
        and if its not move it to the next snap point right or left depending on the trigger you pressed
        - Jason */

        if (gamepad1.leftBumperWasPressed() && motors.spinPosition != 0) motors.spinPosition -= 1;
        if (gamepad1.rightBumperWasPressed() && motors.spinPosition != 26) motors.spinPosition += 1;
        if (motors.spinPosition == 0) {
            motors.spinPosition = 6;
        }
        if (motors.spinPosition == 26) {
            motors.spinPosition = 20;
        }

        // Constantly sets the fidgetTech's position to our current snap point - Jason
        motors.fidgetTech.setPosition(motors.spinPositions[motors.spinPosition]);

        // These lines set the shooter's power to 80% if the circle button is pressed and 0% if its not
        if (gamepad1.left_trigger > 0) {
            motors.shooter.setPower(.8);
            motors.intake.setPower(0);
        } else motors.shooter.setPower(0);

        /* These lines check to see if the fidget tech is in the way of the ball ejector if it's not, when
        you press the triangle it will swing knocking out the ball - Jason */
        if (gamepad1.right_trigger > 0 && motors.spinPosition % 2 == 1) {
            motors.ballEjector.setPosition(.3);
        } else motors.ballEjector.setPosition(0);

        // This if statement makes turns the intake on when the circle is pressed and off when the square is pressed - Jason
        if (gamepad1.circleWasPressed()) {
            motors.intake.setPower(1);
        }
        if (gamepad1.squareWasPressed()) {
            motors.intake.setPower(0);
        }


        // These lines grab the april tag data then write any tags data to the telemetry - Jason
        vision.getAprilTagData();
        if (vision.blueBase != null) telemetry.addData("Blue Base Range", vision.blueBase.ftcPose.range);
        if (vision.redBase != null) telemetry.addData("Red Base Range", vision.redBase.ftcPose.range);
        if (vision.blueBase != null) telemetry.addData("Blue Base Bearing", vision.blueBase.ftcPose.bearing);
        if (vision.redBase != null) telemetry.addData("Red Base Bearing", vision.redBase.ftcPose.bearing);
        if (Vision.pattern != null) {
            telemetry.addData("Pattern Is", Vision.pattern);
        } else {
            telemetry.addData("Pattern Is", "unknown");
        }

        // These lines add the fidget tech's position and the bot's position to the telemetry - Jason
        telemetry.addData("Fidget Tech Position", motors.spinPositions[motors.spinPosition]);
        telemetry.addData("Bot Position", pathing.follower.getPose());
        telemetry.update();

    } // This section holds all the controls used during TeleOp

    @Override
    public void stop() {
        // Sets the bots starting position to our end position - Jason
        pathing.setStartPose(false);
    }
}
