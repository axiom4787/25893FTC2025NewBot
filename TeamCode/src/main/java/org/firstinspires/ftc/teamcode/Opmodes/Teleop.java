//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//Imports LimeLight
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp(name = "DecodeTeleopV4.11 Alaqmar", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Chassis chassis;
    VoltageSensor voltageSensor;
    private volatile boolean threadIsRunning = true;
    double flyWheelVelocity = 0.0;
    long maxLoopTimeout = 2000;
    private Thread driveThread;
    DistanceSensor channelSensor;
    DistanceSensor frontDistanceSensor;

    WebcamName webcamName;
    private Limelight3A limelight;



    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis();
        chassis.init(this);

        //voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
//   chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Intake intake =new Intake();
        intake.init(this);

        Kicker kicker = new Kicker();
        kicker.init(hardwareMap);

        DecodeAprilTag aprilTag  = new DecodeAprilTag(this);
        aprilTag.initCamera();

        Flipper flipper = new Flipper();
        flipper.init(hardwareMap);
        /*
        telemetry.addData("Flipper Position 0 - ",flipper.getPosition());
        flipper.turnFlipper();
        sleep(500);
        telemetry.addData("Flipper Position 1 - ",flipper.getPosition());
        flipper.turnFlipper();
        sleep(500);
        telemetry.addData("Flipper Position 2 - ",flipper.getPosition());
        flipper.turnFlipper();
        sleep(500);
        telemetry.addData("Flipper Position 3 - ",flipper.getPosition());
        flipper.resetFlipper();
        sleep(500);
        telemetry.addData("Flipper Position reset - ",flipper.getPosition());
        telemetry.update();
        sleep(10000);

         */





        channelSensor = hardwareMap.get(DistanceSensor.class, "channelSensor");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");

        chassis.odo.resetPosAndIMU();

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task

        //telemetry.addData("Status", "Initialized");

        //Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, channelSensor, 1100, telemetry);


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            kicker.setGatePosition(Kicker.GATE_SHOOT);
            flyWheel.setPower(0.45);
            telemetry.addData("FlyWheel Power - ", flyWheel.getPower());
            telemetry.addData("FlyWheel Velocity - ", flyWheel.getVelocity());
            telemetry.addData("Distance - ", "Distance - " + Util.getDistance(frontDistanceSensor, telemetry));
            //Util.telemetryFlyWheelVelocity(flyWheel,0.45, frontDistanceSensor, 30000,telemetry);
            telemetry.update();
            */

            Double flyWheelPowerRequired = 0.45;
            Integer flyWheelVelocityRequired =  FlyWheel.FLYWHEEL_SHOOTING_VELOCITY;
            Double robotDistanceFromAprilTagUsingCamera = 0.0;
            Double robotDistanceUsingFrontDistanceSensor = 0.0;
            Double distanceSensor = 0.0;

            AprilTagPoseFtc aprilTagPoseFtc = null;

            if(aprilTag.findAprilTag(DecodeAprilTag.BLUE_APRIL_TAG)){
                aprilTagPoseFtc = aprilTag.getCoordinate(DecodeAprilTag.BLUE_APRIL_TAG);
                if(aprilTagPoseFtc !=null) {
                    robotDistanceFromAprilTagUsingCamera = aprilTagPoseFtc.range;
                }
            }
            robotDistanceUsingFrontDistanceSensor = frontDistanceSensor.getDistance(DistanceUnit.INCH);
            Double robotDistanceFromAprilTag = 0.0;

            if(robotDistanceFromAprilTagUsingCamera != null && robotDistanceFromAprilTagUsingCamera < 180){
                robotDistanceFromAprilTag = robotDistanceFromAprilTagUsingCamera;
            } else if(robotDistanceUsingFrontDistanceSensor != null && robotDistanceUsingFrontDistanceSensor < 80){
                robotDistanceFromAprilTag = robotDistanceUsingFrontDistanceSensor;
            } else {
                robotDistanceFromAprilTag = 45.0;
            }

            flyWheelPowerRequired = Util.getRequiredFlyWheelPower(robotDistanceFromAprilTag);
            flyWheelVelocityRequired = Util.getRequiredFlyWheelVelocity(robotDistanceFromAprilTag);
            distanceSensor = Util.getDistance(channelSensor, telemetry);

            telemetry.addData("Robot dist 4m AprilTag - ", String.valueOf(robotDistanceFromAprilTag));
            telemetry.addData("Robot dist 4m Camera - ", String.valueOf(robotDistanceFromAprilTagUsingCamera));
            telemetry.addData("Robot dist 4m Front Sensor - ", String.valueOf(robotDistanceUsingFrontDistanceSensor));
            telemetry.addData("flyWheelVelocityRequired - ", flyWheelVelocityRequired);
            telemetry.addData("flyWheelPowerRequired - ", flyWheelPowerRequired);
            telemetry.addData("Distance From Channel Sensor - ",distanceSensor);
            telemetry.update();

            // Kicker
            if(gamepad2.dpad_up) {
                kicker.setPosition(Kicker.gateClose);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.resetFlipper();
            }
            if(gamepad2.dpad_down) {
                kicker.setPosition(Kicker.gateShoot);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.turnFlipper();
            }
            if(gamepad2.dpad_left) {
                kicker.setPosition(Kicker.gateIntake);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_right) {
                if (kicker.getGatePosition().equals(Kicker.GATE_CLOSE)) {
                    kicker.setPosition(Kicker.gateShoot);
                    sleep(400);
                    kicker.setPosition(Kicker.gateClose);
                }
                Util.addKickerTelemetry(kicker, telemetry);
                telemetry.update();
            }
            //Shooting
            if (gamepad2.right_bumper) {

                Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);

                intake.setIntakePower(0.5);

                int loopCounter = 0;

                while(!gamepad2.left_bumper && loopCounter<4) {

                    // wait flywheel to get the desired speed
                    Util.startShooting(flyWheel, kicker, flipper, intake, channelSensor, robotDistanceFromAprilTag, telemetry);

                    kicker.setPosition(Kicker.gateShoot);
                    sleep(200);

                    flipper.turnFlipper();
                    sleep(200);
                    kicker.setGatePosition(Kicker.GATE_CLOSE);
                    flipper.resetFlipper();
                    sleep(200);


                    loopCounter = loopCounter+1;

                  //  sleep(200);

                }
                telemetry.addData("gamepad2.right_bumper - loopCounter - ",loopCounter);
                telemetry.update();

                //flipper.resetFlipper();


                Util.prepareFlyWheelToIntake(flyWheel,kicker,intake,flipper, telemetry);


                /*
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                long flyWheelStopDuration = Util.waitForFlyWheelStopVelocity(flyWheel,100,5000, telemetry);
                //telemetry.addData("flyWheelStopDuration (ms) - ",flyWheelStopDuration);
                kicker.setGatePosition(Kicker.GATE_INTAKE);
                telemetry.update();
                //sleep(5000);
                 */
            }

            if (gamepad2.a){
                Util.prepareFlyWheelToShoot(flyWheel,kicker, intake, robotDistanceFromAprilTag, telemetry);
            }

            if (gamepad2.b) {
                Util.prepareFlyWheelToIntake(flyWheel,kicker,intake,flipper, telemetry);
            }

            if (gamepad2.x){
                intake.startIntake();
            }

            if (gamepad2.y) {
                intake.stopIntake();
            }

        }

        // Clean up the thread
        threadIsRunning = false;
        sleep(2000);
        driveThread.interrupt();
        try {
            driveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        }

    private class DriveTask implements Runnable {

        @Override
        public void run() {
            while ( threadIsRunning && !Thread.currentThread().isInterrupted()) {

                // Read gamepad input and set drive motor power
                float axial = -gamepad1.left_stick_y;
                float lateral = -gamepad1.left_stick_x;
                float yaw = -gamepad1.right_stick_x;
                chassis.moveRobot(axial, lateral, yaw);

                sleep(10);
            }
        }
    }
}
