//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "DecodeTeleopV3.83 Alaqmar", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Chassis chassis;
    VoltageSensor voltageSensor;
    private volatile boolean threadIsRunning = true;
    double flyWheelVelocity = 0.0;
    long maxLoopTimeout = 2000;
    private Thread driveThread;
    DistanceSensor channelSensor;
    DistanceSensor frontDistanceSensor;


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

        channelSensor = hardwareMap.get(DistanceSensor.class, "channelSensor");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");

        chassis.odo.resetPosAndIMU();

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            AprilTagPoseFtc aprilTagPoseFtc = null;
            Double robotDistanceFromAprilTagUsingCamera = 0.0;
            Double robotDistanceUsingFrontDistanceSensor = 0.0;
            Double distanceSensor = 0.0;
            Integer flyWheelVelocityRequired =  FlyWheel.FLYWHEEL_SHOOTING_VELOCITY;

            if(aprilTag.findAprilTag(DecodeAprilTag.BLUE_APRIL_TAG)){
                aprilTagPoseFtc = aprilTag.getCoordinate(DecodeAprilTag.BLUE_APRIL_TAG);
                if(aprilTagPoseFtc !=null) {
                    robotDistanceFromAprilTagUsingCamera = aprilTagPoseFtc.range;
                }
            }

            flyWheelVelocityRequired = Util.getFlyWheelVelocityRequiredForDistance(robotDistanceFromAprilTagUsingCamera);
            robotDistanceUsingFrontDistanceSensor = frontDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance From Camera - ", String.valueOf(robotDistanceFromAprilTagUsingCamera));
            telemetry.addData("Distance From Front Sensor - ", String.valueOf(robotDistanceUsingFrontDistanceSensor));
            telemetry.addData("flyWheelVelocityRequired - ", flyWheelVelocityRequired);

            distanceSensor = Util.getDistance(channelSensor, telemetry);
            telemetry.addData("Distance From Channel Sensor - ",distanceSensor);
            telemetry.update();

            // Kicker
            if(gamepad2.dpad_up) {
                kicker.setPosition(Kicker.gateClose);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_down) {
                kicker.setPosition(Kicker.gateShoot);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_left) {
                kicker.setPosition(Kicker.gateIntake);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_right) {
                if(kicker.getGatePosition().equals(Kicker.GATE_CLOSE)){
                    kicker.setPosition(Kicker.gateShoot);
                    sleep(400);
                    kicker.setPosition(Kicker.gateClose);
                }
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }

            //Shooting
            if (gamepad2.right_bumper) {

                Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, channelSensor, flyWheelVelocityRequired, telemetry);

                intake.setIntakePower(0.4);

                while(!gamepad2.left_bumper) {
                    Util.startShooting(flyWheel, kicker, intake, channelSensor, flyWheelVelocityRequired, telemetry);
                    sleep(200);
                }

                intake.startIntake();
                Util.prepareFlyWheelToIntake(flyWheel,kicker,intake, channelSensor,telemetry);


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
                // = Util.getFlyWheelVelocityRequiredForDistance();
                Util.prepareFlyWheelToShoot(flyWheel,kicker, intake, channelSensor, FlyWheel.FLYWHEEL_SHOOTING_VELOCITY, telemetry);
            }

            if (gamepad2.b) {
                Util.prepareFlyWheelToIntake(flyWheel,kicker, intake, channelSensor, telemetry);
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
