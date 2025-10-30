//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "DecodeTeleopV3.47 Alaqmar", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Chassis chassis;
    VoltageSensor voltageSensor;
    private volatile boolean threadIsRunning = true;
    double flyWheelVelocity = 0.0;
    long maxLoopTimeout = 2000;
    private Thread driveThread;
    DistanceSensor channelSensor;


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
        //kicker.setPosition(Kicker.gateIntake);
        //Util.addKickerTelemetry(kicker,telemetry);



        DecodeAprilTag aprilTag  = new DecodeAprilTag(this);
        aprilTag.initCamera();

        channelSensor = hardwareMap.get(DistanceSensor.class, "channelSensor");



        chassis.odo.resetPosAndIMU();

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task

        //Util.telemetryFlyWheelVelocity(flyWheel, 0.65, 3000,telemetry);
        //telemetry.update();

        /*


        Util.readyToLoad(channelSensor,telemetry);
        telemetry.update();
        sleep(5000);
        aprilTag.getCoordinate("BlueTarget");

         */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            chassis.odo.update();
            //telemetry.addData("Odo x", chassis.odo.getEncoderX());
            //telemetry.addData("Odo y", chassis.odo.getEncoderY());



//            if (gamepad1.a) {
//                chassis.resetHeading();
//
//            }
//            // If gamepad x is pressed then switch to field centric
//             If gamepad y is pressed then switch to robot centric
//            if (gamepad1.x) {
//                chassis.resetIMU();
//                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
//            } else if (gamepad1.y) {
//                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
//            }



            /*
            //Running the following code in Thread
                float axial = -gamepad1.left_stick_y;
                float lateral = -gamepad1.left_stick_x;
                float yaw = -gamepad1.right_stick_x;
                chassis.moveRobot(axial, lateral, yaw);

             */


            // Kicker

            long flyWheelReadyTime = 1000;

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
            if(gamepad2.dpad_right) {
                kicker.setPosition(Kicker.gateIntake);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }

            //Shooting
            if (gamepad2.right_bumper) {

                //telemetry.addData("Double.compare",Double.compare(kicker.getPosition(), Kicker.gateIntake));


                switch (kicker.getGatePosition()) {
                    case Kicker.GATE_INTAKE:
                        telemetry.addData("Gate in intake position", " when right_bumper pressed.");
                        //This is not a great condition, as it would waste 2 sec
                        //Ideal condition is that gamepad2.a is pressed before trying to shoot
                        Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, channelSensor, telemetry);
                        break;
                    case Kicker.GATE_SHOOT:
                        telemetry.addData("Gate in shoot position", " when right_bumper pressed.");
                        //This is a condition where shoot is pressed before gate is open
                        break;
                    case Kicker.GATE_CLOSE:
                        telemetry.addData("Gate in close position", " when right_bumper pressed.");
                        //This is a condition where shoot is pressed when gate is closed
                        break;
                    default:
                        Util.addKickerTelemetry(kicker, telemetry);
                        break;
                }

                while(!gamepad2.left_bumper) {
                    Util.startShooting(flyWheel, kicker, intake, channelSensor, telemetry);
                    sleep(300);
                }

                telemetry.addData("gamepad2.left_bumper - ",gamepad2.left_bumper);


                flyWheel.stop();
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                long flyWheelStopDuration = Util.waitForFlyWheelStopVelocity(flyWheel,100,5000, telemetry);
                telemetry.addData("flyWheelStopDuration (ms) - ",flyWheelStopDuration);
                telemetry.update();

                /*

                long startTime = System.currentTimeMillis();
                long intermidiateTime =  System.currentTimeMillis();
                long durationInMillis = intermidiateTime - startTime;

                //intake.intake(0.6);
                kicker.moveToClosePosition();// Middle P
                sleep(1000);

                flyWheel.setPower(-0.65);
                sleep(800);

                Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
                //telemetry.addData("Flywheel warmup time (ms): ",  + durationInMillis );

                intermidiateTime =  System.currentTimeMillis();
                durationInMillis = intermidiateTime - startTime;
                double currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before First Shot: "+ currentVelocity," in time: "+durationInMillis);
                telemetry.update();

                //sleep(1000);
                // First Shot
                kicker.moveToShootingPosition();
                sleep(400);
                kicker.moveToClosePosition();

                 intermidiateTime =  System.currentTimeMillis();
                 durationInMillis = intermidiateTime - startTime;
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity After First Shot: "+ currentVelocity," in time: "+durationInMillis);

                // Turn intake on
                //sleep(flyWheelReadyTime);
                intake.intake(0.6);
                //sleep(200);

                Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
                intermidiateTime =  System.currentTimeMillis();
                durationInMillis = intermidiateTime - startTime;
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before Second Shot: "+ currentVelocity," in time: "+durationInMillis);



                //Second Shot
                kicker.moveToShootingPosition();
                sleep(500);
                kicker.moveToClosePosition();
                //sleep(flyWheelReadyTime);



                // Third Shot
                Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
                intermidiateTime =  System.currentTimeMillis();
                durationInMillis = intermidiateTime - startTime;
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before Third Shot: "+ currentVelocity," in time: "+durationInMillis);

                kicker.moveToShootingPosition();
                //sleep(1000);
                //intake.intake(0.0);
                //kicker.setKickerPos(gateIntake);

                telemetry.update();


                 */

            } else if (gamepad2.left_bumper) {

        } else if (gamepad2.a){
                Util.prepareFlyWheelToShoot(flyWheel,kicker, intake, channelSensor, telemetry);
            } else if (gamepad2.b) {
                Util.prepareFlyWheelToIntake(flyWheel,kicker, intake, channelSensor, telemetry);
            } else if (gamepad2.x){
                intake.startIntake();
            } else if (gamepad2.y) {
                intake.stopIntake();
            }
            else if (gamepad2.dpad_right){

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
