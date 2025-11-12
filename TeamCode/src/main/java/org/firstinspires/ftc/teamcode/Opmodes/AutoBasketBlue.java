package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Helper.Flipper;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;
import org.firstinspires.ftc.teamcode.Helper.Util;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Autonomous(name = "Auto Blue Near 4.55", group = "Autonomous")

public class AutoBasketBlue extends LinearOpMode {

    enum RobotType {
        VORTEX_DECODE_1,
        VORTEX_DECODE_2
    }

    RobotType robotType = RobotType.VORTEX_DECODE_1;

    Chassis chassis = new Chassis();

    FlyWheel flyWheel = new FlyWheel();

    Kicker kicker = new Kicker();

    Intake intake = new Intake();

    Flipper flipper;

    double gateClose = 0.4;
    double gateShooting = 0.25;
    double gateIntake = 0.6;

    enum AutoStages {
        BACK_UP,
        SHOOT,
        GET_MORE_BALLS,
        GO_BACK_TO_SHOOT,
        MOVE_OUT_OF_SHOOTING_ZONE,
        END
    }

    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(this);
        flyWheel.init(this);
        kicker.init(hardwareMap);
        intake.init(this);

        Util.resetToDefaultSpeed();

        chassis.resetODOPosAndIMU();

        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

        if(robotType == RobotType.VORTEX_DECODE_1) {
            aprilTag = new DecodeAprilTag(this);
            aprilTag.initCamera();

            flipper = new Flipper();
            flipper.init(hardwareMap);
        }



        //chassis.odo.resetPosAndIMU();



//        while (opModeInInit()) {
//
//            Util.printOdoPositionTelemetry(chassis.odo, telemetry);
//            Util.printIMUTelemetry(chassis.imu,telemetry);
//            telemetry.update();
//
//        }
        /*
        Util.printOdoPositionTelemetry(chassis.odo, telemetry);
        Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.STRAFE_RIGHT, 12, 0, telemetry);
        Util.printOdoPositionTelemetry(chassis.odo, telemetry);
        telemetry.update();
        sleep(10000);
        Util.printOdoPositionTelemetry(chassis.odo, telemetry);
        Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.STRAFE_LEFT, 12, 0, telemetry);
        Util.printOdoPositionTelemetry(chassis.odo, telemetry);
        telemetry.update();
        sleep(10000);

         */




        waitForStart();

     /*   //chassis.drive(1,0,0);
        if(robotType == RobotType.VORTEX_DECODE_1) {
//            Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.FORWARD, 20, 0, telemetry);
//            sleep(2000);
//            Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.BACKWARD, 20, 0, telemetry);
//            sleep(2000);
//            Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.STRAFE_RIGHT, 20, 0, telemetry);
//            sleep(2000);
//            Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.STRAFE_LEFT, 20, 0, telemetry);
//            sleep(2000);

            //Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.TURN_LEFT, 0, 90, telemetry);
            //sleep(2000);
            //Util.moveRobot(chassis.frontLeftDrive, chassis.backLeftDrive, chassis.frontRightDrive, chassis.backRightDrive, chassis.odo, chassis.imu, Util.MovementDirection.TURN_RIGHT, 0, -90, telemetry);
            //sleep(2000);

        }*/


        AutoStages currentStage = AutoStages.BACK_UP;

        while (opModeIsActive()) {

            Util.AlignmentResult alignmentResult;
            Double robotDistanceFromAprilTag = 0.0;
            AprilTagPoseFtc aprilTagPoseFtc = null;

            if(robotType == RobotType.VORTEX_DECODE_1) {
                if (aprilTag.findAprilTag(DecodeAprilTag.BLUE_APRIL_TAG)) {
                    aprilTagPoseFtc = aprilTag.getCoordinate(DecodeAprilTag.BLUE_APRIL_TAG);
                    if (aprilTagPoseFtc != null) {
                        robotDistanceFromAprilTag = aprilTagPoseFtc.range;
                        telemetry.addData("April Tag Distance", robotDistanceFromAprilTag);
                        telemetry.update();
                    }
                }
            }



            switch (currentStage) {
                case BACK_UP:
                    Util.setSpeed(0.2, 0.8);
                    intake.setIntakePower(0.5);
                    chassis.drive(30);
                    //telemetry.addData("April Tag", aprilTagPoseFtc);
                    //telemetry.update();
                    sleep(200);
                    currentStage = AutoStages.SHOOT;
                    break;

                case SHOOT:
                    alignmentResult = Util.autoAlignWithAprilTag(this, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, chassis, telemetry);
                    Util.shoot(flyWheel, kicker, flipper, intake, alignmentResult.distance, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, telemetry);
                    currentStage = AutoStages.GET_MORE_BALLS;
                    break;

                case GET_MORE_BALLS:
                    chassis.turn(120);
                    sleep(100);
                    chassis.strafe(-12);
                    sleep(100);
                    intake.startIntake();
                    Util.setSpeed(0.2,0.2);
                    chassis.drive(30);
                    Util.setSpeed(0.3, 0.6);
                    chassis.drive(-20);
                    sleep(100);
                    currentStage = AutoStages.GO_BACK_TO_SHOOT;
                    break;

                case GO_BACK_TO_SHOOT:
                    chassis.strafe(12);
                    Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);
                    sleep(100);
                    chassis.turn(-125);
                    sleep(100);
                    alignmentResult = Util.autoAlignWithAprilTag(this, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, chassis, telemetry);
                    Util.shoot(flyWheel, kicker, flipper, intake, alignmentResult.distance, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, telemetry);
                    currentStage = AutoStages.MOVE_OUT_OF_SHOOTING_ZONE;
                    break;

                case MOVE_OUT_OF_SHOOTING_ZONE:
                    Util.setSpeed(0.3, 0.8);
                    chassis.strafe(36);
                    currentStage = AutoStages.END;
                    break;
                case END:
                    break;

            }


            /*
              double destRange = 10;
            double destYaw = 0;
            double destBearing = 0;
            double currentRange = 0;
            double currentYaw = 0;
            double currentBearing = 0;

            while ( Math.abs((destRange - currentRange)) >  0) {
                AprilTagPoseFtc cameraPosition = aprilTag.getCoordinate("BlueTarget");
                if (cameraPosition != null) {
                    currentRange = cameraPosition.range; //distance in inches
                    currentYaw = cameraPosition.yaw; //use straffing to align to 0
                    currentBearing = cameraPosition.bearing; //use turning to align to 0

                    chassis.drive(-0.5,0,0);
                }
            }
            chassis.setPowerToWheels(0,0,0,0);

             */


            // for(int i = 15; i <= 180; i += 15){
            //   chassis.turnToAngle(i);
            //  sleep(1500);
            //}
            //for(int i = -15; i >= -180; i -= 15) {
            //   chassis.turnToAngle(i);
            // sleep(1500);

            //chassis.moveDistance(0,20,0);
            //telemetry.addData("x:0,y:20,yaw:0","");
            //telemetry.update();
            //sleep(2000);
            //chassis.moveDistance(0,20,0);
            //telemetry.addData("x:0,y:-20,yaw:0","");
            //telemetry.update();
            //sleep(2000);
            //chassis.imuTurn(90);
            //chassis.moveDistance(10,10,0);
            //chassis.imuTurn(-90);
            //chassis.moveDistance(10,0,0);
            //chassis.imuTurn(90);
            //chassis.moveDistance(-10,0,0);


            //chassis.printOdoTelemetry();
            //chassis.printIMUTelemetry();
            //  telemetry.update();


//                aprilTag.findAprilTag("BlueAllianceLeft");
//                AprilTagPoseFtc aprilTagPoseFtc = aprilTag.getCoordinate("BlueAllianceLeft");
//                sleep(1000);

            //for (int i = 0; i < 5; i++){
            //chassis.imuTurnRight(90);
            //chassis.turnToHeadingWithImuDegrees(90, 0.5, 30000);
//                if (i % 2 == 0) {
//                    chassis.turnToHeadingWithImuDegrees(90, 0.5, 30000);
//                }else{
//                    chassis.turnToHeadingWithImuDegrees(-90, 0.5, 30000);
//                }
        }
    }

    public void shoot() {
        long startTime = System.currentTimeMillis();
        long intermediateTime =  System.currentTimeMillis();
        long durationInMillis = intermediateTime - startTime;

        //intake.intake(0.6);
        kicker.setPosition(Kicker.gateClose);
        sleep(1000);

        flyWheel.setPower(-0.65);
        sleep(800);

        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000,telemetry);
        //telemetry.addData("Flywheel warmup time (ms): ",  + durationInMillis );

        intermediateTime =  System.currentTimeMillis();
        durationInMillis = intermediateTime - startTime;
        double currentVelocity = flyWheel.getVelocity();
        telemetry.addData("Velocity Before First Shot: "+ currentVelocity," in time: "+durationInMillis);
        telemetry.update();

        //sleep(1000);

        // First Shot
        kicker.setPosition(Kicker.gateShoot);
        sleep(400);
        kicker.setPosition(Kicker.gateClose);

        intermediateTime =  System.currentTimeMillis();
        durationInMillis = intermediateTime - startTime;
        currentVelocity = flyWheel.getVelocity();
        telemetry.addData("Velocity After First Shot: "+ currentVelocity," in time: "+durationInMillis);

        // Turn intake on
        //sleep(flyWheelReadyTime);
        intake.setIntakePower(0.6);
        //sleep(200);

        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000, telemetry);
        intermediateTime =  System.currentTimeMillis();
        durationInMillis = intermediateTime - startTime;
        currentVelocity = flyWheel.getVelocity();
        telemetry.addData("Velocity Before Second Shot: "+ currentVelocity," in time: "+durationInMillis);



        //Second Shot
        kicker.setPosition(Kicker.gateShoot);
        sleep(500);
        kicker.setPosition(Kicker.gateClose);
        //sleep(flyWheelReadyTime);



        // Third Shot
        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000, telemetry);
        intermediateTime =  System.currentTimeMillis();
        durationInMillis = intermediateTime - startTime;
        currentVelocity = flyWheel.getVelocity();
        telemetry.addData("Velocity Before Third Shot: "+ currentVelocity," in time: "+durationInMillis);

        kicker.setPosition(Kicker.gateShoot);

        //sleep(1000);
        //intake.intake(0.0);
        //kicker.setKickerPos(gateIntake);

        telemetry.update();
    }
}

