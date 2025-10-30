package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;
import org.firstinspires.ftc.teamcode.Helper.Util;

@Autonomous(name = "Auto Basket", group = "Autonomous")

public class AutoBasketBlue extends LinearOpMode {

    Chassis chassis = new Chassis();

    FlyWheel flyWheel = new FlyWheel();

    Kicker kicker = new Kicker();

    Intake intake = new Intake();

    double gateClose = 0.4;
    double gateShooting = 0.25;
    double gateIntake = 0.6;

    enum AutoStages {
        BACK_UP,
        ADJUST_POSITION,
        SHOOT,
        GET_MORE_BALLS,
    }

    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(this);
        flyWheel.init(this);
        kicker.init(hardwareMap);
        intake.init(this);

        DecodeAprilTag aprilTag = new DecodeAprilTag(this);
        aprilTag.initCamera();

        while (opModeInInit()) {
            chassis.odo.resetPosAndIMU();
            Util.printOdoPositionTelemetry(chassis.odo, telemetry);
        }

        waitForStart();


        AutoStages currentStage = AutoStages.BACK_UP;

        while (opModeIsActive()) {

            switch (currentStage) {
                case BACK_UP:
                    //chassis.moveWithProportionalDeceleration(Chassis.Direction.FORWARD, 0.3, 60);
                    currentStage = AutoStages.ADJUST_POSITION;
                    break;

                case ADJUST_POSITION:
                    currentStage = AutoStages.SHOOT;
                    break;

                case SHOOT:
                    shoot();
                    flyWheel.stop();
                    intake.stopIntake();

                //case GET_MORE_BALLS:
                    

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

        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
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
        intake.intake(0.6);
        //sleep(200);

        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
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
        Util.waitForFlyWheelShootingVelocity(flyWheel,1500,2000);
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

