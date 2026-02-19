package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, -45, Math.toRadians(234)))
                        .lineToY(-30)
                        .stopAndAdd(new ShootAction())
                        .waitSeconds(0.5)

                        .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12, -45), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
                        .afterDisp(0.0, new IntakeAction())
                        .stopAndAdd(new ShootAction())
                        .waitSeconds(0.5)

                        .strafeToLinearHeading(new Vector2d(12, -25), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
                        .afterDisp(0.0, new IntakeAction())
                        .stopAndAdd(new ShootAction())
                        .waitSeconds(0.5)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static class IntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            try {
                wait(2000);
            } catch (InterruptedException ignore) {}
//            intakeMotor.setPower(0.4);
//            shooterMotor.setPower(-0.4);
//            sleep(2000);
//            intakeMotor.setPower(0.0);
//            shooterMotor.setPower(0.0);
            return false;
        }
    }

    static class ShootAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            try {
                wait(6500);
            } catch (InterruptedException ignore) {}
//            shooterMotor.setPower(1.0);
//            sleep(1500);
//            intakeMotor.setPower(0.3);
//            sleep(5000);
//            shooterMotor.setPower(0.0);
//            intakeMotor.setPower(0.0);
            return false;
        }
    }
}