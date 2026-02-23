package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: Fix Math being weird or something.
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, java.lang.Math.toRadians(180), java.lang.Math.toRadians(180), 15)
                .setDimensions(16, 16)
                .build();

        VelConstraint velConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                return 10;
            }
        };

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, i(-47), java.lang.Math.toRadians(i(234))))
                .stopAndAdd(new StartShooterAction())
                .lineToY(i(-30))
                .waitSeconds(1.0)
                .stopAndAdd(new ShootAction())

                .strafeToLinearHeading(new Vector2d(-12, i(-25)), java.lang.Math.toRadians(i(270)))
                .stopAndAdd(new StartIntakeAction())
                .strafeToLinearHeading(new Vector2d(-12, i(-45)), java.lang.Math.toRadians(i(270)), velConstraint)
                .strafeToLinearHeading(new Vector2d(-43, i(-30)), java.lang.Math.toRadians(i(234)))
                .afterDisp(1.0, new StopIntakeAction())
                .stopAndAdd(new ShootAction())

                .strafeToLinearHeading(new Vector2d(12, i(-25)), java.lang.Math.toRadians(i(270)))
                .stopAndAdd(new StartIntakeAction())
                .strafeToLinearHeading(new Vector2d(12, i(-45)), java.lang.Math.toRadians(i(270)))
                .strafeToLinearHeading(new Vector2d(-43, i(-30)), java.lang.Math.toRadians(i(234)))
                .afterDisp(1.0, new StopIntakeAction())
                .stopAndAdd(new ShootAction())
                .strafeToLinearHeading(new Vector2d(-15, i(-40)), java.lang.Math.toRadians(i(270)))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static double i(double n) {
        return -n;
    }

    static class StartIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
//            intakeMotor.setPower(0.7);
//            indexerMotor.setPower(-0.2);

            return false;
        }
    }

    static class StopIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
//            intakeMotor.setPower(0.0);
//            indexerMotor.setPower(0.0);

            return false;
        }
    }

    static class ShootAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            try {
                wait(6000);
            } catch (Exception ignore) {}

            return false;
        }
    }

    static class StartShooterAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
//            smartShooter.setVelocity(1450);

            return false;
        }
    }
}