package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Autonomous
public class AutonLinear extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int target = 400;
    private int pathState;
    private static double vel = 0;
    private final Pose Start = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(70, 80, Math.toRadians(135));
    private final Pose Collect1 = new Pose(17, 80, Math.toRadians(180));
    private final Pose Grab2 = new Pose(40, 55, Math.toRadians(0));
    private final Pose Collect2 = new Pose(17, 55, Math.toRadians(180));
    private final Pose Grab3 = new Pose(40, 29.5, Math.toRadians(0));
    private final Pose Collect3 = new Pose(17, 29.5, Math.toRadians(180));
    private Path PreloadShoot;
    private PathChain Pickup1, Shoot1, Pickup2, Shoot2, Pickup3, Shoot3, scorePickup3;
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        PreloadShoot = new Path(new BezierLine(Start, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), ScorePosition.getHeading());

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Collect1))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Collect2))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect2.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .build();

        waitForStart();

        robot.intake.setPower(1);

        robot.controller.setPID(robot.p, robot.i, robot.d);
        double presentVoltage = robot.volt.getVoltage();
        vel = vel * robot.alpha + robot.shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - robot.alpha);
        double pid = robot.controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        robot.shooterb.setPower(pid);
        robot.shootert.setPower(-1 * pid);
        followPath(PreloadShoot, false);
        robot.latch.setPosition(1);
        sleep(1000);
        robot.latch.setPosition(0);
        followPath(Pickup1,true);
        sleep(1000);
        robot.intake.setPower(0);

        followPath(Shoot1,true);
        robot.intake.setPower(1);
        robot.latch.setPosition(1);
        sleep(1000);
        robot.latch.setPosition(0);

        followPath(Pickup2, false);
        sleep(1000);
        robot.intake.setPower(0);

        followPath(Shoot2, false);
        robot.intake.setPower(1);
        robot.latch.setPosition(1);
        sleep(1000);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
        }
    }
    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        while (follower.isBusy()) follower.update();
    }
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        while (follower.isBusy()) follower.update();
    }
}