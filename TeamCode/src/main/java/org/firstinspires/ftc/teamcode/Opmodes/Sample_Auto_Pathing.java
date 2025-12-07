package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Pedro Blue Near Auto 0.01", group = "Autonomous")
public class Sample_Auto_Pathing extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;


    public enum PathState{
        //START POSITION_ END POSITION
        //DRIVE-MOVEMENT STATE
        //SHOOT-ATTEPTING TO SCORE


        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }

    PathState pathState;
    private final Pose startPose = new Pose(22,123, Math.toRadians(138));
    private final Pose shootPose = new Pose(48,94, Math.toRadians(138));

    private PathChain driveStartPosShootPos;


    public void buildPaths(){
        // PUT IN CORDINATES FOR START AND END POSITIONS
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);// reset the timer and makes new state.
                break;
            case SHOOT_PRELOAD:
            //check if follower is done with previous path
                if(!follower.isBusy()){
                    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
                }
                break;
            default:
                break;
        }
    }


    public  void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init (){
    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    pathTimer = new Timer();
    opModeTimer = new Timer();
    follower = Constants.createFollower(hardwareMap);
    //TODO add in any other things we need to initialize flywheel, intake, etc.
        buildPaths();
        follower.setStartingPose(startPose);
        
    }


    public void start(){

        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){

        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }


}
