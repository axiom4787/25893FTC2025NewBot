package org.firstinspires.ftc.teamcode.Auto9Ball;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BLUE: 9 ball auto", group = "9ball", preselectTeleOp = "comp ready code")
public class Auto9BallBlue extends RoadRunnerAuto {
    Auto9BallBlue() {
        alliance = Alliance.BLUE;
    }
}
