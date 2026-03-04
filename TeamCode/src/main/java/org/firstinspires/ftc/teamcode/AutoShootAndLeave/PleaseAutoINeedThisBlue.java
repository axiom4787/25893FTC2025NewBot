package org.firstinspires.ftc.teamcode.AutoShootAndLeave;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto BLUE: shoot & move")
@Disabled
public class PleaseAutoINeedThisBlue extends BasePleaseAutoINeedThis {
    PleaseAutoINeedThisBlue() {
        alliance = Alliance.BLUE;
    }
}