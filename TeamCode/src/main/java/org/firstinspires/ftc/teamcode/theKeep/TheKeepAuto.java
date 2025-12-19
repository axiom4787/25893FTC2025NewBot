// The Keep Auto Version 2.0
package org.firstinspires.ftc.teamcode.theKeep;

// Pedro Path imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import org.firstinspires.ftc.teamcode.hardware.Motors;
import org.firstinspires.ftc.teamcode.hardware.PedroPathing;
import org.firstinspires.ftc.teamcode.hardware.Vision;

@Autonomous(name="The Keep Auto",group="The Keep")
public class TheKeepAuto extends OpMode {

    private Motors motors;
    private Vision vision;
    private PedroPathing pathing;

    // These next lines setup the variables used to store prompter values, which define the autonomous is used - Jason
    public enum Alliance {
        RED,
        BLUE
    }

    private Prompter prompter = new Prompter(this);
    public static Alliance alliance;
    public static int startLocation;
    public static boolean robotCentric;


    @Override
    public void init() {
        vision = new Vision();
        pathing = new PedroPathing();
        motors = new Motors();
        // Call their init methods
        vision.initAprilTag(hardwareMap);
        motors.initMotors(hardwareMap);

        //Sets up the prompter - Jason
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", Alliance.RED, Alliance.BLUE))
                .prompt("startLocation", new OptionPrompt<>("Select Start Location", 1, 2))
                .prompt("robotCentric", new BooleanPrompt("Robot Centric",true))
                .onComplete(this::onPromptsComplete);

    } // This initializes all the motors and sensors

    // Function that runs when the prompter is done that stores the values and prints the results - Jason
    public void onPromptsComplete() {
        alliance = prompter.get("alliance");
        startLocation = prompter.get("startLocation");
        robotCentric = prompter.get("robotCentric");
        pathing.setStartPose(true);
        pathing.initFollower(hardwareMap);

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Start Location", startLocation);
        telemetry.addData("Selected Start Position", pathing.follower.getPose());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Runs the prompter - Jason
        prompter.run();
    } // A loop that runs from when the init button is pressed to when the start button is hit

    @Override
    public void loop() {

        // Updates the follower - Jason
        pathing.follower.update();

        // These lines grab the april tag data then write any tags data to the telemetry - Jason
        vision.getAprilTagData();
        if (Vision.pattern != null) {
            telemetry.addData("Pattern Is", Vision.pattern);
        } else {
            telemetry.addData("Pattern Is", "unknown");
        }

        // These lines add the fidget tech's position and the bots position to the telemetry - Jason
        telemetry.addData("Fidget Tech Position", motors.spinPositions[motors.spinPosition]);
        telemetry.addData("Bot Position", pathing.follower.getPose());
        telemetry.update();

    }
    @Override
    public void stop() {
        pathing.setStartPose(false);
    }


}
