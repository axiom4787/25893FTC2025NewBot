import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actuator {
    private final int DOWN = 0; // flush with the floor of platform
    private final int UP = 180; // raised to push it into the flywheel
    private final double DOWN = 0.0; // flush with the floor of platform
    private final double UP = .27; // raised to push it into the flywheel

    private boolean activated;

    @ TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Actuator.java:16 @ public class Actuator {
    }

    public void down() {
        servo.turnToAngle(DOWN);
        servo.setPosition(DOWN);
        activated = false;
    }

    public void up() {
        servo.turnToAngle(UP);
        servo.setPosition(UP);
        activated = true;
    }