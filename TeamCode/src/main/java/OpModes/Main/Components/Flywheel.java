package OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import ProgrammingBoard.ProgrammingBoardOTHER;

public class Flywheel {
    private ProgrammingBoardOTHER board;
    private Telemetry telemetry;
    
    private double flywheelPower = 0.8; // starting power
    private boolean spinning = false;   // flywheel state

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        board = new ProgrammingBoardOTHER();
        board.initializeComponents(hardwareMap);
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void update() {
        board.flyWheelMotor.setPower(spinning ? flywheelPower : 0);
        board.flyWheelMotor2.setPower(spinning ? flywheelPower : 0);
    }

    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    public void setPower(double power) {
        this.flywheelPower = power;
    }

    public double getPower() {
        return flywheelPower;
    }
}

