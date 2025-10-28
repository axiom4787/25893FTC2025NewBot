import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "AutoSimple1")
public class AutoSimple1 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        int counter;
        int counter_loop;
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor launchwheel = hardwareMap.get(DcMotor.class, "launcher_wheel");
        CRServo rightservo = hardwareMap.get(CRServo.class, "right_launch_servo");
        CRServo leftservo = hardwareMap.get(CRServo.class, "left_launch_servo");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // You don't HAVE to do this, but it makes things clear
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        frontLeft.setPower(-0.7);
        frontRight.setPower(-0.7);
        sleep(1500);
        frontLeft.setPower(-0.7);
        frontRight.setPower(0.7);
        sleep(300);
        frontLeft.setPower(-0.7);
        frontRight.setPower(-0.7);
        sleep(1000);
        counter_loop = 0;
        while (counter_loop < 1000) {
            launch_loop();
            sleep(10);
            counter_loop += 1;
        }
    }
    private void launch_loop() {
        int counter = 0;
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor launchwheel = hardwareMap.get(DcMotor.class, "launcher_wheel");
        CRServo rightservo = hardwareMap.get(CRServo.class, "right_launch_servo");
        CRServo leftservo = hardwareMap.get(CRServo.class, "left_launch_servo");
        counter += 1;
        launchwheel.setPower(0.6);
        if (counter == 200) {
            rightservo.setPower(-1);
            leftservo.setPower(1);
        }
        if (counter == 280) {
            rightservo.setPower(0);
            leftservo.setPower(0);
        }
        if (counter == 330) {
            rightservo.setPower(-1);
            leftservo.setPower(1);
            counter = 201;
        }
    }
}
