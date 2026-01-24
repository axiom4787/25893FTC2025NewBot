/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

@TeleOp(name="Test what motors do what", group="Linear OpMode")
public class TestTheMotors extends ThePlantRobotOpMode {
    @Override public void opModeInit() {}
    @Override public void opModeRunOnce() {}

    @Override public void opModeRunLoop() {
        if (gamepad1.dpad_up) {
            frontLeftDrive.setPower(0.5);
            telemetry.addLine("Running `frontLeftDrive`");
        } else frontLeftDrive.setPower(0);

        if (gamepad1.dpad_down) {
            frontRightDrive.setPower(0.5);
            telemetry.addLine("Running `frontRightDrive`");
        } else frontRightDrive.setPower(0);

        if (gamepad1.dpad_left) {
            backLeftDrive.setPower(0.5);
            telemetry.addLine("Running `backLeftDrive`");
        } else backLeftDrive.setPower(0);

        if (gamepad1.dpad_right) {
            backRightDrive.setPower(0.5);
            telemetry.addLine("Running `backRightDrive`");
        } else backRightDrive.setPower(0);

        if (gamepad1.y) {
            intake.setPower(1.0);
            telemetry.addLine("Running `intake`");
        } else intake.setPower(0);

        if (gamepad1.x) {
            shooter.setPower(1.0);
            telemetry.addLine("Running `shooter`");
        } else shooter.setPower(0);
    }
}
