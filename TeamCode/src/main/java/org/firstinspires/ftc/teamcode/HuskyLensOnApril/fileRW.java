// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.HuskyLensOnApril;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "FileDataRW", group = "FTC")
public class fileRW extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Define the file name and content
    String filename = "DecodeRedOrBlueMatch.txt";
    String dataToSave = "This is the data from Autonomous.";

    // Get the file reference using AppUtil (stores in FIRST/settings directory)
    File file = AppUtil.getInstance().getSettingsFile(filename);

    // --- Writing the file ---
    telemetry.addData("Status", "Writing to file: " + filename);
    // Use the static method from ReadWriteFile to write the string data
    ReadWriteFile.writeFile(file, dataToSave);
    telemetry.addData("Status", "Successfully wrote data.");

    // --- Reading the file ---
    telemetry.addData("Status", "Reading from file: " + filename);
    // Use the static method from ReadWriteFile to read the string data
    String readData = ReadWriteFile.readFile(file);
    telemetry.addData("Read Data", readData);

    telemetry.update();

    waitForStart();
    // The opmode will remain active until STOP is pressed
    while (opModeIsActive()) {
      // Keep telemetry updated or run robot actions
      telemetry.addData("Status", "Running OpMode...");
      telemetry.addData("Read Data (Persisted)", readData);
      telemetry.update();
    }
  }
}
