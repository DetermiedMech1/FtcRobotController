package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class AutoOpMode extends LinearOpMode {

    private DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry, opModeIsActive());
        }

        while (opModeIsActive()) {

        }
    }
}
