package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends LinearOpMode{

    private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            driveSubsystem.splitDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
    }


}