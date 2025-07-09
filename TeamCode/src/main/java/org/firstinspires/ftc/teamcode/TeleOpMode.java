package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@TeleOp(name = "TeleOp")
@SuppressWarnings("unused")
public class TeleOpMode extends LinearOpMode {


    private DriveSubsystem driveSubsystem;

    @Override public void runOpMode()
    {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        }

        while (opModeIsActive())
        {
            driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
        }

    }

}