package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@TeleOp(name = "TeleOp")
@SuppressWarnings("unused")
public class TeleOpMode extends LinearOpMode {


    public ArmSubsystem armSubsystem;
    private DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
            armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        }

        while (opModeIsActive()) {
            driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            armSubsystem.moveArm((gamepad1.left_bumper ? 1 : 0) - (gamepad1.right_bumper ? 1 : 0));

            telemetry.update();
        }

    }

}