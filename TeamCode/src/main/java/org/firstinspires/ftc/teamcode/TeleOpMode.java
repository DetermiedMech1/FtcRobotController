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

        double pos = 0;
        double speed = 0;

        while (opModeIsActive()) {
            driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.3);
            armSubsystem.moveArm(gamepad1.right_stick_y, 0.7);

            if (gamepad1.circle || armSubsystem.intaking) {
                armSubsystem.intake(ArmSubsystem.Part.INTAKE);
            }

            if (gamepad1.cross) {
                armSubsystem.outtake(ArmSubsystem.Part.INTAKE);
            }

            driveSubsystem.sendTelemetry();
            armSubsystem.sendTelemetry();
            telemetry.update();

        }

    }

}