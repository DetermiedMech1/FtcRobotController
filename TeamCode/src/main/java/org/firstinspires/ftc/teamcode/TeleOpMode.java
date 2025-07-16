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

        double speed = 0;

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                speed = 0.7;
            } else if (gamepad1.right_bumper) {
                speed = 0.2;
            } else {
                speed = 0.4;
            }

            driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
            armSubsystem.moveArm((gamepad1.dpad_down?1:0) - (gamepad1.dpad_up?1:0), 1);

            if (gamepad1.circle || armSubsystem.intaking) {
                armSubsystem.intake(ArmSubsystem.Part.INTAKE);
            }

            if (gamepad1.cross) {
                armSubsystem.outtake(ArmSubsystem.Part.INTAKE);
            }

            telemetry.addData("Speed: ", speed);
            driveSubsystem.sendTelemetry();
            armSubsystem.sendTelemetry();
            telemetry.update();

        }

    }

}