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
            //driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

            speed = (gamepad1.left_bumper ? 1 : 0) - (gamepad1.right_bumper ? 1 : 0);

            armSubsystem.moveArm(speed);
            armSubsystem.openHand(gamepad1.circle);

            driveSubsystem.sendTelemetry();

            telemetry.update();

            telemetry.addData("arm speed", speed);
            telemetry.addData("arm", armSubsystem.armMotor.getCurrentPosition());
            telemetry.addData("hand", armSubsystem.handMotor.getPosition());
        }

    }

}