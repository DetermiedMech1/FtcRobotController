package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@Autonomous(name = "TestOp")
@SuppressWarnings("unused")
public class TestOpMode extends LinearOpMode {


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

        double desired = 0;
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
            driveSubsystem.sendTelemetry();


            double allowance = 1;
            double yaw = driveSubsystem.getYaw();

            desired += (gamepad1.left_bumper ? 1 : 0) - (gamepad1.right_bumper ? 1 : 0);

            double turn = (yaw > desired - allowance ? 1 : 0) - (yaw < desired + allowance ? 1 : 0);

            driveSubsystem.tankDrive(0, turn);

            telemetry.addData("data", "current %f, target %f, turn %f", yaw, desired, turn);

            //driveSubsystem.rotate(90, 1);

            telemetry.update();
        }

    }

}