package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@Autonomous(name = "TestOp")
@SuppressWarnings("unused")
public class TestOpMode extends LinearOpMode {


    private DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;

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
            driveSubsystem.driveStraight(Constants.DriveConstants.DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
            driveSubsystem.turnToHeading(Constants.DriveConstants.TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
            driveSubsystem.holdHeading(Constants.DriveConstants.TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second

            driveSubsystem.driveStraight(Constants.DriveConstants.DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
            driveSubsystem.turnToHeading(Constants.DriveConstants.TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
            driveSubsystem.holdHeading(Constants.DriveConstants.TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

            driveSubsystem.driveStraight(Constants.DriveConstants.DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
            driveSubsystem.turnToHeading(Constants.DriveConstants.TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
            driveSubsystem.holdHeading(Constants.DriveConstants.TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second

            driveSubsystem.driveStraight(Constants.DriveConstants.TURN_SPEED,-48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)

        }

    }

}