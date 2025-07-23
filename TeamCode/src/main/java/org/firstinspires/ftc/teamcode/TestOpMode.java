package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@Autonomous(name = "TestOp")
@SuppressWarnings("unused")
public class TestOpMode extends LinearOpMode {


    private ArmSubsystem armSubsystem;
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
            driveSubsystem.stopped = isStopRequested();

            while (true) {
                driveSubsystem.setTargetRotation(90);
                driveSubsystem.turnToTargetRotation(10);
                if (!driveSubsystem.turning) {
                    driveSubsystem.setTargetRotation(0);
                    break;
                }
            }

            if (!driveSubsystem.turning) break;

            driveSubsystem.sendTelemetry();
            telemetry.update();
        }

    }

}