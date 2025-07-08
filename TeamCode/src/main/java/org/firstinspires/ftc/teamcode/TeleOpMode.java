package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;


@TeleOp(name = "TeleOp")
@SuppressWarnings("unused")
public class TeleOpMode extends LinearOpMode {


    private DriveSubsystem driveSubsystem;
    private CameraSubsystem cameraSubsystem;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry, opModeIsActive());
            cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry, opModeIsActive());
        }

        while (opModeIsActive()) {
            if (isStopRequested()) {
                driveSubsystem.shouldStop = true;
                cameraSubsystem.shouldStop = true;
                break;
            }

            driveSubsystem.runtime = getRuntime();
            driveSubsystem.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, 1);

            if (driveSubsystem.requestedRuntimeReset) {
                resetRuntime();
                driveSubsystem.grantReset();
            }

            List<AprilTagDetection> detections = cameraSubsystem.findAprilTags();
            double drive, turn;
            while (gamepad1.a) {
                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null) {

                    } else {

                    }

                }


            }

            telemetry.addData("Status", String.format(Locale.ENGLISH, "left %f right %f", gamepad1.left_stick_y, gamepad2.right_stick_x));
            telemetry.update();
        }
    }


}