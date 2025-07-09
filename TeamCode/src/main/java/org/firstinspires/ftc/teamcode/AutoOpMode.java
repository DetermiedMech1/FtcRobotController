package org.firstinspires.ftc.teamcode;

import androidx.core.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "Auto")
@SuppressWarnings("unused")
public class AutoOpMode extends LinearOpMode {
    private final int DESIRED_TAG_ID = -1;
    private DriveSubsystem driveSubsystem;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        boolean targetFound;

        initAprilTag();

        if (Constants.AutoConstants.USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        }

        while (opModeIsActive()) {
            targetFound = false;
            AprilTagDetection desiredTag = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);


            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            double drive, turn;
            if (gamepad1.left_bumper && targetFound) {
                Pair<Double, Double> movement = goToAprilTag(desiredTag);

                drive = movement.first;
                turn = movement.second;

                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            } else {
                drive = gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;

                telemetry.addData("Manual", "Drive %5.2f, Turn %5.2f", drive, turn);
            }


            driveSubsystem.tankDrive(drive, turn);
            telemetry.update();
        }
    }

    //*************************CAMERA METHODS*************************//

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.

        aprilTag = new AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary()).build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (Constants.AutoConstants.USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "camera")).addProcessor(aprilTag).build();
        } else {
            visionPortal = new VisionPortal.Builder().setCamera(BuiltinCameraDirection.BACK).addProcessor(aprilTag).build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    public Pair<Double, Double> goToAprilTag(AprilTagDetection desiredTag) {
        double drive, turn;
        // Determine heading and range error so we can use them to control the robot automatically.
        double rangeError = (desiredTag.ftcPose.range - Constants.AutoConstants.DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;

        // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
        drive = Range.clip(rangeError * Constants.AutoConstants.SPEED_GAIN, -Constants.AutoConstants.MAX_AUTO_SPEED, Constants.AutoConstants.MAX_AUTO_SPEED);
        turn = Range.clip(headingError * Constants.AutoConstants.TURN_GAIN, -Constants.AutoConstants.MAX_AUTO_TURN, Constants.AutoConstants.MAX_AUTO_TURN);

        telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);

        return Pair.create(-drive, -turn);
    }
}
