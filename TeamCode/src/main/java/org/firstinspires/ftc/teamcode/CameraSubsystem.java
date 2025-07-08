package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;
import androidx.core.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraSubsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Boolean opModeIsActive;

    private Camera camera;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    public Boolean shouldStop = false;

    CameraSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, @NonNull Boolean opModeIsActive) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opModeIsActive = opModeIsActive;

        if (opModeIsActive) {
            camera = hardwareMap.get(Camera.class, "camera");

        }

        initAprilTag();
        // setManualExposure();
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (AutoConstants.USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!shouldStop && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!shouldStop)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    public List<AprilTagDetection> findAprilTags() {
        return aprilTag.getDetections();
    }

    public Pair<Double, Double> goToAprilTag(AprilTagDetection aprilTag) {
        double drive, turn;
        // Determine heading and range error so we can use them to control the robot automatically.
        double  rangeError   = (desiredTag.ftcPose.range - AutoConstants.DESIRED_DISTANCE);
        double  headingError = desiredTag.ftcPose.bearing;

        // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
        drive = Range.clip(rangeError * AutoConstants.SPEED_GAIN, -AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_SPEED);
        turn = Range.clip(headingError * AutoConstants.TURN_GAIN, -AutoConstants.MAX_AUTO_TURN, AutoConstants.MAX_AUTO_TURN) ;

        telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);

        return Pair.create(drive, turn);
    }
}
