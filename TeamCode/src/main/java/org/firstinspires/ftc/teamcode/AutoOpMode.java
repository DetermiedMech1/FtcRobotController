package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "AutoOp")
@SuppressWarnings("unused")
public class AutoOpMode extends LinearOpMode {

    private final int DESIRED_TAG_ID = -1;
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private enum AutoState {
        FIRST, SECOND, LAST, DONE
    }

    private AutoState state;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        state = AutoState.FIRST;

        boolean targetFound;

        initAprilTag();


        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (opModeIsActive()) {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
            armSubsystem = new ArmSubsystem(hardwareMap, telemetry);

        }

        while (opModeIsActive()) {
            driveSubsystem.stopped = isStopRequested();

            telemetry.addData("state:", state);
            telemetry.addData("ball", driveSubsystem.getDistance());

            switch (state) {
                case FIRST:
                    driveSubsystem.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armSubsystem.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (driveSubsystem.getDistance() > 25) {
                        driveSubsystem.tankDrive(-0.5, 0, 0.2);
                        telemetry.addData("1", "driving forward");

                        if (!armSubsystem.intaking)
                            armSubsystem.intake(ArmSubsystem.Part.INTAKE);
                        telemetry.addData("2","intaking");
                    }

                    if (driveSubsystem.getDistance() < 25) {

                        driveSubsystem.tankDrive(0,0,0);
                        armSubsystem.moveArmToPosition(-210, 0.1);
                        telemetry.addData("3","moving arm");

                        while (armSubsystem.isBusy()) sleep(100);

                        state = AutoState.SECOND;
                    }

                    break;

                case SECOND:
                    driveSubsystem.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addData("4","resetting encoders");

                    armSubsystem.outtake(ArmSubsystem.Part.INTAKE);
                    telemetry.addData("5","outtaking");

                    state = AutoState.LAST;
                    break;

                case LAST:
                    armSubsystem.moveArmToPosition(0, 0.1);
                    telemetry.addData("7","moving arm");
                    while (armSubsystem.isBusy()) sleep(100);

                    while (Math.round(driveSubsystem.getYaw()) != 90) {
                        driveSubsystem.turn(10);
                    }
                    telemetry.addData("8", "turning");

                    driveSubsystem.drive(DistanceUnit.MM.fromInches(2),0.2);
                    telemetry.addData("9", "driving back");
                    while (driveSubsystem.isBusy()) sleep(100);

                    state = AutoState.DONE;
                    break;

                default:
                    break;

            }

            telemetry.update();
        }
    }

    //*************************CAMERA METHODS*************************//

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.

        aprilTag = new AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()).build();


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
            visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "camera0")).addProcessor(aprilTag).build();
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

        return Pair.of(-drive, -turn);
    }
}
