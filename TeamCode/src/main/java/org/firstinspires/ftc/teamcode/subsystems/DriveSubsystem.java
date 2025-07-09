package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

public class DriveSubsystem {

    private final Telemetry telemetry;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final IMU imu;
    private double IMUheadingError;

    public DriveSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

        this.telemetry = telemetry;

        leftMotor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.LEFT_MOTOR_ID);
        rightMotor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.RIGHT_MOTOR_ID);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        imu.resetYaw();

    }

    /**
     * use lSpeed and rSpeed to move each side of the robot independently
     *
     * @param lSpeed left side speed
     * @param rSpeed right side speed
     */
    private void splitDrive(double lSpeed, double rSpeed) {
        leftMotor.setPower(lSpeed);
        rightMotor.setPower(rSpeed);
    }

    /**
     * Basic tank drive
     *
     * @param forward forward power
     * @param turn    turn power
     */
    public void tankDrive(double forward, double turn) {

        double ogForward = forward;
        double ogTurn = turn;
        double mag;

        mag = Math.sqrt((forward * forward) + (turn * turn));

        if (mag > 0) {
            forward /= mag;
            turn /= mag;
        }

        double lSpeed = forward + turn;
        double rSpeed = forward - turn;

        splitDrive(lSpeed, rSpeed);

        telemetry.addData("Status", String.format(Locale.ENGLISH, "forward %f  %f \n turn %f %f", ogForward, ogTurn, forward, turn));

    }

    /*
     * ======================================FROM EXAMPLE CODE=============================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * Constants.DriveConstants.COUNTS_PER_INCH);
        int leftTarget = leftMotor.getCurrentPosition() + moveCounts;
        int rightTarget = rightMotor.getCurrentPosition() + moveCounts;

        double turnSpeed;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        tankDrive(maxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftMotor.isBusy() && rightMotor.isBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.DriveConstants.P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0) turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            tankDrive(maxDriveSpeed, turnSpeed);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        tankDrive(0, 0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        double turnSpeed;

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, Constants.DriveConstants.P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while ((Math.abs(this.IMUheadingError) > Constants.DriveConstants.HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.DriveConstants.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            tankDrive(0, turnSpeed);

            // Display drive status for the driver.
        }

        // Stop all motion;
        tankDrive(0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        double turnSpeed;

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while ((holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.DriveConstants.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            tankDrive(0, turnSpeed);
        }

        // Stop all motion;
        tankDrive(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    private double getHeadingError(double desiredHeading) {
        return desiredHeading - getHeading(); // Determine the heading current error
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        this.IMUheadingError = getHeadingError(desiredHeading);
        double steeringCorrection;

        // Normalize the error to be within +/- 180 degrees
        while (this.IMUheadingError > 180) this.IMUheadingError -= 360;
        while (this.IMUheadingError <= -180) this.IMUheadingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        steeringCorrection = Range.clip(this.IMUheadingError * proportionalGain, -1, 1);

        return steeringCorrection;
    }

    /*
    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     *
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
    }
    */

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
