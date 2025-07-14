package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem {

    private final Telemetry telemetry;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final IMU imu;
    private double globalAngle;
    private Orientation lastAngles;

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

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

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

        telemetry.addData("Status", "forward %f  %f \n turn %f %f", ogForward, ogTurn, forward, turn);

    }

    public double getPitch() {
        double pitch = imu.getRobotYawPitchRollAngles().getPitch();
        if (pitch < 0) {
            return pitch + 360;
        } else {
            return pitch;
        }
    }

    public double getRoll() {
        double roll = imu.getRobotYawPitchRollAngles().getRoll();
        if (roll < 0) {
            return roll + 360;
        } else {
            return roll;
        }
    }

    public double getYaw() {
        double pitch = imu.getRobotYawPitchRollAngles().getYaw();
        if (pitch < 0) {
            return pitch + 360;
        } else {
            return pitch;
        }
    }

    public void turn(double degrees, double allowance) {

        double yaw = Double.NaN, distance;

        while (yaw != degrees) {
            yaw = getYaw();

            distance = Math.abs(yaw - degrees);
            distance = Math.min(distance, 360 - distance);
            if (distance > 180 ) distance -= 180;

            if (distance < 0) tankDrive(0, -1);
            else tankDrive(0, 1);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) throws InterruptedException {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        splitDrive(leftPower, rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            while (getAngle() > degrees);
        } else {    // left turn.
            while (getAngle() < degrees);
        }

        // turn the motors off.
        splitDrive(0,0);


        // wait for rotation to stop.
        while (leftMotor.isBusy() || rightMotor.isBusy()) sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public void sendTelemetry() {
        telemetry.addData("angles", "p %f \n r %f \n y %f", getPitch(), getRoll(), getYaw());
    }
}
