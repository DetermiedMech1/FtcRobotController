package org.firstinspires.ftc.teamcode.subsystems;

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
    public void tankDrive(double forward, double turn, double speed) {

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

        splitDrive(lSpeed * speed, rSpeed * speed);

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

    public void sendTelemetry() {
        telemetry.addData("angles", "p %f \n r %f \n y %f", getPitch(), getRoll(), getYaw());
    }
}
