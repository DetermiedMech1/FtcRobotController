package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem {

    private final Telemetry telemetry;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final IMU imu;
    private double globalAngle;
    private double targetRotation;
    private Orientation lastAngles;

    private Rev2mDistanceSensor distanceSensor = null;
    public boolean turning;
    public boolean stopped;

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

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, Constants.RobotConstants.DIST_SENSOR_ID);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    /**
     * use lpower and rpower to move each side of the robot independently
     *
     * @param lpower left side power
     * @param rpower right side power
     */
    private void splitDrive(double lpower, double rpower) {
        DcMotor.RunMode[] modes = {leftMotor.getMode(), rightMotor.getMode()};

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(lpower);
        rightMotor.setPower(rpower);

        leftMotor.setMode(modes[0]);
        rightMotor.setMode(modes[1]);
    }

    /**
     * drive forward some number of millimeters
     * @param distance distance to drive
     */
    public void drive(double distance, double power) {
        DcMotor.RunMode[] modes = {leftMotor.getMode(), rightMotor.getMode()};

        leftMotor.setPower(power);
        rightMotor.setPower(power);
        
        leftMotor.setTargetPosition((int) (distance * Constants.DriveConstants.COUNTS_PER_MM));
        rightMotor.setTargetPosition((int) (distance * Constants.DriveConstants.COUNTS_PER_MM));

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setMode(modes[0]);
        rightMotor.setMode(modes[1]);
    }

    public void turnWithEncoder(int ticks, double power) {
        DcMotor.RunMode[] modes = {leftMotor.getMode(), rightMotor.getMode()};

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(-ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setMode(modes[0]);
        rightMotor.setMode(modes[1]);
    }

    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy() || turning;
    }

    /**
     * turn by some number of degrees
     * @param degrees degrees to turn
     */
    public void turn(double degrees, double maxError) {
        turning = true;
        double yaw = getYaw();

        if (Math.abs(degrees - yaw) > maxError) {
            yaw = getYaw();
            telemetry.addData("yaw", yaw);

            double turn = (Math.abs(yaw - degrees) < 180) ? degrees - yaw : degrees - 360 + yaw;
            tankDrive(0, turn,0.1);
            telemetry.addData("turn", turn);

            telemetry.update();
        } else {
            turning = false;
        }
    }

    public void turn(double degrees) { turn(degrees, 0.1); }

    public void turnToTargetRotation(double maxError) {
        double yaw = getYaw();

        if (Math.abs(targetRotation - yaw) < maxError) {
            turning = false;
            return;
        }

        telemetry.addData("yaw", yaw);

        double turn = (Math.abs(yaw - targetRotation) < 180) ? targetRotation - yaw : targetRotation - 360 + yaw;
        tankDrive(0, turn,0.1);
        telemetry.addData("turn", turn);

        telemetry.update();
    }

    public void setTargetRotation(double degrees) {
        targetRotation = degrees;
        turning = true;
    }


    /**
     * Basic tank drive
     *
     * @param forward forward power
     * @param turn    turn power
     */
    public void tankDrive(double forward, double turn, double power) {

        double ogForward = forward;
        double ogTurn = turn;
        double mag;

        mag = Math.sqrt((forward * forward) + (turn * turn));

        if (mag > 0) {
            forward /= mag;
            turn /= mag;
        }

        double lpower = forward + turn;
        double rpower = forward - turn;

        splitDrive(lpower * power, rpower * power);

        telemetry.addData("Status", "forward %f  %f \n turn %f %f", ogForward, ogTurn, forward, turn);

    }

    public double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch();
    }

    public double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll();

    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw();
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

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public void sendTelemetry() {
        //telemetry.addData("angles", "p %f \n r %f \n y %f", getPitch(), getRoll(), getYaw());
    }
}
