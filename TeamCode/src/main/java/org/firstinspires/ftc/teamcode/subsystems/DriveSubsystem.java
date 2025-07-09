package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

public class DriveSubsystem {

    private final Telemetry telemetry;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private RevRoboticsCoreHexMotor armMotor;

    public DriveSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

        this.telemetry = telemetry;

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.LEFT_MOTOR_ID);
        rightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.RIGHT_MOTOR_ID);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");

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
     * @param forward    forward power
     * @param turn       turn power
     * @param driveSpeed drive speed (0 - 1)
     */
    public void tankDrive(double forward, double turn, double driveSpeed) {

        double ogForward = forward;
        double ogTurn = turn;
        double ogDriveSpeed = driveSpeed;
        double mag;

        mag = Math.sqrt((forward * forward) + (turn * turn));

        if (mag > 0) {
            forward /= mag;
            turn /= mag;
        }

        double lSpeed = forward + turn;
        double rSpeed = forward - turn;

        driveSpeed = driveSpeed < -1 ? -1 : driveSpeed > 1 ? 1 : driveSpeed;

        splitDrive(lSpeed * driveSpeed, rSpeed * driveSpeed);

        telemetry.addData("Status", String.format(Locale.ENGLISH, "forward %f  %f \n turn %f %f \n speed %f %f", ogForward, ogTurn, ogDriveSpeed, forward, turn, driveSpeed));

    }
}
