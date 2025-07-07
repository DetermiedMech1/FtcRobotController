package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.MotorConstants;



public class DriveSubsystem {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final Telemetry telemetry;

    /**
     * Drive Subsystem <br>
     * Contains driving functions
     *
     * @param hardwareMap hardware map from caller
     * @param telemetry telemetry from caller
     *
     */
    public DriveSubsystem(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        //use telemetry stuff from caller
        this.telemetry = telemetry;

        //assign and set up motors
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor = hardwareMap.get(DcMotor.class, "1");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);


    }

    /**
     * use lspeed and rspeed to move each side of the robot independently
     *
     * @param lSpeed left side speed
     * @param rSpeed right side speed
     */
    public void splitDrive(double lSpeed, double rSpeed) {
        leftMotor.setPower(lSpeed * MotorConstants.motorSpeed);
        rightMotor.setPower(rSpeed * MotorConstants.motorSpeed);
    }

    /**
     * Basic tank drive
     *
     * @param forward forward power
     * @param turn turn power
     */
    public void tankDrive(double forward, double turn) {
        //TODO test the math here for whether it is actually correct
        double lSpeed = forward - turn;
        double rSpeed = forward + turn;

        splitDrive(lSpeed, rSpeed);
    }
    public void encoderForward() {

    }

}