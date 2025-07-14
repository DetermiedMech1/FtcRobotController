package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem {

    private final Telemetry telemetry;
    public final DcMotor armMotor;
    public final Servo handMotor;
    private int armHoldPosition;

    public ArmSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.ARM_MOTOR_ID);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        handMotor = hardwareMap.get(Servo.class, Constants.RobotConstants.HAND_MOTOR_ID);
        handMotor.setDirection(Servo.Direction.REVERSE);
        handMotor.scaleRange(0,0.13);


    }

    /**
     * move the arm at some speed
     *
     * @param speed to move at
     */
    public void moveArm(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(speed * 0.5);
        armHoldPosition = armMotor.getCurrentPosition();
        stop();
    }

    /**
     *
     */
    public void openHand(boolean open) {
        if (open) {
            handMotor.setPosition(0);
        } else {
            handMotor.setPosition(1);
        }
    }


    /**
     *
     */
    /*
    public void runIntake(boolean direction) {

    }
    */

    /**
     * stop the arm
     */
    public void stop() {
        armMotor.setTargetPosition(armHoldPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0.3);
    }


}
