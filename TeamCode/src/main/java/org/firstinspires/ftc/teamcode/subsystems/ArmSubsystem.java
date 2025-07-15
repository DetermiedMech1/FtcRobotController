package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem {

    private final Telemetry telemetry;
    public final DcMotor armMotor;
    //public final Servo handMotor;
    public final CRServo intakeMotor;
    public final Rev2mDistanceSensor distanceSensor;
    public int armHoldPosition;

    public ArmSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.ARM_MOTOR_ID);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor = hardwareMap.get(CRServo.class, Constants.RobotConstants.HAND_MOTOR_ID);

        /*
        handMotor = hardwareMap.get(Servo.class, Constants.RobotConstants.HAND_MOTOR_ID);
        handMotor.setDirection(Servo.Direction.REVERSE);
        handMotor.scaleRange(0, 0.13);
        */

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, Constants.RobotConstants.DIST_SENSOR_ID);

    }

    /**
     * move the arm at some speed
     *
     * @param speed to move at
     */
    public void moveArm(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(speed * 2/5);

        if (speed != 0) {
            armHoldPosition = armMotor.getCurrentPosition();
        }

        stop();
    }

    /**
     *
     */
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    /**
     *
     */
    public void intake() {
        intakeMotor.setPower(1);
        if (distanceSensor.getDistance(DistanceUnit.MM) <= 20.0) {
            stopIntake();
        }
    }

    /**
     *
     */
    public void outtake() {
        intakeMotor.setPower(-0.3);
    }

    /**
     * stop the arm
     */
    public void stop() {
        armMotor.setTargetPosition(armHoldPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void sendTelemetry() {
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.MM) < 20.0);
    }

}
