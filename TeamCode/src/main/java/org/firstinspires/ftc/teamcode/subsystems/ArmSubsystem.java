package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem {

    private final Telemetry telemetry;
    public final DcMotor armMotor;
    public Servo clawMotor = null;
    public CRServo intakeMotor = null;
    public final Rev2mDistanceSensor distanceSensor;
    public int armHoldPosition;
    public boolean intaking;
    public enum Part {
        ARM, CLAW, INTAKE
    }

    public ArmSubsystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.ARM_MOTOR_ID);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try {
            intakeMotor = hardwareMap.get(CRServo.class, Constants.RobotConstants.CLAW_MOTOR_ID);
        } catch (Exception intakeException) {
            try {
                clawMotor = hardwareMap.get(Servo.class, Constants.RobotConstants.CLAW_MOTOR_ID);
                clawMotor.setDirection(Servo.Direction.REVERSE);
                clawMotor.scaleRange(0, 0.13);
            } catch (Exception clawException) {
                intakeException.addSuppressed(clawException);
                throw intakeException;
            }
        }

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, Constants.RobotConstants.DIST_SENSOR_ID);


    }

    /**
     * move the arm at some speed
     * @param power percentage of speed to move at
     * @param speed to move at
     */
    public void moveArm(double power, double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(power * speed);

        if (power != 0) {
            armHoldPosition = armMotor.getCurrentPosition();
        }

        stop(Part.ARM);
    }

    /**
     * set the claw or intake to take in an item
     */
    public void intake(@NonNull Part part) {
        switch (part) {
            case CLAW:
                clawMotor.setPosition(1);

            case INTAKE:
                if (distanceSensor.getDistance(DistanceUnit.MM) < 25.0) {
                    stop(Part.INTAKE);
                } else {
                    intakeMotor.setPower(1);
                    intaking = true;
                }

                telemetry.update();

        }
    }

    /**
     * set the claw or intake to release an item
     */
    public void outtake(@NonNull Part part) {
        intaking = false;

        switch (part) {
            case CLAW:
                clawMotor.setPosition(0);

            case INTAKE:
                intakeMotor.setPower(-0.3);
        }

    }

    /**
     * stop
     */
    public void stop(@NonNull Part part) {
        switch (part) {
            case ARM:
                armMotor.setTargetPosition(armHoldPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                break;

            case CLAW:
                clawMotor.setPosition(0);
                break;

            case INTAKE:
                intakeMotor.setPower(0);
                break;
        }
    }

    public void sendTelemetry() {

        telemetry.addData("object distance: ", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("disable intake: ", distanceSensor.getDistance(DistanceUnit.MM) < 25.0);
    }

}
