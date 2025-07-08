package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;


@TeleOp(name = "TeleOp")
public class TeleOpMode extends LinearOpMode{



    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotor.class, Constants.MotorConstants.leftMotorId);
        rightMotor = hardwareMap.get(DcMotor.class, Constants.MotorConstants.rightMotorId);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Status", String.format(Locale.ENGLISH, "left %f right %f", gamepad1.left_stick_y, gamepad2.left_stick_y));
            telemetry.update();
        }
    }

    /**
     * use lSpeed and rSpeed to move each side of the robot independently
     *
     * @param lSpeed left side speed
     * @param rSpeed right side speed
     */
    public void splitDrive(double lSpeed, double rSpeed) {
        leftMotor.setPower(lSpeed * Constants.MotorConstants.motorSpeed);
        rightMotor.setPower(rSpeed * Constants.MotorConstants.motorSpeed);
    }

    /**
     * Basic tank drive
     *
     * @param forward forward power
     * @param turn turn power
     */
    public void tankDrive(double forward, double turn) {
        //TODO test the math here for whether it is actually correct
        double lSpeed = forward + turn;
        double rSpeed = forward - turn;

        /* idk if this is actually useful
        lSpeed = lSpeed > 1 ? 1 : lSpeed < -1 ? -1 : lSpeed;
        rSpeed = rSpeed > 1 ? 1 : rSpeed < -1 ? -1 : rSpeed;
        //*/

        splitDrive(lSpeed, rSpeed);
    }
    public void encoderForward() {

    }


}