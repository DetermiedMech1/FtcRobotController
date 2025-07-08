package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends LinearOpMode{



    private DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
    private DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            splitDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
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
        double lSpeed = forward - turn;
        double rSpeed = forward + turn;

        splitDrive(lSpeed, rSpeed);
    }
    public void encoderForward() {

    }


}