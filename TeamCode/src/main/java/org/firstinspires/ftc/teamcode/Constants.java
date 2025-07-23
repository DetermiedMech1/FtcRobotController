package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static class RobotConstants {
        public static final String LEFT_MOTOR_ID = "motor0";
        public static final String RIGHT_MOTOR_ID = "motor1";
        public static final String ARM_MOTOR_ID = "motor2";
        public static final String CLAW_MOTOR_ID = "servo0";
        public static final String DIST_SENSOR_ID = "distance0";
    }

    public static class DriveConstants {

        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
        public static final double COUNTS_PER_MOTOR_REV = 20;
        public static final double DRIVE_GEAR_REDUCTION = 1;
        public static final double WHEEL_DIAMETER_MM = 90;
        public static final double COUNTS_PER_MM = 1.89873418/* (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415) */;

        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        public static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
        public static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    }

    public static class AutoConstants {
        public static final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public static final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
        public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    }

    public static class ArmConstants {
        public static final double ENCODER_TICKS_PER_ROTATION = 288;
        public static final double GEARBOX_RATIO = 72.0/1.0;
        public static final double EXT_GEAR_RATIO = 90.0/15.0;
        public static final double TICKS_PER_DEGREE = 1.0 / 360.0;
        public static final double ARM_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION * EXT_GEAR_RATIO * GEARBOX_RATIO * TICKS_PER_DEGREE;

    }
}
