package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static class DriveConstants {
        public static final String LEFT_MOTOR_ID = "0";
        public static final String RIGHT_MOTOR_ID = "1";
        public static final double COUNTS_PER_INCH = 1024;
    }

    public static class AutoConstants {
        public static final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public static final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
        public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    }
}
