//TODO implement ArmSubsystem

package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ArmSubsystem {
    private final Telemetry telemetry;

    public ArmSubsystem(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

    }

    /**
     * set arm angle
     *
     * @param angle
     */
    //TODO determine angle units
    public void setArmAngle(double angle) {

    }

}
