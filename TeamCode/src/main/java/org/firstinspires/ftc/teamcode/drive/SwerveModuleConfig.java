package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveModuleConfig {

    public final String driveMotorName, angleServoName, absoluteEncoderName;
    public final double offset;
    public final int moduleNumber;
    public PIDFController drivePIDFController, anglePIDFController;
    public DcMotorSimple.Direction angleReverse;

    public SwerveModuleConfig(int modNumber, PIDFController drivePIDFController, PIDFController anglePIDFController,
                              String driveMotorName, String angleServoName, String absoluteEncoderName, double offset, DcMotorSimple.Direction angleReverse) {
        this.driveMotorName = driveMotorName;
        this.angleServoName = angleServoName;
        this.absoluteEncoderName = absoluteEncoderName;
        this.drivePIDFController = drivePIDFController;
        this.anglePIDFController = anglePIDFController;
        this.angleReverse = angleReverse;
        this.offset = offset;
        this.moduleNumber = modNumber;
    }
}
