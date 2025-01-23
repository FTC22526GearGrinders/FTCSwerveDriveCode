package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

public final class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE = 3.5 * Math.PI;
    public static final double TICKS_PER_REVOLUTION = 1;

    private static final double inchesToMeters = 0.0254;

    public static final double maxSpeedMeters = 2.4638; //I have absolutely no clue if this is the right number
    public static final double maxRadiansPerSecond = 3.63 * Math.PI; //2PI / (0.4318PI / maxSpeed)

    public static final double trackWidth = 11.5 ;
    public static final double wheelBase = 12.5;

    public static final double trackWidthMeters = trackWidth * inchesToMeters;
    public static final double wheelBaseMeters = wheelBase * inchesToMeters;

    public static final Translation2d flModuleOffset = new Translation2d(wheelBaseMeters/ 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);


    //For controlling the robot Values can be adjusted depending on who is controlling
    public static final double rotationMultiplier = 0.5;

}
