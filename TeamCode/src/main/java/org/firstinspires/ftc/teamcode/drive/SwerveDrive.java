package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private final PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    private final PIDFController turnController = new PIDFController(0.018, 0.005 , 0.025, 0);


    private BHI260IMU imu;

    private double period = 0;
    private double lastTimeStamp = 0;

    public Telemetry telemetry;

    public SwerveDrive(CommandOpMode opMode) {
        SwerveModuleConfig fl = new SwerveModuleConfig(0, driveController, turnController,
                "driveMotor1", "angleServo1", "angleInput1", 113.9, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig fr = new SwerveModuleConfig(1, driveController, turnController,
                "driveMotor2", "angleServo2", "angleInput2", 33.6, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig bl = new SwerveModuleConfig(2, driveController, turnController,
                "driveMotor3", "angleServo3", "angleInput3", 48.2, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig br = new SwerveModuleConfig(3, driveController, turnController,
                "driveMotor4", "angleServo4", "angleInput4", 125.8, DcMotorSimple.Direction.REVERSE);

        modules = new SwerveModule[] {
                new SwerveModule(fl, opMode),
                new SwerveModule(fr, opMode),
                new SwerveModule(bl, opMode),
                new SwerveModule(br, opMode)
        };

        imu = opMode.hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    public void drive(double translation, double strafe, double rotation, boolean fieldRelative) {

        double new_translation = translation * SwerveDriveConstants.maxSpeedMeters;
        double new_strafe = strafe * SwerveDriveConstants.maxSpeedMeters;
        double new_rotation = rotation * SwerveDriveConstants.maxRadiansPerSecond * SwerveDriveConstants.rotationMultiplier;

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(new_translation, new_strafe, new_rotation, getHeading())
                : new ChassisSpeeds(new_translation, new_strafe, new_rotation);


        //----------I'm not sure if any of this is needed------------copied from FRC--------------
//        double currentTimeStamp = (double) System.nanoTime() / 1E9; //to seconds
//        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
//        period = currentTimeStamp - lastTimeStamp;
//        if (period == 0) period = 1/40;
//
//        lastTimeStamp = currentTimeStamp;
//
//        speeds = discretize(speeds.vxMetersPerSecond,
//                speeds.vyMetersPerSecond,
//                speeds.omegaRadiansPerSecond,
//                period);
        //---------------------------------------------------------------------------------------

        SwerveModuleState[] states = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, SwerveDriveConstants.maxSpeedMeters);

        for (SwerveModule module : modules) {
            module.setState(states[module.moduleNumber]);
        }
    }

    public Rotation2d getHeading() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public void resetYaw() {
        imu.resetYaw();
    }
    public ChassisSpeeds discretize( //Imported Directly From WPILIB not sure if necessary
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double dtSeconds) {

        Pose2d desiredDeltaPose =
                new Pose2d(
                        vxMetersPerSecond * dtSeconds,
                        vyMetersPerSecond * dtSeconds,
                        new Rotation2d(omegaRadiansPerSecond * dtSeconds));


        Twist2d twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }

    @Override
    public void periodic() {
        
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Heading", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));

    }
}
