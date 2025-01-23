package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveModule extends SubsystemBase {

    public CRServo angleServo;
    public DcMotorEx driveMotor;
    public AnalogInput servoPotentiometer;
    public double angleOffset;
    public double angleSetPoint;

    public CustomPIDFController angleController;
    public PIDFController driveController;

    public int moduleNumber;
    public Telemetry telemetry;

    public double setpoint = 0;
    public double anglePID = 0;
    public double wheelDegs = 0;
    public double drivePower = 0;
    public double driveSpeedMetersPerSecond = 0;


    private final boolean showTelemetry = true;

    public SwerveModule(SwerveModuleConfig config, CommandOpMode opMode) {
        driveMotor = opMode.hardwareMap.get(DcMotorEx.class, config.driveMotorName);

        angleServo = opMode.hardwareMap.get(CRServo.class, config.angleServoName);
        angleServo.setDirection(config.angleReverse);

        servoPotentiometer = opMode.hardwareMap.get(AnalogInput.class, config.absoluteEncoderName);

        angleOffset = config.offset;

        //angleController = config.anglePIDFController;
        angleController = new CustomPIDFController(config.anglePIDFController.getP(), 0, 0, 0);
        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(3);

        driveController = config.drivePIDFController;

        moduleNumber = config.moduleNumber;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState newState = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngleRad()));

        setSpeed(newState);
        setAngle(newState);
    }

    public void setSpeed(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        driveSpeedMetersPerSecond = speed;
        double power = speed / SwerveDriveConstants.maxSpeedMeters;
        if (power > 1) power = 1;
        if (power < -1) power = -1;

        drivePower = power;
        driveMotor.setPower(power); //-1.0 to 1.0
    }

    public void setAngle(SwerveModuleState state) {
        angleSetPoint = state.angle.getDegrees();

        setpoint = angleSetPoint;
        angleController.setSetPoint(setpoint);

        wheelDegs = getWheelAngleDeg();
        double pidout = angleController.calculate(wheelDegs);

        if (pidout > 1) pidout = 1;
        if (pidout < -1) pidout = -1;

        anglePID = pidout;

        angleServo.setPower(pidout);
    }

    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeed();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleRadians * 180 / Math.PI));
    }

    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    double getWheelAngleDeg() {
        double volts = servoPotentiometer.getVoltage();
        double potAngle = volts * 360 / 3.3;
        return Math.IEEEremainder((potAngle + angleOffset), 360);
    }

    public double getWheelSpeed() {
        double motorTicksPerSecond = driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / SwerveDriveConstants.TICKS_PER_REVOLUTION;
        return wheelRevolutionsPerSecond * SwerveDriveConstants.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void periodic() {
        if (showTelemetry) {
        telemetry.addData("CurrentDegrees" + moduleNumber, getWheelAngleDeg());
        telemetry.addData("SetPoint" + moduleNumber, setpoint);
        telemetry.addData("PIDOut" + moduleNumber, anglePID);
        telemetry.addData("DrivePower" + moduleNumber, drivePower);
        telemetry.addData("DriveSpeed" + moduleNumber, driveSpeedMetersPerSecond);
    }
        if (angleController.atSetPoint()) {
            angleServo.setPower(0);
        }
    }
}
