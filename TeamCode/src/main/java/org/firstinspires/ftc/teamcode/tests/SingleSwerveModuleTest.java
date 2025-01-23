package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleConfig;

@TeleOp(name = "SingleSwerveModuleTest1", group = "Test")
@Config
public class SingleSwerveModuleTest extends CommandOpMode {


    private final PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    private final PIDFController turnController = new PIDFController(0.015, 0, 0, 0);

    private SwerveModuleConfig config = new SwerveModuleConfig(0, driveController, turnController,
            "driveMotor4", "angleServo4", "angleInput4", 0, DcMotorSimple.Direction.REVERSE);

    public double targetAngle = 0;

    SwerveModule module;


    public void initialize() {
        module = new SwerveModule(config, this);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            run();
            if (gamepad1.right_bumper) {
                if (gamepad1.a) targetAngle = 45;
                if (gamepad1.b) targetAngle = 135;
                if (gamepad1.x) targetAngle = -135;
                if (gamepad1.y) targetAngle = -45;
                module.setState(new SwerveModuleState(0, new Rotation2d((targetAngle / 180) * Math.PI)));
            }
        }
    }

}
