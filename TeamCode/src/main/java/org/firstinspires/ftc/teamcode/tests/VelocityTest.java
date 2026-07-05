package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.constants.panelConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class VelocityTest extends LinearOpMode {
    Robot robot = new Robot();
    JoinedTelemetry joinedTele;
    double targetX = 136.5, targetY = 8, vx, vy;
    double distance;
    boolean shootOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        targetX = 136.5;
        targetY = 8;
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);
            Pose2D current = robot.drivetrain.getPosition();
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));


            if (gamepad1.dpadUpWasPressed()) VELOCITY += 20;
            if (gamepad1.dpadDownWasPressed()) VELOCITY -= 20;
            if (gamepad1.dpadLeftWasPressed()) IN_POWER += 0.01;
            if (gamepad1.dpadRightWasPressed()) IN_POWER -= 0.01;
            IN_POWER = Math.max(0, Math.min(IN_POWER, 1));
            if (gamepad1.bWasPressed()) shootOn = !shootOn;

            if (shootOn)robot.shooter.setShooterVelocity(VELOCITY);
            else robot.shooter.shooterStop();

            if (gamepad1.yWasPressed()) POSITION += 0.01;
            if (gamepad1.aWasPressed()) POSITION -= 0.01;
            POSITION = Math.max(0, Math.min(POSITION, 1));

            if (gamepad1.right_trigger > 0) robot.intake.intakeFire(IN_POWER);
            else robot.intake.intakeStop();

            if (gamepad1.right_bumper) robot.intake.gateOpen();
            if (gamepad1.left_bumper) robot.intake.gateClose();


            robot.shooter.panelTo(POSITION);

            joinedTele.addData("distance", distance);
            joinedTele.addData("intakePower", IN_POWER);
            joinedTele.addData("target velocity", VELOCITY);
            joinedTele.addData("current velocity", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("POSITION", POSITION);
            joinedTele.update();
        }
    }
}
