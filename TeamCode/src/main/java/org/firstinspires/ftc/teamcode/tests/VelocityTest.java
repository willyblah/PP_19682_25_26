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
    double targetX = 136.5, targetY = 8;
    double distance;
    boolean shootOn = false;
    double targetATAN, drivetrainHeading;
    int turretTargetHeading = 0;
    int turretCorrection = 0;


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
            drivetrainHeading = current.getHeading(AngleUnit.DEGREES);
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - drivetrainHeading) <= 175) {
                turretTargetHeading = (int) (targetATAN - drivetrainHeading);
            } else {
                turretTargetHeading = 0;
            }


            if (gamepad1.dpadUpWasPressed()) VELOCITY += 20;
            if (gamepad1.dpadDownWasPressed()) VELOCITY -= 20;
            if (gamepad1.dpadLeftWasPressed()) IN_POWER += 0.01;
            if (gamepad1.dpadRightWasPressed()) IN_POWER -= 0.01;
            IN_POWER = Math.max(0, Math.min(IN_POWER, 1));

            if (gamepad1.yWasPressed()) POSITION += 0.01;
            if (gamepad1.aWasPressed()) POSITION -= 0.01;
            POSITION = Math.max(0, Math.min(POSITION, 1));

            if (gamepad1.xWasPressed()) turretCorrection -= 2;
            if (gamepad1.bWasPressed()) turretCorrection += 2;

            if (gamepad1.right_trigger > 0) robot.intake.intakeFire(IN_POWER);
            else robot.intake.intakeStop();

            if (gamepad1.rightBumperWasPressed()) {
                shootOn = true;
                robot.intake.gateOpen();
                robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
            }
            if (gamepad1.leftBumperWasPressed()) {
                robot.intake.gateClose();
                shootOn = false;
                robot.shooter.turretToDegree(0);
            }

            if (shootOn)robot.shooter.setShooterVelocity(VELOCITY);
            else robot.shooter.shooterStop();
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
