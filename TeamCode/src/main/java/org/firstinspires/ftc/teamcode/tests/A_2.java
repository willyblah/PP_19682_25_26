package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.TOTAL_SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.SHOOT_MS;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class A_2 extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 136.5, targetY = 138;
//    double targetX = 136.5, targetY = 6;
    int turretTargetHeading = 0;
    double targetATAN, drivetrainHeading;
    double panelPos = 0.5, shooterVelocity = 2300;
    boolean shooterOn = false, autoTurret = true, autoShooter = true;
    double distance;
    int turretCorrection = 0;
    double distanceCorrection = 0;
    ElapsedTime timer = new ElapsedTime();
    double vx, vy, at;
    boolean shootAll = false, moveShoot = true;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.right_bumper) {
                if (autoShooter) {
                    robot.intake.intakeIn(robot.shooter.calculateIntakePower());
                } else {
                    robot.intake.intakeIn(INTAKE_POWER);
                }
            } else if (!shootAll) {
                robot.intake.intakeStop();
            }

            if (shootAll && timer.milliseconds() > TOTAL_SHOOT_TIME) {
                shootAll = false;
            }

            Pose2D current = robot.drivetrain.getPosition();
            drivetrainHeading = current.getHeading(AngleUnit.DEGREES);
            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            if (moveShoot) {
                at = Math.abs(Math.hypot(teleOpTargetY - current.getY(DistanceUnit.INCH), teleOpTargetX - current.getX(DistanceUnit.INCH))) * 0.00575 + 0.4;
            } else {
                at = 0.0;
            }
            targetX = teleOpTargetX - at * vx;
            targetY = teleOpTargetY - at * vy;
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - drivetrainHeading) <= 80) {
                turretTargetHeading = (int) (targetATAN - drivetrainHeading);
            } else {
                turretTargetHeading = 0;
            }
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

            if (gamepad1.squareWasPressed()) moveShoot = !moveShoot;

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                robot.intake.gateOpen();

                if (autoTurret)
                    robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
                else
                    robot.shooter.turretToDegree(0);

                if (autoShooter)
                    robot.shooter.setShooterByDis(distance + distanceCorrection);
                else {
                    robot.shooter.setShooterVelocity(shooterVelocity);
                    robot.shooter.panelTo(panelPos);
                }
            } else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
                robot.shooter.turretToDegree(0);
            }


            if (gamepad2.leftBumperWasPressed()) autoTurret = !autoTurret;
            if (gamepad2.rightBumperWasPressed()) autoShooter = !autoShooter;

            if (gamepad2.triangle) {
                shooterVelocity = 1980;
                panelPos = 0.65;
                INTAKE_POWER = 0.65;
            }
            if (gamepad2.cross) {
                shooterVelocity = 1380;
                panelPos = 0.545;
                INTAKE_POWER = 1;
            }

            if (gamepad2.dpadUpWasPressed()) distanceCorrection += 2;
            if (gamepad2.dpadDownWasPressed()) distanceCorrection -= 2;

            if (gamepad2.dpadLeftWasPressed()) turretCorrection -= 2;
            if (gamepad2.dpadRightWasPressed()) turretCorrection += 2;

            joinedTele.addData("autoTurret", autoTurret);
            joinedTele.addData("autoShooter", autoShooter);
            joinedTele.addData("x", current.getX(DistanceUnit.INCH));
            joinedTele.addData("y", current.getY(DistanceUnit.INCH));
            joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
//            joinedTele.addData("target", targetATAN);
//            joinedTele.addData("turretTo", turretTargetHeading);
//            joinedTele.addData("turretTicks", robot.shooter.getTurretPosition());
//            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("distance", distance);
//            joinedTele.addData("panel", panelPos);
//            joinedTele.addData("panelActPos", robot.shooter.panel.getPosition());
            joinedTele.addData("shooterT", autoShooter ? targetVelocity : shooterVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("shootAllTime", TOTAL_SHOOT_TIME);
            joinedTele.update();
        }
    }
}
