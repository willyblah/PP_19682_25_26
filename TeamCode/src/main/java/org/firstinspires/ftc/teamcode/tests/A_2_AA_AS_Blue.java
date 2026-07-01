package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.hoodCorrection;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class A_2_AA_AS_Blue extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 136.5, targetY = 136, vx, vy;
    int turretTargetHeading = 0;
    double targetATAN, drivetrainHeading;
    boolean shooterOn = false;
    double distance;
    int turretCorrection = 0;
    double distanceCorrection = 2;
    long gap = 0;
    ElapsedTime timer = new ElapsedTime();
    boolean intakeOn = false, intakeLeft = false, triggered = false, shootAll = false;
    boolean lh = false, rh = false, yh = false, ch = false;
    JoinedTelemetry joinedTele;
    public static double at = 0.64;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.right_trigger > 0.1) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.rightBumperWasPressed()) {
                timer.reset();
            } else if (gamepad1.right_bumper) {
                if (timer.milliseconds() > 180) hoodCorrection = 0.045;
                robot.intake.intakeIn(robot.shooter.calculateIntakePower());
            } else {
                robot.intake.intakeStop();
            }

            Pose2D current = robot.drivetrain.getPosition();
            drivetrainHeading = current.getHeading(AngleUnit.DEGREES);
            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            at = Math.abs(Math.hypot(teleOpTargetY - current.getY(DistanceUnit.INCH), teleOpTargetX - current.getX(DistanceUnit.INCH))) * 0.00575 + 0.4;
            targetX = teleOpTargetX - at * vx;
            targetY = teleOpTargetY - at * vy;
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - drivetrainHeading) <= 175) {
                turretTargetHeading = (int) (targetATAN - drivetrainHeading);
            } else {
                turretTargetHeading = 0;
            }
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

            if (gamepad2.dpadUpWasPressed()) distanceCorrection += 2;
            if (gamepad2.dpadDownWasPressed()) distanceCorrection -= 2;

            if (gamepad2.dpadLeftWasPressed()) turretCorrection -= 2;
            if (gamepad2.dpadRightWasPressed()) turretCorrection += 2;

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
                if (shooterOn) {
                    hoodCorrection = 0;
                }
            }

            if (shooterOn) {
                robot.intake.gateOpen();
//                robot.shooter.setShooterVelocity(shooterVelocity);
                robot.shooter.setShooterByDis(distance + distanceCorrection);
                robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
//                robot.shooter.turretToDegPP(turretTargetHeading);
            }
            else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
//                robot.shooter.shooterStop();
//                robot.shooter.turretToDegPP(drivetrainHeading);
                robot.shooter.turretToDegree(0);
            }

//            robot.shooter.panelTo(panelPos);

            joinedTele.addData("x", current.getX(DistanceUnit.INCH));
            joinedTele.addData("y", current.getY(DistanceUnit.INCH));
            joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
            joinedTele.addData("target", targetATAN);
//            joinedTele.addData("turretTarget", turretTargetHeading);
//            joinedTele.addData("turretHeading", robot.shooter.getPPHeading());
            joinedTele.addData("turretTo", turretTargetHeading);
//            joinedTele.addData("turretTicks", robot.shooter.getTurretPosition());
            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("distance", distance);
//            joinedTele.addData("panel", panelPos);
//            joinedTele.addData("panelActPos", robot.shooter.panel.getPosition());
            joinedTele.addData("shooterT", targetVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
//            joinedTele.addData("shootAllTime", TOTAL_SHOOT_TIME);
            joinedTele.update();
        }
    }
}
