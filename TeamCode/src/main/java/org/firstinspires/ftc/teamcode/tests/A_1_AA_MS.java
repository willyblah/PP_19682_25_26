package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.TOTAL_SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.SHOOT_MS;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;

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
public class A_1_AA_MS extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 136.5, targetY = 138, vx, vy;
//    double targetX = 136.5, targetY = 6;
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;
    double panelPos = 0.5, shooterVelocity = 2300;
    boolean shooterOn = false;
    double distance;
    int turretCorrection = 0;
    double distanceCorrection = 0;
    long gap = 0;
    ElapsedTime timer = new ElapsedTime();
    JoinedTelemetry joinedTele;
    public static double time = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        robot.shooter.reset();
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
                robot.intake.intakeIn(INTAKE_POWER);
            } else {
                robot.intake.intakeStop();
            }

            Pose2D current = robot.drivetrain.getPosition();
            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            targetX = teleOpTargetX - time * vx;
            targetY = teleOpTargetY - time * vy;
            turretCurrentHeading = current.getHeading(AngleUnit.DEGREES);
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - turretCurrentHeading) <= 80) {
                turretTargetHeading = (int) (targetATAN - turretCurrentHeading);
            } else {
                turretTargetHeading = 0;
            }
            distance = Math.abs(Math.hypot(teleOpTargetY - current.getY(DistanceUnit.INCH), teleOpTargetX - current.getX(DistanceUnit.INCH)));

//            if (g1.getDpadUp()) distanceCorrection += 2;
//            if (g1.getDpadDown()) distanceCorrection -= 2;
//
            if (gamepad1.circleWasPressed()) turretCorrection += 2;
            if (gamepad1.squareWasPressed()) turretCorrection -= 2;

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                robot.intake.gateOpen();
                robot.shooter.setShooterVelocity(shooterVelocity);
//                robot.shooter.setShooterByDis(distance + distanceCorrection);
                robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
//                robot.shooter.turretToDegree(0);
            }
            else {
                robot.intake.gateClose();
//                robot.shooter.shooterHold();
                robot.shooter.shooterStop();
                robot.shooter.turretToDegree(0);
            }

            robot.shooter.panelTo(panelPos);

            if (gamepad1.dpad_up && panelPos < 1) panelPos += 0.005;
            if (gamepad1.dpad_down && panelPos > 0) panelPos -= 0.005;
            if (gamepad1.dpadLeftWasPressed()) shooterVelocity -= 20;
            if (gamepad1.dpadRightWasPressed()) shooterVelocity += 20;

//            if (gamepad1.dpad_up) {
//                shooterVelocity = 1980;
//                panelPos = 0.65;
//                INTAKE_POWER = 0.65;
//            }
//            if (gamepad1.dpad_down) {
//                shooterVelocity = 1380;
//                panelPos = 0.545;
//                INTAKE_POWER = 1;
//            }

            joinedTele.addData("x", current.getX(DistanceUnit.INCH));
            joinedTele.addData("y", current.getY(DistanceUnit.INCH));
            joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
            joinedTele.addData("target", targetATAN);
            joinedTele.addData("turretTo", turretTargetHeading);
            joinedTele.addData("turretTicks", robot.shooter.getTurretPosition());
            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("distance", distance);
            joinedTele.addData("panel", panelPos);
            joinedTele.addData("panelActPos", robot.shooter.panel.getPosition());
            joinedTele.addData("shooterT", shooterVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("shootAllTime", TOTAL_SHOOT_TIME);
            joinedTele.update();
        }
    }
}
