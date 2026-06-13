package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "A_1 AutoAim Subsystem Test", group = "Tests")
@Configurable
public class A_1_AutoAimSubsystemTest extends LinearOpMode {
    public static double drivePower = 1.0;
    public static double yawCorrection = 0.0;
    public static double velocityCorrection = 0.0;
    public static double distanceCorrection = 0.0;
    public static boolean shootOnTheMove = true;
    public static boolean manualDistanceMode = false;
    public static double manualDistance = 60.0;
    public static double brakeStickThreshold = 0.15;

    private final Robot robot = new Robot();
    private final AutoAimSubsystem autoAim = new AutoAimSubsystem();
    private JoinedTelemetry joinedTele;

    private boolean shooterOn = false;
    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        autoAim.init(hardwareMap);
        setStartingPose();
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        setStartingPose();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, drivePower);

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.right_bumper) {
                robot.intake.intakeIn(robot.shooter.calculateIntakePower());
            } else {
                robot.intake.intakeStop();
            }

            boolean currentA = gamepad1.a;
            boolean currentY = gamepad1.y;
            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }
            if (currentY && !lastY) {
                shootOnTheMove = !shootOnTheMove;
            }
            if (currentA && !lastA) {
                manualDistanceMode = !manualDistanceMode;
            }
            lastA = currentA;
            lastY = currentY;
            if (gamepad1.dpadUpWasPressed()) {
                distanceCorrection += 2.0;
            }
            if (gamepad1.dpadDownWasPressed()) {
                distanceCorrection -= 2.0;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                yawCorrection -= 1.0;
            }
            if (gamepad1.dpadRightWasPressed()) {
                yawCorrection += 1.0;
            }

            Pose2D current = robot.drivetrain.getPosition();
            double robotX = current.getX(DistanceUnit.INCH);
            double robotY = current.getY(DistanceUnit.INCH);
            double headingDeg = current.getHeading(AngleUnit.DEGREES);
            double vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            double vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            double omegaDeg = robot.drivetrain.pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            double manualDist = Math.max(0.0, manualDistance + distanceCorrection);
            boolean isBraking = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) < brakeStickThreshold;

            AutoAimSubsystem.SHOT_DISTANCE_OFFSET = distanceCorrection;
            AutoAimSubsystem.TurretCommand command = autoAim.update(
                    robotX, robotY, vx, vy,
                    headingDeg, omegaDeg,
                    teleOpTargetX, teleOpTargetY,
                    manualDistanceMode, manualDist,
                    shootOnTheMove, isBraking, yawCorrection
            );

            if (shooterOn && command.hasTarget) {
                robot.intake.gateOpen();
                targetVelocity = command.targetRpm + velocityCorrection;
                robot.shooter.setShooterVelocity(targetVelocity);
            } else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
            }

            joinedTele.addData("x", robotX);
            joinedTele.addData("y", robotY);
            joinedTele.addData("h", headingDeg);
            joinedTele.addData("targetX", teleOpTargetX);
            joinedTele.addData("targetY", teleOpTargetY);
            joinedTele.addData("vx", vx);
            joinedTele.addData("vy", vy);
            joinedTele.addData("omegaDeg", omegaDeg);
            joinedTele.addData("autoAimTarget", command.hasTarget);
            joinedTele.addData("aimLocked", command.isAimLocked);
            joinedTele.addData("targetDist", command.targetDist);
            joinedTele.addData("flightTime", command.flightTime);
            joinedTele.addData("targetTurret", command.targetTurretAngle);
            joinedTele.addData("turretDeg", autoAim.getCurrentTurretAngle());
            joinedTele.addData("filteredTurretDeg", autoAim.getFilteredTurretAngle());
            joinedTele.addData("turretVel", autoAim.getFilteredTurretVelocity());
            joinedTele.addData("aimError", command.aimError);
            joinedTele.addData("yawCorrection", yawCorrection);
            joinedTele.addData("targetVelocity", targetVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("targetPitch", command.targetPitch);
            joinedTele.addData("panelActPos", robot.shooter.panel.getPosition());
            joinedTele.addData("shootOnMove", shootOnTheMove);
            joinedTele.addData("manualMode", manualDistanceMode);
            joinedTele.addData("isBraking", isBraking);
            joinedTele.addData("battery", autoAim.getCurrentBatteryVoltage());
            joinedTele.update();
        }

        autoAim.stop();
        robot.shooter.shooterStop();
        robot.intake.intakeStop();
    }

    private void setStartingPose() {
        robot.drivetrain.pinPoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                autoEndY,
                144.0 - autoEndX,
                AngleUnit.RADIANS,
                autoEndH - Math.PI / 2.0
        ));
    }
}
