package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
@Configurable
public class A_2_AA_AS_NoPP extends LinearOpMode {
    Robot robot = new Robot();
    int turretTargetHeading = 0;
    double targetATAN;
    boolean shooterOn = false, movingShoot = false;
    double manualVelocity = 1800, manualPanel = 0.3;
    double distance;
    int turretCorrection = 0;
    double distanceCorrection = 2;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.right_trigger > 0.1) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.right_bumper) {
                robot.intake.intakeIn(robot.shooter.calculateIntakePower());
            } else {
                robot.intake.intakeStop();
            }

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                robot.intake.gateOpen();
                robot.shooter.setShooter(manualPanel, manualVelocity);
            } else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
            }
            robot.shooter.turretToDegree(0);

            if (gamepad2.triangleWasPressed()) {
                manualVelocity = 1420;
                manualPanel = 0.4;
            }
            if (gamepad2.squareWasPressed()) {
                manualVelocity = 1580;
                manualPanel = 0.8;
            }

            if (gamepad2.dpadLeftWasPressed()) manualVelocity += 40;
            if (gamepad2.dpadRightWasPressed()) manualVelocity -= 40;
            if (gamepad2.yWasPressed()) manualPanel += 0.01;
            if (gamepad2.aWasPressed()) manualPanel -= 0.01;

            joinedTele.addData("target", targetATAN);
            joinedTele.addData("turretTo", turretTargetHeading);
            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("movingShoot", movingShoot);
            joinedTele.addData("distance", distance);
            joinedTele.addData("shooterT", targetVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
            joinedTele.addData("panel", robot.shooter.panel.getPosition());
            joinedTele.update();
        }
    }
}
