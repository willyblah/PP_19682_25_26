package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.*;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class VelocityTest extends LinearOpMode {
    Robot robot = new Robot();
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.dpadUpWasPressed()) VELOCITY += 40;
            if (gamepad1.dpadDownWasPressed()) VELOCITY -= 40;
            if (gamepad1.dpadLeftWasPressed()) IN_POWER += 0.02;
            if (gamepad1.dpadRightWasPressed()) IN_POWER -= 0.02;
            if (gamepad1.bWasPressed()) {
                if (VELOCITY > 0) VELOCITY = 0;
                else VELOCITY = 1600;
            }

            if (gamepad1.y) POSITION += 0.005;
            if (gamepad1.a) POSITION -= 0.005;

            if (gamepad1.right_trigger > 0) robot.intake.intakeFire(IN_POWER);
            else robot.intake.intakeStop();

            if (gamepad1.right_bumper) robot.intake.gateOpen();
            if (gamepad1.left_bumper) robot.intake.gateClose();

            robot.shooter.setShooterVelocity(VELOCITY);
            robot.shooter.panelTo(POSITION);

            joinedTele.addData("intakePower", IN_POWER);
            joinedTele.addData("target velocity", VELOCITY);
            joinedTele.addData("current velocity", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("POSITION", POSITION);
            joinedTele.update();
        }
    }
}
