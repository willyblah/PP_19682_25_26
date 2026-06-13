package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
@Configurable
public class VelocityTest extends LinearOpMode {
    Robot robot = new Robot();
    double velocity = 0, position = 0.35, inPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.dpadUpWasPressed()) velocity += 20;
            if (gamepad1.dpadDownWasPressed()) velocity -= 20;
            if (gamepad1.dpadLeftWasPressed()) inPower += 0.01;
            if (gamepad1.dpadRightWasPressed()) inPower -= 0.01;

            if (gamepad1.y) position += 0.005;
            if (gamepad1.a) position -= 0.005;

            if (gamepad1.right_trigger > 0) robot.intake.intakeFire(inPower);
            else robot.intake.intakeStop();

            if (gamepad1.right_bumper) robot.intake.gateOpen();
            if (gamepad1.left_bumper) robot.intake.gateClose();

            robot.shooter.setShooterVelocity(velocity);
            robot.shooter.panelTo(position);

            telemetry.addData("intakePower", inPower);
            telemetry.addData("target velocity", velocity);
            telemetry.addData("current velocity", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
