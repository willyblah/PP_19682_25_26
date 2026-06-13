package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
@Configurable
public class TurretTest extends LinearOpMode {
    Shooter shooter = new Shooter();
    public static double targetHeading = 0.0, tor = 0.012;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();

        while (opModeIsActive()) {
//            shooter.turretToDegPP(targetHeading);

            joinedTele.addData("enc", shooter.turret.getCurrentPosition());
            joinedTele.addData("target", targetHeading);
            joinedTele.update();
        }
    }
}
