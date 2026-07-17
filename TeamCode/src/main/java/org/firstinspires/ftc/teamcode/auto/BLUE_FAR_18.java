package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCurrentToPoint;
import org.firstinspires.ftc.teamcode.constants.BluePathChains;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class BLUE_FAR_18 extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    Robot robot = new Robot();
    BluePathChains bluePathChains;
    double distance = FAR_HOLD_DISTANCE;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(BLUE_FAR_START.copy());
        robot.autoInit(hardwareMap);
        telemetryM.update();
        bluePathChains = new BluePathChains(follower.follower);
        TOTAL_SHOOT_TIME = 550;
        Drawing.init();
    }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower.follower);
    }

    @Override
    public void init_loop() {
        follower.follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.follower.update();
        draw();
        autoEndX = follower.follower.getPose().getX();
        autoEndY = follower.follower.getPose().getY();
        autoEndH = follower.follower.getPose().getHeading();
        robot.shooter.setShooterByDis(distance);
        teleOpTargetX = BLUE_TARGET_X;
        teleOpTargetY = BLUE_TARGET_Y;
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.follower.activateAllPIDFs();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // 发射预载
                        new InstantCommand(() -> distance = FAR_FIRE_DISTANCE_PRELOAD),
                        new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_FAR_TURRET)),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new WaitCommand(AUTO_FAR_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 收集第三排
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_THIRD_CONTROL, BLUE_FAR_INTAKE_THIRD_END),
                        new InstantCommand(() -> robot.intake.intakeStop()),

                        // 发射第三排
                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 第一次
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new ParallelRaceGroup(
                                new WaitCommand(2500),
                                new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_HP_1_START, BLUE_FAR_INTAKE_HP_1_MID, BLUE_FAR_INTAKE_HP_1_END)
                        ),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_HP),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 第二次
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new ParallelRaceGroup(
                                new WaitCommand(2500),
                                new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_HP_1_START, BLUE_FAR_INTAKE_HP_1_MID, BLUE_FAR_INTAKE_HP_1_END)

                        ),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_HP),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 第三次
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new ParallelRaceGroup(
                                new WaitCommand(2500),
                                new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_HP_1_START, BLUE_FAR_INTAKE_HP_1_MID, BLUE_FAR_INTAKE_HP_1_END)

                        ),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_HP),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 第四次
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new ParallelRaceGroup(
                                new WaitCommand(2500),
                                new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_HP_1_START, BLUE_FAR_INTAKE_HP_1_MID, BLUE_FAR_INTAKE_HP_1_END)

                        ),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_HP),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),

                        // 停靠
                        new DriveCurrentToPoint(follower, BLUE_FAR_PARK),

                        new InstantCommand(() -> autoEndX = follower.follower.getPose().getX()),
                        new InstantCommand(() -> autoEndY = follower.follower.getPose().getY()),
                        new InstantCommand(() -> autoEndH = follower.follower.getPose().getHeading()),
                        new InstantCommand(() -> teleOpTargetX = BLUE_TARGET_X),
                        new InstantCommand(() -> teleOpTargetY = BLUE_TARGET_Y),
                        new InstantCommand(() -> distance = 0),
                        new InstantCommand(this::stop)
                )
        );
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        follower.breakFollowing();
        autoEndX = follower.follower.getPose().getX();
        autoEndY = follower.follower.getPose().getY();
        autoEndH = follower.follower.getPose().getHeading();
        teleOpTargetX = BLUE_TARGET_X;
        teleOpTargetY = BLUE_TARGET_Y;
    }
}
