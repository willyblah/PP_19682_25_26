package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCurrentToPoint;
import org.firstinspires.ftc.teamcode.commands.DrivePointToPoint;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.constants.BluePathChains;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class BLUE_CLOSE_21 extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    Robot robot = new Robot();
    double distance = 50;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(BLUE_CLOSE_START.copy());
        robot.autoInit(hardwareMap);
        telemetryM.update();
        TOTAL_SHOOT_TIME = 350;
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
        telemetryM.addData("posX:",follower.follower.getPose().getX());
        telemetryM.addData("posY:",follower.follower.getPose().getY());
        telemetryM.addData("heading:",follower.follower.getPose().getHeading());
        telemetryM.update();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.follower.update();
        draw();
        robot.shooter.setShooterByDis(distance);
        autoEndX = follower.follower.getPose().getX();
        autoEndY = follower.follower.getPose().getY();
        autoEndH = follower.follower.getPose().getHeading();
        teleOpTargetX = BLUE_TARGET_X;
        teleOpTargetY = BLUE_TARGET_Y;

        telemetryM.addData("posX:",follower.follower.getPose().getX());
        telemetryM.addData("posY:",follower.follower.getPose().getY());
        telemetryM.addData("heading:",follower.follower.getPose().getHeading());
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.follower.activateAllPIDFs();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // 发射预载三球
                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                        new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new DrivePointToPoint(follower, BLUE_CLOSE_START, BLUE_CLOSE_SHOOT_PRELOAD),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第一次收球: 收集第二组球
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower,
                                BLUE_CLOSE_INTAKE_SECOND_CONTROL,
                                BLUE_CLOSE_INTAKE_SECOND_END
                        ),
                        new WaitCommand(INTAKE_TIME), // 等待吸取时间
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        // 移动到发射位置
                        new ParallelCommandGroup(
                            new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                            new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                            new DriveCurrentToPoint(follower,
                                    BLUE_CLOSE_SHOOT_PRELOAD
                            )
                        ),
                        // 发射第二组球
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第一次闸门cycle
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_INTAKE_GATE_CONTROL, BLUE_CLOSE_INTAKE_GATE),
                        new WaitCommand(INTAKE_TIME),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        BLUE_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第二次闸门cycle
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_INTAKE_GATE_CONTROL, BLUE_CLOSE_INTAKE_GATE),
                        new WaitCommand(INTAKE_TIME),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        BLUE_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第三次闸门cycle
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_INTAKE_GATE_CONTROL, BLUE_CLOSE_INTAKE_GATE),
                        new WaitCommand(INTAKE_TIME),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        BLUE_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第四次闸门cycle
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_INTAKE_GATE_CONTROL, BLUE_CLOSE_INTAKE_GATE),
                        new WaitCommand(INTAKE_TIME),
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        BLUE_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第一组球
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower,
                                BLUE_CLOSE_INTAKE_FIRST_CONTROL,
                                BLUE_CLOSE_INTAKE_FIRST_END
                        ),
                        new WaitCommand(INTAKE_TIME), // 等待吸取时间
                        new InstantCommand(() -> robot.intake.intakeStop()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        BLUE_CLOSE_SHOOT_PRELOAD
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 停靠
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_PARK),

                        new WaitCommand(100),
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