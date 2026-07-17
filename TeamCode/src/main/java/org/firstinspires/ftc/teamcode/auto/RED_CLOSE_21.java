package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCurrentToPoint;
import org.firstinspires.ftc.teamcode.commands.DrivePointToPoint;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class RED_CLOSE_21 extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    Robot robot = new Robot();
    double distance = 50;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(RED_CLOSE_START.copy());
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
        teleOpTargetX = RED_TARGET_X;
        teleOpTargetY = RED_TARGET_Y;

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
                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new DrivePointToPoint(follower, RED_CLOSE_START, RED_CLOSE_SHOOT_PRELOAD),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第一次收球: 收集第二组球
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new ParallelDeadlineGroup(
                                new WaitCommand(1500), // 等待吸取时间
                                new DriveCurrentToPoint(follower,
                                    RED_CLOSE_INTAKE_SECOND_CONTROL,
                                    RED_CLOSE_INTAKE_SECOND_END
                                )
                        ),

//                        new WaitCommand(INTAKE_TIME), // 等待吸取时间

                        // 移动到发射位置
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                    RED_CLOSE_SHOOT_PRELOAD)
                        ),

                        // 发射第二组球
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第一次闸门cycle
                        new ParallelCommandGroup(
                            new InstantCommand(() -> robot.intake.intakeIn()),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(2200),
                                    new DriveCurrentToPoint(follower,
                                            RED_CLOSE_INTAKE_GATE_CONTROL,
                                            RED_CLOSE_INTAKE_GATE)
                                    )
                        ),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        RED_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),

                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第二次闸门cycle
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeIn()),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new DriveCurrentToPoint(follower,
                                                RED_CLOSE_INTAKE_GATE_CONTROL,
                                                RED_CLOSE_INTAKE_GATE)
                                )
                        ),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        RED_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),

                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第三次闸门cycle
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeIn()),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2300),
                                        new DriveCurrentToPoint(follower,
                                                RED_CLOSE_INTAKE_GATE_CONTROL,
                                                RED_CLOSE_INTAKE_GATE)
                                )
                        ),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        RED_CLOSE_SHOOT_INTAKE_GATE
                                )
                        ),

                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 第四次闸门cycle
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeIn()),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new DriveCurrentToPoint(follower,
                                                RED_CLOSE_INTAKE_GATE_CONTROL,
                                                RED_CLOSE_INTAKE_GATE)
                                )
                        ),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new DriveCurrentToPoint(follower,
                                        RED_CLOSE_SHOOT_INTAKE_GATE
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
                                RED_CLOSE_INTAKE_FIRST_CONTROL,
                                RED_CLOSE_INTAKE_FIRST_END
                        ),
                        new WaitCommand(300), // 等待吸取时间
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.intakeStop()),
                                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                                new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                                new InstantCommand(() -> robot.intake.gateOpen()),
                                new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                                new DriveCurrentToPoint(follower,
                                        RED_CLOSE_SHOOT_PRELOAD
                                )
                        ),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // 停靠
                        new DriveCurrentToPoint(follower, RED_CLOSE_PARK),

                        new WaitCommand(100),
                        new InstantCommand(() -> autoEndX = follower.follower.getPose().getX()),
                        new InstantCommand(() -> autoEndY = follower.follower.getPose().getY()),
                        new InstantCommand(() -> autoEndH = follower.follower.getPose().getHeading()),
                        new InstantCommand(() -> teleOpTargetX = RED_TARGET_X),
                        new InstantCommand(() -> teleOpTargetY = RED_TARGET_Y),
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
        teleOpTargetX = RED_TARGET_X;
        teleOpTargetY = RED_TARGET_Y;
    }
}