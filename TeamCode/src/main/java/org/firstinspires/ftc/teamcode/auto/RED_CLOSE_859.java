package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.AUTO_CLOSE_WAIT_FOR_SHOOT;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.CLOSE_FIRE_DISTANCE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.CLOSE_FIRE_DISTANCE_LAST;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.CLOSE_HOLD_DISTANCE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_FIRE_TURRET;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_FIRE_TURRET_LAST;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_INTAKE_THIRD_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_SHOOT_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_SHOOT_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_SHOOT_SECOND_ROW;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_SHOOT_THIRD_ROW_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_CLOSE_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.TOTAL_SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.RED_TARGET_X;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.RED_TARGET_Y;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.constants.RedPathChains;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class RED_CLOSE_859 extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    Robot robot = new Robot();
    RedPathChains redPathChains;
    double distance = 50;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(RED_CLOSE_START.copy());
        robot.autoInit(hardwareMap);
        telemetryM.update();
        redPathChains = new RedPathChains(follower.follower);
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
        telemetryM.update();
    }

    @Override
    public void start() {
        follower.follower.activateAllPIDFs();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // shoot preload
                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new DrivePointToPoint(follower, RED_CLOSE_START, RED_CLOSE_SHOOT_PRELOAD),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // intake second row
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_SECOND_ROW_CHAIN()),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new InstantCommand(() -> robot.intake.intakeIn()),
                                        new WaitCommand(1900),
                                        new InstantCommand(() -> robot.intake.intakeStop()),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE)
                                )
                        ),

                        // shoot second row
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // open gate for intake 1
                        new ParallelCommandGroup(
                                new WaitCommand(3200),
                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_GATE_CHAIN(), true),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new InstantCommand(() -> robot.intake.intakeIn())
                                )
                        ),

                        // shoot gate intake 1
                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                        new ParallelCommandGroup(
                                new DriveCurrentToPoint(follower, RED_CLOSE_SHOOT_CONTROL, RED_CLOSE_SHOOT_INTAKE_GATE),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new InstantCommand(() -> robot.intake.intakeStop())
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

//                        // open gate for intake 2
//                        new ParallelCommandGroup(
//                                new WaitCommand(3200),
//                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_GATE_CHAIN(), true),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new InstantCommand(() -> robot.intake.intakeIn())
//                                )
//                        ),
//
//                        // shoot gate intake 2
//                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
//                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
//                        new ParallelCommandGroup(
//                                new DriveCurrentToPoint(follower, RED_CLOSE_SHOOT_CONTROL, RED_CLOSE_SHOOT_INTAKE_GATE),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(150),
//                                        new InstantCommand(() -> robot.intake.intakeStop())
//                                )
//                        ),
//                        new InstantCommand(() -> robot.intake.gateOpen()),
//                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
//                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
//                        new WaitCommand(TOTAL_SHOOT_TIME),
//                        new InstantCommand(() -> robot.intake.gateClose()),
//                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

//                        new ParallelCommandGroup(
//                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_THIRD_ROW_CHAIN(), true),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(800),
//                                        new InstantCommand(() -> robot.intake.intakeIn()),
//                                        new WaitCommand(2000),
//                                        new InstantCommand(() -> robot.intake.intakeStop())
//                                )
//                        ),

                        new DriveCurrentToPoint(follower, RED_CLOSE_SHOOT_THIRD_ROW_CONTROL, RED_CLOSE_INTAKE_THIRD_START),
                        new InstantCommand(() -> robot.intake.intakeIn()),
                        new DriveCurrentToPoint(follower, RED_CLOSE_INTAKE_THIRD_END),
                        new InstantCommand(() -> robot.intake.intakeStop()),

                        new ParallelCommandGroup(
                                new DriveCurrentToPoint(follower, RED_CLOSE_SHOOT_THIRD_ROW_CONTROL, RED_CLOSE_SHOOT_INTAKE_GATE),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> robot.intake.gateOpen())
                                )
                        ),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),


                        // open gate for intake 3
                        new ParallelCommandGroup(
                                new WaitCommand(3200),
                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_GATE_CHAIN(), true),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new InstantCommand(() -> robot.intake.intakeIn())
                                )
                        ),

                        // shoot gate intake 3
                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET)),
                        new ParallelCommandGroup(
                                new DriveCurrentToPoint(follower, RED_CLOSE_SHOOT_CONTROL, RED_CLOSE_SHOOT_INTAKE_GATE),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new InstantCommand(() -> robot.intake.intakeStop())
                                )
                        ),
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

                        // intake 1st row
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower.follower, redPathChains.getCLOSE_INTAKE_FIRST_ROW_CHAIN()),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE_LAST),
                                        new InstantCommand(() -> robot.shooter.turretToDegree(RED_CLOSE_FIRE_TURRET_LAST)),
                                        new InstantCommand(() -> robot.intake.intakeIn())
                                )
                        ),
                        new InstantCommand(() -> robot.intake.intakeStop()),

                        // shoot 1st row
                        new InstantCommand(() -> robot.intake.gateOpen()),
                        new WaitCommand(AUTO_CLOSE_WAIT_FOR_SHOOT),
                        new InstantCommand(() -> robot.intake.intakeFire(robot.shooter.calculateIntakePower())),
                        new WaitCommand(TOTAL_SHOOT_TIME),
                        new InstantCommand(() -> robot.intake.gateClose()),
                        new InstantCommand(() -> distance = CLOSE_HOLD_DISTANCE),

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