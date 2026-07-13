package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_ENCODER;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * ============================================================================
 * AutoAimSubsystem —— 完整版：转塔自瞄 + 飞轮转速/浮板角度解算 + 移动发射预判
 * ============================================================================
 *
 * 【这个类整体在做什么】
 * 每个循环调用一次 update()，输入"机器人当前在哪、朝哪、走多快"+"目标点在哪"，
 * 输出"转塔该转多少度(并且这个类自己会把转塔实际转过去)"+"飞轮该开多快转速
 * (targetRpm，需要调用方自己拿去喂给Shooter)"+"浮板该摆多少角度(这个类自己
 * 会把浮板舵机转过去，不需要调用方管)"。
 *
 * 整个计算链条大致是：
 *   1. 读取机器人位姿/速度 → 做滤波(减少里程计噪声)
 *   2. 如果开了"移动发射"(isShootOnTheMove)，预测"再过 flightTime 秒球飞到目标时，
 *      机器人(以及球)会移动到哪个位置" → 拿这个"未来位置"去解算瞄准点，而不是
 *      拿"当前位置"解算，这样才能在移动中打准
 *   3. 用预测出来的落点距离，查表(三次多项式拟合)算出飞轮转速+浮板角度
 *   4. 用 atan2 算出目标点的绝对方位角，转成转塔该转的相对角度
 *   5. 检查这个角度有没有超出转塔物理安全限位，超了就放弃瞄准、转回中心
 *   6. 用前馈(kS静摩擦/kV速度/kA加速度)+PID位置环，把转塔实际驱动过去
 *
 * 【调用说明——怎么在 TeleOp 里用这个类】
 *   1. 声明一个实例：`AutoAimSubsystem autoAim = new AutoAimSubsystem();`
 *   2. `runOpMode()` 里、`waitForStart()` 之前调用一次：`autoAim.init(hardwareMap);`
 *      （这一步会去拿转塔电机、浮板舵机的硬件引用，并把浮板转到 PANEL_MIN 的初始位置）
 *   3. 主循环 `while (opModeIsActive())` 里，每一帧都调用一次 `update(...)`，
 *      参数说明：
 *        robotX, robotY           —— 机器人当前场地坐标(英寸)，从里程计(PinPoint)读
 *        globalVx, globalVy       —— 机器人当前"场地坐标系"下的速度分量(英寸/秒)，
 *                                     PinPoint的getVelX()/getVelY()
 *        currentHeadingDeg        —— 机器人当前朝向(度)
 *        robotAngularVelocityDeg  —— 机器人当前角速度(度/秒)，PinPoint的getHeadingVelocity()
 *        targetX, targetY         —— 要瞄准的目标点(球门)场地坐标(英寸)，这里传的是"静止的
 *                                     目标点本身"，不需要调用方自己再算什么提前量——
 *                                     移动预判这个类内部自己会算(通过 flightTime 那一套)
 *        isManualMode, manualDist —— 调试/台测用：不走里程计解算距离，直接用 manualDist
 *                                     这个固定值去查表算转速/浮板角度(转塔仍然会正常瞄准)
 *        isShootOnTheMove         —— 是否启用"移动中发射"的这套预判(chassis滤波+刹车预测+
 *                                     飞行时间提前量)，对应操作手的一个按键开关
 *        isBraking                —— 是否处于"正在刹车/摇杆已经回中"的状态，配合
 *                                     isShootOnTheMove使用，只有为true时才会启用"预判会
 *                                     刹停"这个假设，否则按当前测得的加速度直接外推
 *        yawOffset                —— 操作手方向键手动微调的角度偏移(度)
 *   4. 用返回的 `TurretCommand` 结果：
 *        command.hasTarget       —— 是否有有效瞄准解(距离太近/角度超限位都会是false)
 *        command.targetRpm       —— 算出来的飞轮目标转速，需要调用方自己
 *                                     `shooter.setShooterVelocity(command.targetRpm)`
 *        command.targetPitch     —— 浮板目标角度，⚠️不需要调用方自己设置，这个类内部
 *                                     update()/disable分支里已经自己调 setPitchServo()
 *                                     把浮板转过去了，command.targetPitch只是给你看/打
 *                                     遥测用的
 *        command.isAimLocked    —— 误差是否已经收敛到容差以内，可以用来判断"能不能开火"
 *        command.targetDist / flightTime / targetTurretAngle / aimError —— 都是调试用的
 *          中间量，正常逻辑不需要用到，打在遥测里方便排查问题
 *   5. `stop()` 在 OpMode 结束前调用一次：转塔停转，并把所有滤波器的初始化标记复位
 *      （避免下次进OpMode时滤波器还带着上一局的陈旧数据）。
 *
 * 【调试指南】
 *
 * ▶ 转塔抖动/震荡：
 *   最常见的原因排查顺序：
 *   1. 先看 `getFilteredTurretVelocity()` 是不是在目标角度附近来回正负跳变——
 *      如果是，大概率是 kV/kS/kA 或者 TURRET_kP 没标定好，前馈+PID打架。
 *   2. 再看 `command.targetTurretAngle` 是不是频繁在 TURRET_MIN/MAX_SAFE_ANGLE
 *      边界附近来回蹦——如果是，说明安全限位卡得太紧，瞄准解算频繁"够得到/够不到"反复横跳，
 *      这种情况下应该放宽安全限位(前提是转塔线材允许)，而不是调PID。
 *   3. ⚠️ 已知问题：本文件第456行 `driveTurretToAngle()` 里的
 *      `Range.clip(targetAngle, -TURRET_FULL_RANGE_DEGREE, TURRET_FULL_RANGE_DEGREE)`
 *      用的是 TURRET_FULL_RANGE_DEGREE(编码器换算比例常量)，而不是
 *      TURRET_MIN_SAFE_ANGLE/TURRET_MAX_SAFE_ANGLE(真正的安全限位)。如果你只改了
 *      安全限位、没改这一行，外层判断放宽/收紧了，这里还是会按旧的换算比例把角度夹一遍，
 *      两处不一致，是个隐藏的耦合bug，改安全限位之前务必先看看这行要不要一起改。
 *
 * ▶ 移动发射(shoot-on-the-move)打不准：
 *   1. 先确认 isShootOnTheMove 有没有真的传 true 进来(遥测里加一行打印确认)。
 *   2. 看 `getLastFlightTime()` 是否落在 [MIN_FLIGHT_TIME, MAX_FLIGHT_TIME] 合理区间，
 *      如果一直卡在边界值，说明 BASE_FLIGHT_TIME/FLIGHT_TIME_PER_INCH 这套线性模型
 *      跟实际弹道差太远，需要重新拟合。
 *   3. `isBraking` 判断不准会导致预判方向反了(明明在加速，却按"要刹车"去预判减速)，
 *      检查这个布尔值的判断逻辑(通常是摇杆量是否接近0)是否符合预期。
 *   4. FORWARD_BRAKE_DECEL / LATERAL_BRAKE_DECEL 这两个"假设的刹车减速度"是拍脑袋
 *      给的初始值还是实测的？不对的话移动发射提前量会系统性偏多或偏少。
 *
 * ▶ 飞轮转速/浮板角度不对：
 *   注意 `AimCalculator.interpolate()` 里的两套三次多项式系数，是这个类自己独立
 *   拟合的一套，**跟 `Shooter.java` 里 `f(a,b,c,d,distance)` 用的系数不是同一套**，
 *   两边如果都在用、又没同步更新，会出现"同样距离、两套代码打出不同转速"的情况。
 *   确认清楚当前项目到底是用这个类里的插值、还是 Shooter.java 里那套在实际生效。
 *
 * ▶ 转塔追踪的绝对角度总是跟实际差一截：
 *   检查 `smoothHeading` 的滤波是不是没跟上实际朝向——比如某段时间没有持续调用 update()
 *   (被跳过/暂停)，`smoothHeading` 会失去同步，下次调用时会有一次性的角度纠偏冲击。
 */
@Configurable
public class AutoAimSubsystem {
    private DcMotorEx turret;      // 转塔电机
    private Servo panel;           // 浮板(抬升角度)舵机
    private HardwareMap hardwareMap;
    private PIDFController turretPIDF;   // 转塔角度位置环(ftclib的PIDF控制器)

    // ========================================================================
    // 转塔前馈系数——由 TurretFullAutoTuner 自动标定出来的，含义：
    //   kS: 静摩擦补偿(多大功率能让转塔刚好开始动)
    //   kV: 速度前馈(稳态时"功率-速度"线性关系的斜率)
    //   kA: 加速度前馈(转塔的等效转动惯量)
    //   kLinearBraking / kQuadraticFriction: 转塔自身刹车距离模型的线性/二次项系数，
    //     用来预判"转塔冲过目标会不会刹不住车"，注释里的旧值是标定前的默认值
    // ========================================================================
    public static double TURRET_kP = 30.0;   // 位置环比例增益(误差角度→速度指令)
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.0;
    public static double TURRET_kF = 0.0;
    public static double TURRET_kV = 0.001339; // 0.001394;
    public static double TURRET_kS = 0.024370; // 0.03;
    public static double TURRET_kA = 0.000007; // 0.000069;
    public static double TURRET_kLinearBraking = 0.069085; // 0.026905;
    public static double TURRET_kQuadraticFriction = 0.0; // 0.000078;

    public static double TURRET_POWER_SIGN = 1.0;    // 电机转向是否需要反相(标定得出，1或-1)
    public static double TURRET_LATENCY = 0.012;     // 系统延迟补偿(秒)，用于移动目标角速度超前量
    public static double TURRET_MAX_POWER = 1.0;     // 转塔功率上限(0~1)
    public static double TURRET_FILTER_ALPHA = 0.8;         // 转塔角度低通滤波系数(越大越跟手、越小越平滑抗噪)
    public static double TURRET_VEL_FILTER_ALPHA = 0.8;     // 转塔速度低通滤波系数
    public static double TURRET_CMD_ACCEL_FILTER_ALPHA = 0.15; // 目标加速度低通滤波系数(给kA前馈用，避免噪声放大)
    public static double TUNING_VOLTAGE = 13.84;    // 标定前馈系数时的电池电压，运行时按当前电压做补偿

    // ========================================================================
    // 底盘速度滤波 + 刹车预判系数——只在"移动发射"(isShootOnTheMove)开启时生效
    // ========================================================================
    public static double CHASSIS_VEL_FILTER_ALPHA = 0.3;    // 底盘前向/侧向速度低通滤波系数
    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.4;  // 底盘前向/侧向加速度低通滤波系数
    public static double BRAKE_PREDICTION_WEIGHT = 0.55;    // 刹车预判权重：0=完全信任实测加速度，1=完全信任理论刹车减速度
    public static double FORWARD_BRAKE_DECEL = 33.09567766; // 假设/实测的底盘前向刹车减速度(英寸/秒²)
    public static double LATERAL_BRAKE_DECEL = 52.88478403; // 假设/实测的底盘侧向刹车减速度(英寸/秒²)
    public static double OMEGA_FILTER_ALPHA = 0.7;          // 机器人角速度低通滤波系数
    public static double HEADING_CORRECTION_ALPHA = 0.5;    // 朝向滤波纠偏系数(把积分外推的朝向拉回实测朝向的力度)

    // ========================================================================
    // 浮板行程限位 + 转塔安全限位
    // ========================================================================
    public static double PANEL_MIN = 0.2;    // 浮板舵机位置下限(0~1)
    public static double PANEL_MAX = 0.59;   // 浮板舵机位置上限(0~1)
    public static double TURRET_MIN_SAFE_ANGLE = -175.0;// -175   转塔安全限位(度)，超出这个范围就放弃瞄准、转回中心
    public static double TURRET_MAX_SAFE_ANGLE = 175.0; // 175    —— 务必按实测的线材缠绕极限来填，留几度余量
    public static double TURRET_CENTER_ANGLE = 0.0;     // 转塔"归中"时的目标角度
    public static boolean RESET_TURRET_ENCODER_ON_INIT = false; // init()时是否把当前位置清零成0度(转塔物理归中后再开机时用)

    // ========================================================================
    // 移动发射的飞行时间模型——线性近似："距离越远，球飞得越久"
    // ========================================================================
    public static double BASE_FLIGHT_TIME = 0.4;       // 飞行时间基础值(秒)，对应距离=0时的理论飞行时间(含发射延迟等)
    public static double FLIGHT_TIME_PER_INCH = 0.00575; // 每英寸距离增加的飞行时间(秒/英寸)，线性拟合斜率
    public static double MIN_FLIGHT_TIME = 0.35;       // 飞行时间下限(秒)，防止近距离算出不合理的极小值
    public static double MAX_FLIGHT_TIME = 1.25;       // 飞行时间上限(秒)，防止远距离算出不合理的极大值
    public static double MECHANICAL_SHOOT_DELAY = 0.0; // 机械结构本身的发射延迟(秒)，比如喂球到出膛的滞后
    public static double SHOT_DISTANCE_OFFSET = 0.0;   // 距离修正(英寸)，对应操作手方向键trim

    // ---------- 转塔角度/速度滤波状态(运行时内部使用，不需要外部关心) ----------
    private double initialTurretOffset = 0.0;         // 转塔角度零点偏移(一般不用，配合非零起始角度用)
    private double filteredTurretRelAngle = 0.0;      // 低通滤波后的转塔相对角度(度)
    private boolean isTurretFilterInitialized = false; // 滤波器是否已经用第一帧数据初始化过
    private double lastTurretRelAngle = 0.0;          // 上一帧的原始转塔角度，用于差分算速度
    private double filteredTurretVel = 0.0;           // 低通滤波后的转塔角速度(度/秒)
    private double lastTargetVel = 0.0;               // 上一帧PID+前馈算出的目标速度，用于差分算目标加速度
    private double filteredTargetAccel = 0.0;         // 低通滤波后的目标加速度(给kA前馈用)
    private long lastTime = 0;                        // 上一次调用的系统时间(纳秒)，算dt用
    private long lastVoltageReadTime = 0;              // 上一次读电池电压的时间戳(电压读取有节流，不用每帧都读)
    private double currentBatteryVoltage = 12.0;       // 当前电池电压缓存值，用于电压补偿

    // ---------- 底盘速度滤波状态(移动发射用) ----------
    private boolean isChassisFilterInitialized = false;
    private double filteredForward = 0.0;    // 低通滤波后的"机器人朝向前方"的速度分量(英寸/秒)
    private double filteredLateral = 0.0;    // 低通滤波后的"机器人侧向"的速度分量(英寸/秒)
    private double filteredForwardAccel = 0.0;
    private double filteredLateralAccel = 0.0;
    private double lastFilteredForward = 0.0;
    private double lastFilteredLateral = 0.0;
    private double filteredRobotOmega = 0.0;   // 低通滤波后的机器人角速度(度/秒)
    private double smoothHeading = 0.0;        // 平滑后的机器人朝向(度)——用角速度积分外推+朝向纠偏融合得到
    private boolean isSmoothHeadingInit = false;

    // ---------- 上一帧的调试用中间量，配合get方法给外部打遥测 ----------
    private double lastTargetRelAngle = 0.0;
    private double lastAimError = 0.0;
    private double lastTargetAbsAngle = 0.0;
    private double lastFlightTime = 0.0;

    /** 每次 update() 返回的完整瞄准+射击参数结果。 */
    public static class TurretCommand {
        public boolean hasTarget = false;      // 是否有有效瞄准解(距离太近/超出安全限位都会是false)
        public double targetRpm = 0.0;         // 飞轮目标转速——调用方需要自己拿去喂给Shooter
        public double targetPitch = 0.0;       // 浮板目标角度——仅供参考/打遥测，实际浮板已经被这个类自己转过去了
        public boolean isAimLocked = false;    // 瞄准误差是否已收敛到容差以内
        public double currentTolerance = 1.0;  // 当前距离对应的容差(度)，距离越近容差越宽
        public double targetDist = 0.0;        // 解算出的目标距离(英寸)
        public double targetTurretAngle = 0.0; // 转塔目标相对角度(度)
        public double aimError = 0.0;          // 转塔角度误差(度)
        public double flightTime = 0.0;        // 本次解算用的飞行时间(秒)
    }

    /**
     * ⚠️ 内部私有的飞轮转速/浮板角度插值计算器。
     * 注意：这里的三次多项式系数是这个类自己独立拟合的一套，
     * 跟 Shooter.java 里 f(a,b,c,d,distance) 用的系数完全不一样，
     * 两边如果都在实际使用、又没同步更新，会出现"同样距离两套代码打出不同结果"的问题，
     * 使用前务必确认清楚项目里到底是这一套在生效、还是Shooter.java那一套。
     */
    private static class AimCalculator {
        /** 一次瞄准解算的完整结果（距离、绝对方位角、转速、浮板角度、飞行时间）。 */
        private static class AimResult {
            final double dist;
            final double algYaw;
            final double rpm;
            final double pitch;
            final double flightTime;

            AimResult(double dist, double algYaw, double rpm, double pitch, double flightTime) {
                this.dist = dist;
                this.algYaw = algYaw;
                this.rpm = rpm;
                this.pitch = pitch;
                this.flightTime = flightTime;
            }
        }

        /**
         * 根据"预测的未来机器人位置"和目标点，解算距离/方位角/转速/浮板角度。
         * 距离小于0.1英寸(几乎重合)时返回null，代表"没法瞄准"(比如就在目标点正上方)。
         */
        static AimResult solveAim(double futureRobotX, double futureRobotY, double targetX, double targetY, double flightTime) {
            double dx = targetX - futureRobotX;
            double dy = targetY - futureRobotY;
            double dist = Math.hypot(dx, dy);
            if (dist < 0.1) {
                return null;
            }
            double shotDist = Math.max(0.0, dist + SHOT_DISTANCE_OFFSET);
            return new AimResult(
                    dist,
                    Math.toDegrees(Math.atan2(dy, dx)),
                    interpolate(shotDist, 1),
                    interpolate(shotDist, 2),
                    flightTime
            );
        }

        /** type=1 查转速多项式，其他值查浮板角度多项式(并clip到PANEL_MIN~MAX范围)。 */
        static double interpolate(double dist, int type) {
            if (type == 1) {
                return f(-0.0004276526, 0.1345118, -5.930571, 1286.731, dist);
            }
            return Range.clip(f(0.0, 0.000002082898, 0.003827418, 0.05630374, dist), PANEL_MIN, PANEL_MAX);
        }

        /** 标准三次多项式 a*x³ + b*x² + c*x + d。 */
        private static double f(double a, double b, double c, double d, double x) {
            return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
        }
    }

    /** 空构造：需要之后手动调用 init(hardwareMap)。 */
    public AutoAimSubsystem() {
    }

    /** 构造时直接初始化硬件(等价于 new + init 两步合成一步)。 */
    public AutoAimSubsystem(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    /**
     * 硬件初始化，在 runOpMode() 里 waitForStart() 之前调用一次。
     * 会拿转塔电机+浮板舵机的硬件引用，配置转塔电机方向/模式/零功率行为，
     * 创建PID控制器，并把浮板转到 PANEL_MIN 的初始位置。
     */
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RESET_TURRET_ENCODER_ON_INIT) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // 把当前物理位置清零成0度
        }
        // 用 RUN_WITHOUT_ENCODER 而不是电机固件自带的 RUN_TO_POSITION——
        // 转塔怎么转、怎么减速全部交给下面自己写的前馈+PID控制(driveTurretToAngle)
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panel = hardwareMap.get(Servo.class, PANEL);
        currentBatteryVoltage = getBatteryVoltage();
        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
        setPitchServo(PANEL_MIN);
    }

    /** 读取所有电压传感器里的最大值作为当前电池电压，供电压补偿使用，下限8V防止读数异常。 */
    private double getBatteryVoltage() {
        double maxVoltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
            }
        }
        return Math.max(8.0, maxVoltage > 0.0 ? maxVoltage : TUNING_VOLTAGE);
    }

    // ==================== 以下都是给外部(TeleOp)打遥测/调试用的只读方法 ====================

    public double getCurrentBatteryVoltage() {
        return currentBatteryVoltage;
    }

    /** 转塔编码器原始换算角度(没滤波)。 */
    public double getCurrentTurretAngle() {
        return getTurretRelativeAngle();
    }

    /** 转塔滤波后的角度——正常应该跟"编码器原始角度"很接近，如果差很多说明滤波系数/编码器有问题。 */
    public double getFilteredTurretAngle() {
        return filteredTurretRelAngle;
    }

    /** 转塔滤波后的角速度(度/秒)——排查抖动时重点看这个是不是在来回跳变。 */
    public double getFilteredTurretVelocity() {
        return filteredTurretVel;
    }

    public double getLastTargetTurretAngle() {
        return lastTargetRelAngle;
    }

    /** 上一帧的瞄准误差(度)——aimLocked为true时这个应该很接近0。 */
    public double getLastAimError() {
        return lastAimError;
    }

    public double getLastTargetAbsAngle() {
        return lastTargetAbsAngle;
    }

    /** 上一帧解算用的飞行时间(秒)——移动发射调试时重点看这个是否在合理区间。 */
    public double getLastFlightTime() {
        return lastFlightTime;
    }

    /** 把浮板舵机转到指定位置，自动clip在[PANEL_MIN, PANEL_MAX]范围内，防止意外指令把浮板打坏。 */
    private void setPitchServo(double targetPitch) {
        panel.setPosition(Range.clip(targetPitch, PANEL_MIN, PANEL_MAX));
    }

    /** 编码器原始ticks换算成角度(度)，换算比例来自 robotConstants.TURRET_FULL_RANGE_ENCODER/DEGREE。 */
    private double getTurretRelativeAngle() {
        return turret.getCurrentPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE + initialTurretOffset;
    }

    /**
     * 预测"刹车中"的加速度：如果 isBraking=true 且当前有速度，就不完全相信实测加速度
     * (实测加速度可能噪声大/滞后)，而是按 BRAKE_PREDICTION_WEIGHT 的权重去混合"理论刹车
     * 减速度"(方向跟速度相反，大小固定为brakeDecel)。isBraking=false时直接用实测加速度。
     */
    private double predictAxisAccel(double velocity, double measuredAccel, boolean isBraking, double brakeDecel) {
        if (!isBraking || Math.abs(velocity) <= 0.0001) {
            return measuredAccel;
        }
        double brakeAccel = -Math.signum(velocity) * brakeDecel;
        return measuredAccel * (1.0 - BRAKE_PREDICTION_WEIGHT) + brakeAccel * BRAKE_PREDICTION_WEIGHT;
    }

    /**
     * 防止"预测值"把速度方向预测反了(比如实际在往正方向减速到0，预测值却算出穿过0变成负数)——
     * 如果当前值和预测值符号相反(说明预测"冲过头到反方向了")，强制钳位到0，更符合物理直觉
     * (刹车顶多刹到0，不会自己变成反向加速)。
     */
    private double stopAtZero(double currentValue, double predictedValue) {
        if (currentValue > 0.0 && predictedValue < 0.0) {
            return 0.0;
        }
        if (currentValue < 0.0 && predictedValue > 0.0) {
            return 0.0;
        }
        return predictedValue;
    }

    /** 把任意角度差归一化到 (-180, 180] 区间，保证走的是"最短路径"夹角。 */
    private double normalizeDeg(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * 每个循环调用一次，具体每个参数的含义见类顶部的"调用说明"。
     * 副作用：这个方法内部会直接驱动转塔电机(turret.setPower)和浮板舵机(panel.setPosition)，
     * 不需要调用方再额外操作硬件——除了 command.targetRpm 需要调用方自己喂给 Shooter。
     */
    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy,
            double currentHeadingDeg, double robotAngularVelocityDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist,
            boolean isShootOnTheMove, boolean isBraking, double yawOffset) {

        // PIDF系数每帧都重新set一遍，这样如果在FTC Dashboard上实时改了kP/kI/kD/kF，
        // 不用重新部署代码/重启OpMode就能立刻生效
        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
        TurretCommand command = new TurretCommand();

        // 电压读取有节流(每250ms才读一次)，避免每帧都去查所有电压传感器浪费时间
        long currentTimeForVolts = System.nanoTime();
        if (currentTimeForVolts - lastVoltageReadTime > 250_000_000L) {
            currentBatteryVoltage = getBatteryVoltage();
            lastVoltageReadTime = currentTimeForVolts;
        }

        // 算这一帧距上一帧过了多久(dt)，所有微分/积分类的滤波都要用到
        long currentTimeNanos = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (currentTimeNanos - lastTime) / 1e9;
        lastTime = currentTimeNanos;

        updateTurretFilters(dt);   // 转塔角度/速度滤波

        updateHeadingFilter(currentHeadingDeg, robotAngularVelocityDeg, dt);       // 机器人朝向滤波
        updateChassisFilters(globalVx, globalVy, isShootOnTheMove, dt);           // 底盘前向/侧向速度滤波

        // 把机器人坐标系下的"前向/侧向"速度分量，转换回场地坐标系下的(x,y)速度分量，
        // 后面预测"未来位置"要用场地坐标系
        double headingRad = Math.toRadians(smoothHeading);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double effectiveFieldVx = filteredForward * cosH - filteredLateral * sinH;
        double effectiveFieldVy = filteredForward * sinH + filteredLateral * cosH;

        AimCalculator.AimResult aimResult;
        if (isManualMode) {
            // 台测/调试模式：不用里程计算距离，直接拿 manualDist 这个固定值去查表算转速/浮板角度，
            // 转塔仍然会按 smoothHeading 正常瞄准(这里传smoothHeading当"方位角"，相当于瞄准正前方)
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, smoothHeading, manualRpm, manualPitch, 0.0);
        } else {
            // ---- 正常模式：先算飞行时间，再预测"球飞行期间机器人+球会移动到哪" ----
            double currentDistToTarget = Math.hypot(targetX - robotX, targetY - robotY);
            double flightTime = Range.clip(
                    BASE_FLIGHT_TIME + FLIGHT_TIME_PER_INCH * currentDistToTarget,
                    MIN_FLIGHT_TIME,
                    MAX_FLIGHT_TIME
            );

            // 只有开启"移动发射"才会启用刹车感知的加速度预测，否则预测加速度=0(不做任何提前量修正的前置准备)
            double predictedForwardAccel = 0.0;
            double predictedLateralAccel = 0.0;
            if (isShootOnTheMove) {
                predictedForwardAccel = predictAxisAccel(filteredForward, filteredForwardAccel, isBraking, FORWARD_BRAKE_DECEL);
                predictedLateralAccel = predictAxisAccel(filteredLateral, filteredLateralAccel, isBraking, LATERAL_BRAKE_DECEL);
            }

            // "发射延迟"期间(MECHANICAL_SHOOT_DELAY，喂球到出膛的滞后)机器人会继续移动/减速，
            // 先算出"发射那一刻"的速度(releaseForward/Lateral)，如果在刹车就钳位到0(不会反向)
            double releaseForward = filteredForward + predictedForwardAccel * MECHANICAL_SHOOT_DELAY;
            double releaseLateral = filteredLateral + predictedLateralAccel * MECHANICAL_SHOOT_DELAY;
            if (isBraking) {
                releaseForward = stopAtZero(filteredForward, releaseForward);
                releaseLateral = stopAtZero(filteredLateral, releaseLateral);
            }

            // 用"现在"和"发射那一刻"速度的平均值，估算发射延迟期间机器人实际挪动的距离(releaseX/Y)
            double avgForward = 0.5 * (filteredForward + releaseForward);
            double avgLateral = 0.5 * (filteredLateral + releaseLateral);
            double releaseX = robotX + (avgForward * cosH - avgLateral * sinH) * MECHANICAL_SHOOT_DELAY;
            double releaseY = robotY + (avgForward * sinH + avgLateral * cosH) * MECHANICAL_SHOOT_DELAY;

            // 再假设发射那一刻的速度在飞行时间(flightTime)内保持不变，匀速外推球最终落点对应的
            // "机器人+球系统"未来位置(futureX/Y)——这是整套移动发射预判的核心近似假设
            double releaseFieldVx = releaseForward * cosH - releaseLateral * sinH;
            double releaseFieldVy = releaseForward * sinH + releaseLateral * cosH;
            double futureX = releaseX + releaseFieldVx * flightTime;
            double futureY = releaseY + releaseFieldVy * flightTime;

            // 拿"未来位置"而不是"当前位置"去解算距离/转速/浮板角度/方位角，
            // 这样打出去的球才能在飞行时间内"追上"移动中的机器人该到的落点
            aimResult = AimCalculator.solveAim(futureX, futureY, targetX, targetY, flightTime);
        }

        // aimResult为null说明距离太近(<0.1英寸，几乎贴在目标点上)，没法算方位角，直接停转塔
        if (aimResult == null) {
            turret.setPower(0.0);
            return command;
        }

        command.hasTarget = true;
        command.targetRpm = aimResult.rpm;
        command.targetPitch = aimResult.pitch;
        command.targetDist = aimResult.dist;
        command.flightTime = aimResult.flightTime;
        lastFlightTime = aimResult.flightTime;

        // 距离越远，容差越小(要求瞄得更准才能开火)；距离越近，容差越宽松(角度差一点也无所谓)
        command.currentTolerance = calculateTolerance(aimResult.dist);

        // ---- 移动目标的角速度超前补偿：机器人在移动，目标点相对机器人的"视线方向"本身也在转 ----
        // 用叉乘算出"如果机器人保持当前有效场地速度不变，目标点方位角每秒会转多少度"(translationalOmegaDeg)，
        // 配合 TURRET_LATENCY(系统延迟)提前把这部分转速补偿进目标角度里，减少"打了但目标已经转过去了"的滞后
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distSq = dx * dx + dy * dy;
        double translationalOmegaDeg = 0.0;
        if (distSq > 0.001) {
            double omegaRad = (-dx * effectiveFieldVy + dy * effectiveFieldVx) / distSq;
            translationalOmegaDeg = Math.toDegrees(omegaRad);
        }

        // 目标绝对方位角 = 解算出的方位角 + 操作手手动微调 + 移动目标角速度超前量
        double compensatedTargetAbsAngle = aimResult.algYaw + yawOffset + translationalOmegaDeg * TURRET_LATENCY;
        // 转塔当前绝对朝向 = 机器人朝向 + 转塔相对角度
        double currentTurretAbsAngle = smoothHeading + filteredTurretRelAngle;
        // 归一化取最短路径夹角误差，再加到"转塔当前相对角度"上，得到"转塔该转到的新相对角度"
        double error = normalizeDeg(compensatedTargetAbsAngle - currentTurretAbsAngle);
        double targetTurretRelAngle = filteredTurretRelAngle + error;

        lastTargetAbsAngle = compensatedTargetAbsAngle;
        lastAimError = error;
        lastTargetRelAngle = targetTurretRelAngle;
        command.targetTurretAngle = targetTurretRelAngle;
        command.aimError = error;

        // ---- 安全限位检查：超出转塔物理安全范围就放弃瞄准，强制转回中心，浮板也收回最小位置 ----
        if (targetTurretRelAngle < TURRET_MIN_SAFE_ANGLE || targetTurretRelAngle > TURRET_MAX_SAFE_ANGLE) {
            command.hasTarget = false;
            command.isAimLocked = false;
            command.targetTurretAngle = TURRET_CENTER_ANGLE;
            driveTurretToAngle(filteredTurretRelAngle, TURRET_CENTER_ANGLE, -filteredRobotOmega, dt);
            setPitchServo(PANEL_MIN);
            return command;
        }

        command.isAimLocked = Math.abs(error) <= command.currentTolerance;

        // ---- 转塔自身的刹车距离预判：如果转塔正在往一个方向转、但目标要求反向修正 ----
        // (说明转塔"冲过头了"，需要往回转)，提前把"预计凭惯性还会多转的距离"算进当前角度里，
        // 让PID提前感知到"实际已经更接近/更超过目标"，从而更早开始减速/反向修正，避免真正冲过头再弹回来
        double predictedRelAngle = filteredTurretRelAngle;
        double brakingDist = TURRET_kLinearBraking * Math.abs(filteredTurretVel)
                + TURRET_kQuadraticFriction * filteredTurretVel * filteredTurretVel;
        if (Math.signum(error) != Math.signum(filteredTurretVel)) {
            predictedRelAngle += Math.signum(filteredTurretVel) * brakingDist;
        }

        // 最终驱动转塔：前馈速度里包含"抵消机器人自转"(-filteredRobotOmega)和"移动目标角速度超前"(translationalOmegaDeg)两部分
        driveTurretToAngle(predictedRelAngle, targetTurretRelAngle, -filteredRobotOmega + translationalOmegaDeg, dt);
        setPitchServo(command.targetPitch);
        return command;
    }

    /**
     * 转塔角度/速度低通滤波。第一次调用直接用原始值初始化(避免从0开始的假过渡)，
     * 之后每帧用指数滤波(EMA)平滑角度，并用差分算角速度再滤波一遍。
     */
    private void updateTurretFilters(double dt) {
        double rawTurretRelAngle = getTurretRelativeAngle();
        if (!isTurretFilterInitialized) {
            filteredTurretRelAngle = rawTurretRelAngle;
            lastTurretRelAngle = rawTurretRelAngle;
            filteredTurretVel = 0.0;
            lastTargetVel = 0.0;
            filteredTargetAccel = 0.0;
            isTurretFilterInitialized = true;
        } else {
            filteredTurretRelAngle = TURRET_FILTER_ALPHA * rawTurretRelAngle
                    + (1.0 - TURRET_FILTER_ALPHA) * filteredTurretRelAngle;
            if (dt > 0.0001) {
                double rawVel = (rawTurretRelAngle - lastTurretRelAngle) / dt;
                filteredTurretVel = TURRET_VEL_FILTER_ALPHA * rawVel
                        + (1.0 - TURRET_VEL_FILTER_ALPHA) * filteredTurretVel;
            }
        }
        lastTurretRelAngle = rawTurretRelAngle;
    }

    /**
     * 机器人朝向的"角速度积分外推 + 实测朝向纠偏"融合滤波。
     * 不是直接用瞬时朝向(会有跳变噪声)，而是让 smoothHeading 按滤波后的角速度自己积分往前走，
     * 再用"实测朝向-积分朝向"的误差按 HEADING_CORRECTION_ALPHA 的比例往回拉，类似互补滤波。
     * ⚠️ 这个方法必须每帧持续调用才能保持准确——如果中途长时间不调用(比如被跳过)，
     * smoothHeading会跟实际朝向脱节，下次调用时会有一次性的较大纠偏冲击。
     */
    private void updateHeadingFilter(double currentHeadingDeg, double robotAngularVelocityDeg, double dt) {
        if (!isSmoothHeadingInit) {
            smoothHeading = currentHeadingDeg;
            filteredRobotOmega = robotAngularVelocityDeg;
            isSmoothHeadingInit = true;
            return;
        }

        filteredRobotOmega += OMEGA_FILTER_ALPHA * (robotAngularVelocityDeg - filteredRobotOmega);
        smoothHeading += filteredRobotOmega * dt;
        double headingError = normalizeDeg(currentHeadingDeg - smoothHeading);
        smoothHeading = normalizeDeg(smoothHeading + HEADING_CORRECTION_ALPHA * headingError);
    }

    /**
     * 把机器人场地坐标系速度(globalVx/Vy)转换成机器人自身坐标系下的"前向/侧向"分量并滤波。
     * isShootOnTheMove=false 时不滤波，直接用瞬时值(没开移动发射的时候没必要平滑，
     * 而且能避免"刚打开移动发射"那一帧还带着上次关闭时的滤波残留)。
     */
    private void updateChassisFilters(double globalVx, double globalVy, boolean isShootOnTheMove, double dt) {
        double headingRad = Math.toRadians(smoothHeading);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double rawForward = globalVx * cosH + globalVy * sinH;
        double rawLateral = -globalVx * sinH + globalVy * cosH;

        if (!isChassisFilterInitialized || !isShootOnTheMove) {
            filteredForward = rawForward;
            filteredLateral = rawLateral;
            filteredForwardAccel = 0.0;
            filteredLateralAccel = 0.0;
            lastFilteredForward = rawForward;
            lastFilteredLateral = rawLateral;
            isChassisFilterInitialized = true;
            return;
        }

        filteredForward += CHASSIS_VEL_FILTER_ALPHA * (rawForward - filteredForward);
        filteredLateral += CHASSIS_VEL_FILTER_ALPHA * (rawLateral - filteredLateral);
        if (dt > 0.0001) {
            double rawForwardAccel = (filteredForward - lastFilteredForward) / dt;
            double rawLateralAccel = (filteredLateral - lastFilteredLateral) / dt;
            filteredForwardAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawForwardAccel - filteredForwardAccel);
            filteredLateralAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawLateralAccel - filteredLateralAccel);
        }
        lastFilteredForward = filteredForward;
        lastFilteredLateral = filteredLateral;

    }

    /** 距离≤20英寸容差30度(很宽松，近距离角度差一点也能命中)，≥90英寸容差5度(远距离必须打得准)，中间线性插值。 */
    private double calculateTolerance(double dist) {
        if (dist <= 20.0) {
            return 30.0;
        }
        if (dist >= 90.0) {
            return 5.0;
        }
        return 30.0 + (5.0 - 30.0) / (90.0 - 20.0) * (dist - 20.0);
    }

    /**
     * 转塔的实际驱动函数：PID位置环 + 前馈(kS/kV/kA) + 电压补偿，最终算出功率下发给电机。
     *
     * ⚠️⚠️ 已知的隐藏耦合bug（调试指南里也提过，这里再标一次方便你直接定位）：
     * 下面这行 clip 用的是 TURRET_FULL_RANGE_DEGREE(编码器换算比例常量)，而不是
     * TURRET_MIN_SAFE_ANGLE / TURRET_MAX_SAFE_ANGLE(真正的安全限位)。正常情况下调用方
     * (update()方法里)已经提前用安全限位过滤过targetAngle了，所以这里目前不会触发出问题；
     * 但如果以后有别的调用路径绕过了外层的安全限位检查、直接调这个私有方法，这里的clip
     * 范围可能跟你以为的安全限位对不上，建议改成 Range.clip(targetAngle, TURRET_MIN_SAFE_ANGLE, TURRET_MAX_SAFE_ANGLE) 更保险。
     *
     * @param currentAngle   当前(或预测惯性冲出去后的)转塔相对角度，PID的"过程量"
     * @param targetAngle    目标相对角度，PID的"设定值"
     * @param feedforwardVel 前馈速度(度/秒)，比如抵消机器人自转/移动目标超前量
     * @param dt             距上次调用过了多久(秒)，用来做微分/低通滤波
     */
    private void driveTurretToAngle(double currentAngle, double targetAngle, double feedforwardVel, double dt) {
        targetAngle = Range.clip(targetAngle, -TURRET_FULL_RANGE_DEGREE, TURRET_FULL_RANGE_DEGREE);
        double pidOutputVel = turretPIDF.calculate(currentAngle, targetAngle); // 位置误差 → PID给出的期望速度
        double finalTargetVel = pidOutputVel + feedforwardVel;                 // PID输出 + 前馈速度 = 最终期望速度

        // 对"最终期望速度"求导得到"期望加速度"，再低通滤波(避免直接微分带来的噪声放大)，供kA前馈使用
        double rawTargetAccel = 0.0;
        if (dt > 0.0001) {
            rawTargetAccel = (finalTargetVel - lastTargetVel) / dt;
        }
        lastTargetVel = finalTargetVel;
        filteredTargetAccel = TURRET_CMD_ACCEL_FILTER_ALPHA * rawTargetAccel
                + (1.0 - TURRET_CMD_ACCEL_FILTER_ALPHA) * filteredTargetAccel;

        // kS只在"确实需要动"(期望速度 > 0.5度/秒)的时候才加，避免在几乎静止时因为噪声正负跳变而抖动
        double ksSign = Math.abs(finalTargetVel) > 0.5 ? Math.signum(finalTargetVel) : 0.0;
        double turretPower = filteredTargetAccel * TURRET_kA
                + finalTargetVel * TURRET_kV
                + ksSign * TURRET_kS;

        // 电压补偿：标定时电压是多少(TUNING_VOLTAGE)，现在电压如果更低，同样的功率实际输出的
        // 扭矩/速度会变小，这里按比例放大功率去补偿，让电池电量变化时表现更一致
        turretPower *= TUNING_VOLTAGE / currentBatteryVoltage;
        turret.setPower(Range.clip(TURRET_POWER_SIGN * turretPower, -TURRET_MAX_POWER, TURRET_MAX_POWER));
    }

    /** OpMode结束前调用：转塔停转，所有滤波器初始化标记复位(避免下次进OpMode用到上一局的陈旧滤波状态)。 */
    public void stop() {
        turret.setPower(0.0);
        isTurretFilterInitialized = false;
        isChassisFilterInitialized = false;
        isSmoothHeadingInit = false;
        lastTime = 0;
    }
}