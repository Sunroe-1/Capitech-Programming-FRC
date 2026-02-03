package frc.robot;

// === IMPORTAÇÕES ===
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.studica.frc.AHRS;

public class Robot extends TimedRobot {

    // === AUTONOMOUS CHOOSER ===
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // === DRIVE MOTORS ===
    private final SparkMax RightNEO1 = new SparkMax(1, MotorType.kBrushed);
    private final SparkMax LeftNEO1  = new SparkMax(4, MotorType.kBrushed);
    private final SparkMax RightNEO2 = new SparkMax(3, MotorType.kBrushed);
    private final SparkMax LeftNEO2  = new SparkMax(5, MotorType.kBrushed);

    // === GARRA ===
    private final SparkMax clawMotor = new SparkMax(6, MotorType.kBrushed);
    private final SparkMaxConfig armConfig = new SparkMaxConfig();
    private RelativeEncoder clawEncoder;

    // === PID GARRA ===
    private double armTarget = 0;
    private double armLastError = 0;

    private final double kP_arm = 0.6;
    private final double kD_arm = 0.05;

    // === INTAKE ===
    private final SparkMax intakeMotor = new SparkMax(7, MotorType.kBrushed);

    // === SHOOTER (SEM ENCODER) ===
    private final SparkMax shooter1 = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax shooter2 = new SparkMax(8, MotorType.kBrushed);
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    private double shooterPower = -0.65;

    // === NAVx2 9-EIXOS ===
    private AHRS navx;
    private double lastYawError = 0;

    // === OUTROS ===
    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    private final Timer tempoTimer = new Timer();
    private final XboxController controller1 = new XboxController(0);

    // ==================== ROBOT INIT ====================
    @Override
    public void robotInit() {

        // Chooser
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // === DRIVE CONFIG ===
        driveConfig.idleMode(IdleMode.kBrake).inverted(false);

        RightNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightNEO2.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftNEO2.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // === GARRA ===
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.smartCurrentLimit(35);
        clawMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        clawEncoder = clawMotor.getEncoder();
        clawEncoder.setPosition(0);

        // === SHOOTER ===
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooter2.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter Power", shooterPower);

        // === NAVx2 9-EIXOS ===
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

        System.out.println("=== Sistema iniciado ===");
    }

    // ==================== AUTONOMOUS ====================
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        tempoTimer.reset();
        tempoTimer.start();
    }

    @Override
    public void autonomousPeriodic() {}

    // ==================== TELEOP ====================
    @Override
    public void teleopPeriodic() {

        // === JOYSTICK ===
        double eixoY = applyDeadband(-controller1.getLeftY(), 0.1);
        double eixoX = applyDeadband(controller1.getLeftX(), 0.1);
        double speedMultiplier = controller1.getLeftBumper() ? 1.0 : 0.5;

        // === ARCADE DRIVE ===
        double leftSpeed  = clamp((eixoY + eixoX) * speedMultiplier, -0.7, 0.7);
        double rightSpeed = clamp((eixoY - eixoX) * speedMultiplier, -0.7, 0.7);

        // === NAVx2 9-EIXOS/PID ===
        double yaw = navx.getYaw();
        double yawError = -yaw;
        double yawDerivative = yawError - lastYawError;

        double kP_turn = 0.02;
        double kD_turn = 0.003;

        double turnPower = (kP_turn * yawError) + (kD_turn * yawDerivative);
        turnPower = clamp(turnPower, -0.4, 0.4);

        lastYawError = yawError;

        LeftNEO1.set(leftSpeed + turnPower);
        LeftNEO2.set(leftSpeed + turnPower);
        RightNEO1.set(rightSpeed - turnPower);
        RightNEO2.set(rightSpeed - turnPower);

        // === SHOOTER ===
        shooterPower = SmartDashboard.getNumber("Shooter Power", -0.65);

        if (controller1.getRightTriggerAxis() > 0.1) {
            shooter1.set(shooterPower);
            shooter2.set(shooterPower);
        } else {
            shooter1.set(0);
            shooter2.set(0);
        }

        // === GARRA ===
        double pos = clawEncoder.getPosition();

        if (controller1.getAButton()) armTarget = 2.5;
        if (controller1.getYButton()) armTarget = -2.5;

        double armError = armTarget - pos;
        double armDerivative = armError - armLastError;

        double armPower = (kP_arm * armError) + (kD_arm * armDerivative);
        armPower = clamp(armPower, -0.8, 0.8);

        clawMotor.set(armPower);
        armLastError = armError;

        // === INTAKE/OUTTAKE ===
        if (controller1.getBButton()) {
            intakeMotor.set(0.8);
        } else if (controller1.getXButton()) {
            intakeMotor.set(-0.8);
        } else {
            intakeMotor.set(0);
        }
    }

    // ==================== UTIL ====================
    private double applyDeadband(double val, double lim) {
        return (Math.abs(val) < lim ? 0 : val);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ==================== EMPTY ====================
    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void testInit() {}
    @Override public void testPeriodic() {}
    @Override public void simulationInit() {}
    @Override public void simulationPeriodic() {}
}
