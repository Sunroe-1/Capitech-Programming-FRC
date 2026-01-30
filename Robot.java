package frc.robot;

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

public class Robot extends TimedRobot {

    // === AUTONOMOUS CHOOSER ===
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // === DRIVE MOTORS ===
    private final SparkMax RightNEO1   = new SparkMax(1, MotorType.kBrushed);
    private final SparkMax LeftNEO1   = new SparkMax(4, MotorType.kBrushed);
    private final SparkMax RightNEO2 = new SparkMax(3, MotorType.kBrushed);
    private final SparkMax LeftNEO2  = new SparkMax(5, MotorType.kBrushed);

    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    private final Timer tempoTimer = new Timer();
    private final XboxController controller1 = new XboxController(0);

    // === BRAÇO ===
    private final SparkMax armMotor = new SparkMax(6, MotorType.kBrushless);
    private final SparkMaxConfig armConfig = new SparkMaxConfig();
    private RelativeEncoder armEncoder;

    // === SHOOTER ===
    private final SparkMax shooter1 = new SparkMax(2, MotorType.kBrushed);
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    private double shooterPower = -0.65;


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

        // === ARM ===
        armConfig.idleMode(IdleMode.kBrake);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armEncoder = armMotor.getEncoder();

        System.out.println("Posição inicial do braço: " + armEncoder.getPosition());

        // === SHOOTER ===
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter Power", shooterPower);

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
    public void autonomousPeriodic() {
    }


    // ==================== TELEOP ====================
    @Override
    public void teleopPeriodic() {

        // === Joystick ===
        double eixoY = applyDeadband(-controller1.getLeftX(), 0.1);
        double eixoX = applyDeadband(controller1.getLeftY(), 0.1);

        double speedMultiplier = controller1.getRightBumperButton() ? 1.0 : 0.5;

        // === Arcade Drive ===
        double leftSpeed  = clamp((eixoY + eixoX) * speedMultiplier, -0.7, 0.7);
        double rightSpeed = clamp((eixoY - eixoX) * speedMultiplier, -0.7, 0.7);

        LeftNEO1.set(leftSpeed);
        LeftNEO2.set(leftSpeed);
        RightNEO1.set(rightSpeed);
        RightNEO2.set(rightSpeed);

        // === Shooter ===
        shooterPower = SmartDashboard.getNumber("Shooter Power",  -0.65);

        if (controller1.getXButton()) {
            shooter1.set(shooterPower);
        } else {
            shooter1.set(0);
        }

        // === ARM MANUAL ===
        double pos = armEncoder.getPosition();
        boolean subir = controller1.getAButton();
        boolean descer = controller1.getYButton();

        double limiteSuperior = 3.0;
        double limiteInferior = -3.0;
        double power = 0;

        if (subir && pos < limiteSuperior) power = 0.15;
        else if (descer && pos > limiteInferior) power = -0.05;

        armMotor.set(power);

        System.out.printf("Braço | pos: %.3f | power: %.3f%n", pos, power);
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
