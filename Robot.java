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
    
    // === GARRA ===
    private final SparkMax clawMotor = new SparkMax(6, MotorType.kBrushed);
    private final SparkMaxConfig armConfig = new SparkMaxConfig();
    private RelativeEncoder clawEncoder;

    // === INTAKE ===
    private final SparkMax intakeMotor = new SparkMax(7, MotorType.kBrushed);

    // === SHOOTER ===
    private final SparkMax shooter1 = new SparkMax(2, MotorType.kBrushed);
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    private double shooterPower = -0.65;

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

        System.out.println("Posição inicial da Garra: " + clawEncoder.getPosition());

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

        // === JOYSTICK ===
        double eixoY = applyDeadband(-controller1.getLeftX(), 0.1);
        double eixoX = applyDeadband(controller1.getLeftY(), 0.1);

        double speedMultiplier = controller1.getLeftBumperButton() ? 1.0 : 0.5;

        // === ARCADE DRIVE===
        double leftSpeed  = clamp((eixoY + eixoX) * speedMultiplier, -0.7, 0.7);
        double rightSpeed = clamp((eixoY - eixoX) * speedMultiplier, -0.7, 0.7);

        LeftNEO1.set(leftSpeed);
        LeftNEO2.set(leftSpeed);
        RightNEO1.set(rightSpeed);
        RightNEO2.set(rightSpeed);

        // === SHOOTER ===
        shooterPower = SmartDashboard.getNumber("Shooter Power",  -0.65);

        if (controller1.getRightTriggerAxis() > 0.1) {
            shooter1.set(shooterPower);
        } else {
            shooter1.set(0);
        }

        // === GARRA ===
        double pos = clawEncoder.getPosition();
        boolean fechar = controller1.getAButton();
        boolean abrir = controller1.getYButton();
        boolean hangmode = controller1.getRightBumperButton();

        double limiteFechado = 2.5;
        double limiteAberto = -2.5;

        double corrente = clawMotor.getOutputCurrent();
        double correnteHang = 34;
        double correnteAlvo = 25;

        double powerFechar = 0.9;
        double powerAbrir = -0.6;
        double powerFirme = 0.25;
        double powerhang = 0.3;

        double power = 0;

if (hangmode) {
    if (corrente < correnteHang && pos < limiteFechado){
        power = powerhang;
    }
} else {
    power = powerFirme;
}

        if (fechar && pos < limiteFechado) power = powerFechar;
        else if (abrir && pos > limiteAberto) power = powerAbrir;

        else {
            if (corrente < correnteAlvo) {
                power = powerFirme;
            } else {
                power = 0.1;
            }
        }

        clawMotor.set(power);

        System.out.printf("Garra  | HANG: %b | pos: %.2f | power: %.1fA", hangmode, pos, corrente, power);
    
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
