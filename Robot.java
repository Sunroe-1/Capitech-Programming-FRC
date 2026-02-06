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

    // === PID GARRA ===
    private double armTarget = 0;
    private double armLastError = 0;

    private final double kP_arm = 0.6;
    private final double kD_arm = 0.05;

    // === SHOOTER ===
    private final SparkMax shooter1 = new SparkMax(7, MotorType.kBrushed);
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    private double shooterPower = -0.9;

    // ===INDEXTER ===
    private final SparkMax Indexter = new SparkMax(2, MotorType.kBrushed );

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

    // === CHOOSER ===
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

    // === DRIVE CONFIG ===
        driveConfig.idleMode(IdleMode.kBrake).inverted(false);

        RightNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // === SHOOTER ===
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        double eixoY = -applyDeadband(controller1.getLeftY(), 0.1);
        double eixoX = applyDeadband(controller1.getLeftX(), 0.1);
        double speedMultiplier = controller1.getLeftBumper() ? 1.0 : 0.5;

        // === ARCADE DRIVE ===
        double leftSpeed  = clamp((eixoY + eixoX) * speedMultiplier, -0.7, 0.7);
        double rightSpeed = clamp((eixoY - eixoX) * speedMultiplier, -0.7, 0.7);

        // === NAVx2 9-EIXOS/PID ===
        double turnPower = 0;
        double yaw = navx.getYaw();
        double kP_turn = 0.02;
        double kD_turn = 0.003;
        
        if (Math.abs(eixoX) < 0.05) {
        
            double yawError = -yaw;
            double yawDerivative = yawError - lastYawError;
        
            //turnPower = (kP_turn * yawError) + (kD_turn * yawDerivative);
            //turnPower = clamp(turnPower, -0.4, 0.4);

            turnPower = 0;
        
            lastYawError = yawError;
        
        }
        
        LeftNEO1.set(leftSpeed + turnPower);
        RightNEO1.set(rightSpeed - turnPower);

        // === SHOOTER/INTAKE/OUTTAKE ===
        double shooterCmd = 0;

        if (controller1.getRightTriggerAxis() > 0.1) {
           shooterCmd = shooterPower;
              }

        else if (controller1.getBButton()) {
             shooterCmd = -0.45;
              }

        else if (controller1.getXButton()) {
             shooterCmd = 0.45;
              }

        shooter1.set(shooterCmd);
      
        // === INDEXTER ===
        if (controller1.getBButton()) {
            Indexter.set(-0.9);
        } else if (controller1.getXButton()) {
            Indexter.set(0.43);
        } else {
            Indexter.set(0);
        }
    }

  
    @Override
    public void teleopInit() {
        navx.zeroYaw();
        lastYawError = 0;
    }

    // ==================== UTIL ====================
    private double applyDeadband(double val, double lim) {
        return (Math.abs(val) < lim ? 0 : val);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ==================== VAZIO ====================
    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void testInit() {}
    @Override public void testPeriodic() {}
    @Override public void simulationInit() {}
    @Override public void simulationPeriodic() {}
}
