package frc.robot; //Pacote de importações para FRC

// === IMPORTAÇÕES ===
import edu.wpi.first.wpilibj.RobotState; //Importação da máquina estado
import edu.wpi.first.wpilibj.TimedRobot; //Importação para o robô executar seu código em 20ms
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; //Importação para decidir qual versão do autônomo usar
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Importação para mostrar informações do robô em tempo real
import edu.wpi.first.wpilibj.Timer; //Importação para o robô possuir um tempo determinado, útil para autônomo
import edu.wpi.first.wpilibj.XboxController; //Importação para o robô utilizar o controle, útil no TeleOp
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Counter; //Importação para identificação de período de som
import edu.wpi.first.wpilibj.DigitalOutput; //Importação para utilizar meios digitais implementados no RoboRIO

import com.revrobotics.spark.SparkMax; //Importação para utilizar SparkMax
import com.revrobotics.spark.SparkLowLevel.MotorType; //Importação para decidir qual motor vai ser (Brushed ou Brusheless)
import com.revrobotics.spark.SparkBase.ResetMode; //Importação para o robô continuar com a programação ou voltar ao padrão
import com.revrobotics.spark.SparkBase.PersistMode; //Importação para o código ficar salvo na memória ao invés de perder
import com.revrobotics.spark.config.SparkMaxConfig; // Importação para criar e aplicar configurações no SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;  // Importação para definir o modo de parada do motor (Brake ou Coast)

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder; //Importação para a utilização do Encoder

import com.studica.frc.AHRS; //Importação do sensor Navx2

public class Robot extends TimedRobot { //Classe para definir motores e outras coisas

    // === AUTONOMOUS CHOOSER ===
    private static final String kDefaultAuto = "Default"; //Define o modo base do autonômo
    private static final String kCustomAuto = "My Auto"; //Define o modo customizado do autonômo
    private String m_autoSelected; //Define qual modo do autonômo será escolhido
    private final SendableChooser<String> m_chooser = new SendableChooser<>(); 

    // === DRIVE MOTORS ===
    private final SparkMax RightNEO1 = new SparkMax(2, MotorType.kBrushed); //Definição do motor e sparkmax do lado direito
    private final SparkMax LeftNEO1  = new SparkMax(1, MotorType.kBrushed); //Definição do motor e sparkmax do lado esquerdo

    // === PID GARRA ===
    private double armTarget = 0; //Definição do alvo da garra para derivado do PID
    private double armLastError = 0; //Definição do último erro da garra para derivado do PID
    private final double kD_arm = 0.05; //Definição do valor do derivado no PID

    // === SENSOR AJ-SR04M-2 ===
    private final DigitalOutput Trig = new DigitalOutput(0); //Definição para utilizar meios digitais implementados no RoboRIO
    private final Counter Echo = new Counter(1); //Definição para identificação de período de som
    private double DistanceFilter = 0; //Definição do filtro de distância
    private double ShooterPercentage = 0; //Definição da porcentagem de acerto do shooter

    // === SHOOTER ===
    private final SparkMax shooter1 = new SparkMax(3, MotorType.kBrushed); //Definição de motor do Shooter
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig(); //Definição da configuração de motor do Shooter
    private double shooterPower = -0.8; //Definição da potência do shooter

    // === INTAKE ===
    private final SparkMax intake = new SparkMax(4, MotorType.kBrushed); //Definição do motor do intake e seu sparkmax
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); //Definição de configuração do motor do intake

    // ===INDEXTER ===
    private final SparkMax indexter = new SparkMax(5, MotorType.kBrushed); //Definição do motor do indexter e seu sparkmax
    private final SparkMax indexter2 = new SparkMax(6, MotorType.kBrushed);
    private final SparkMaxConfig indexterConfig = new SparkMaxConfig(); //Definição da configuração

    // === NAVx2 9-EIXOS ===
    private AHRS navx; //Definição do sensor NAVX2
    private double yawerror = 0; //Definição do erro de ângulo
    private double lastYawError = 0; //Definição do último erro de ângulo
    private double robotvelocity = 0; //Definição da velocidade do robô

    // === STATE MACHINE ===
    private RobotState currenttState = RobotState.OFF; //Definição do estado atual do robô e seu controle, sendo ele "desligado"
    private boolean lastBButton = false; //Definição da última vez que o botão B foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastAButton = false; //Definição da última vez que o botão A foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastXButton = false; //Definição da última vez que o botão X foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastRightTriggerAxis = false; //Definição da última vez que o botão de Gatilho Direito foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastYButton = false;
    private boolean lastRightBumperButton = false;
    private boolean lastLeftTriggerAxis = false;
    private boolean lastLeftBumprButton = false;

    // === OUTROS ===
    private final SparkMaxConfig driveConfig = new SparkMaxConfig(); //Definição da configuração dos motores de movimentação
    private final Timer tempo = new Timer(); //Definição do tempo, funcionando para autônomo e cronômetro
    private final XboxController controller1 = new XboxController(0); //Definição do controle para o piloto de movimentação/1
    private final XboxController controller2 = new XboxController(1); //Definição do controle para o piloto de funções mecâ

    // ==================== ROBOT INIT ====================
    @Override
    public void robotInit() { //Criação da classe "Iniciação do Robô"

    // === CHOOSER ===
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

    // === DRIVE CONFIG ===
        driveConfig.idleMode(IdleMode.kBrake).inverted(false);

        RightNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // === SENSOR AJ-SRR04M-2
        Echo.setSemiPeriodMode(true);
        Echo.reset();


    // === SHOOTER ===
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter Power", shooterPower);

    // === INTAKE/OUTTAKE ===
        intakeConfig.idleMode(IdleMode.kCoast);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // === INDEXTER ===
        indexterConfig.idleMode(IdleMode.kCoast);
        indexter.configure(indexterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // === NAVx2 9-EIXOS ===
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

        System.out.println("=== Sistema iniciado ===");
    }

    public enum RobotState{
        OFF,
        INTAKE,
        GARRA,
        SHOOTER,
        INDEXTER,
        OUTTAKE
    }

    public double getDistanceMeters() {
        Trig.set(true);
        Timer.delay(0.00001);
        Trig.set(false);
        
        double period = Echo.getPeriod();

        double distance = (period * 343) /2;
        return distance;
    }

    public void CalculateAccuracy (double getDistanceMeters) {
        DistanceFilter = (DistanceFilter * 0.8) + (getDistanceMeters * 0.2);

        double idealDistance = 3;
        double distanceError = Math.abs(DistanceFilter - idealDistance);

        double distanceTotal = 100 - (distanceError * 15);
        distanceTotal = Math.max(0, Math.min(100, distanceTotal));

        double angleTotal = 100 - (yawerror * 3);
        angleTotal = Math.max(0, Math.min(100, angleTotal));

        double movimentTotal = 100 - ((Math.abs(robotvelocity) < 0.1) ? 100 : 50);
        movimentTotal = Math.max(0, Math.min(100, movimentTotal));

        ShooterPercentage = movimentTotal * 0.45 + distanceTotal * 0.35 + angleTotal * 0.2;
    }

    // ==================== AUTONOMOUS ====================
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        tempo.reset();
        tempo.start();
    }

    @Override
    public void autonomousPeriodic() {}

    // ==================== TELEOP ====================
    @Override
    public void teleopPeriodic() {

        // === STATE MACHINE ===
        boolean BPressed = controller2.getBButton();
        boolean APressed = controller2.getAButton();
        boolean XPressed = controller2.getXButton();
        boolean YPressed = controller2.getYButton();
        boolean RightBumperButtonPressed = controller2.getRightBumperButton();
        boolean LeftBumperButtonPressed = controller2.getLeftBumperButton();
        boolean RightTriggerAxisPressed = controller2.getRightTriggerAxis() > 0.1;
        boolean LeftTriggerAxisPressed = controller2.getLeftTriggerAxis() > 0.1;
        boolean APressed2 = controller1.getAButton();

        // === JOYSTICK ===
        double eixoY = applyDeadband(controller1.getLeftX(), 0.1);
        double eixoX = applyDeadband(controller1.getRightY(), 0.1);
        double speedMultiplier = controller1.getLeftBumper() ? 1.0 : 0.5;

        // === ARCADE DRIVE ===
        double leftSpeed  = clamp((eixoY + eixoX) * speedMultiplier, -0.7, 0.7);
        double rightSpeed = clamp((eixoY - eixoX) * speedMultiplier, -0.7, 0.7);

        // === SELECTIVE DRIVE ISOLATION ===
        if (controller1.getRightBumperButton()) {
            RightNEO1.set(0);
        } else if (controller1.getLeftBumperButton()) {
            LeftNEO1.set(0);
        } else {
            RightNEO1.set(rightSpeed);
            LeftNEO1.set(leftSpeed);
        }

        LeftNEO1.set(leftSpeed);
        RightNEO1.set(rightSpeed);

        // === SHOOTER ===
        if (RightTriggerAxisPressed && !lastRightTriggerAxis) {
            if (currenttState == RobotState.SHOOTER) {
               currenttState = RobotState.OFF; 
            } else{
                currenttState = RobotState.SHOOTER;
            }
        }
      
        // === INDEXTER ===
        if (BPressed && !lastBButton) {
          if (currenttState == RobotState.INDEXTER) {
            currenttState = RobotState.OFF;
          } else{
            currenttState = RobotState.INDEXTER;
          }  
        }

        // === INTAKE/OUTTAKE ===
        if (BPressed && !lastBButton) {
            if (currenttState == RobotState.INTAKE) {
               currenttState = RobotState.OFF; 
            } else {
                currenttState = RobotState.INTAKE;
            }
        }
        if (APressed && !lastAButton) {
            if (currenttState == RobotState.OUTTAKE) {
                currenttState = RobotState.OFF;
            } else{
                currenttState = RobotState.OUTTAKE;
            }
        }

        // === UPDATE BUTTONS STATES ===
        lastBButton = BPressed;
        lastAButton = APressed;
        lastRightTriggerAxis = RightTriggerAxisPressed;

        switch (currenttState) {
            case OFF:
                shooter1.set(0);
                indexter.set(0);
                intake.set(0);
                break;

            case SHOOTER:
            shooter1.set(shooterPower);
            indexter.set(0.6);
            intake.set(0);
            break;

            case INTAKE:
            shooter1.set(0);
            indexter.set(0.9);
            intake.set(0.65);
            break;

            case OUTTAKE:
            shooter1.set(0);
            indexter.set(-0.9);
            intake.set(-0.65);
            break;
            
            default:
                break;
        }

        // === SHOOTER PERCENTAGE ===
        double ShooterPercentage = 0;

        double distance = getDistanceMeters();
        SmartDashboard.putNumber("Distancia (m)", distance);

        CalculateAccuracy(distance);
        SmartDashboard.putNumber("Chance de Acerto %", ShooterPercentage);
        boolean canShoot = ShooterPercentage >= 90;
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
