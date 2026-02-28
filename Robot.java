package frc.robot; //Pacote de importações para FRC

// === IMPORTAÇÕES ===
import edu.wpi.first.wpilibj.TimedRobot; //Importação para o robô executar seu código em 20ms
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; //Importação para decidir qual versão do autônomo usar
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Importação para mostrar informações do robô em tempo real
import edu.wpi.first.wpilibj.Timer; //Importação para o robô possuir um tempo determinado, útil para autônomo
import edu.wpi.first.wpilibj.XboxController; //Importação para o robô utilizar o controle, útil no TeleOp
import edu.wpi.first.wpilibj.Counter; //Importação para identificação de período de som
import edu.wpi.first.wpilibj.DigitalOutput; //Importação para utilizar meios digitais implementados no RoboRIO
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax; //Importação para utilizar SparkMax
import com.revrobotics.spark.SparkLowLevel.MotorType; //Importação para decidir qual motor vai ser (Brushed ou Brusheless)
import com.revrobotics.spark.SparkBase.ResetMode; //Importação para o robô continuar com a programação ou voltar ao padrão
import com.revrobotics.spark.SparkBase.PersistMode; //Importação para o código ficar salvo na memória ao invés de perder
import com.revrobotics.spark.config.SparkMaxConfig; // Importação para criar e aplicar configurações no SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;  // Importação para definir o modo de parada do motor (Brake ou Coast)
import org.opencv.core.Mat; //Importação para utilizar o OpenCV, muito utilizado na Telemetria para calcular distância

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TeleBee extends TimedRobot { //Classe para definição de partes, motores e sensores

    // === AUTONOMOUS CHOOSER ===
    private static final String kDefaultAuto = "Default"; //Define o modo base do autonômo
    private static final String kCustomAuto = "My Auto"; //Define o modo customizado do autonômo
    private String m_autoSelected; //Define qual modo do autonômo será escolhido
    private final SendableChooser<String> m_chooser = new SendableChooser<>();  //Definição de SmartDashboard no autonômo

    // === DRIVE MOTORS ===
    private final SparkMax RightNEO1 = new SparkMax(2, MotorType.kBrushed); //Definição do motor e sparkmax do lado direito
    private final SparkMax LeftNEO1  = new SparkMax(1, MotorType.kBrushed); //Definição do motor e sparkmax do lado esquerdo

    // === GARRA E DERIVATIVO ===
    private final SparkMax Claw = new SparkMax(8, MotorType.kBrushed);
    private final SparkMaxConfig ClawConfig = new SparkMaxConfig();

    private double ClawTargetPower = 0;
    private double ClawLastPower = 0;
    private double kD_Claw = 0.5;

    // === SENSOR AJ-SR04M-2 (ULTRASSÔNICO) ===
    private final DigitalOutput Trig = new DigitalOutput(0); //Definição para utilizar meios digitais implementados no RoboRIO
    private final Counter Echo = new Counter(1); //Definição para identificação de período de som
    private double DistanceFilter = 0; //Definição do filtro de distância
    private double ShooterPercentage = 0; //Definição da porcentagem de acerto do shooter

    // === SHOOTER ===
    private final SparkMax shooter1 = new SparkMax(4, MotorType.kBrushed); //Definição de motor do Shooter
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig(); //Definição da configuração de motor do Shooter
    private double shooterPower1 = 0.4; //Definição da potência do shooter para longa distância
    private double shooterPower2 = 0.4; //Definição da potência do shooter para curta distância
        
    // === INTAKE ===
    private final SparkMax intake = new SparkMax(6, MotorType.kBrushed); //Definição do motor do intake e seu sparkmax
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); //Definição de configuração do motor do intake

    // === INDEXTER ===
    private final SparkMax indexter = new SparkMax(3, MotorType.kBrushless); //Definição do motor do indexter da esquerda e seu sparkmax
    private final SparkMax indexter2 = new SparkMax(5, MotorType.kBrushed); //Definição do motor indexter de cima e seu sparkmax
    private final SparkMax esteira = new SparkMax(7, MotorType.kBrushed);
    private final SparkMaxConfig indexterConfig = new SparkMaxConfig(); //Definição da configuração

    // === TELEMTRY ===
    PhotonCamera camera = new PhotonCamera("Brutal");
    PIDController rotPID = new PIDController(0.02, 0, 0.001);
    PIDController distPID = new PIDController(0.8, 0, 0.05);

    double desiredDistance = 2;

    // === STATE MACHINE ===
    private RobotState currenttState = RobotState.OFF; //Definição do estado atual do robô e seu controle, sendo ele "desligado"
    private boolean lastBButton = false; //Definição da última vez que o botão B foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastAButton1 = false; //Definição da última vez que o botão A foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastAButton2 = false;
    private boolean lastXButton = false; //Definição da última vez que o botão X foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastXButton1 = false; // NOVO: Ultima vez que o X do controle 1 foi apertado
    private boolean lastRightTriggerAxis = false; //Definição da última vez que o botão de Gatilho Direito foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastYButton = false; //Definição da última vez que o botão Y foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastRightBumperButton = false; //Definição da última vez que o botão RB foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastLeftTriggerAxis = false; //Definição da última vez que o botão de Gatilho Esquerdo foi apertado, sendo considerado um "Falso Verdadeiro"
    private boolean lastLeftBumprButton = false; //Definição da última vez que o botão LB foi apertado, sendo considerado um "Falso Verdadeiro"

    // === OUTROS ===
    private final SparkMaxConfig driveConfig = new SparkMaxConfig(); //Definição da configuração dos motores de movimentação
    private final Timer tempo = new Timer(); //Definição do tempo, funcionando para autônomo e cronômetro
    private final Timer Delay = new Timer();
    private final XboxController controller1 = new XboxController(0); //Definição do controle para o piloto de movimentação/1
    private final XboxController controller2 = new XboxController(1); //Definição do controle para o piloto de funções mecânica/2

    // ==================== ROBOT INIT ====================
    @Override
    public void robotInit() { //Criação da classe "Iniciação do Robô"

    // === CHOOSER ===
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto); //Definição do modo normal do auto
        m_chooser.addOption("My Auto", kCustomAuto); //Definição do Meu Auto
        SmartDashboard.putData("Auto choices", m_chooser); //Definição de Escolhas de Auto

    // === DRIVE CONFIG ===
        driveConfig.idleMode(IdleMode.kBrake).inverted(false); //Configuração do DriveConfig, onde ele não está invertido

        RightNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor direito do Drive
        LeftNEO1.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor esquerdo do Drive

    // === SENSOR AJ-SRR04M-2
        Echo.setSemiPeriodMode(true); //Definição do semi período do Echo do sensor ultrassônco
        Echo.reset(); //Definição do reset do Echo do sensor ultrassônico

    // === SHOOTER ===
        shooterConfig.idleMode(IdleMode.kCoast); //Configuração do ShooterConfig, onde ele é Coast, sem parar totalmente
        shooter1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor de Shooter do ShooterConfig

        SmartDashboard.putNumber("Shooter Power", shooterPower1); //Aparição do power do shooter de longa distância no SmartDashboard, para os pilotos verem.

    // === INTAKE/OUTTAKE ===
        intakeConfig.idleMode(IdleMode.kCoast); //Configuração do Intake, onde ele é Coast, sem parar totalmente
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor de Intake do IntakeConfig

    // === INDEXTER ===
        indexterConfig.idleMode(IdleMode.kCoast); //Configuração do Indexter, onde ele é Coast, sem parar totalmente
        indexter.configure(indexterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor de Indexter do IndexterConfig
        indexter2.configure(indexterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Configuração dos parâmetros do motor de Indexter2 do IndexterConfig
        esteira.configure(indexterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ClawConfig.idleMode(IdleMode.kBrake);
        Claw.configure(ClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("=== Sistema iniciado ==="); //Aparecer  no console quando o robô for ligado
    }

    public enum RobotState{ //Definição da classe RobotState
        OFF, //Motores em 10% de potência
        INTAKE, //Intake ligado
        GARRAUP, //Garra para subir
        GARRADOWN, //Garra para descer
        SHOOTER1, //Shooter para longa distância
        SHOOTER2, //Shooter para curta distância
        OFFALL, //Motores em 0%
        INDEXTER, //Indexter ligado
        OUTTAKE, //Outtake ligado
        AUTO_TRACKING //Modo de alinhamento e tiro automático
    }

    public double getDistanceMeters() { //Classe para pegar a distância em metros
        Trig.set(true); //Colocar o Trig para um valor "Verdadeiro"
        Timer.delay(0.00001); //Delay de tempo de emissão de código para 10 microssegundos
        Trig.set(false); //Colocar o Trig para um valor "Falso Verdadeiro"
        
        double period = Echo.getPeriod(); //Colocar o Echo para período

        double distance = (period * 343) /2; //Colocar a distância para ser divida pela metade do período multiplicado pela velocidade do som e dividido pela metade
        return distance; //Retorna a distância
    }

    public void CalculateAccuracy (double getDistanceMeters, double camYaw, double joyMovement) { //Classe para Calcular a chance de acerto
        DistanceFilter = (DistanceFilter * 0.8) + (getDistanceMeters * 0.2); //Criação do cálculo do filtro de distância somado com a distância em metros

        double distanceTotal = 0;
        if (DistanceFilter >= 1.5) {
            double idealDistance = desiredDistance; //Distância ideal é 2 metros
            double distanceError = Math.abs(DistanceFilter - idealDistance); //Cálculo do erro de distância
            distanceTotal = 100 - (distanceError * 25); //Cálculo do total da distância diminuido pelos erros
        }
        distanceTotal = Math.max(0, Math.min(100, distanceTotal)); //Mínimo e máximo da distância 

        double angleTotal = 100 - (Math.abs(camYaw) * 10); //Cálculo total do ângulo diminuido pelos erros (10% por grau)
        angleTotal = Math.max(0, Math.min(100, angleTotal)); //Mínimo e máximo do ângulo

        double movimentTotal = 100 - (joyMovement > 0.1 ? 60 : 0); //Cálculo do total do movimento diminuido pelos erros
        movimentTotal = Math.max(0, Math.min(100, movimentTotal)); //Mínimo e máximo do movimento

        ShooterPercentage = movimentTotal * 0.2 + distanceTotal * 0.4 + angleTotal * 0.4; //Média da junção de todos os erros para cálcular a porcentagem de Shooter
    }

    // ==================== AUTONOMOUS ====================
    @Override
    public void autonomousInit() { //Classe do autônomo e sua iniciação
        m_autoSelected = m_chooser.getSelected(); //Seleção do auto selecionado
        tempo.reset(); //Reset do tempo
        tempo.start(); //Início do tempo
    }

    @Override
    public void autonomousPeriodic() {} //Classe onde fica a programação do Autônomo

    // ==================== TELEOP ====================
    @Override
    public void teleopPeriodic() { //Classe onde fica a programação do TeleOp

        // === STATE MACHINE ===
        boolean BPressed = controller2.getBButton(); //Caso o botão B seja pressionado no controle 2, será efetuado o botão B
        boolean APressed1 = controller1.getAButton(); //Caso o botão A seja pressionado no controle 1, será efetuado o botão A
        boolean XPressed = controller2.getXButton(); //Caso o botão X seja pressionado no controle 2, será efetuado o botão X
        boolean XPressed1 = controller1.getXButton(); // NOVO: Botão X no Controle 1 para modo automático
        boolean YPressed = controller1.getYButton(); //Caso o botão Y seja pressionado no controle 2, será efetuado o botão Y
        boolean RightBumperButtonPressed = controller2.getRightBumperButton(); //Caso o botão RB seja pressionado no controle 2, será efetuado o botão rB
        boolean LeftBumperButtonPressed = controller2.getLeftBumperButton(); //Caso o botão LB seja pressionado no controle 2, será efetuado o botão LB
        boolean RightTriggerAxisPressed = controller2.getRightTriggerAxis() > 0.1; //Caso o Gatilho Direito seja pressionado no controle 2, será efetuado o Gatilho Direito
        boolean LeftTriggerAxisPressed = controller2.getLeftTriggerAxis() > 0.1; //Caso o Gatilho Esquerdo seja pressionado no controle 2, será efetuado o Gatilho Esquerdo
        boolean APressed2 = controller2.getAButton(); //Caso o botão A seja pressionado no controle 2, será efetuado o botão A

        // === JOYSTICK ===
        double eixoX = applyDeadband(-controller1.getLeftX(), 0.1);
        double eixoY = applyDeadband(controller1.getRightY(), 0.1);

        // === TELEMETRY
        PhotonPipelineResult result = camera.getLatestResult();
        double currentYaw = result.hasTargets() ? result.getBestTarget().getYaw() : 999;
        double joystickSum = Math.abs(eixoX) + Math.abs(eixoY);

        CalculateAccuracy(getDistanceMeters(), currentYaw, joystickSum);

        // === TELEMETRY ===
        if (XPressed1 && !lastXButton1) {
            if (currenttState == RobotState.AUTO_TRACKING) {
                currenttState = RobotState.OFF;
            } else {
                currenttState = RobotState.AUTO_TRACKING;
            }
        }
        if (currenttState == RobotState.AUTO_TRACKING) {
            if (result.hasTargets()) {
                double rotOutput = rotPID.calculate(currentYaw, 0);
                if (Math.abs(currentYaw) < 0.5) rotOutput = 0;
                rotOutput = MathUtil.clamp(rotOutput, -0.5, 0.5);
                LeftNEO1.set(-rotOutput);
                RightNEO1.set(rotOutput);
            } else {
                LeftNEO1.set(0); RightNEO1.set(0);
            }
        } else {
            // === ARCADE DRIVE ===
            double finalLeftSpeed  = clamp(eixoY - eixoX, -0.8, 0.8);
            double finalRightSpeed = clamp(eixoY + eixoX, -0.8, 0.8);

            // === SELECTIVE DRIVE ISOLATION ===
            if (controller1.getRightBumperButton()) { finalRightSpeed = 0; }
            if (controller1.getLeftBumperButton()) { finalLeftSpeed = 0; }

            RightNEO1.set(finalRightSpeed);
            LeftNEO1.set(finalLeftSpeed);
        }
        
        // === SHOOTER LONG DISTANCE ===
        if (RightTriggerAxisPressed && !lastRightTriggerAxis) { //Se o Gatilho Direito do controle 2 for pressionado uma vez
            if (currenttState == RobotState.SHOOTER1) { //Ativa o modo SHOOTER1 
               currenttState = RobotState.OFF; //Caso seja apertado novamente, será mudado para o modo OFF
            } else{ //Caso não seja pressionado para o modo OFF
                currenttState = RobotState.SHOOTER1; //Modo SHOOTER1 continua ligado
                Delay.reset();
                Delay.start();
            }
        }

        // === SHOOTER SHORT DISTANCE ===
        if (RightBumperButtonPressed && !lastRightBumperButton) { //Se o botão RB do controle 2 for pressionado uma vez
            if (currenttState == RobotState.SHOOTER2) { //Ativa o modo SHOOTER2
                currenttState = RobotState.OFF; //Caso seja apertado novamente, será mudado para o modo OFF
            } else{ //Caso não seja pressionado para o modo OFF
                currenttState = RobotState.SHOOTER2; //Modo SHOOTER2 continua ligado
                Delay.reset();
                Delay.start();
            }
        }
      
        // === INDEXTER ===
        if (BPressed && !lastBButton) { //Caso o botão B do controle 2 for pressionado uma vez
          if (currenttState == RobotState.INDEXTER) { //Ativa o modo INDEXTER
            currenttState = RobotState.OFF; //Caso seja apertado novamente, será mudado para o modo OFF
          } else{ //Caso não seja pressionado para o modo OFF
            currenttState = RobotState.INDEXTER; //Modo INDEXTER continua ligado
          }  
        }

        // === INTAKE/OUTTAKE ===
        if (LeftBumperButtonPressed && !lastLeftBumprButton) { //Caso o botão LB seja pressionado uma vez
            if (currenttState == RobotState.INTAKE) { //Ativa o modo INTAKE
               currenttState = RobotState.OFF; //Caso seja apertado novamente, será mudado para o modo OFF
            } else { //Caso não seja pressionado para o modo OFF
                currenttState = RobotState.INTAKE; //Modo INTAKE continua ligado
            }
        }
        if (XPressed && !lastXButton) { //Caso o botão X seja pressionado uma vez
            if (currenttState == RobotState.OUTTAKE) { //Ativa o modo OUTTAKE
                currenttState = RobotState.OFF; //Caso seja apertado novamente, será mudado para o modo OFF
            } else{ //Caso não seja pressionado para o modo OFF
                currenttState = RobotState.OUTTAKE; //Modo OUTTAKE continua ligado
            }
        }

        // === OFFALL ===
        if (APressed2 && !lastAButton2) { //Caso o botão A do controle 2 seja apertado uma vez
            if (currenttState == RobotState.OFFALL) { //Ativa o modo OFFALL
                currenttState = RobotState.OFF; //Caso seja apertado novamnte, será mudado para o modo OFF
            } else { //Caso não seja pressionado para o modo OFF
                currenttState = RobotState.OFFALL; //Modo OFFALL continua ligado
            }
        }

        // === GARRA E DERIVATIVO ===
        if (controller1.getRightTriggerAxis() > 0.1) {
            ClawTargetPower = 1.0;
        }
        else if (controller1.getLeftTriggerAxis() > 0.1) {
            ClawTargetPower = -0.7;
        }
        else {
            ClawTargetPower = 0;
        }
        double derivative = ClawTargetPower - ClawLastPower;
        double output = ClawTargetPower - (kD_Claw * derivative);

        output = MathUtil.clamp(output, -1.0, 1.0);

        Claw.set(output);
        ClawLastPower = output;
        
        // === UPDATE BUTTONS STATES ===
        lastBButton = BPressed; //Atualiza a última vez que o botão B foi apertado
        lastAButton1 = APressed1; //Atualiza a última vez que o botão A do controle 1 foi apertado
        lastXButton1 = XPressed1; //Atualiza a última vez que o botão X do controle 1 foi apertado
        lastRightTriggerAxis = RightTriggerAxisPressed; //Atualiza a última vez que o Gatilho Direito foi apertado
        lastXButton = XPressed; //Atualiza a última vez que o botão X foi apertado
        lastYButton = YPressed; //Atualiza a última vez que o botão Y foi apertado
        lastLeftBumprButton = LeftBumperButtonPressed; //Atualiza a última vez que o botão LB foi apertado
        lastRightBumperButton = RightBumperButtonPressed; //Atualiza a última vez que o botão RB foi apertado
        lastLeftTriggerAxis = LeftTriggerAxisPressed; //Atualiza a última vez que o Gatilho Esquerdo foi apertado
        lastAButton2 = APressed2; //Atualiza a última vez que o botão A foi apertado

        switch (currenttState) { //Troca de estado atual da funções
            case AUTO_TRACKING:
                if (ShooterPercentage > 85) {
                    shooter1.set(shooterPower2);
                    indexter.set(-0.35); 
                    indexter2.set(0.35); 
                    esteira.set(-0.4);
                } else {
                    shooter1.set(0); 
                    indexter.set(0); 
                    indexter2.set(0.2); 
                    esteira.set(0);
                }
                break;

            case OFF: //Definição da classe OFF e suas funções
                shooter1.set(0.25); //Motor do shooter com potência de 10%
                indexter.set(0); //Motor do indexter do lado esquerdo com potência de 10%
                intake.set(0.2); //Motor do intake com potência de 10%
                indexter2.set(0); //Motor do indexter superior com potência de 10%
                esteira.set(0);
                break; //Termino da classe OFF e suas funções

            case SHOOTER1: //Definição da classe SHOOTER1 e suas funções
                shooter1.set(shooterPower1); //Potência do motor do shooter sendo a correspondente com a ShooterPower1
                if (Delay.get() >= 1.0)  {
                indexter.set(-0.3); //Motor do indexter do lado esquerdo com potência de 38%
                indexter2.set(0.3); //Motor do indexter superior com potência de 35%, sendo inverso dos outros
                esteira.set(0.2);
                } else{
                indexter.set(0); //Motor do indexter do lado esquerdo com potência de 38%
                indexter2.set(0); //Motor do indexter superior com potência de 35%, sendo inverso dos outros
                esteira.set(0);
                }
                intake.set(0.2);
                break; //Termino da classe SHOOTER1 e suas funções

            case SHOOTER2: //Definição da classe SHOOTER2  e suas funções
            shooter1.set(shooterPower2); //Potência do motor do shooter sendo a correspondente com a shooterPower2
            if (Delay.get() >= 1.0) {
                indexter.set(-0.3); //Motor indexter do lado esquerdo com potência de 38%
                indexter2.set(0.3); //Motor do indexter superior com potência de 35%, sendo inverso dos outros
                esteira.set(0.2);
            } else{
                indexter.set(0); //Motor indexter do lado esquerdo com potência de 38%
                indexter2.set(0); //Motor do indexter superior com potência de 35%, sendo inverso dos outros
                esteira.set(0);
            }
                intake.set(0.2);
                break; //Termino da classe SHOOTER2 e suas funções

            case INTAKE: //Definição da classe INTAKE e suas funções
                shooter1.set(0.25); //Potência do motor do shooter sendo 0%
                indexter.set(0); //Potência do motor do indexter do lado esquerdo sendo 20%
                indexter2.set(0); //Potênca do motor do indexter superior sendo 0%
                intake.set(0.8); //Potência do motor do intake sendo 50%
                esteira.set(0.2);
                break;

            case OFFALL:
                shooter1.set(0); 
                indexter.set(0); 
                indexter2.set(0); 
                intake.set(0); 
                esteira.set(0);
                break;

            case OUTTAKE:
                shooter1.set(0.25); 
                indexter.set(0); 
                intake.set(-0.6); 
                indexter2.set(0.2); 
                esteira.set(0);
                break;
            
            default:
                break;
        }

        // === SHOOTER PERCENTAGE DASHBOARD ===
        SmartDashboard.putNumber("Distancia (m)", getDistanceMeters());
        SmartDashboard.putNumber("Chance de Acerto %", ShooterPercentage);
        SmartDashboard.putBoolean("Pode Atirar", ShooterPercentage > 85);
    }
  
    @Override public void teleopInit() {}
    private double applyDeadband(double val, double lim) { return (Math.abs(val) < lim ? 0 : val); }
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }

    // ==================== VAZIO ====================
    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void testInit() {}
    @Override public void testPeriodic() {}
    @Override public void simulationInit() {}
    @Override public void simulationPeriodic() {}
}
