    //===============================IMPORTS===============================//
    package frc.robot;//Pacote que lista os códigos exclusivamente para robôs da FRC


    //Importações da biblioteca da WPIlib
    import edu.wpi.first.wpilibj.TimedRobot;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.CommandScheduler;
    import edu.wpi.first.wpilibj.DigitalInput;
    import edu.wpi.first.wpilibj.RobotState;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj.XboxController;
    import edu.wpi.first.math.util.Units; 
    import static edu.wpi.first.units.Units.Newton;//Import que permite a utilizaçao da unidade de medida em ''Newtons''


    //Importações da Rev para utilização dos SparkMax
    import com.revrobotics.spark.SparkMax;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
    import com.revrobotics.spark.SparkBase.ResetMode;
    import com.revrobotics.spark.SparkBase.PersistMode;
    import com.revrobotics.spark.config.SparkBaseConfig;
    import com.revrobotics.spark.config.SparkMaxConfig;
    import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

    import org.opencv.aruco.EstimateParameters;

    //Imports do photonvision

    import org.photonvision.PhotonCamera; // Classe principal para a câmera
    import org.photonvision.targeting.PhotonPipelineResult; // Resultado do processamento
    import org.photonvision.targeting.PhotonTrackedTarget; // Dados de um alvo específico
    import org.photonvision.PhotonUtils; // (Opcional) Útil para cálculos de distância
    import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

    import edu.wpi.first.wpilibj.DriverStation;
    import java.util.Optional;
    import edu.wpi.first.wpilibj.DriverStation.Alliance;

    import com.revrobotics.RelativeEncoder;//import da Rev para utiização do encoder

    import com.studica.frc.AHRS;//import da biblioteca utilizada para o sensor NAVx2


    public class Robot extends TimedRobot {

    // --- VARIÁVEIS DE VISÃO ---
    PhotonCamera camera = new PhotonCamera("Fifine_K420");
    private final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

    // Variáveis de calibração baseadas nos seus dados
    final double ALTURA_CAMERA_METROS = 0.53; 
    final double ALTURA_HUB_METROS = 1.12;
    final double ALTURA_TORRE_METROS = 0.55;
    final double ANGULO_MONTAGEM_RADIANOS = Units.degreesToRadians(0.0); // Mude se a câmera estiver inclinada

    // Controle de estado
    private boolean jaAlinhou = false;

    private static final String kDefaultAuto = "Normal"; // saída 0

    private static final String kCustomAuto = "Auto 1"; // saida 1

    private static final String kAuto2 = "Auto 2"; // saída 2

    private static final String kAuto3 = "Auto 3"; // saída 3

    private static final String kAuto4 = "Auto 4"; // saída 4

    private static final String kAuto5 = "Auto 5"; // saída 5

    private static final String kAutoProtoEsp = "Normal Esp"; // saída 6

    private static final String kAuto7 = "Auto 7"; // saída 7

    private static final String kAuto8 = "Auto 8"; // saida 8

    private static final String kAuto9 = "Auto 9"; // saida 9

    private static final String kAuto10 = "Auto 10";

    
    private String m_autoSelect; // Guarda qual autônomo foi escolhido no dashboard

    private final SendableChooser<String> m_chooser = new SendableChooser<>(); // Menu no dashboard para selecionar o autônomo

    Timer timer = new Timer();  // Timer para controle de tempo

    DigitalInput botao = new DigitalInput(1);  // Entrada digital

    private Command m_autonomousCommand; // Comando que roda durante o autônomo


    private final RobotContainer m_robotContainer;// Container onde ficam subsistemas e comandos
    
        
    private final SparkMaxConfig leftConfig = new SparkMaxConfig();  // Configurações esquerda e direita 
    private final SparkMaxConfig rightConfig = new SparkMaxConfig(); //dos motores SparkMax
    
    private final SparkMaxConfig shooterConfig = new SparkMaxConfig(); //Configuração do motor do Shooter

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); //Configuração do motor do Intake

    private final SparkMaxConfig indexPConfig = new SparkMaxConfig(); //Configuração do motor do Primeiro Indexer

    private final SparkMaxConfig indexSConfig = new SparkMaxConfig(); //COnfiguração do motor do segundo Indexer
    
    private final SparkMaxConfig esteiraConfig = new SparkMaxConfig(); //Configuração do motor da esteira

    //Motores direito e esquerdo, respectivamente, ambos tipo Brushed
    private final SparkMax RightNEO1 = new SparkMax(1, MotorType.kBrushed);
    private final SparkMax LeftNEO1 = new SparkMax(2, MotorType.kBrushed);

    private final SparkMax Shooter = new SparkMax(3, MotorType.kBrushed); //Motor do Shooter
    
    private final SparkMax Intake = new SparkMax(4, MotorType.kBrushed); //Motor do Intake
    
    private final SparkMax IndexP = new SparkMax(5, MotorType.kBrushed); //Motor do Primeiro indexer
    
    private final SparkMax IndexS = new SparkMax(6, MotorType.kBrushless); //Motor do Segundo Indexer


    private final SparkMax esteira = new SparkMax(8, MotorType.kBrushed); //Esteira


    public Robot() {
        
        m_robotContainer = new RobotContainer();
    }


    public void robotInit(){

    // Tabela de chute: Distância (metros) -> Potência (0.0 a 1.0)
        shotMap.put(1.0, 0.50);
        shotMap.put(2.0, 0.70);
        shotMap.put(3.0, 0.85); // O exemplo que você pediu (3m = 0.8)
        shotMap.put(4.0, 0.95);
    SmartDashboard.putData("Auto Choices", m_chooser);


    m_chooser.setDefaultOption("Normal", kDefaultAuto);
    m_chooser.addOption("Saída1", kCustomAuto);
    m_chooser.addOption("Saida 2", kAuto2);
    m_chooser.addOption("Saida 3", kAuto3);
    m_chooser.addOption("Saida 4", kAuto4);
    m_chooser.addOption("Saida 5", kAuto5);
    m_chooser.addOption("Normal Esp", kAutoProtoEsp);
    m_chooser.addOption("Saida 7", kAuto7);
    m_chooser.addOption("Saida 8", kAuto8);
    m_chooser.addOption("Saida 9", kAuto9); 
    m_chooser.addOption("Saida 10", kAuto10);

    SmartDashboard.putData("Auto Choices", m_chooser);

    leftConfig.idleMode(IdleMode.kBrake).inverted(false);
    rightConfig.idleMode(IdleMode.kBrake).inverted(true);


    RightNEO1.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    LeftNEO1.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterConfig.idleMode(IdleMode.kCoast);

    Shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexPConfig.idleMode(IdleMode.kBrake);

    IndexP.configure(indexPConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexSConfig.idleMode(IdleMode.kBrake);

    IndexS.configure(indexSConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig.idleMode(IdleMode.kCoast);

    Intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    esteiraConfig.idleMode(IdleMode.kBrake);

    esteira.configure(esteiraConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
    

    @Override
    public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /* 
    if (m_autonomousCommand != null) {
    CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }*/

    m_autoSelect = m_chooser.getSelected();
    System.out.println("Normal" + m_autoSelect);

    timer.reset();
    timer.start();
    jaAlinhou = false; // Reseta o flag de alinhamento
    allMotorsStop();

    }

    
    @Override 
    public void autonomousPeriodic() {
        double time = timer.get();

        switch (m_autoSelect) {
    case kDefaultAuto:
    logicaNormal(time);
    break;
    case kCustomAuto:
    default:
    logicaSaida1(time);
    break;
    case kAuto2:
    logicaSaida2(time);
    break;
    case kAuto3:
    logicaSaida3(time);
    break;
    case kAuto4:
    logicaSaida4(time);
    break;
    case kAuto5:
    logicaSaida5(time);
    break;
    case kAutoProtoEsp:
    logicaNormalEsp(time);
    break;
    case kAuto7:
    logicaSaida7(time);
    break;
    case kAuto8:
    logicaSaida8(time);
    break;
    case kAuto9:
    logicaSaida9(time);
    break;
    case kAuto10:
    logicaSaida10(time);
    break;
    }
    }


    private void logicaNormal(double time) {
        // 1. FASE DE POSICIONAMENTO E PRIMEIRO TIRO (0 a 6.0s)
        if (time < 6.0) { 
            if (time < 2.0) {
                RightNEO1.set(-0.4); LeftNEO1.set(-0.4); 
            } else {
                RightNEO1.set(0); LeftNEO1.set(0);  
            }
            Shooter.set(0.85); 
            if (time > 2.5) { IndexS.set(0.5); }
            if (time > 3.5) { IndexP.set(0.4); esteira.set(0.2); }
        } 
        // 2. GIRA 180° PARA BUSCAR NOVAS BOLAS (6.0s a 8.0s)
        else if (time < 8.0) { 
            Shooter.set(0.55); 
            IndexP.set(0); IndexS.set(0); esteira.set(0);
            RightNEO1.set(-0.5); LeftNEO1.set(0.5); 
        }
        // 3. ANDA PARA FRENTE E COLETA (8.0s a 10.0s)
        else if (time < 10.0) { 
            RightNEO1.set(0.5); LeftNEO1.set(0.5); 
            Intake.set(0.8); IndexP.set(0.2); esteira.set(0.2); 
        }
        // 4. GIRA 180° DE VOLTA (10.0s a 12.0s)
        else if (time < 12.0) { 
            RightNEO1.set(0.5); LeftNEO1.set(-0.5); 
            Intake.set(0); IndexP.set(0); esteira.set(0); 
            Shooter.set(0.95);
        }
        // 5. SEGUE UM POUCO PARA FRENTE ANTES DE VIRAR (12.0s a 13.0s)
        else if (time < 13.0) {
            RightNEO1.set(0.4); LeftNEO1.set(0.4); // Deslocamento para ganhar espaço
        }
        // 6. VIRA 45° PARA A ESQUERDA (13.0s a 13.7s)
        else if (time < 13.7) {
            RightNEO1.set(0.4); LeftNEO1.set(-0.4); 
        }
        // 7. PARA E ATIRA (13.7s a 15.5s)
        else if (time < 15.5) { 
            RightNEO1.set(0); LeftNEO1.set(0); 
            if (time > 14.0) { 
                IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2);
            }
        }
        // 8. COMPENSA OS 45° PARA A DIREITA (15.5s a 16.2s)
        else if (time < 16.2) {
            Shooter.set(0); IndexS.set(0); IndexP.set(0); esteira.set(0);
            RightNEO1.set(-0.4); LeftNEO1.set(0.4); 
        }
        // 9. CORRIDA FINAL PARA A RAMPA (16.2s a 20.0s)
        else if (time < 20.0) {
            RightNEO1.set(0.65); 
            LeftNEO1.set(0.65); 
        }
        else { 
            allMotorsStop(); 
        }
    }
    
    private void logicaSaida1(double time) {
        // 1. ATIRA (0 a 6.0s)
        if (time < 6.0) {
            if (time < 1.5) { 
                RightNEO1.set(-0.4); LeftNEO1.set(-0.4); // Pequena ré para alinhar
            } else { 
                RightNEO1.set(0); LeftNEO1.set(0);
            }
            
            Shooter.set(0.85);
            if (time > 2.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } 
        // 2. DESLOCAMENTO LONGO PARA A TORRE (6.0s a 16.0s)
        else if (time < 16.0) {
            allMotorsStop();
            // Aqui você pode colocar um giro se a torre não estiver atrás do robô
            // Supondo que precise apenas dar ré até ela:
            RightNEO1.set(-0.4); LeftNEO1.set(-0.4); 
        } 
        // 3. ESCALADA (16.0s a 20.0s)
        else if (time < 20.0) {
            RightNEO1.set(-0.2); // Mantém pressão contra a torre
            LeftNEO1.set(-0.2);
        }
        else { 
            allMotorsStop(); 
        }
    }

    private void logicaSaida2(double time) {
        if (time < 6.0) { // TIRO
            if (time < 2.0) { RightNEO1.set(-0.3); LeftNEO1.set(-0.3); } 
            else { RightNEO1.set(0); LeftNEO1.set(0); }
            Shooter.set(0.85);
            if (time > 2.5) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } 
        else if (time < 9.0) { // GIRA PARA ALINHAR A TRASEIRA COM A TORRE
            allMotorsStop();
            RightNEO1.set(-0.4); LeftNEO1.set(0.4); // Ajuste o tempo se o giro for muito ou pouco
        } 
        else if (time < 16.0) { // RÉ ATÉ A TORRE
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); 
        }
        else if (time < 20.0) { //  
            RightNEO1.set(-0.1); LeftNEO1.set(-0.1);
        } else { allMotorsStop(); }
    }


    private void logicaSaida3(double time) {
        if (time < 6.0) { // TIRO
            if (time < 2.0) { RightNEO1.set(-0.3); LeftNEO1.set(-0.3); }
            else { RightNEO1.set(0); LeftNEO1.set(0); }
            Shooter.set(0.85);
            if (time > 2.5) { IndexS.set(0.4); IndexP.set(0.5); esteira.set(0.2); }
        } 
        else if (time < 8.5) { // 
            RightNEO1.set(0.4); LeftNEO1.set(-0.4);
        } 
        else if (time < 10.5) { // GIRA 90°
            RightNEO1.set(0.4); LeftNEO1.set(0.4);
        } 
        else if (time < 16.0) { // RÉ FINAL ATÉ A TORRE
            RightNEO1.set(-0.4); LeftNEO1.set(-0.4);
        } 
        else if (time < 20.0) { // ESCALADA
            RightNEO1.set(-0.1); LeftNEO1.set(-0.1);
        } else { allMotorsStop(); }
    }

    void logicaSaida4(double time){
    // 1. ATIRAR IMEDIATAMENTE (0 a 4s)
        if (time < 4.0) {
            Shooter.set(0.85);
            if (time > 1.5) { 
                IndexS.set(0.4); // Indexer superior
                IndexP.set(0.5); esteira.set(0.2); // Indexer primeiro e esteira juntos para evitar engasgos
            }
        } 
        // 2. PRIMEIRO GIRO DE 90° (1/4 de círculo) (4 a 5s)
        else if (time < 5.0) {
            // Desliga tiro para poupar energia
            Shooter.set(0.3); IndexS.set(0); IndexP.set(0); esteira.set(0);
            
            // Gira 90 graus (ajuste o tempo se ele girar mais ou menos que isso)
            RightNEO1.set(-0.4); 
            LeftNEO1.set(0.4); 
        } 
        // 3. VAI PARA FRENTE (5 a 7s)
        else if (time < 7.0) {
            RightNEO1.set(0.3); 
            LeftNEO1.set(0.3); 
        } 
        // 4. SEGUNDO GIRO DE 90° (7 a 8s)
        else if (time < 8.0) {
            // Gira para alinhar com as bolas no chão
            RightNEO1.set(-0.4); 
            LeftNEO1.set(0.4); 
        } 
        // 5. LIGA INTAKE E VAI PARA FRENTE COLETAR (8 a 11s)
        else if (time < 11.0) {
            Intake.set(0.8);   // Puxa a bola
            IndexP.set(0.4); esteira.set(0.2);  // Esteira liga para levar a bola pro fundo e liberar espaço
            
            RightNEO1.set(0.4); 
            LeftNEO1.set(0.4); 
        } 
        // 6. DÁ RÉ NA MESMA QUANTIDADE (11 a 14s)
        else if (time < 14.0) {
            // Mantém o intake ligado para garantir que a bola não escape na ré
            Intake.set(0.3); 
            IndexP.set(0); esteira.set(0);
            
            // Volta (mesmo tempo que andou na fase 5, ou seja, 3 segundos)
            RightNEO1.set(-0.4); 
            LeftNEO1.set(-0.4); 
        } 
        // FINAL: SEGURANÇA
        else {
            allMotorsStop();
        }
    }

    private void logicaSaida5(double time) {
        // 1. TIRO INICIAL (0 a 3.0s)
        if (time < 3.0) { 
            Shooter.set(0.85);
            if (time > 1.2) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } 
        // 2. VIRA 90 GRAUS PARA A TRINCHEIRA (3.0s a 4.5s)
        else if (time < 4.5) {
            allMotorsStop(); // Limpa inércia
            RightNEO1.set(0.4); LeftNEO1.set(-0.4); // Vira 90°
        } 
        // 3. CURVA ARC TURN PARA FRENTE - COLETANDO (4.5s a 7.5s)
        else if (time < 7.5) {
            Intake.set(0.7); IndexP.set(0); esteira.set(0);
            LeftNEO1.set(0.5); RightNEO1.set(0.2); // Curva aberta para frente
        }
        // 4. MESMA CURVA, MAS VOLTANDO DE RÉ (7.5s a 10.5s)
        else if (time < 10.5) {
            Intake.set(0); // Para de coletar
            LeftNEO1.set(-0.5); RightNEO1.set(-0.2); // Ré fazendo o mesmo arco
        }
        // 5. VIRA 90 GRAUS DE VOLTA - APONTANDO PRO HUB (10.5s a 12.0s)
        else if (time < 12.0) {
            RightNEO1.set(0.4); LeftNEO1.set(-0.4); // Giro contrário ao passo 2
        }
        // 6. SEGUNDO TIRO (12.0s a 15.0s)
        else if (time < 15.0) {
            RightNEO1.set(0); LeftNEO1.set(0);
            Shooter.set(0.85);
            if (time > 13.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        }
        // 7. MANOBRA DE ESCALADA - RÉ ATÉ A TORRE (15.0s a 20.0s)
        else if (time < 20.0) {
            Shooter.set(0); IndexS.set(0); IndexP.set(0);
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); // Ré para encostar a garra
        } 
        else { 
            allMotorsStop(); 
        }
    }

    private void logicaNormalEsp(double time) {
        // 1. FASE DE POSICIONAMENTO E PRIMEIRO TIRO (0 a 6.0s)
        if (time < 6.0) { 
            if (time < 2.0) {
                RightNEO1.set(-0.4); LeftNEO1.set(-0.4); 
            } else {
                RightNEO1.set(0); LeftNEO1.set(0);  
            }
            Shooter.set(0.85); 
            if (time > 2.5) { IndexS.set(0.5); }
            if (time > 3.5) { IndexP.set(0.4); esteira.set(0.2); }
        } 
        // 2. GIRA 180° - INVERTIDO (8.0s a 10.0s)
        // Se o original era (-0.5, 0.5), o espelhado é (0.5, -0.5)
        else if (time < 8.0) { 
            Shooter.set(0.55); 
            IndexP.set(0); IndexS.set(0); esteira.set(0);
            RightNEO1.set(0.5); LeftNEO1.set(-0.5); 
        }
        // 3. ANDA PARA FRENTE E COLETA (8.0s a 10.0s)
        else if (time < 10.0) { 
            RightNEO1.set(0.4); LeftNEO1.set(0.4); 
            Intake.set(0.8); IndexP.set(0.2); esteira.set(0.2); 
        }
        // 4. GIRA 180° DE VOLTA - INVERTIDO
        else if (time < 12.0) { 
            RightNEO1.set(-0.5); LeftNEO1.set(0.5); 
            Intake.set(0); IndexP.set(0); esteira.set(0); 
            Shooter.set(0.95);
        }
        // 5. SEGUE UM POUCO PARA FRENTE (12.0s a 13.0s)
        else if (time < 13.0) {
            RightNEO1.set(0.4); LeftNEO1.set(0.4); 
        }
        // 6. VIRA 45° PARA A DIREITA (Espelhado da Esquerda)
        else if (time < 13.7) {
            RightNEO1.set(-0.4); LeftNEO1.set(0.4); 
        }
        // 7. PARA E ATIRA (13.7s a 15.5s)
        else if (time < 15.5) { 
            RightNEO1.set(0); LeftNEO1.set(0); 
            if (time > 14.0) { 
                IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2);
            }
        }
        // 8. COMPENSA OS 45° PARA A ESQUERDA (Volta ao eixo)
        else if (time < 16.2) {
            Shooter.set(0); IndexS.set(0); IndexP.set(0); esteira.set(0);
            RightNEO1.set(0.4); LeftNEO1.set(-0.4); 
        }
        // 9. CORRIDA FINAL PARA A RAMPA (16.2s a 20.0s)
        else if (time < 20.0) {
            RightNEO1.set(0.65); 
            LeftNEO1.set(0.65); 
        }
        else { 
            allMotorsStop(); 
        }
    }

    private void logicaSaida7(double time) {
        if (time < 3.0) { // 1. Ré e atira inicial
            if(time < 1.0) { RightNEO1.set(-0.3); LeftNEO1.set(-0.3); }

            else 
            { RightNEO1.set(0); LeftNEO1.set(0); }
            Shooter.set(0.85);

            if(time > 1.5) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } 
        else if (time < 4.5) { // 2. Vira para o HP (Giro de ~90°)
            RightNEO1.set(0.4); LeftNEO1.set(-0.4); Shooter.set(0.3);
        } 
        else if (time < 8.0) { // 3. Vai até o HP e Coleta
            Intake.set(0.8); IndexP.set(0.4); esteira.set(0.2);
            RightNEO1.set(0.4); LeftNEO1.set(0.4);
        } 
        else if (time < 11.0) { // 4. Dá ré voltando
            Intake.set(0); IndexP.set(0); esteira.set(0);
            RightNEO1.set(-0.4); LeftNEO1.set(-0.4);
        } 
        else if (time < 12.5) { // 5. Vira para o Hub (Direita)
            RightNEO1.set(-0.4); LeftNEO1.set(0.4);
        } 
        else if (time < 16.0) { // 6. Atira segunda carga
            RightNEO1.set(0); LeftNEO1.set(0);
            Shooter.set(0.85);
            if(time > 14.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } 
        /*else if (time < 20.0) { // 7. ESCALADA (Traseira na torre)
            Shooter.set(0); IndexS.set(0); IndexP.set(0); esteira.set(0);
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); // Ré final encostando
        }  */
        else { allMotorsStop(); }
    }

        private void logicaSaida8(double time) {
            if (time < 3.0) { // Atira parado
                Shooter.set(0.85);
                if (time > 1.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
            } else if (time < 4.0) { // Vira 90 Graus Direita
                allMotorsStop();
                RightNEO1.set(-0.4); LeftNEO1.set(0.4);
            } else if (time < 5.0) { // Frente 
                RightNEO1.set(0.3); LeftNEO1.set(0.3);
            } else if (time < 7.5) { // CURVA COAST MODE: Coletando
                Intake.set(0.7); IndexP.set(0.4); esteira.set(0.2);
                // Força no esquerdo, "Coast" no direito enviando quase 0
                LeftNEO1.set(0.4); RightNEO1.set(0.0); 
            } else if (time < 9.5) { // Dá ré
                Intake.set(0); IndexP.set(0); esteira.set(0);
                RightNEO1.set(-0.3); LeftNEO1.set(-0.3);
            } else if (time < 10.5) { // Vira Esquerda (Subir rampa)
                RightNEO1.set(0.4); LeftNEO1.set(-0.4);
            } else if (time < 12.5) { // Frente subindo rampa
                RightNEO1.set(0.4); LeftNEO1.set(0.4);
            } else if (time < 13.5) { // Vira para o Hub (Esquerda)
                RightNEO1.set(0.4); LeftNEO1.set(-0.4);
            } else if (time < 15.0) { // Atira
                RightNEO1.set(0); LeftNEO1.set(0);
                Shooter.set(0.85);
                if(time > 14.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
            } else { allMotorsStop(); }
        }

    private void logicaSaida9 (double time) {
        if (time < 3.0) { // Atira parado
            Shooter.set(0.85);
            if (time > 1.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } else if (time < 5.0) { // Giro e posicionamento
            allMotorsStop();
            RightNEO1.set(0.4); LeftNEO1.set(-0.4);
        } else if (time < 9.0) { // Curva Coast e Coleta
            Intake.set(0.7); IndexP.set(0.4); esteira.set(0.2);
            LeftNEO1.set(0.4); RightNEO1.set(0.1); 
        } else if (time < 12.0) { // Sobe rampa
            Intake.set(0);
            RightNEO1.set(0.5); LeftNEO1.set(0.5);
        } else if (time < 16.0) { // Atira no Hub
            RightNEO1.set(0); LeftNEO1.set(0);
            Shooter.set(0.85);
            if(time > 14.0) { IndexS.set(0.5); IndexP.set(0.4); esteira.set(0.2); }
        } else if (time < 20.0) { // ESCALADA
            Shooter.set(0); IndexS.set(0); 
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); 
        } else { allMotorsStop(); }
    }


    private void logicaSaida10(double time) {
        if (time < 2.0) { // 1. Sai da parede
            Shooter.set(0.3);
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3);
        } 
        else if (time < 16.0) { // 2. Mira e Atira via Câmera
            boolean prontoParaAtirar = prepararDisparo(time); 

            if (prontoParaAtirar) {
                IndexS.set(0.6); 
                if (time > 5.0) { IndexP.set(0.5); esteira.set(0.2); }
            } else {
                IndexS.set(0); IndexP.set(0);
            }
        } 
        else if (time < 20.0) { // 3. ESCALADA (Pós-mira)
            allMotorsStop();
            // Como o prepararDisparo termina apontando a traseira pro Hub...
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); // Ré até a torre
        }
        else {
            allMotorsStop();
        }
    }



    @Override
    public void teleopPeriodic() {}

    private void allMotorsStop() {
            RightNEO1.set(0); LeftNEO1.set(0);
            Shooter.set(0); Intake.set(0);
            IndexP.set(0); IndexS.set(0);
            esteira.set(0);
        }


    ///////////////VAZIO//////////////////
    @Override
    public void testInit() {}
    @Override
    public void testPeriodic() {}
    @Override
    public void simulationInit() {}
    @Override
    public void simulationPeriodic() {}
    @Override
    public void robotPeriodic() {}
    @Override
    public void disabledInit() {}
    @Override
    public void disabledPeriodic() {}


    ///////////|Telemtria|\\\\\\\\\\\
    private boolean prepararDisparo(double time) {
        var result = camera.getLatestResult();
        
        // Detecta a aliança atual para filtrar as Tags
        var alliance = DriverStation.getAlliance();
        int[] tagsAlvo = (alliance.isPresent() && alliance.get() == Alliance.Red) 
                        ? new int[]{3, 4, 7} : new int[]{1, 2, 6};

        if (result.hasTargets()) {
            PhotonTrackedTarget targetDesejado = null;

            for (var t : result.getTargets()) {
                if (ehTagDoMeuHub(t.getFiducialId(), tagsAlvo)) {
                    if (targetDesejado == null || Math.abs(t.getYaw()) < Math.abs(targetDesejado.getYaw())) {
                        targetDesejado = t;
                    }
                }
            }

            if (targetDesejado != null) {
                double yaw = targetDesejado.getYaw();
                
                if (Math.abs(yaw) > 1.5) { 
                    RightNEO1.set(yaw * 0.035);
                    LeftNEO1.set(-(yaw * 0.035));
                    return false; 
                } else {
                    RightNEO1.set(0); LeftNEO1.set(0);
                    
                    // CÁLCULO DE DISTÂNCIA E POTÊNCIA
                    double distancia = PhotonUtils.calculateDistanceToTargetMeters(
                        ALTURA_CAMERA_METROS, ALTURA_HUB_METROS,
                        ANGULO_MONTAGEM_RADIANOS, Units.degreesToRadians(targetDesejado.getPitch())
                    );

                    double forcaCerta = shotMap.get(distancia); 
                    Shooter.set(forcaCerta > 0 ? forcaCerta : 0.85);
                    return true; 
                }
            }
        }
        
        RightNEO1.set(0.2); LeftNEO1.set(-0.2);
        return false;
    }

    private boolean ehTagDoMeuHub(int id, int[] listaHub) {
        for (int idAlvo : listaHub) {
            if (id == idAlvo) {
                return true;
            }
        }
        return false;
    }
    }