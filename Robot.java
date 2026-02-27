// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Importações do WPILib
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController; // Adicionada para o controle
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Importações da REV Robotics para o motor NEO (Temporada 2025+)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * A classe principal do seu robô.
 */
public class Robot extends TimedRobot {
  // --- SEÇÃO DE CONTROLE AUTÔNOMO (do seu código original) ---
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // --- SEÇÃO DE CONTROLE DO MOTOR E OPERADOR ---
  // Constantes para as portas e IDs. MUDE ESTES VALORES!
  private static final int MOTOR_NEO_CAN_ID = 1;     // CAN ID do seu SPARK MAX
  private static final int CONTROLE_PORTA_USB = 0;   // Porta USB do seu controle

  // Declaração dos objetos do motor e do controle
  private SparkMax m_motorNEO;
  private XboxController m_controle;


  /**
   * Esta função é executada quando o robô é ligado. Use-a para inicialização.
   */
  @Override
  public void robotInit() { // O construtor `public Robot()` foi movido para cá para seguir o padrão mais moderno do WPILib.
    // Configuração do seletor de autônomo (do seu código original)
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // --- INICIALIZAÇÃO DO MOTOR E CONTROLE ---
    // 1. Inicializa o objeto do motor
    m_motorNEO = new SparkMax(MOTOR_NEO_CAN_ID, MotorType.kBrushless);

    // 2. Configura o motor para um estado conhecido e seguro
    m_motorNEO.restoreFactoryDefaults();      // Limpa configurações antigas
    m_motorNEO.setInverted(false);            // Mude para 'true' se o motor girar ao contrário
    m_motorNEO.setIdleMode(IdleMode.kBrake);  // Freia o motor quando o comando é zero

    // 3. Inicializa o controle
    m_controle = new XboxController(CONTROLE_PORTA_USB);
  }

  /**
   * Esta função é chamada a cada 20ms, independentemente do modo.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * Esta função é chamada uma vez no início do modo autônomo.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * Esta função é chamada periodicamente durante o modo autônomo.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Coloque o código do seu autônomo customizado aqui
        break;
      case kDefaultAuto:
      default:
        // Coloque o código do seu autônomo padrão aqui
        break;
    }
  }

  /**
   * Esta função é chamada uma vez quando o modo teleoperado é habilitado.
   */
  @Override
  public void teleopInit() {
      // Você pode colocar aqui código para ser executado uma vez no início do teleop,
      // como resetar encoders ou garantir que o motor esteja parado.
      m_motorNEO.set(0);
  }

  /**
   * Esta função é chamada periodicamente durante o controle do operador.
   * É aqui que o robô é controlado pelo piloto.
   */
  @Override
  public void teleopPeriodic() {
    // --- LÓGICA DE CONTROLE DO MOTOR ---
    // 1. Pega o valor do eixo Y do analógico esquerdo do controle (-1.0 a 1.0)
    // O sinal negativo inverte o eixo para que "para frente" no controle seja positivo.
    double velocidade = -m_controle.getLeftY();

    // 2. Comanda o motor com o valor lido
    m_motorNEO.set(velocidade);
  }

  // --- Funções de Disabled e Test (não modificadas) ---

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
  
  // As funções de simulação não são padrão no template, mas foram mantidas
  @Override
  public void simulationInit() {}
  
  @Override
  public void simulationPeriodic() {}
}