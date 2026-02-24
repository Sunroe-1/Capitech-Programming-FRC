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

//Importações da Rev para utilização dos SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Newton;//Import que permite a utilizaçao da unidade de medida em ''Newtons''

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.revrobotics.RelativeEncoder;//import da Rev para utiização do encoder

import com.studica.frc.AHRS;//import da biblioteca utilizada para o sensor NAVx2


public class AutonomousPrime extends TimedRobot {

  private static final String kDefaultAuto = "Auto 0"; // 

  private static final String kCustomAuto = "Auto 1"; // 

  private static final String kAuto2 = "Auto 2"; //

  private static final String kAuto3 = "Auto 3"; // 

  private static final String kAuto4 = "Auto 4"; // 

  private static final String kAuto5 = "Auto 5"; // 

  private static final String kAuto6 = "Auto 6"; //
  
  private String m_autoSelect; // Guarda qual autônomo foi escolhido no dashboard

  private final SendableChooser<String> m_chooser = new SendableChooser<>(); // Menu no dashboard para selecionar o autônomo

  Timer timer = new Timer();  // Timer para controle de tempo

  DigitalInput botao = new DigitalInput(1);  // Entrada digital

  private Command m_autonomousCommand; // Comando que roda durante o autônomo


  private final RobotContainer m_robotContainer;// Container onde ficam subsistemas e comandos
  
      
  private final SparkMaxConfig leftConfig = new SparkMaxConfig();  // Configurações esquerda e direita 
  private final SparkMaxConfig rightConfig = new SparkMaxConfig(); //dos motores SparkMax
  
  private final SparkMaxConfig shooterConfig = new SparkMaxConfig(); //Configurações do motor do Shooter

  private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); //Configurações do motor do Intake

  private final SparkMaxConfig indexPConfig = new SparkMaxConfig(); //Configurações do motor do Primeiro Indexer

  private final SparkMaxConfig indexSConfig = new SparkMaxConfig(); //COnfigurações do motor do segundo Indexer

//Motores direito e esquerdo, respectivamente, ambos tipo Brushed
  private final SparkMax RightNEO1 = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax LeftNEO1 = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax Shooter = new SparkMax(3, MotorType.kBrushed); //Motor do Shooter
  
  private final SparkMax Intake = new SparkMax(4, MotorType.kBrushed); //Motor do Intake
  
  private final SparkMax IndexP = new SparkMax(5, MotorType.kBrushed); //Motor do Primeiro indexer
  
  private final SparkMax IndexS = new SparkMax(6, MotorType.kBrushless); //Motor do Segundo Indexer


  public AutonomousPrime() {
    
    m_robotContainer = new RobotContainer();
  }


  public void robotInit(){

m_chooser.setDefaultOption("AutoProto", kDefaultAuto);
m_chooser.addOption("Saída1", kCustomAuto);
m_chooser.addOption("Auto 2", kAuto2);
m_chooser.addOption("Auto 3", kAuto3);
m_chooser.addOption("Auto 4", kAuto4);
m_chooser.addOption("Auto 5", kAuto5);
m_chooser.addOption("Auto 6", kAuto6);

SmartDashboard.putData("Auto Choices", m_chooser);

leftConfig.idleMode(IdleMode.kBrake).inverted(false);
rightConfig.idleMode(IdleMode.kBrake).inverted(true);

shooterConfig.idleMode(IdleMode.kCoast);

RightNEO1.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
LeftNEO1.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

Shooter.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }
 

@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
//m_autonomousCommand = m_robotContainer.getAutonomousCommand();

/* 
if (m_autonomousCommand != null) {
CommandScheduler.getInstance().schedule(m_autonomousCommand);
}*/

m_autoSelect = m_chooser.getSelected();
System.out.println("AutoProto" + m_autoSelect);

timer.reset();
timer.start();
allMotorsStop();

  }

  
  @Override 
  public void autonomousPeriodic() {
      double time = timer.get();

      switch (m_autoSelect) {
case kDefaultAuto:
logicaAutoProto(time);
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
       }
 
   }


private void logicaAutoProto(double time) {
   // 1. FASE DE POSICIONAMENTO E TIRO (0 a 6.0s)
  if (time < 6.0) { 
            
  // --- MOVIMENTAÇÃO DO CHASSI ---
  if (time < 2.0) {
  // Dá ré para se afastar (0s a 2s)
  RightNEO1.set(-0.3); LeftNEO1.set(-0.3); 
  } else {
  // PARA TOTALMENTE antes de começar o fluxo da bola (2s+)
  RightNEO1.set(0); LeftNEO1.set(0);       
  }

  // =====SHOOTER E ALIMENTAÇÃO (Esteira e Indexer)=====
   Shooter.set(0.85); // Shooter já liga desde o tempo 0 para ganhar rotação

  if (time > 2.5) {
  // APÓS 2.5s: O robô já parou há meio segundo (estabilizado)
  IndexS.set(0.6); // Libera a primeira bola para o shooter
  }
            
  if (time > 3.5) {
  // APÓS 3.5s: A esteira (IndexP) traz a segunda bola
  IndexP.set(0.5); 
  }
  } 
  // 2. GIRA 180° PARA BUSCAR NOVAS FUELS (6s a 7.5s)
  else if (time < 7.5) { 
  Shooter.set(0.3); // Abaixa o shooter para economizar bateria
  IndexP.set(0); IndexS.set(0); // Para a alimentação durante o giro
  RightNEO1.set(-0.4); LeftNEO1.set(0.4); 
  }
  // 3. ANDA PARA FRENTE E COLETA (6.5 a 9.5s)
  else if (time < 9.5) { 
  RightNEO1.set(0.5); LeftNEO1.set(0.5); // Anda
            
  Intake.set(0.7);  // Puxa do chão
  IndexP.set(0.5);  // Esteira leva a bola para cima...
  IndexS.set(0);    // Segura a bola antes do shooter.
  }
  // 4. GIRA 180° DE VOLTA (9.5 a 11.0s)
  else if (time < 11.0) { 
  RightNEO1.set(0.4); LeftNEO1.set(-0.4); // Gira de volta
            
  Intake.set(0); IndexP.set(0); // Desliga a coleta e a esteira para a bola não mexer
  Shooter.set(0.85); // Acelera o shooter para o próximo tiro
  }
  // 5. PARA O CHASSI E ATIRA (11.0 a 15.0s)
  else if (time < 15.0) { 
  RightNEO1.set(0); LeftNEO1.set(0); // Breca o robô
            
  // Espera 1 segundo para o chassi estabilizar e o shooter pegar rotação
  if (time > 12.0) { 
  IndexS.set(0.6); 
  IndexP.set(0.6); 
  }
  }
  // FIM DO TEMPO
  else { 
  allMotorsStop(); 
  }
  }
 
   private void logicaSaida1(double time) {
    if (time < 6.0) {

  if (time < 2.0){
  RightNEO1.set(-0.3); LeftNEO1.set(-0.3);
  }else{
  RightNEO1.set(0); LeftNEO1.set(0);
  }
            Shooter.set(0.85);
            // Liga esteira e indexer para atirar
            if (time > 1.5) { IndexS.set(0.6); IndexP.set(0.5); }
        } 
        else if (time < 7.0) {
            // Desliga os indexers e diminui a potência do shooter para economizar bateria
            Shooter.set(0.3); IndexP.set(0); IndexS.set(0); Intake.set(0);
            
            // Dá ré (como o intake é a frente, dar ré afasta ele da parede)
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); 
        } 
        else { 
            allMotorsStop(); 
        }
    }

 private void logicaSaida2(double time) {
    if (time < 5.0) { // PASSO 1 e 2
        if (time < 2.0) {
            RightNEO1.set(-0.3); LeftNEO1.set(-0.3); // Vai para trás
        } else {
            RightNEO1.set(0); LeftNEO1.set(0); // Para para atirar
        }
        Shooter.set(0.85);
        if (time > 2.0) { IndexS.set(0.6); IndexP.set(0.5); } // Atira após parar
    } 
    else if (time < 6.5) { // PASSO 3: Gira para a torre (Ajuste o 6.5 se precisar)
        Shooter.set(0.3); IndexP.set(0); IndexS.set(0);
        RightNEO1.set(-0.3); LeftNEO1.set(0.3); 
    } 
    else if (time < 9.0) { // PASSO 4: Estaciona (Ajuste o 9.0 para a distância)
        RightNEO1.set(0.3); LeftNEO1.set(0.3); 
    }
    else {
        allMotorsStop(); // Fim do autônomo
    }
}

private void logicaSaida3(double time) {
// FASE 1: AFASTAR DO HUB (0 a 2s)
    if (time < 2.0) {
        // Ambos os lados negativos para dar ré (afastar o intake do Hub)
        RightNEO1.set(-0.3);  LeftNEO1.set(-0.3); 
       
        // Shooter já acelera para ganhar inércia
        Shooter.set(0.85); 
        IndexS.set(0); IndexP.set(0);
    } 
    // FASE 2: PARAR E ATIRAR (2s a 5s)
    else if (time < 5.0) {
        // Chassi parado para máxima precisão no tiro
        RightNEO1.set(0); 
        LeftNEO1.set(0); 
        
        Shooter.set(0.85);
        // Staging: Indexer superior primeiro, esteira depois para não engasgar
        if (time > 2.5) { IndexS.set(0.6); }
        if (time > 3.5) { IndexP.set(0.5); }
    } 
    // FASE 3: GIRAR PARA A TORRE (5s a 6.5s)
    else if (time < 6.5) {
        Shooter.set(0); IndexP.set(0); IndexS.set(0);
        
        // Giro no eixo: Lados com sinais opostos
        // Se ele girar para o lado errado, inverta os sinais abaixo
        RightNEO1.set(-0.4); 
        LeftNEO1.set(0.4); 
    } 
    // FASE 4: SEGUIR EM FRENTE E ESTACIONAR (6.5s a 9s)
    else if (time < 9.0) {
        // Ambos os lados positivos para andar para frente (direção do intake)
        RightNEO1.set(0.3); 
        LeftNEO1.set(0.3); 
    }
    // FINAL: SEGURANÇA
    else {
        allMotorsStop(); // Para todos os motores após o tempo definido
    }
}

private void logicaSaida4(double time){
// 1. ATIRAR IMEDIATAMENTE (0 a 4s)
    if (time < 4.0) {
        Shooter.set(0.85);
        if (time > 1.5) { 
            IndexS.set(0.6); // Indexer superior
            IndexP.set(0.5); // Esteira
        }
    } 
    // 2. PRIMEIRO GIRO DE 90° (1/4 de círculo) (4 a 5s)
    else if (time < 5.0) {
        // Desliga tiro para poupar energia
        Shooter.set(0); IndexS.set(0); IndexP.set(0);
        
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
        Intake.set(0.7);   // Puxa a bola
        IndexP.set(0.4);   // Esteira liga para levar a bola pro fundo e liberar espaço
        
        RightNEO1.set(0.3); 
        LeftNEO1.set(0.3); 
    } 
    // 6. DÁ RÉ NA MESMA QUANTIDADE (11 a 14s)
    else if (time < 14.0) {
        // Mantém o intake ligado para garantir que a bola não escape na ré
        Intake.set(0.4); 
        IndexP.set(0);
        
        // Volta (mesmo tempo que andou na fase 5, ou seja, 3 segundos)
        RightNEO1.set(-0.3); 
        LeftNEO1.set(-0.3); 
    } 
    // FINAL: SEGURANÇA
    else {
        allMotorsStop();
    }
}

private void logicaSaida5(double time) {
   // 1. ATIRAR IMEDIATAMENTE (Igual ao original)
    if (time < 4.0) {
        Shooter.set(0.85);
        if (time > 1.5) { 
            IndexS.set(0.6); 
            IndexP.set(0.5); 
        }
    } 
    // 2. PRIMEIRO GIRO DE 90°(4 a 5s)
    else if (time < 5.0) {
        Shooter.set(0); IndexS.set(0); IndexP.set(0);
        
        // ANTES: Right(-0.4) Left(0.4) -> Girava para um lado
        // AGORA: Right(0.4) Left(-0.4) -> Gira para o lado oposto
        RightNEO1.set(0.4); 
        LeftNEO1.set(-0.4); 
    } 
    // 3. VAI PARA FRENTE (Igual ao original)
    else if (time < 7.0) {
        RightNEO1.set(0.3); 
        LeftNEO1.set(0.3); 
    } 
    // 4. SEGUNDO GIRO DE 90° - ESPELHADO (7 a 8s)
    else if (time < 8.0) {
        // Invertido também para manter a simetria do "L"
        RightNEO1.set(0.4); 
        LeftNEO1.set(-0.4); 
    } 
    // 5. LIGA INTAKE E VAI PARA FRENTE COLETAR (8 a 11s)
    else if (time < 11.0) {
        Intake.set(0.7);   
        IndexP.set(0.4);   
        
        RightNEO1.set(0.3); 
        LeftNEO1.set(0.3); 
    } 
    // 6. DÁ RÉ (11 a 14s)
    else if (time < 14.0) {
        Intake.set(0.4); 
        
        RightNEO1.set(-0.3); 
        LeftNEO1.set(-0.3); 
    } 
    else {
        allMotorsStop();
    }
}

   @Override
  public void teleopInit() {

   /*  if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }*/

    allMotorsStop();
  }

  @Override
  public void teleopPeriodic() {}

 private void allMotorsStop() {
        RightNEO1.set(0); LeftNEO1.set(0);
        Shooter.set(0); Intake.set(0);
        IndexP.set(0); IndexS.set(0);
    }

@Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
}