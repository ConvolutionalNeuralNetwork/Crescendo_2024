package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    

    CANSparkMax leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorID, MotorType.kBrushless);
    CANSparkMax rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorID, MotorType.kBrushless);

    RelativeEncoder leftEncoder = leftMotor.getEncoder();
    RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private SparkPIDController rightPIDController;
    private SparkPIDController leftPIDController;

    private double targetVelocity = 0; 
    private int rollingAvg = 0;



    public Shooter() {

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        
        // leftEncoder.setPosition(0);
        // rightEncoder.setPosition(0);

        leftMotor.follow(rightMotor);
        // rightMotor.follow(rightMotor);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftPIDController = leftMotor.getPIDController();
        rightPIDController = rightMotor.getPIDController();

        leftPIDController.setP(ShooterConstants.proportialPIDConstant);
        leftPIDController.setI(ShooterConstants.integralPIDConstant);
        leftPIDController.setD(ShooterConstants.derivativePIDConstant);
        leftPIDController.setIZone(ShooterConstants.integralPIDConstant);
        leftPIDController.setFF(ShooterConstants.leftFeedForwardPIDConstant);
        leftPIDController.setOutputRange(ShooterConstants.minPIDOutput, ShooterConstants.maxPIDOutput);
    
        rightPIDController.setP(ShooterConstants.proportialPIDConstant);
        rightPIDController.setI(ShooterConstants.integralPIDConstant);
        rightPIDController.setD(ShooterConstants.derivativePIDConstant);
        rightPIDController.setIZone(ShooterConstants.integralPIDConstant);
        rightPIDController.setFF(ShooterConstants.rightFeedForwardPIDConstant);
        rightPIDController.setOutputRange(ShooterConstants.minPIDOutput, ShooterConstants.maxPIDOutput);
        stop();

        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        leftPIDController.setReference(targetVelocity, ControlType.kVelocity);
        rightPIDController.setReference(targetVelocity, ControlType.kVelocity);
      }
    
      public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
      }
    
      public void stop() {
        setSpeed(0.0);
      }
    
      // Finds the average velocity of the two motors 
      public double getVelocity() {
        double sum = leftEncoder.getVelocity() + rightEncoder.getVelocity();
        double average = sum / 2;
        return average;
      }
    

    


    
  
        
    
      }
