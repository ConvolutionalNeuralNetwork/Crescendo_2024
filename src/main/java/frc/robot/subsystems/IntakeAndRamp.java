package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.IntakeAndRampConstants;


public class IntakeAndRamp extends SubsystemBase {
    

    CANSparkMax intakeMotor= new CANSparkMax(IntakeAndRampConstants.intakeMotorID, MotorType.kBrushless);
    CANSparkMax rampMotor = new CANSparkMax(IntakeAndRampConstants.rampMotorID, MotorType.kBrushless);

    RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    RelativeEncoder rampEncoder = rampMotor.getEncoder();




    public IntakeAndRamp() {
        intakeMotor.restoreFactoryDefaults();
        rampMotor.restoreFactoryDefaults();

        
        intakeEncoder.setPosition(0);
        rampEncoder.setPosition(0);

        intakeMotor.follow(intakeMotor);
        rampMotor.follow(rampMotor);

    }

    public void setIntakeMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setRampMotorSpeed(double speed) {
        rampMotor.set(speed);
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
