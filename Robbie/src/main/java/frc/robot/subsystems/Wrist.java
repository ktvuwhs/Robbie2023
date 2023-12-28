package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {

    private enum ArmState{
        OFF,      // off...
        JOG,      // motor power (e.g. when we need to reduce it when its going down bc of gravity (overshoot, etc...))
        POSITION, // gets position of wrist (aka angle)
        ZERO      // default position
    }
    
    private CANSparkMax armMaster = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_MASTER, MotorType.kBrushless);
    private CANSparkMax armSlave  = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_SLAVE,  MotorType.kBrushless);

    private ArmState state = ArmState.OFF; //defaults to being off

    private double jogValue = 0;                          // off
    private Rotation2d setpoint = new Rotation2d();       // make object of rotation2d
    //rotation2d does math 4 us

    private static Wrist instance = null;  // so we can call that instance of a class across many files

    private Wrist() {
        configMotors();                    // now motor belong to wrist
    }

    // static mean i can call it straight from the class instead of making an object
    // e.g., Wrist.getInstance()
    public static Wrist getInstance() {  // Singleton design pattern
        if (instance == null) { instance = new Wrist(); }
        return instance;
    }

    @Override
    public void periodic() {    // periodic ~ constantly
        logData();
        
        switch(state){          // switch statements ~ if-else clone
            case OFF:
                set(0);
                break;
            case JOG:
                set(jogValue);
                break;
            case POSITION:
                goToSetpoint();
                break;
            case ZERO:
                zero();
                break;
        }   
        
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(armMaster.getEncoder().getPosition() / ArmConstants.ARM_GEAR_RATIO);
        // getEncoder ~ # motor turns
        // math: getEncoder / gearRatio --> rotation(2d) 
    }

    public void zero(){
      if(!zero_indicator)
        jog(0.1);   // whatever speed is comfortable
      else{
        zeroEncoders();
        jog(0);
        setState(SubsystemState.STOW);
      }
    }

    public void set(double value){      //set speed of speed controllor (motor)
        armMaster.set(value);           //value = speed (from -1 to 1)
    }
    
    public void setJog(double jog){     //...sets jog
        jogValue = jog;
    }

    private void goToSetpoint(){     
        armMaster.getPIDController().setReference(setpoint.getRotations() * ArmConstants.ARM_GEAR_RATIO, ControlType.kSmartMotion);
    }       //setReference ~ know where we at, just put in where go and it go
            //aka our setpoint (it take in rotations... how many rotations does it take to get to my setpoint?)
            //gear ratio ~ # of motor turns per rotation
            //math: getRotations * gearRatio --> motor turns

    public void setSetpoint(Rotation2d setpoint){   //...sets setpoint (arm position)
        setState(ArmState.POSITION);
        this.setpoint = setpoint;
    }

    public Rotation2d getSetpoint(){                //...gets setpoint
        return setpoint;
    }

    public void setState(ArmState state){           //...sets state
        this.state = state;
    }

    public ArmState getState(){
        return state; 
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint.minus( this.getAngle() ).getRotations()) < ArmConstants.TOLERANCE.getRotations();
        // I have reached the setpoint if (distance from setpoint) < TOLERANCE
        // as in, I am "close enough" to my setpoint
        // in math: |setpoint - this.angle()| < TOLERANCE
        // english: setpoint minus this angle from this rotation is less than the tolerance
        // tolerance ~ how much can u take
    }

    public void logData(){
        SmartDashboard.putNumber("Wrist Position", getAngle().getDegrees());
        SmartDashboard.putBoolean("Wrist At Setpoint", atSetpoint());
        SmartDashboard.putString("Wrist State", getState().toString());
        SmartDashboard.putNumber("Wrist Setpoint", getSetpoint().getDegrees());
    }


    public void configMotors(){
        armMaster.restoreFactoryDefaults();                           //reset, blank slate
        armSlave.restoreFactoryDefaults();

        armMaster.setInverted(false);                      //invert so motor face same direction
        armSlave.setInverted(!armMaster.getInverted());

        armMaster.setIdleMode(IdleMode.kBrake);                       //when robot off = in idle mode
        armSlave.setIdleMode(armMaster.getIdleMode());

        armMaster.setSmartCurrentLimit(40, 40);  //freeLimit = max current when running freely
        armSlave.setSmartCurrentLimit(40, 40);   //stallLimit = max current when forcing (don't overdo it)

        SparkMaxPIDController armController = armMaster.getPIDController();             //pid = tuner
        armController.setP(ArmConstants.ARM_kP);
        armController.setD(ArmConstants.ARM_kD);

        armController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        armController.setSmartMotionMaxAccel(ArmConstants.MAX_ACCELERATION, 0);
        armController.setSmartMotionMaxVelocity(ArmConstants.MAX_VELOCITY, 0);

        armSlave.follow(armMaster);
    }
}
