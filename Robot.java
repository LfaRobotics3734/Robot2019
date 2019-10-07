/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//@author Danny Luo
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // declare variables
 // private AHRS ahrs;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private String m_autoSelected;
  //choose options (for autonomous)
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // public static final ADIS16470_IMU imu = new ADIS16470_IMU();

  // wheel motors:
  private VictorSPX motorFrontR;
  private VictorSPX motorFrontL;
  private VictorSPX motorBackR;
  private VictorSPX motorBackL;
  // lift motors
  private TalonSRX motorLift;
  private VictorSP motorDoor;


  private DigitalInput limitSwitch;
  private Compressor compressor;
 
  private Joystick _joy;
  
 // private AnalogInput input;

 // private int bits;
  private DoubleSolenoid _solenoid;
  private boolean[] buttonPressed = new boolean[8];
  private Joystick stick;

 // private Timer timer = new Timer();
  private final double k_updatePeriod = 0.005;
  private double liftWeightOffsetSpeed = 0;
  final int kTimeoutMs = 30;
	
  /**
 * If the measured travel has a discontinuity, Note the extremities or
 * "book ends" of the travel.
 */
private final boolean kDiscontinuityPresent = true;
private final int kBookEnd_0 = 910;		/* 80 deg */
private final int kBookEnd_1 = 1137;	/* 100 deg */

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("start robotInit method");
   //assign variables to PWM port
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto); //parameters: "Default Auto", "Default"
    m_chooser.addOption("My Auto", kCustomAuto); //parameters: "My Auto", "My Auto"
    //frc smart dashboard
    SmartDashboard.putData("Auto choices", m_chooser);
    //set motors into robot rio ports
    motorFrontL = new VictorSPX(0);
    motorFrontR = new VictorSPX(1);
    motorBackL = new VictorSPX(2);
    motorBackR = new VictorSPX(3);
    motorLift = new TalonSRX(4);
    motorDoor = new VictorSP(0);

    stick = new Joystick(5); //in reference to driver station slot
    limitSwitch = new DigitalInput(1);
    _solenoid = new DoubleSolenoid(5, 4, 3); // first number is the PCM ID (usually zero), second number is the solenoid channel
    compressor = new Compressor(5);
    
    /*
    /* Factory Default Hardware to prevent unexpected behaviour */
    motorLift.configFactoryDefault();
		/* Seed quadrature to be absolute and continuous */
		initQuadrature();
		/* Configure Selected Sensor for Talon */
		motorLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	// Feedback
											                  0, 											                  // PID ID
                                        kTimeoutMs);                              // Timeout         
  }

  /**
	 * Get the selected sensor register and print it 
	 */
  public void disabledPeriodic() {
		/**
		 * When button is pressed, seed the quadrature register. You can do this
		 * once on boot or during teleop/auton init. If you power cycle the 
		 * Talon, press the button to confirm its position is restored.
     * 
     * !!!
     * getRawButton(1) is already used for offset? Should we think about
     * change this to another button in order to incorporate Motor Encoders?
		 */
		if (stick.getRawButton(4))  {
			initQuadrature();
		}

		/**
		 * Quadrature is selected for soft-lim/closed-loop/etc. initQuadrature()
		 * will initialize quad to become absolute by using PWD
		 */
		int selSenPos = motorLift.getSelectedSensorPosition(0);
		int pulseWidthWithoutOverflows = 
				motorLift.getSensorCollection().getPulseWidthPosition() & 0xFFF;

		/**
		 * Display how we've adjusted PWM to produce a QUAD signal that is
		 * absolute and continuous. Show in sensor units and in rotation
		 * degrees.
		 */
    SmartDashboard.putString("pulseWidPos:", ""+selSenPos);
    SmartDashboard.putString("pulseWidDeg",  ToDeg(pulseWidthWithoutOverflows) +
    "   =>    " + "selSenDeg:" + ToDeg(selSenPos));
	
		
  }
  
  /**
	 * Seed the quadrature position to become absolute. This routine also
	 * ensures the travel is continuous.
	 */
	public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = motorLift.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (kDiscontinuityPresent) {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		motorLift.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
	}

	/**
	 * @param units CTRE mag encoder sensor units 
	 * @return degrees rounded to tenths.
	 */
	String ToDeg(int units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return "" + deg;
	}


  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //TBD


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //complicated auto?
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      //call the method for control
      this.teleopPeriodic();
      break;
    case kDefaultAuto:
    default:
      // call the method for control
      this.teleopPeriodic();
      break;
    }
  }

  public void teleopInit() {
    System.out.println("Default teleopInit() method... Overload me!");
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    //Everything after Auto-period

    //Pneumatics
    if(stick.getRawButton(5))
    {
      compressor.setClosedLoopControl(false);
    }else{
      compressor.setClosedLoopControl(true);
    }
    if(stick.getRawButton(7))
    {
      _solenoid.set(DoubleSolenoid.Value.kReverse);
    }else if(stick.getRawButton(8)){
      _solenoid.set(DoubleSolenoid.Value.kForward);
    }else{
      _solenoid.set(DoubleSolenoid.Value.kOff);
    }
   
    SmartDashboard.putBoolean("button 7", stick.getRawButton(7));
    SmartDashboard.putBoolean("button 8", stick.getRawButton(8));
    SmartDashboard.putBoolean("pressure switch", compressor.getPressureSwitchValue());







    //  Door code
    //  Press X and B button to rotate the door
    if (stick.getRawButton(2)) {
      motorDoor.set(0.55);
    } else if (stick.getRawButton(3)) {
      motorDoor.set(-0.55);
    } else {
      motorDoor.set(0);
    }
    SmartDashboard.putBoolean("switch", limitSwitch.get());



    //  Lift weight offset code
    //  Press A to offset gear weight or reverse to no offset
    if (singlePressDetection(1)) { 
      if (liftWeightOffsetSpeed == 0.1) {
        liftWeightOffsetSpeed = 0;
        System.out.println("Offset of Lift at 0");
      } else {
        liftWeightOffsetSpeed = 0.1;
        System.out.println("Offset of Lift at 0.1");
      }
    }


    //  Lift code 
    //  Press L/R triggers to move lift UP/DOWN respectively
    double rightTrigger = stick.getRawAxis(3);
    double leftTrigger = stick.getRawAxis(2);
    //maybe change the sensitivity to 0.1 for lift?
    if (stick.getRawAxis(3) >= 0.2) { // right trigger active, down
      //was sqrt
      motorLift.set(ControlMode.PercentOutput, -rightTrigger);

    } else if (stick.getRawAxis(2) >= 0.2) { // left trigger is active, up
      
      if (!limitSwitch.get()) {
        //Limit switch unpressed: return FALSE:
        motorLift.set(ControlMode.PercentOutput, leftTrigger);
      } else {
        //limit switch pressed: return TRUE: 
        motorLift.set(ControlMode.PercentOutput, 0);
      }

    } else { // no trigger is active, set lift offset speed
      motorLift.set(ControlMode.PercentOutput, liftWeightOffsetSpeed);
    }

    //  Emergency/Reset all motors code
    //  Press Y button to set all motors to 0
    /*
    if (singlePressDetection(4)) {
      motorFrontL.set(ControlMode.PercentOutput, 0);
      motorFrontR.set(ControlMode.PercentOutput, 0);
      motorBackL.set(ControlMode.PercentOutput, 0);
      motorBackR.set(ControlMode.PercentOutput, 0);
      motorLift.set(ControlMode.PercentOutput, 0);
      motorDoor.set(0);
    }
    */
    //  MecanumDrive code
    double x = 0, y = 0, z = 0;
    //  Stick
    //  Maybe reduce detection tolerance interval?

    if (!(stick.getX() < 0.2 && stick.getX() > -0.2)) {
      z = stick.getX();
    }
    if (!(stick.getY() < 0.2 && stick.getY() > -0.2)) {
      x = -stick.getY();
    }
    if (!(stick.getRawAxis(4) < 0.2 && stick.getRawAxis(4) > -0.2)) {
      y = stick.getRawAxis(4);
    }

    //  Logic
    double[] w = new double[4];
    w[0] = x + y + z;
    w[1] = x - y - z;
    w[2] = x - y + z;
    w[3] = x + y - z;

    double wMax = 1;

    for (int i = 0; i < 4; i++) {
      if (w[i] > wMax) {
        wMax = w[i];
      }
    }
    //change from 2 to 1.
    /*
    double fL=w[0] / (wMax*1.4 );
    double fR=w[1] / (wMax*1.4 );
    double bL=w[2] / (wMax*1.4 );
    double bR=w[3] / (wMax*1.4 );

    motorFrontL.set(ControlMode.PercentOutput, maxSpeedCheck(Math.pow(fL,3)));
    motorFrontR.set(ControlMode.PercentOutput, -maxSpeedCheck(Math.pow(fR,3)));
    motorBackL.set(ControlMode.PercentOutput, maxSpeedCheck(Math.pow(bL,3)));
    motorBackR.set(ControlMode.PercentOutput, -maxSpeedCheck(Math.pow(bR,3)));
    */
    
    motorFrontL.set(ControlMode.PercentOutput, maxSpeedCheck(w[0] / (wMax*2 )));
    motorFrontR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[1] / (wMax*2 )));
    motorBackL.set(ControlMode.PercentOutput, maxSpeedCheck(w[2] / (wMax*2 )));
    motorBackR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[3] / (wMax*2 )));
    

    //do we need this? Most likely keep it
    Timer.delay(k_updatePeriod);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  
  //  return true when button is pressed for the first time and false while the
  //  button is held
  //  button:00000011111110000011111111100
  //  output:00000010000000000010000000000
  //  Function: so that driver doesn't need to hold down the button coninuously for a single command
  
  public boolean singlePressDetection(int INDEX) {
    if (!stick.getRawButton(INDEX + 1)) {
      buttonPressed[INDEX] = false;
      System.out.println("continuous button press in singlePressDetection()");
    }
    if (stick.getRawButton(INDEX + 1) && (buttonPressed[INDEX] == false)) {
      buttonPressed[INDEX] = true;
      return true;
    } //else {
      return false;
   // }
  }

  public double maxSpeedCheck(double inputSpeed) {
    if (inputSpeed > 1) {
      return 1.0;
    } else {
      return inputSpeed;
    }
  }
}