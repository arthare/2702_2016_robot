#include "WPILib.h"
#include "Interfaces.h"
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>
#include <mutex>


class SmoothMotionManager
{
float lastPos;
float lastTime;

public:
	SmoothMotionManager(float initialPos, float time):lastPos(initialPos),
													lastTime(time)
	{

	}
	bool tick(const float currentPos,
					const float targetPos,
					const float powerIfUnder,
					const float powerIfOver,
					const float accelRate,
					const float currentTime,
					const float stoppingSpeedTolerance,
					const float stoppingDistanceTolerance,
					float* powerOutput)
	{
		const float deltaTime = currentTime - lastTime;
		lastTime = currentTime;

		const float distToTarget = fabs(targetPos - currentPos);

		if (deltaTime == 0){
			return false;
		}
		const float curSpeed = (currentPos - lastPos)/deltaTime;
		lastPos = currentPos;

		const float absCurSpeed = fabs(curSpeed);
		if (distToTarget < stoppingDistanceTolerance && absCurSpeed < stoppingSpeedTolerance)
		{
			return true;
		}

		const float maxa = accelRate;
		float targetSpeed = sqrt(2*maxa*distToTarget);

		if(currentPos > targetPos)
		{
			targetSpeed = targetSpeed * -1;
		}

		if(curSpeed < targetSpeed)
		{
			*powerOutput = powerIfUnder;
		}
		else
		{
			*powerOutput = powerIfOver;
		}
		return false;
	}
};


class SimpleRobotDemo;
void receiveTask(SimpleRobotDemo *myRobot);
int receivePacket();

struct networkData {
	int x;
	int y;
};

int networkCount =0 ;
#if PRACTICE_ROBOT
	SEM_ID g_semaphore;
	#define ACQUIRE_MUTEX() Synchronized lock(g_semaphore)
#else
	std::mutex g_i_mutex;  // protects g_i
	#define ACQUIRE_MUTEX() std::lock_guard < std::mutex > lock(g_i_mutex)
#endif
networkData lastNetworkData;

int endianFlip32(int input)
{
	const unsigned char* pInput = (const unsigned char*)&input;

	int output = 0;
	unsigned char* pOutput = (unsigned char*)&output;
	for(int x = 0;x < 4; x++)
	{
		pOutput[x] = pInput[3 - x];
	}
	return output;
}
#ifdef PRACTICE_ROBOT
	typedef AnalogChannel AnalogType;
#else
	typedef AnalogInput AnalogType;
#endif
class VoltagePIDInput: public PIDSource
{
public:
	VoltagePIDInput(AnalogType& analog):analog(analog)
	{
	}
	virtual double PIDGet()
	{
		float ret = analog.GetVoltage();
		return ret;
	}
	AnalogType& analog;
};



enum PICKUP_STATE {
  PICKUP,
  PICKUP_STOWING,
  PREPSHOOT,
  PREPSHOOT_STOWING, // prepshoot_stowing differs from normal stowing because we need to keep roller and shooter running
  STOWED,
  PREPSHOOT_HOLDON,
};
class PickupStateMachine {
public:
  PickupStateMachine(PICKUP_STATE state) :
    myState(state) {
  }
  PickupStateMachine() :
    myState(PICKUP_STOWING) {

  }
  void DoStateAction(PICKUP_STATE newState,
    const bool shooterWasRunning,
    const int currentTime,
    float* pickupTarget,
    float* rollerSpeed,
    bool* shooterRunning,
    bool* shooterJustStarted,
    bool* canPickupGoUp,
    float* shooterSpeed) {

    const bool stateChanged = newState != myState;

    if (stateChanged) {
      lastStateSwitch = currentTime;
    }
    myState = newState;
    switch (myState) {
    case PICKUP:
      *pickupTarget = MIN_PICKUP_POS;
      *rollerSpeed = 1;
      *canPickupGoUp = false;
      *shooterSpeed = 0;
      *shooterJustStarted = false;
      *shooterRunning = false;
      break;
    case PICKUP_STOWING:
      *pickupTarget = MAX_PICKUP_POS;
      *rollerSpeed = 0;
      *canPickupGoUp = true;
      *shooterSpeed = 0;
      *shooterJustStarted = false;
      *shooterRunning = false;
      break;
    case PREPSHOOT_HOLDON:
      *pickupTarget = UNJAM_PICKUP_POS;
      *shooterSpeed = SHOOTER_SPEED;
      *shooterRunning = true;
      *canPickupGoUp = false;
      *rollerSpeed = 1;
      break;
    case PREPSHOOT:
      if (stateChanged) {
        *shooterJustStarted = (*shooterJustStarted) || !(shooterWasRunning);
      }

      *pickupTarget = PREP_SHOOT_PICKUP_POS;
      *shooterSpeed = SHOOTER_SPEED;
      *shooterRunning = true;
      *canPickupGoUp = false;
      *rollerSpeed = 0;
      break;
    case PREPSHOOT_STOWING:
      *shooterSpeed = SHOOTER_SPEED;
      *shooterRunning = true;
      *rollerSpeed = 1;
      *canPickupGoUp = true;
      *pickupTarget = UNJAM_PICKUP_POS;
      break;
    case STOWED:
      *shooterRunning = false;
      *pickupTarget = MAX_PICKUP_POS;
      *canPickupGoUp = true;
      *rollerSpeed = 0;
      *shooterSpeed = 0;
      break;
    }
  }
  void Do(const bool pickupButton,
    const bool prepShootButton,
    const float pickupVoltage,
    const bool shooterWasRunning,
    const int currentTime,
    float* pickupTarget,
    float* rollerSpeed,
    bool* shooterRunning,
    bool* shooterJustStarted,
    bool* canPickupGoUp,
    float* shooterSpeed) {

    DoStateAction(myState, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
    switch (myState) {
    case PICKUP:
      if (!pickupButton) {
        DoStateAction(PICKUP_STOWING, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    case PICKUP_STOWING:
      // just lift the thing up to MAX_PICKUP_POS
      if (pickupButton) {
        DoStateAction(PICKUP, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      else if (prepShootButton) {
        if (pickupVoltage > PREP_SHOOT_PICKUP_POS) {
          // they want to go to prepshoot, and they're high enough that the logic will still work...
          DoStateAction(PREPSHOOT, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
        }
        else {
          // they're too low to switch to prepshoot.  We can write code to support this later,
          // but really, if they just keep holding the button, the arm will eventually lift high enough
        }
      }
      else if (pickupVoltage >= MAX_PICKUP_POS) {
        DoStateAction(STOWED, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    case PREPSHOOT:
      if (!prepShootButton) {
        // if they're not pressing the prepshoot button, then stow from prepshoot
        DoStateAction(PREPSHOOT_STOWING, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      if (pickupButton) {
        // should be safe to go from prepshoot to pickup
        DoStateAction(PICKUP, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    case PREPSHOOT_HOLDON:
      if (timeSinceLastState(currentTime) > 1000) {
        DoStateAction(STOWED, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      else if (pickupButton) {
        // should be safe to go from prepshoot to pickup
        DoStateAction(PICKUP, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      else if (prepShootButton) {
        // if they're not pressing the prepshoot button, then stow from prepshoot
        DoStateAction(PREPSHOOT, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    case PREPSHOOT_STOWING:
      // prepshoot stowing: this is to keep the roller and shooter spinning while
      // we bring the arm in, so that the ball will shoot
      if (pickupVoltage >= MAX_PICKUP_POS) {
        DoStateAction(PREPSHOOT_HOLDON, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      if (pickupButton) {
        // should be safe to go from prepshoot to pickup
        DoStateAction(PICKUP, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      if (prepShootButton) {
        // if they're not pressing the prepshoot button, then stow from prepshoot
        DoStateAction(PREPSHOOT, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    case STOWED:
      if (prepShootButton) {
        // if they're not pressing the prepshoot button, then stow from prepshoot
        DoStateAction(PREPSHOOT, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      if (pickupButton) {
        // should be safe to go from prepshoot to pickup
        DoStateAction(PICKUP, shooterWasRunning, currentTime, pickupTarget, rollerSpeed, shooterRunning, shooterJustStarted, canPickupGoUp, shooterSpeed);
      }
      break;
    default:
      // oh crap
      break;
    }

  }
  int timeSinceLastState(int currentTime) const {
    return currentTime - lastStateSwitch;
  }
public:
  PICKUP_STATE myState;
  int lastStateSwitch;
};


/**
 * This starter template is for building a robot program from the
 * SimpleRobot base class.  This template does nothing - it merely
 * provides method stubs that you can use to begin your implementation.
 */
class SimpleRobotDemo: public SampleRobot {
	RobotDrive myRobot; // robot drive system
	Joystick driveStick;
	Joystick opStick;

	Solenoid highShifter;
	Solenoid lowShifter;
	Solenoid liftArm;
	Solenoid lowerArm;

	Compressor compressor;

	Encoder rightencoder;
	Encoder leftencoder;

	Talon shooter;
	Talon turret;
	Talon pickup;
	Talon pickupRoller;
	Talon climber;

	AnalogType turretPot;
	AnalogType pickupPot;

	PickupStateMachine pickupStateMachine;
	//	DigitalInput rightencA;
	//	DigitalInput rightencB;
	//	DigitalInput leftencA;
	//	DigitalInput leftencB;

	Gyro* gyro;
	float targetVolts; //the target pot reading the turret want to achieve
#if PRACTICE_ROBOT
	Task networkTask;
#else
	std::thread networkThread;
#endif

	//interfaces
	//SolenoidInterface* shiftersolenoid;

	float p;
	float i;
	float d;
	VoltagePIDInput turretPIDInput;
	PIDController turretController;

	int shooterStartTime;

	SendableChooser autoChooser;
	SendableChooser posnChooser;
	SendableChooser defenseChooser;

	float initailAngle;

	// at the start of operatorControl/autonomous
	networkData newNetworkData;
	int imagesSinceHitVoltTarget;
	int onCameraTargetCount;

	// bunch of parameters for art/billy/taylor/benjamin/kevin's spring 2016 dead-reckoning tests
	float speedControlDistance;
	float speedControlP;
	float speedControlForwardValue;
	float speedControlReverseValue;
	float speedControlAccelRate;
	float speedControlSpeedTolerance;
public:
	SimpleRobotDemo() :
#if PRACTICE_ROBOT
			myRobot(DRIVE_LEFT_PWM, DRIVE_RIGHT_PWM),// initialize the RobotDrive to use motor controllers on ports 0 and 1
#else
			myRobot(DRIVE_RIGHT_PWM, DRIVE_LEFT_PWM),// initialize the RobotDrive to use motor controllers on ports 0 and 1
#endif
			driveStick(DRIVER_JOYSTICK),
			opStick(OP_JOYSTICK),
			highShifter(HIGH_SHIFT_SOLENOID),
			lowShifter(LOW_SHIFT_SOLENOID),
			liftArm(LIFT_SCALE_ARMS_SOLENOID),
			lowerArm(LOWER_SCALE_ARMS_SOLENOID),
//			rightencA(RIGHT_ENCODER_A_DIO),
//			rightencB(RIGHT_ENCODER_B_DIO),
//			leftencA(LEFT_ENCODER_A_DIO),
//			leftencB(LEFT_ENCODER_B_DIO),
#if PRACTICE_ROBOT
					compressor( PRESSURE_SWITCH_INPUT, COMPRESSOR_RELAY ),
#else
					compressor( COMPRESSOR),
#endif
			rightencoder(RIGHT_ENCODER_A_DIO, RIGHT_ENCODER_B_DIO),
			leftencoder(LEFT_ENCODER_A_DIO, LEFT_ENCODER_B_DIO),
			shooter(SHOOTER_PWM),
			turret(TURRET_PWM),
			pickup(PICKUP_PWM),
			pickupRoller(PICKUP_ROLLER_PWM),
			climber(CLIMBER_PWM),
			turretPot(TURRET_POT),
			pickupPot(PICKUP_POT),
			onCameraTargetCount(0),
			imagesSinceHitVoltTarget(0),
#if PRACTICE_ROBOT
					networkTask("NetworkTask", (FUNCPTR)&receiveTask, 150),
#else
					networkThread(receivePacket),
#endif
					p(2.35),
					i(0.089),
					d(3.075),
					turretPIDInput(turretPot),
					turretController(p, i, d, &turretPIDInput, &turret),
					shooterStartTime(0x7fffffff),
					initailAngle(0)
     {
#if PRACTICE_ROBOT
		g_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
		networkTask.Start((UINT32)this);
		gyro = new Gyro(GYRO_ANALOG_CHANNEL);
#else
		gyro = new AnalogGyro(GYRO_ANALOG_CHANNEL);
		leftencoder.SetReverseDirection(true);
		rightencoder.SetReverseDirection(true);
#endif
		myRobot.SetExpiration(0.1);

		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);

		targetVolts=2.5;

	}

	void RobotInit( void )
	{
		printf("RobotInit(): -- i'm running");
#ifdef PRACTICE_ROBOT
		leftencoder.Start();
		rightencoder.Start();
#else
		//robo rio doesnt need start calls for encoders
		CameraServer* instance = CameraServer::GetInstance();
		if(instance)
		{
			instance->SetSize(2);
			instance->SetQuality(80);
			instance->StartAutomaticCapture();
		}
#endif

		compressor.Start();

		speedControlDistance = SmartDashboard::GetNumber("speedControlDistance", 1.5);
		speedControlForwardValue = SmartDashboard::GetNumber("speedControlForwardValue", 0.3);
		speedControlReverseValue = SmartDashboard::GetNumber("speedControlReverseValue", 0.0);
		speedControlAccelRate = SmartDashboard::GetNumber("speedControlAccelRate", 1.0);
		speedControlP = SmartDashboard::GetNumber("speedControlP", 100);
		speedControlSpeedTolerance = SmartDashboard::GetNumber("speedControlSpeedTolerance", 0.05);
		SmartDashboard::PutNumber("speedControlDistance", speedControlDistance);
		SmartDashboard::PutNumber("speedControlForwardValue", speedControlForwardValue);
		SmartDashboard::PutNumber("speedControlReverseValue", speedControlReverseValue);
		SmartDashboard::PutNumber("speedControlAccelRate", speedControlAccelRate);
		SmartDashboard::PutNumber("speedControlP", speedControlP);
		SmartDashboard::PutNumber("speedControlSpeedTolerance", speedControlSpeedTolerance);

		autoChooser.AddDefault( "Do Nothing", (void *)"1");
		autoChooser.AddObject( "Touch Outer", (void *)"2");
		autoChooser.AddObject( "Cross Outer", (void *)"3");
		autoChooser.AddObject( "Score", (void *)"4");
		SmartDashboard::PutData( "Autonomous Mode", &autoChooser );

		posnChooser.AddDefault( "Pos 2", (void *)"2");
		posnChooser.AddObject( "Pos 3", (void *)"3");
		posnChooser.AddObject( "Pos 4", (void *)"4");
		posnChooser.AddObject( "Pos 5", (void *)"5");
		SmartDashboard::PutData( "Start Posn", &posnChooser );

		defenseChooser.AddDefault( "Moat", (void *)"1");
		defenseChooser.AddObject( "Rough Terrain", (void *)"2");
		defenseChooser.AddObject( "Rock Wall", (void *)"3");
		defenseChooser.AddObject( "Ramparts", (void *)"4");
		defenseChooser.AddObject( "Chaval de Frise", (void *)"5");
		SmartDashboard::PutData( "Defense", &defenseChooser );
		reportSmartDashboard();
		SmartDashboard::PutNumber("Camera X", 0 );
		SmartDashboard::PutNumber("Camera Y", 0 );


		LiveWindow *plw = LiveWindow::GetInstance();
		plw->AddActuator( "Turret", "Controller", &turret );
		plw->AddSensor( "Turret", "Pot", &turretPot );
		plw->AddActuator( "Turret", "PID", &turretController);
	}
	void DriveForward(float speed, float distance)
	{
		int cnt = 100;

		int32_t nTargetTicks = (int32_t)(distance * fEncoderTicksPerFoot);


		leftencoder.Reset();
		rightencoder.Reset();

		while(IsAutonomous() && IsEnabled() && rightencoder.GetRaw() >= nTargetTicks )
		{
			float currentAngle = (gyro->GetAngle() - initailAngle)*.25f;
			float fError = (rightencoder.GetRaw() - nTargetTicks);
			float fPwr = fError * (1.f/1000.f);

			if( currentAngle > 1)
			{
				currentAngle=1;
			}
			else if( currentAngle < -1)
			{
				currentAngle=-1;
			}

			if( fPwr > speed )
				fPwr = speed;
			myRobot.ArcadeDrive(-fPwr, -currentAngle);
			turret.Set( 0. );
			shooter.Set( 0. );
			pickup.Set( 0. );
			if( cnt++ >10 )
			{
				cnt = 0;
				printf( "E: %.1f P: %.3f, A:%.3f\n", fError, fPwr, currentAngle );
			}
			pickupRoller.Set( 0. );
			climber.Set( 0 );
			Wait( 0.005);
		}
		myRobot.ArcadeDrive( 0., 0. );
	}

	void TouchOuterworks( int nDefence )
	{
		printf( "Touch Outerworks\n");
		DriveForward( .75, 5. );
	}
	void CrossOuterworks(int nDefence)
	{
		printf("Cross Outerworks\n");

		switch(nDefence)
		{
		case 1:  // moat
			DriveForward(.80, 11.);
			break;
		case 2: //rough terrain
			DriveForward(1, 11.);
			break;
		case 3: //rock wall
			DriveForward(.95, 13.);
			break;
		case 4: //ramparts
			DriveForward(1, 11.);
			break;
		case 5: //caval de fris
			// DriveForward(.5, 11.);
			break;

		}

	}
	void Turn(float turnDegAmount)
	{
		int cnt = 100;
		float targetAngle=initailAngle+turnDegAmount;

		while(IsAutonomous() && IsEnabled())
		{
			float currentAngle = (gyro->GetAngle() - targetAngle);
			printf("current angle %.2f \n" ,currentAngle);
			if( fabs( currentAngle ) < 5 )
				break;

			currentAngle *= .25;
			if( currentAngle > 1)
			{
				currentAngle=1;
			}
			else if( currentAngle < -1)
			{
				currentAngle=-1;
			}
			myRobot.ArcadeDrive(0, -currentAngle);

			turret.Set( 0. );
			shooter.Set( 0. );
			pickup.Set( 0. );
			if( cnt++ >10 )
			{
				cnt = 0;
				printf( "A:%.3f\n", currentAngle );
			}
			pickupRoller.Set( 0. );
			climber.Set( 0 );
			Wait( 0.005);
		}
		myRobot.ArcadeDrive( 0., 0. );
		printf("Turn Completed \n");

	}
	void AutonomousScore()
	{
		bool shooterSpunUp = false;
		uint32_t uEndTime = GetFPGATime() + 1000000;
		shooterStartTime = GetFPGATime();

		while( IsAutonomous() && IsEnabled() && GetFPGATime() < uEndTime )
		{
			pickupDeploy(PREP_SHOOT_PICKUP_POS, 0, false);
			shooter.Set(SHOOTER_SPEED);
			Wait(.0025);
		}
		printf( "After .5 Wait\n");

		updateNetworkData();

		const float offsetPx = newNetworkData.x - CORRECT_PIX_POS;
		const float offsetDeg = (offsetPx / 160.0f) * CAMERA_FOV + CAMERA_FOV_OFFSET;
		const float offsetVolts = offsetDeg * TURRET_V_PER_DEGREES;
		targetVolts = turretPot.GetVoltage() - offsetVolts;
		printf("%f.2  %f.2  %f.2  %f.2 \n", offsetPx, offsetDeg, offsetVolts, targetVolts);
		if (targetVolts < MIN_TURRET_POS || targetVolts > MAX_TURRET_POS)
		{
			printf("turret out of range\n");
			return;
		}
		turretController.Reset();
		turretController.SetSetpoint(targetVolts);
		turretController.Enable();
		while( IsAutonomous() && IsEnabled() && fabs( turretPot.GetVoltage() - targetVolts) > TURRET_TOLERANCE
				&& !shooterSpunUp )
		{
			pickupDeploy(PREP_SHOOT_PICKUP_POS, 0, false);
			shooter.Set(SHOOTER_SPEED);
			Wait(.0025);
			shooterSpunUp = (GetFPGATime() - shooterStartTime) > 2000000;
		}
		printf( "A:%d %.2f\n", IsAutonomous(), turretPot.GetVoltage() - targetVolts );
		while( IsAutonomous() && IsEnabled() && !shooterSpunUp )
		{
			pickupDeploy(PREP_SHOOT_PICKUP_POS, 0, false);
			shooter.Set(SHOOTER_SPEED);
			Wait(.0025);
			shooterSpunUp = (GetFPGATime() - shooterStartTime) > 2000000;
		}
		pickupDeploy(PREP_SHOOT_PICKUP_POS, 1, false);
		turretController.Disable();
		uEndTime = GetFPGATime() + 1500000;
		//kicks ball in
		while( IsAutonomous() && IsEnabled() && GetFPGATime() < uEndTime )
		{
			pickupDeploy(PREP_SHOOT_PICKUP_POS, 1, false );
			shooter.Set(SHOOTER_SPEED);
			Wait(.0025);
		}
		//unjams ball
		uEndTime = GetFPGATime() + 2000000;
		while( IsAutonomous() && IsEnabled() && GetFPGATime() < uEndTime ){
			pickupDeploy(UNJAM_PICKUP_POS, 1, true);
			shooter.Set(SHOOTER_SPEED);
			Wait(0.0025);
		}
		shooter.Set( 0 );
		pickupRoller.Set(0);
		pickup.Set(0);
	}

	/**
	 * Your code for autonomous goes here.  You must be certain to exit the function when
	 * the Autonomous period is over!  Otherwise your program will never transition to
	 * OperatorControl.  You can use a loop similar to below, or otherwise frequently check
	 * IsAutonomous() and return when it returns false.
	 */

	int encCalls;
	int driveCalls;
	class EncoderLink
	{
	public:
		EncoderLink(SimpleRobotDemo* me) : me(me) {};
		double PIDGet() const{
			me->encCalls++;
			int ret= (me->leftencoder.GetRaw()+me->rightencoder.GetRaw())/2;

			const float tickspermeter = 6351;
			const float retInMeters = (float)ret / tickspermeter;
			return retInMeters;
		}

	private:
		SimpleRobotDemo* me;

	};

	class DriveLink: public PIDOutput
	{
	public:
		DriveLink(SimpleRobotDemo* me) : me(me) {};
		void PIDWrite(float output){
			me->driveCalls++;
			me->myRobot.ArcadeDrive(output,0,false);
		}

	private:
		SimpleRobotDemo* me;
	};

	void Autonomous() {
		leftencoder.Reset();
		rightencoder.Reset();

		int c=0;
		encCalls = 0;
		driveCalls = 0;

		myRobot.SetSafetyEnabled(false);

		EncoderLink enclnk(this);


		DriveLink drvlnk(this);

		speedControlDistance = SmartDashboard::GetNumber("speedControlDistance", 1.5);
		speedControlForwardValue = SmartDashboard::GetNumber("speedControlForwardValue", 0.3);
		speedControlReverseValue = SmartDashboard::GetNumber("speedControlReverseValue", 0.0);
		speedControlAccelRate = SmartDashboard::GetNumber("speedControlAccelRate", 1.0);
		speedControlP = SmartDashboard::GetNumber("speedControlP", 100);
		speedControlSpeedTolerance = SmartDashboard::GetNumber("speedControlSpeedTolerance", 0.05);

		const float targetMeters = enclnk.PIDGet() + speedControlDistance;
		//pid.Enable();

		SmoothMotionManager robotDistance(enclnk.PIDGet(),(float)GetFPGATime()/1000000.0f);
		while (IsAutonomous()&& IsEnabled())
		{
			Wait(.005);
			const float currentTime =(float) GetFPGATime()/1000000.0f;
			float motorPower = 0;
			bool finished = robotDistance.tick(enclnk.PIDGet(),
					targetMeters,
					speedControlForwardValue,
					speedControlReverseValue,
					speedControlAccelRate,
					currentTime,
					speedControlSpeedTolerance,
					0.015,
					&motorPower);

			if(finished) {
				break;
			}
			myRobot.ArcadeDrive(motorPower, 0, false);
			c++;
			if(c % 20 == 0)
			{
				printf("targeted motor power was %.2f \n", motorPower);
			}
		}
		myRobot.ArcadeDrive(0.0,0.0);


		const float targetAngle = this->gyro->GetAngle() + 180;
		SmoothMotionManager robotTurn(this->gyro->GetAngle(),(float)GetFPGATime()/1000000.0f);
		while (IsAutonomous()&& IsEnabled())
		{
			Wait(.005);
			const float currentTime =(float) GetFPGATime()/1000000.0f;
			float motorPower = 0;
			bool finished = robotTurn.tick(this->gyro->GetAngle(),
					targetAngle,
					1,
					-1,
					1000,
					currentTime,
					0.1,
					0.5,
					&motorPower);

			if(finished) {
				break;
			}
			myRobot.ArcadeDrive(0, motorPower, false);
			c++;
			if(c % 20 == 0)
			{
				printf("turning: targeted motor power was %.2f \n", motorPower);
			}
		}
		myRobot.ArcadeDrive(0.0,0.0);
		Wait(0.25);
	}

	void Test()
	{
		int count = 0;
		myRobot.SetSafetyEnabled( false );
		//LiveWindow *plw = LiveWindow::GetInstance();
		while(this->IsTest() && IsEnabled())
		{
			//plw->Run();
//			reportSmartDashboard();
			//printf("arm pot: %f.2 \n" ,pickupPot.GetVoltage());
			this->pickup.Set(this->opStick.GetX());
//			this->turret.Set(this->opStick.GetY());
//			this->pickupRoller.Set(this->driveStick.GetX());
			if(count++ >=50)
			{
				printf("pickup pot:  %.2f turret pot: %.2f \n", pickupPot.GetVoltage(), turretPot.GetVoltage());
				printf("left encoder %d, right encoder %d \n", leftencoder.GetRaw(), rightencoder.GetRaw());
				count = 0;
			}
			Wait(0.005);
		}
		myRobot.SetSafetyEnabled( true );
	}

	void reportSmartDashboard()
	{

		SmartDashboard::PutBoolean("shooter is running",shooter.Get() > 0.5);
		SmartDashboard::PutNumber("turretPot",turretPot.GetVoltage());
		SmartDashboard::PutNumber("armPot",pickupPot.GetVoltage());
		SmartDashboard::PutNumber("rightEncoder",rightencoder.GetRaw());
		SmartDashboard::PutNumber("leftEncoder",leftencoder.GetRaw());
		SmartDashboard::PutNumber("Gyro",gyro->GetAngle());
		SmartDashboard::PutBoolean("turretCentered",centeredShooter());
		SmartDashboard::PutNumber("Camera X", newNetworkData.x );
		SmartDashboard::PutNumber("networkCount", networkCount);
		SmartDashboard::PutNumber("On Voltage Target Count", imagesSinceHitVoltTarget);
		SmartDashboard::PutNumber("On Camera Target Count", onCameraTargetCount);
	}
	void updateNetworkData()
	{
		static int lastFrameID = -1;
		ACQUIRE_MUTEX();
		newNetworkData = ::lastNetworkData;
		float distanceToTarget = fabs(targetVolts - turretPot.GetVoltage());
		if (lastFrameID != networkCount)
		{
			if (distanceToTarget < TURRET_TOLERANCE)
			{
				imagesSinceHitVoltTarget++;
			}
			else
			{
				imagesSinceHitVoltTarget = 0;
			}
			if (newNetworkData.x >= MIN_PIX_POS && newNetworkData.x <= MAX_PIX_POS)
			{
				onCameraTargetCount++;
			}
			else
			{
				onCameraTargetCount = 0;
			}
		}
		lastFrameID = networkCount;
	}
	bool centeredShooter()
	{

		if(newNetworkData.x >= MIN_PIX_POS && newNetworkData.x <= MAX_PIX_POS)
		{
			return true;
		} else {
			return false;
		}

	}
	/**
	 * \Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		int printcnt = 0;
		bool lastOpButtons[12] = {0};
		bool lastDriveButtons[12] = {0};

		float fP = Preferences::GetInstance()->GetFloat( "PID.P", p );
		float fI = Preferences::GetInstance()->GetFloat( "PID.I", i );
		float fD = Preferences::GetInstance()->GetFloat( "PID.D", d );
		float fPIDMax = Preferences::GetInstance()->GetFloat( "PID.Max", 1 );
		printf( "OperatorControl:: P:%.3f I:%.3f D:%.3f Max:%.3f\n", fP, fI, fD, fPIDMax );

		turretController.SetPID(fP,fI,fD);
		turretController.SetOutputRange( -fPIDMax, fPIDMax );

		while (IsOperatorControl() && IsEnabled())
		{
			updateNetworkData();

			bool opButtonClicked[12] = {0};
			bool driveButtonClicked[12] = {0};
			for (int x = 1;x < 12; x++)
			{
				const bool thisOpButton = opStick.GetRawButton(x);
				opButtonClicked[x] = (thisOpButton && !lastOpButtons[x]);
				lastOpButtons[x] = thisOpButton;

				const bool thisDriveButton = driveStick.GetRawButton(x);
				driveButtonClicked[x] = (thisDriveButton && !lastDriveButtons[x]);
				lastDriveButtons[x] = thisDriveButton;
			}
#if 0
			if(opButtonClicked[6]) {
				p*=1.05;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}
			if(opButtonClicked[7]) {
				p/=1.04;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}
			if(opButtonClicked[11]) {
				i*=1.05;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}
			if(opButtonClicked[10]) {
				i/=1.04;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}
			if(opButtonClicked[8]) {
				d*=1.05;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}
			if(opButtonClicked[9]) {
				d/=1.04;
				printf("p = %f, i = %f, d = %f\n", p, i, d);
			}


			turretController.SetPID(p,i,d);

#endif
			reportSmartDashboard();

			bool shooterJustStarted = false;
			bool shooterRunning = false;

			float turretSpeed = 0;
			float rollerSpeed = 0;
			float pickupTarget = MAX_PICKUP_POS;
			float shooterSpeed = 0;
			bool canPickupGoUp = true;
			static bool lastclicked = false;
			//printf("turretpot %f \n", this->turretPot.GetVoltage());


			if (opStick.GetRawButton(CAMERA_BUTTON))
			{
				//camera mode
				const bool stalledOut = imagesSinceHitVoltTarget >= 10 && onCameraTargetCount == 0;
				if (!lastclicked || stalledOut)
				{
					imagesSinceHitVoltTarget = 0; // retargeting;
					if (stalledOut)
					{
						printf("stalled out, retargeting \n");
					}
					const float offsetPx = newNetworkData.x - CORRECT_PIX_POS;
					const float offsetDeg = (offsetPx / 160.0f) * CAMERA_FOV + CAMERA_FOV_OFFSET;
					const float offsetVolts = offsetDeg * TURRET_V_PER_DEGREES;
					targetVolts = turretPot.GetVoltage() - offsetVolts;
					printf("%f.2  %f.2  %f.2  %f.2 \n", offsetPx, offsetDeg, offsetVolts, targetVolts);
					if (targetVolts < MIN_TURRET_POS || targetVolts > MAX_TURRET_POS)
					{
						printf("turret out of range\n");
					}
					else
					{
						turretController.Reset();
						turretController.SetSetpoint(targetVolts);
						turretController.Enable();
					}
				}

				lastclicked = true;


				//turretSpeed = turretPIDResult.lastResult;
			}
			else
			{

				// camera button not clicked, go back to manual-ish turret control
				static bool wasRecentering = false;
				const int timeSinceLastState = pickupStateMachine.timeSinceLastState(GetFPGATime()/1000);
				if(opStick.GetRawButton(RECENTER_BUTTON) || (pickupStateMachine.myState == STOWED && timeSinceLastState >= 500 && timeSinceLastState < 1500)) {
					// its been between 500 ms to 1500 ms the turret should center
					if(!wasRecentering) {
						turretController.Enable();
					}
					wasRecentering = true;
					turretController.SetSetpoint((MAX_TURRET_POS + MIN_TURRET_POS)/2);
				} else {
					wasRecentering = false;
					turretController.Disable();
					lastclicked = false;
					turretSpeed = opStick.GetX() * -0.25f;
					turretControl(turretSpeed);
					//not camera mode
				}

			}


			if(driveStick.GetRawButton(LOW_GOAL))
			{
				pickupDeploy(PREP_SHOOT_PICKUP_POS, 0, false);
				shooterSpeed = 0.55;
			}

			float kidMode = (driveStick.GetZ()+1)/2;
			printf("kidMode = %f\n", kidMode);
			myRobot.ArcadeDrive(driveStick.GetY() * kidMode, driveStick.GetX() * kidMode); // drive with arcade style (use right stick)
			static int highStartedAt = GetFPGATime();
			static int lowStartedAt = GetFPGATime();
			static bool wasPressedLast = false;
			if(driveStick.GetRawButton(HI_SHIFT_BUTTON)){
				wasPressedLast = true;
				if(driveButtonClicked[HI_SHIFT_BUTTON]) {
					highStartedAt = GetFPGATime();
				}
				const int timeSinceStarted = GetFPGATime() - highStartedAt;
				if(timeSinceStarted < 250000) {
					highShifter.Set(true);
					lowShifter.Set(false);
				} else {
					// we've been going long enough
					highShifter.Set(false);
					lowShifter.Set(false);
				}
			} else {
				if(wasPressedLast) {
					lowStartedAt = GetFPGATime();
				}
				wasPressedLast = false;

				const int timeSinceStarted = GetFPGATime() - lowStartedAt;
				if(timeSinceStarted < 250000) {
					highShifter.Set(false);
					lowShifter.Set(true);
				} else {
					// we've been going long enough
					highShifter.Set(false);
					lowShifter.Set(false);
				}
			}

			pickupStateMachine.Do(	driveStick.GetRawButton(PICKUP_BALL),
									driveStick.GetRawButton(PREP_SHOOT_BUTTON),
									pickupPot.GetVoltage(),
									shooterRunning,
									GetFPGATime() / 1000,
									&pickupTarget,
									&rollerSpeed,
									&shooterRunning,
									&shooterJustStarted,
									&canPickupGoUp,
									&shooterSpeed);

			if(opButtonClicked[REV_UP_SHOOTER_BUTTON]){
				static bool shooterToggledOn = false;
				shooterToggledOn = !shooterToggledOn;
				shooterJustStarted = shooterJustStarted || (!shooterRunning && shooterToggledOn);
				shooterRunning=shooterRunning || shooterToggledOn;
			}
			const bool shooterSpunUp = GetFPGATime() - shooterStartTime > 2000000;
			const bool cameraOnTarget = newNetworkData.x >= CORRECT_PIX_POS - 3 && newNetworkData.x <= CORRECT_PIX_POS + 3;
			if(driveStick.GetRawButton(SHOOTER_BUTTON)&&
					(!opStick.GetRawButton(CAMERA_BUTTON)||cameraOnTarget) &&
						shooterSpunUp)
			{
				// feed the ball into the shooter
				pickupRoller.Set(1.0);
			}
			else
			{
				pickupRoller.Set(0.0);
			}

# if 1
			if( ++printcnt >100 )
			{
				printcnt = 0;
			}
#endif
			//Pickup Solenoids
			liftArm.Set(driveStick.GetRawButton(LIFT_SCALE_ARM));
			lowerArm.Set(driveStick.GetRawButton(LOWER_SCALE_ARM));
			//Enable Shooter: Drive Trigger

			//Climber Motor: OP 8
			climber.Set(opStick.GetY());
# if 0
			if (++printcnt > 100) {
				printf("Pickup Speed: %.2f\n", pickupSpeed );
				printcnt = 0;
			}
#endif

			//opstick overides
			if(opStick.GetRawButton(ROLLER_IN_BUTTON))
			{
				rollerSpeed = 1;
			}
			else if(opStick.GetRawButton(ROLLER_OUT_BUTTON))
			{
				rollerSpeed = -1;
			}
			if(opStick.GetRawButton(SHOOTER_FWD_BUTTON))
			{
				shooterSpeed = 1;
			}
			else if(opStick.GetRawButton(SHOOTER_REV_BUTTON))
			{
				shooterSpeed = -1;
			}

			pickupDeploy(pickupTarget, rollerSpeed, canPickupGoUp);
//			turretControl(turretSpeed);
			shooterControl(shooterSpeed, shooterRunning, shooterJustStarted);
			pickupRoller.Set(rollerSpeed);

			Wait(0.0025);
		}
	}
	void shooterControl(float shooterSpeed, bool shooterRunning, bool shooterStarted)
	{
		if(shooterStarted || !shooterRunning)
		{
			shooterStartTime = GetFPGATime();
		}
		shooter.Set(shooterSpeed);
	}
	void turretControl(float rateOfTurn){
		float turretPos = turretPot.GetVoltage();

		if (rateOfTurn > 0 && turretPos > MAX_TURRET_POS) {
			rateOfTurn = 0;
		}
		else if (rateOfTurn < 0 && turretPos < MIN_TURRET_POS) {
			rateOfTurn = 0;
		}

		// if the PID is running, DON'T talk directly to the turret.
		// the PID talks directly to the turret on its own thread.
		if(!turretController.IsEnabled()) {
			turret.Set(rateOfTurn);
		}
	}

	void pickupDeploy(float targetVoltage, float rollerSpeed, bool isUp){

		float pickupPos = pickupPot.GetVoltage();

		float tempPickupSpeed = 0;
		float pickupSpeedUp = 0;
		float pickupSpeedDown = 0;
		//				pickup.Set(((opStick.GetZ()+1)/2)*0.1);
		//Pickup Enable: Drive 6,7


		//Pickup Speed: Op Throttle
		tempPickupSpeed = (pickupPos
				- MIN_PICKUP_POS) / (MAX_PICKUP_POS - MIN_PICKUP_POS);

		pickupSpeedUp = ((MIN_PICKUP_POWER - 1.0f) * tempPickupSpeed) + 1.0f;
		pickupSpeedDown = ((-1.0f + MIN_PICKUP_POWER) * tempPickupSpeed) - MIN_PICKUP_POWER;

		const bool up = pickupPos <= targetVoltage && isUp;
		const bool down = pickupPos >= targetVoltage && !isUp;
		float pickupOutputPower = 0;
		if(up && pickupPos < MAX_PICKUP_POS)
		{
			pickupOutputPower = pickupSpeedUp;
		}
		else if(down && pickupPos > MIN_PICKUP_POS)
		{
			pickupOutputPower = pickupSpeedDown;
		}
		pickup.Set(pickupOutputPower);
		pickupRoller.Set(rollerSpeed);
	}



};

void receiveTask(SimpleRobotDemo *myRobot) {
	receivePacket();
}

int receivePacket() {

	int iResult = 0;
	int RecvSocket = 0;
	sockaddr_in RecvAddr;

	unsigned short Port = 2702;

//	char RecvBuf[1024];
//	int BufLen = 1024;

	sockaddr_in SenderAddr;
#ifdef PRACTICE_ROBOT
	int SenderAddrSize = sizeof(SenderAddr);
#else
	socklen_t SenderAddrSize = sizeof(SenderAddr);
#endif

	//-----------------------------------------------
	// Create a receiver socket to receive datagrams
	RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (RecvSocket < 0) {
		return 1;
	}
	//-----------------------------------------------
	// Bind the socket to any address and the specified port.
	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_port = htons(Port);
	RecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	iResult = bind(RecvSocket, (sockaddr*) &RecvAddr, (int)sizeof(RecvAddr));
	if (iResult != 0) {
		return 1;
	}
	//-----------------------------------------------
	// Call the recvfrom function to receive datagrams
	// on the bound socket.
	printf("about to try to recv from pi\n");
	while (true) {
		networkData dataRecv;

		iResult = recvfrom(RecvSocket, (char*)&dataRecv, (int)sizeof(dataRecv), 0,
				(sockaddr*) &SenderAddr, &SenderAddrSize);
		networkCount++;
		dataRecv.x = (dataRecv.x);
		dataRecv.y = (dataRecv.y);
		//printf("net: %d %d \n",dataRecv.x ,dataRecv.y);

		{
			ACQUIRE_MUTEX();
			lastNetworkData = dataRecv;
		}
	}
	//-----------------------------------------------
	// Close the socket when finished receiving datagrams
	iResult = close(RecvSocket);
	if (iResult < 0) {

		return 1;
	}

	//-----------------------------------------------
	// Clean up and exit.
	return 0;
}

/*
 * This macro invocation tells WPILib that the named class is your "main" robot class,
 * providing an entry point to your robot code.
 */
START_ROBOT_CLASS(SimpleRobotDemo);

