package localization;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 30;

	private Odometer odo;
	private Navigation nav;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	
	//Constants
	private final int ROTATE_SPEED = 100;
	private final int NO_WALL_THRESHOLD = 45;
	private final int NOISE_MARGIN = 1;
	
	private boolean isWall = false; //True if there is a wall, false if there is no wall
	private boolean isDone = true; //True if the falling/rising edge is detected, false if not
	private double deltaTheta; //the angle to be added to the odometer angle to correct it
	private double thresholdUpperHeading, thresholdLowerHeading; //Threshold values for calculating angle
	
	
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.nav = new Navigation(odo);
	}
	
	public void doLocalization() {
		double [] pos = new double [3];
		double angleA, angleB;
		
		if (locType == LocalizationType.FALLING_EDGE) {
						
			//Rotate the robot cw until it sees a wall
			turnToWall(ROTATE_SPEED);
			
			//Stop motors
			odo.setSpeeds(0,0);
			
			
			//Record first angle
			angleA = Odometer.fixDegAngle((thresholdUpperHeading + thresholdLowerHeading)/2);

			//Rotate robot ccw until it sees a wall
			turnToWall(-ROTATE_SPEED);

			odo.setSpeeds(0,0);
			
			
			//Record second angle
			angleB = Odometer.fixDegAngle((thresholdUpperHeading + thresholdLowerHeading)/2);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			
			
			deltaTheta = calculateHeading(angleA, angleB);
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, Odometer.fixDegAngle(this.odo.getAng()) + deltaTheta}, new boolean [] {false, false, true});
			
			
		} else  if(locType == LocalizationType.RISING_EDGE){
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			///Rotate robot cw until it sees no wall
			turnFromWall(ROTATE_SPEED);

			//Stop motors
			odo.setSpeeds(0,0);
			
			//Record first angle
			angleA = Odometer.fixDegAngle((thresholdUpperHeading + thresholdLowerHeading)/2);
			
			//Rotate robot ccw until it sees no wall
			turnFromWall(-ROTATE_SPEED);

			odo.setSpeeds(0,0);
			
			//Record second angle
			angleB = Odometer.fixDegAngle((thresholdUpperHeading + thresholdLowerHeading)/2);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			deltaTheta = Odometer.fixDegAngle(calculateHeading(angleA, angleB));
			
			//Odometer.minimumAngleFromTo(Odometer.fixDegAngle(Math.toDegrees(this.odo.getAng())), Math.toDegrees(this.odo.getAng())+heading);
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, Odometer.fixDegAngle(this.odo.getAng()) + deltaTheta}, new boolean [] {false, false, true});
	
		}
		
		
		
		nav.turnTo(0, true);
	}
	
	//Polls the USS
	protected double getFilteredData() {
		usSensor.fetchSample(usData, 0);
		double distance = (usData[0]*100.0);
				
		return distance;
	}
	
	//Localization for rising edge
	private void turnFromWall(int speed)
	{
		this.isWall = true;
		this.isDone = false;
		
		
		while(!isDone)
		{
			odo.setSpeeds(speed, -speed); //Rotates the robot, default is clockwise
			
			//If the USS distance is smaller than threshold's lower bound,
			//there is a wall, commence rising edge localization
			if(getFilteredData() < (NO_WALL_THRESHOLD - NOISE_MARGIN))
			{
				isWall = false;
			}
			
			//If the USS distance is bigger than threshold's lower bound
			//and the localization has started, there is no wall
			else if(getFilteredData() > (NO_WALL_THRESHOLD - NOISE_MARGIN) && !isWall)
			{
				thresholdLowerHeading = Odometer.fixDegAngle(this.odo.getAng());
				//If the USS distance is bigger than threshold's upper bound,
				//rising edge has been detected, stop the loop
				if(getFilteredData() > (NO_WALL_THRESHOLD + NOISE_MARGIN))
				{
					thresholdUpperHeading = Odometer.fixDegAngle(this.odo.getAng());
					isDone = true;
				}	
			}
		}
	}
	
	//Localization for rising edge
	private void turnToWall(int speed)
	{
		this.isWall = false;
		this.isDone = false;
		
		while(!isDone)
		{
			odo.setSpeeds(speed, -speed); //Rotates robot cw
			
			//If the USS distance is bigger than threshold's upper bound,
			//there is no wall, commence falling edge localization
			if(getFilteredData() > (NO_WALL_THRESHOLD + NOISE_MARGIN))
			{
					isWall = true;
					continue;
			}
			
			//If the USS distance is smaller than threshold's upper bound
			//and the localization has started, there is a wall
			else if(getFilteredData() < (NO_WALL_THRESHOLD + NOISE_MARGIN) && isWall)
			{
				thresholdUpperHeading = Odometer.fixDegAngle(this.odo.getAng());
				//If the USS distance is smaller that the threshold's lower bound,
				//falling edge has been detected, stop the loop
				if(getFilteredData() < (NO_WALL_THRESHOLD - NOISE_MARGIN))
					{
						thresholdLowerHeading = Odometer.fixDegAngle(this.odo.getAng());
						isDone = true;
						Sound.beep();
						continue;
					}
			}
		}
	}
	
	//Calculates the heading to be added to the odometer angle based on the tutorial math
	private double calculateHeading(double thetaA, double thetaB)
	{
		if(locType == LocalizationType.FALLING_EDGE)
		{
			if(thetaA >= thetaB)
			{
				return (225-((thetaA+thetaB)/2));
			}
			else
			{
				return (45-((thetaA+thetaB)/2));
			}
		}
		else
		{
			if(thetaA <= thetaB)
			{
				return (225-((thetaA+thetaB)/2));
			}
			else
			{
				return (45-((thetaA+thetaB)/2));
			}
		}
		
		
			
	}

}
