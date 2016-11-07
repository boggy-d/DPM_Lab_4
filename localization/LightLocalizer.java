package localization;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private Navigation nav;
	
	private double[] lineAngles, linePos; 
	private int lineCount;
	private double thetaY, thetaX, posY, posX, deltaTheta;
	private double previousLightValue, currentLightValue;
	private boolean isNavigating;
	
	//Constants
	private final double LIGHT_THRESHOLD = 5;
	private final int FORWARD_SPEED = 150;
	private final int ROTATE_SPEED = 125;
	private final int LINE_OFFSET = 10;
	private static final int PERIOD = 400;
	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.nav = new Navigation(odo);
	}
	
	public void doLocalization() {
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		
		//Rotate to 45 deg i.e. the diagonal
		nav.turnTo(45, true);
		
		
		
		isNavigating = true;
		previousLightValue = getFilteredData(); //Get the default board color value
		
		//Run the motors forward
		odo.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		
		//Runs until a line is found
		while(isNavigating)
		{
			
			currentLightValue = getFilteredData();
			
			//If the color contrast is bigger than our threshold, line was detected, stop
			if(Math.abs(currentLightValue-previousLightValue) >= LIGHT_THRESHOLD)
			{
				odo.setSpeeds(0, 0);
				isNavigating = false;
			}
		}
		
		//Theoretically, we should arrive at the grid lines' intersection, so advance
		//a bit for the line sencor to be able to detect lines while turning
		nav.goForward(LINE_OFFSET);
		
		//Set up variables for localization
		lineCount = 0;
		lineAngles = new double[4];
		linePos = new double[3];
		previousLightValue = getFilteredData(); //Get the default board color value
		
		
		
		//Runs until it has detected 4 lines i.e. a full circle
		while(lineCount < 4)
		{
			currentLightValue = getFilteredData();
			
			//If the color contrast is bigger than our threshold, line was detected, stop
			if(Math.abs(currentLightValue-previousLightValue) >= LIGHT_THRESHOLD)
			{
				//Gets odometer readings for future use
				this.odo.getPosition(linePos);
				
				//Store angles in variable for future calculations
				lineAngles[lineCount] = linePos[2];
				
				//Debug statement
				Sound.beep();
				
				lineCount++;
				
				//Makes the thread sleep as to not detect the same line twice
				sleepThread();
			}
			
			
			//Run the motors
			odo.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
		}
		
		odo.setSpeeds(0, 0);
		
		//Trigonometry calculations from tutorial
		thetaY = lineAngles[3] - lineAngles[1];
		thetaX = lineAngles[2] - lineAngles[0];
		
		posY = LINE_OFFSET *Math.cos(Math.toRadians(thetaY/2));
		posX = LINE_OFFSET *Math.cos(Math.toRadians(thetaX/2));
		
		deltaTheta = 270 + (thetaY/2) - lineAngles[3];
		
		//Wraps the angle for positive Y axis
		if (deltaTheta > 180) {
			deltaTheta += 180;
		}
		
		//Updates odometer to actual values
		this.odo.setPosition(new double[] {posX, posY, this.odo.getAng() + deltaTheta}, new boolean[] {true, true, true});
		
		//Travel to origin
		nav.travelTo(0, 0);
		sleepThread();
		
		//Turn to 0deg
		nav.turnTo(0, true);
	}
	
	//Polls the color sensor
	private double getFilteredData() {
		colorSensor.fetchSample(colorData, 0);
		double color = (colorData[0] * 100.0);
				
		return color;
	}
	
	//Sleeps the thread for specified time
	public static void sleepThread() {
		try {
			Thread.sleep(PERIOD);
		} catch (InterruptedException e) {
		}
	}
}
