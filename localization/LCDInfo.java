package localization;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	
	// arrays for displaying data
	private double [] pos;
	private double dist;
	private float[] usData;
	private SampleProvider usSensor;
	
	public LCDInfo(Odometer odo, SampleProvider usSensor, float[] usData) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.usSensor = usSensor;
		this.usData = usData;
		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("D: ", 0, 3);
		LCD.drawInt((int)(pos[0] * 10), 3, 0);
		LCD.drawInt((int)(pos[1] * 10), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawInt((int)getFilteredData(), 3, 3);
	}
	
	private double getFilteredData() {
		usSensor.fetchSample(usData, 0);
		dist = (usData[0]*100.0);
				
		return dist;
	}
}
