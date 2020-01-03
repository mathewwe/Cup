package project;

import jssc.SerialPort;
import jssc.SerialPortException;

/**
 * Test Arduino communication.
 */
public class SerialFast
{
	
	SerialPort port;
	double ticksPerRevolution = 1024;
	double pulleyDiameter = 2.44;
	
	public void sendToGantryCoordinate(double x, double y) {
		double inchesPerTick = Math.PI*pulleyDiameter/ticksPerRevolution;
		double ticksPerInch = 1/inchesPerTick;
		double distanceFromOrigin = Math.pow(Math.pow(x, 2)+Math.pow(y, 2), 0.5);
		double angle = Math.atan2(y,x);
		double newAngle = angle + Math.PI/4;
		int leftMotorTicks = (int)(Math.sin(newAngle)*distanceFromOrigin*ticksPerInch*Math.sqrt(2));
		int rightMotorTicks = (int)(Math.cos(newAngle)*distanceFromOrigin*ticksPerInch*Math.sqrt(2));
		sendToTickCoordinate(leftMotorTicks, rightMotorTicks);
	}
	
	 public void sendToTickCoordinate(int leftMotor, int rightMotor) {
		    int diamondSum = Math.abs(leftMotor - 7580) + Math.abs(rightMotor - 0);
		    String data;
		    if(diamondSum <= 7580) {
		    	data = Integer.toString(leftMotor) + ';' + Integer.toString(rightMotor) + '&';
		    }else {
		    	double lMotor = (7580/(double)diamondSum)*(leftMotor-7580) + 7580;
		    	double rMotor = (7580/(double)diamondSum)*(rightMotor);
		    	data = Integer.toString((int)lMotor) + ';' + Integer.toString((int)rMotor) + '&';
		    	//data = Integer.toString(7580) + ';' + Integer.toString(0) + '&';
		    }
		    char[] dataArray = data.toCharArray();
		    for(char c:dataArray) {
		    	try {
		    	port.writeByte((byte)c);
		    	System.out.println(c);
		    	Thread.sleep(1);
		    	}catch(Exception e) {
		    		e.printStackTrace();
		    	}
		    }
		    }
	 
  private static void testComm(SerialPort port) throws SerialPortException {
    long runningSeconds = 0;
    long time = System.currentTimeMillis();
    long numberOfPackets = 0;
    boolean packetSent = false;
    while (runningSeconds < 10) {
      long currentTime = System.currentTimeMillis();
      if (currentTime - time > 1000) {
        runningSeconds++;
        time = currentTime;
        System.out.println(numberOfPackets + " packets/s");
        numberOfPackets = 0;
      }

      if (!packetSent) {
        packetSent = true;
        port.writeByte((byte) 'd');
        for (int i = 0; i < 48; i++) {
          port.writeByte((byte) i);
        }
      } else {
        byte[] received = port.readBytes();
        if (received != null) {
          if (received.length > 1) {
            throw new IllegalStateException("One byte expected, instead got: " + received.length);
          }

          char cmd = (char) received[0];
          if ('k' != cmd) {
            throw new IllegalStateException("Expected response 'k', instead got: " + cmd);
          }
          packetSent = false;
          numberOfPackets++;
        }
      }

    }
  }

  public void startPort(String portValue)
  {
	  port = new SerialPort(portValue);

    try {
      if (!port.openPort()) {
        throw new IllegalStateException("Failed to open port.");
      }
      port.setParams(115200, 8, 1, 0);
    } catch (SerialPortException e) {
      throw new IllegalStateException("Exception while setting up port.", e);
    }

    try {
      // Wait 1.5sec for Arduino to boot successfully.
      Thread.sleep(1500);
    } catch (InterruptedException e) {
      throw new IllegalStateException("Interrupt while waiting?", e);
    }
  }
  public void endPort() {
	  try {
	        if (!port.closePort()) {
	          throw new IllegalStateException("Failed to close port.");
	        }
	      } catch (SerialPortException e) {
	        throw new IllegalStateException("Exception while closing port.", e);
	      }
  }
}



