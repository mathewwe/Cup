package project;

import javax.sql.rowset.serial.SerialException;

public class JSSCTest {
	SerialFast sf;
	
	
	public JSSCTest() throws SerialException {
		//sf = new SerialFast("COM3",9600,'N',8,(float)1,false,false);
		
	}
	public static void main(String args[]) {
		try {
		JSSCTest js = new JSSCTest();
		js.begin();
		}catch(Exception e) {
			e.printStackTrace();
		}
	}
	public void begin() {
		while(true) {
			String data = Integer.toString(500) + ';' + Integer.toString(500);
			//sf.write(data.getBytes());
			try {
				Thread.sleep(1500);
			}catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
}
