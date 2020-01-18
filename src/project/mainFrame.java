package project;


import java.io.IOException;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;

import java.util.Enumeration;


import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.AbstractButton;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTextField;


import org.opencv.videoio.Videoio;


import javafx.geometry.Point3D;

public class mainFrame extends JFrame{
	
	double[] times;
	Point3D[] gantryCoordinates;
	SerialFast sf;
	CameraManager cm = new CameraManager();
	double startTime;
	double iterations;
	int cameraNumber;
	Container contentPane;
	JTextField File;
	JTextField correctionCoefficient;
	JTextField exposure = new JTextField();
	JTextField definition = new JTextField();
	JTextField LocationX, LocationY, LocationZ,
	TLX, TLY, TLZ,
	TRX, TRY, TRZ,
	BLX, BLY, BLZ,
	BRX, BRY, BRZ;
	JRadioButton cameraButton0, cameraButton1, cameraButton2, cameraButton3;
	JRadioButton modeButton0, modeButton1;
	JLabel Point, X, Y, Z, Location, TL, TR, BL, BR;
	JLabel blank0;
	JLabel CC;
	JLabel Exposure;
	JLabel Definition;
	JLabel cameraSettings;
	//JLabel openFile, saveFile;
	JLabel trackingMode; 
	JLabel FileName = new JLabel("File Name:");
	JButton Submit;
	JButton Open;
	JButton Save;
	JButton clear;
	JPanel Panel0;
	JPanel Panel1;
	JPanel Panel2;
	JPanel Panel3;
	JPanel containerBox;
	JPanel inputContainer;
	JPanel submitContainer;
	JPanel saveContainer;
	JPanel openContainer;
	JPanel bufferPanel0;
	JPanel bufferPanel1;
	JPanel holdingBox;
	JPanel fileBox;
	JPanel bufferBox;
	JPanel clearBox;
	JPanel modePanel;
	ButtonGroup group;
	ButtonGroup cameraMode;
	
	public mainFrame(){
		this.setDefaultCloseOperation(EXIT_ON_CLOSE);
		contentPane = this.getContentPane(); 
		this.addWindowListener(new WindowAdapter() {
	          public void windowClosing(WindowEvent e) {
	        	  cm.Cameras.get(0).VC.cap.release();
	        	  cm.Cameras.get(1).VC.cap.release();
	        	  cm.Cameras.get(2).VC.cap.release();
	        	  cm.Cameras.get(3).VC.cap.release();
	        	  sf.endPort();
	              System.exit(0);
	          }
	     });
		sf = new SerialFast();
		sf.startPort("COM3");
		times = new double[2];
		times[1] = System.currentTimeMillis()/1000;
		gantryCoordinates = new Point3D[2];
		gantryCoordinates[1] = new Point3D(0,0,0);
		cameraNumber = 0;
		LocationX = new JTextField();LocationY = new JTextField();LocationZ = new JTextField();
		TLX = new JTextField();TLY = new JTextField();TLZ = new JTextField();
		TRX = new JTextField();TRY = new JTextField();TRZ = new JTextField();
		BLX = new JTextField();BLY = new JTextField();BLZ = new JTextField();
		BRX = new JTextField();BRY = new JTextField();BRZ = new JTextField();
		File = new JTextField();
		exposure = new JTextField();
		definition = new JTextField();
		correctionCoefficient = new JTextField();
		File = new JTextField();
		clear = new JButton("Terminate Tracking Rectangle");
		Submit = new JButton("Submit Camera Configuration");
		Open = new JButton("Open Camera Configuration");
		Save = new JButton("Save Camera Configuration");
		Point = new JLabel("Point ");
		X = new JLabel("     X");Y = new JLabel("     Y");Z = new JLabel("     Z");
		Location = new JLabel("Location ");
		TL = new JLabel("TL ");
		TR = new JLabel("TR ");
		BL = new JLabel("BL ");
		BR = new JLabel("BR ");
		FileName = new JLabel("File Name:");
		blank0 = new JLabel("");
		CC = new JLabel("CC");
		Exposure = new JLabel("Exposure");
		Definition = new JLabel("Resolution");
		cameraSettings = new JLabel("Camera Settings");
		trackingMode = new JLabel("Tracking Mode:");
		bufferBox = new JPanel();
		containerBox = new JPanel();
		inputContainer = new JPanel();
		bufferPanel0 = new JPanel();
		bufferPanel1 = new JPanel();
		holdingBox = new JPanel();
		fileBox = new JPanel();
		submitContainer = new JPanel();
		clearBox = new JPanel();
		cameraButton0 = new JRadioButton("Camera0");
	    cameraButton0.setActionCommand("0");
	    cameraButton1 = new JRadioButton("Camera1");
	    cameraButton1.setActionCommand("1");
	    cameraButton2 = new JRadioButton("Camera2");
	    cameraButton2.setActionCommand("2");
	    cameraButton3 = new JRadioButton("Camera3");
	    cameraButton3.setActionCommand("3");
	    modeButton0 = new JRadioButton("AUTO");
	    modeButton0.setActionCommand("0");
	    modeButton1 = new JRadioButton("MANUAL");
	    modeButton1.setActionCommand("1");
	    group = new ButtonGroup();
	    cameraMode = new ButtonGroup();
	    
	}public static void main(String[] args) {
		
		mainFrame myFrame = new mainFrame();
		myFrame.create();
	}public void create() {
		modePanel = new JPanel();
		fileBox.setLayout(new GridLayout(2,2));
		holdingBox.setLayout(new BoxLayout(holdingBox, BoxLayout.Y_AXIS));
		bufferBox.setPreferredSize(new Dimension(400, 300));
		holdingBox.setPreferredSize(new Dimension(400,10));
		bufferPanel0.setPreferredSize(new Dimension(0,400));
		bufferPanel1.setPreferredSize(new Dimension(100,400));
		inputContainer.setLayout(new GridLayout(8,4));
		clearBox.add(clear);
		fileBox.add(Open);fileBox.add(Save);fileBox.add(FileName);fileBox.add(File);
		inputContainer.add(Point);inputContainer.add(X);inputContainer.add(Y);inputContainer.add(Z);
		inputContainer.add(Location);inputContainer.add(LocationX);inputContainer.add(LocationY);inputContainer.add(LocationZ);
		inputContainer.add(TL);inputContainer.add(TLX);inputContainer.add(TLY);inputContainer.add(TLZ);
		inputContainer.add(TR);inputContainer.add(TRX);inputContainer.add(TRY);inputContainer.add(TRZ);
		inputContainer.add(BL);inputContainer.add(BLX);inputContainer.add(BLY);inputContainer.add(BLZ);
		inputContainer.add(BR);inputContainer.add(BRX);inputContainer.add(BRY);inputContainer.add(BRZ);
		inputContainer.add(blank0);inputContainer.add(CC);inputContainer.add(Exposure);inputContainer.add(Definition);
		inputContainer.add(cameraSettings);inputContainer.add(correctionCoefficient);inputContainer.add(exposure);inputContainer.add(definition);
		submitContainer.add(Submit);
		holdingBox.add(inputContainer);
		holdingBox.add(submitContainer);
		holdingBox.add(fileBox);
		holdingBox.add(modePanel);
		holdingBox.add(clearBox);
		holdingBox.add(bufferBox);
		
	    group.add(cameraButton0);
	    group.add(cameraButton1);
	    group.add(cameraButton2);
	    group.add(cameraButton3);
	    cameraMode.add(modeButton0);
	    cameraMode.add(modeButton1);
	    
	    ActionListener alisten = new CameraActionListener();
	    ActionListener clearListener = new ClearActionListener();
	    ActionListener trackingModeListener = new TrackingModeListener();
	    cameraButton0.addActionListener(alisten);
	    cameraButton1.addActionListener(alisten);
	    cameraButton2.addActionListener(alisten);
	    cameraButton3.addActionListener(alisten);
	    modeButton0.addActionListener(trackingModeListener);
	    modeButton1.addActionListener(trackingModeListener);
	    clear.addActionListener(clearListener);
	    
	    JPanel c = new JPanel();
	    c.add(cameraButton0);
	    c.add(cameraButton1);
	    c.add(cameraButton2);
	    c.add(cameraButton3);
	    
	    modePanel.add(trackingMode);
		modePanel.add(modeButton0);
		modePanel.add(modeButton1);
	   
	    ActionListener submitListen = new SubmitActionListener();
	    Submit.addActionListener(submitListen);
	    
	    ActionListener openListener = new OpenActionListener();
	    Open.addActionListener(openListener);
	    
	    ActionListener saveListener = new SaveActionListener();
	    Save.addActionListener(saveListener);
		this.setSize(new Dimension(1000, 600));
		
		containerBox.setLayout(new BoxLayout(containerBox, BoxLayout.X_AXIS));	
		Panel0 = new ImagePanel(0, cm, 0, "camera0", "TrackerKCF");
		Panel1 = new ImagePanel(1, cm, 1, "camera1", "TrackerKCF");
		Panel1.setVisible(false);
		Panel2 = new ImagePanel(2, cm, 2, "camera2", "TrackerKCF");
		Panel2.setVisible(false);
		Panel3 = new ImagePanel(3, cm, 3, "camera2", "TrackerKCF");
		Panel3.setVisible(false);
		Panel0.setPreferredSize(new Dimension(800, 600));
		Panel1.setPreferredSize(new Dimension(800, 600));
		Panel2.setPreferredSize(new Dimension(800, 600));
		Panel3.setPreferredSize(new Dimension(800, 600));
		containerBox.add(Panel0);
		containerBox.add(Panel1);
		containerBox.add(Panel2);
		containerBox.add(Panel3);
		containerBox.add(bufferPanel0);
		containerBox.add(holdingBox);
		containerBox.add(bufferPanel1);
		contentPane.add(c, BorderLayout.NORTH);
		contentPane.add(containerBox, BorderLayout.CENTER);
		
		this.setVisible(true);
		imageThread thread0 = new imageThread((ImagePanel)Panel0);
		imageThread thread1 = new imageThread((ImagePanel)Panel1);
		imageThread thread2 = new imageThread((ImagePanel)Panel2);
		imageThread thread3 = new imageThread((ImagePanel)Panel3);
		calculateThread cT = new calculateThread(cm);
		thread0.setDaemon(true);
		thread1.setDaemon(true);
		thread2.setDaemon(true);
		thread3.setDaemon(true);
		cT.setDaemon(true);
		thread0.start();
		thread1.start();
		thread2.start();
		thread3.start();
		cT.start();
	}
	
	@Override
	public void paint(Graphics g) {
		super.paint(g);
	}

	class imageThread extends Thread{
		ImagePanel PANEL;
		
		public imageThread(ImagePanel panel) {
			PANEL = panel;
		}
		@Override
		public void run() {
			for (;;){
				cm.update(PANEL.INDEX);
				PANEL.repaint();
				if(System.currentTimeMillis() % 500 < 100) {
				//System.out.println(PANEL.INDEX + " " + cm.Cameras.get(PANEL.INDEX).TrackingSTATUS);
				}
				try { Thread.sleep(5);
				} catch (InterruptedException e) {    }
			}
		}
	}
	
	class calculateThread extends Thread{
		CameraManager CAMERA_MANAGER;
		double acceleration = 386.2205;
		double averageVelocityZ;
		double instantaneousVelocityZ;
		double impactTime;
		double impactX;
		double impactY;
		Point3D realPoint;
		Point3D gantryPoint;

		public calculateThread(CameraManager cameraManager) {
			CAMERA_MANAGER = cameraManager;
		}
		@Override
		public void run() {
			for (;;){
				realPoint = CAMERA_MANAGER.calculatePositionUsingCameras();
				gantryPoint = new Point3D(realPoint.getX() - 93, realPoint.getY() - 89, realPoint.getZ() - 8);
				System.out.println(gantryPoint);
				//93 89 8
				
				times[0] = (double)System.currentTimeMillis()/1000;
				
				gantryCoordinates[0] = gantryPoint;
				acceleration = 386.2205;
				averageVelocityZ = (gantryCoordinates[0].getZ() - gantryCoordinates[1].getZ())/
											(times[0] - times[1]);
				instantaneousVelocityZ = averageVelocityZ - acceleration*(times[0] - times[1])/2;
				impactTime = (instantaneousVelocityZ + Math.sqrt(Math.pow(instantaneousVelocityZ,2) + 2*acceleration*gantryPoint.getZ()))/acceleration;
				impactX = impactTime*(gantryCoordinates[0].getX() - gantryCoordinates[1].getX())/(times[0] - times[1]) + gantryCoordinates[0].getX(); 
				impactY = impactTime*(gantryCoordinates[0].getY() - gantryCoordinates[1].getY())/(times[0] - times[1]) + gantryCoordinates[0].getY();
				
				/*if(Double.isNaN(impactX) || Double.isNaN(impactX)) {
					sf.sendToGantryCoordinate(50, 30);
				}else {*/
				//sf.sendToGantryCoordinate(impactX, impactY);
				sf.sendToGantryCoordinate(gantryPoint.getX(), gantryPoint.getY());
				//}
				
				times[1] = times[0];
				gantryCoordinates[1] = gantryCoordinates[0];
				
				//sf.sendToGantryCoordinate((int)CAMERA_MANAGER.Cameras.get(0).getX(), (int)CAMERA_MANAGER.Cameras.get(0).getY());
				try { Thread.sleep(5);
				} catch (InterruptedException e) {    }
			}
		}
	}
	 class CameraActionListener implements ActionListener {
	      public void actionPerformed(ActionEvent ev) {
	        System.out.println(group.getSelection().getActionCommand());
	        if(group.getSelection().getActionCommand() == "0") {
	        	cm.currentCamera = 0;
	        	cameraNumber = 0;
	        	Panel0.setVisible(true);
	        	Panel1.setVisible(false);
	        	Panel2.setVisible(false);
	        	Panel3.setVisible(false);
	        	if(cm.Cameras.get(cm.currentCamera).TrackingMode == "AUTO") {
		        	modeButton0.doClick();
	        	}else if(cm.Cameras.get(cm.currentCamera).TrackingMode == "MANUAL") {
	        	modeButton1.doClick();
	        	}
	        } else if(group.getSelection().getActionCommand() == "1") {
	        	cm.currentCamera = 1;
	        	cameraNumber = 1;
	        	Panel0.setVisible(false);
	        	Panel1.setVisible(true);
	        	Panel2.setVisible(false);
	        	Panel3.setVisible(false);
	        	if(cm.Cameras.get(cm.currentCamera).TrackingMode == "AUTO") {
		        	modeButton0.doClick();
	        	}else if(cm.Cameras.get(cm.currentCamera).TrackingMode == "MANUAL") {
	        	modeButton1.doClick();
	        	}
	        } else if(group.getSelection().getActionCommand() == "2") {
	        	cm.currentCamera = 2;
	        	cameraNumber = 2;
	        	Panel0.setVisible(false);
	        	Panel1.setVisible(false);
	        	Panel2.setVisible(true);
	        	Panel3.setVisible(false);
	        	if(cm.Cameras.get(cm.currentCamera).TrackingMode == "AUTO") {
		        	modeButton0.doClick();
	        	}else if(cm.Cameras.get(cm.currentCamera).TrackingMode == "MANUAL") {
	        	modeButton1.doClick();
	        	}
	        }else if(group.getSelection().getActionCommand() == "3") {
	        	cm.currentCamera = 3;
	        	cameraNumber = 3;
	        	Panel0.setVisible(false);
	        	Panel1.setVisible(false);
	        	Panel2.setVisible(false);
	        	Panel3.setVisible(true);
	        	if(cm.Cameras.get(cm.currentCamera).TrackingMode == "AUTO") {
		        	modeButton0.doClick();
	        	}else if(cm.Cameras.get(cm.currentCamera).TrackingMode == "MANUAL") {
	        	modeButton1.doClick();
	        	}
	        }
	        LocationX.setText(Double.toString(cm.Cameras.get(cameraNumber).getX()));
       	LocationY.setText(Double.toString(cm.Cameras.get(cameraNumber).getY()));
       	LocationZ.setText(Double.toString(cm.Cameras.get(cameraNumber).getZ()));
       	TLX.setText(Double.toString(cm.Cameras.get(cameraNumber).getTL().getX()));
       	TLY.setText(Double.toString(cm.Cameras.get(cameraNumber).getTL().getY()));
       	TLZ.setText(Double.toString(cm.Cameras.get(cameraNumber).getTL().getZ()));
       	TRX.setText(Double.toString(cm.Cameras.get(cameraNumber).getTR().getX()));
       	TRY.setText(Double.toString(cm.Cameras.get(cameraNumber).getTR().getY()));
       	TRZ.setText(Double.toString(cm.Cameras.get(cameraNumber).getTR().getZ()));
       	BLX.setText(Double.toString(cm.Cameras.get(cameraNumber).getBL().getX()));
       	BLY.setText(Double.toString(cm.Cameras.get(cameraNumber).getBL().getY()));
       	BLZ.setText(Double.toString(cm.Cameras.get(cameraNumber).getBL().getZ()));
       	BRX.setText(Double.toString(cm.Cameras.get(cameraNumber).getBR().getX()));
       	BRY.setText(Double.toString(cm.Cameras.get(cameraNumber).getBR().getY()));
       	BRZ.setText(Double.toString(cm.Cameras.get(cameraNumber).getBR().getZ()));
       	correctionCoefficient.setText(Double.toString(cm.Cameras.get(cameraNumber).getCC()));
       	exposure.setText(Double.toString(cm.Cameras.get(cameraNumber).getExposure()));
       	definition.setText(Double.toString(cm.Cameras.get(cameraNumber).getCapturedMat().width()));
	      }
	    }
	    
	    class SubmitActionListener implements ActionListener {
	    	public void actionPerformed(ActionEvent ev) {
	    		cm.Cameras.get(cameraNumber).setX(Double.parseDouble(LocationX.getText()));
	    		cm.Cameras.get(cameraNumber).setY(Double.parseDouble(LocationY.getText()));
	    		cm.Cameras.get(cameraNumber).setZ(Double.parseDouble(LocationZ.getText()));
	    		cm.Cameras.get(cameraNumber).setTL(new Point3D(Double.parseDouble(TLX.getText()),
	    											Double.parseDouble(TLY.getText()),
	    											Double.parseDouble(TLZ.getText())));
	    		cm.Cameras.get(cameraNumber).setTR(new Point3D(Double.parseDouble(TRX.getText()),
													Double.parseDouble(TRY.getText()),
													Double.parseDouble(TRZ.getText())));
	    		cm.Cameras.get(cameraNumber).setBL(new Point3D(Double.parseDouble(BLX.getText()),
													Double.parseDouble(BLY.getText()),
													Double.parseDouble(BLZ.getText())));
	    		cm.Cameras.get(cameraNumber).setBR(new Point3D(Double.parseDouble(BRX.getText()),
													Double.parseDouble(BRY.getText()),
													Double.parseDouble(BRZ.getText())));
	    		cm.Cameras.get(cameraNumber).orthogonalizeFrustum();
	    		cm.Cameras.get(cameraNumber).setCC(Double.parseDouble(correctionCoefficient.getText()));
	    		cm.Cameras.get(cameraNumber).setExposure(Double.parseDouble(exposure.getText()));
	    		cm.Cameras.get(cameraNumber).VC.cap.set(Videoio.CAP_PROP_FRAME_WIDTH, (int)Double.parseDouble(definition.getText()));
	    		cm.Cameras.get(cameraNumber).VC.cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, (int)Double.parseDouble(definition.getText())*0.66);
	    		
	    	}
	    }
	    class ClearActionListener implements ActionListener {
	    	public void actionPerformed(ActionEvent ev) {
	    	cm.Cameras.get(cameraNumber).setTrackingStatus(false);
	    	}
	    }
	    class OpenActionListener implements ActionListener {
	    	public void actionPerformed(ActionEvent ev) {
	    		String fileName = File.getText();

	            try {
	                // Use this for reading the data.
	                byte[] buffer = new byte[1000];

	                FileInputStream inputStream = 
	                    new FileInputStream(fileName);

	                // read fills buffer with data and returns
	                // the number of bytes read (which of course
	                // may be less than the buffer size, but
	                // it will never be more).
	                int total = 0;
	                int nRead = 0;
	                while((nRead = inputStream.read(buffer)) != -1) {
	                    // Convert to String so we can display it.
	                    // Of course you wouldn't want to do this with
	                    // a 'real' binary file.
	                	String str = new String(buffer);
	                	List<String> myList = Arrays.asList(str.split(";"));
	                	int i = 0;
	                	for(String s : myList) {
	                		List<String> theList = Arrays.asList(s.split(","));
	                		for(int x=0;x<theList.size();x++) {
	                            System.out.println(i+" : "+Double.parseDouble(theList.get(x)));
	                            if(x == 0) {
	                            	cm.Cameras.get(i).setX(Double.parseDouble(theList.get(x)));
	                            }else if(x == 1) {
	                            	cm.Cameras.get(i).setY(Double.parseDouble(theList.get(x)));
	                            }else if(x == 2) {
	                            	cm.Cameras.get(i).setZ(Double.parseDouble(theList.get(x)));
	                            }else if(x == 5) {
	                            	cm.Cameras.get(i).setTL(new Point3D(Double.parseDouble(theList.get(x-2)),
	                            			Double.parseDouble(theList.get(x-1)),
	                            			Double.parseDouble(theList.get(x))));
	                            }else if(x == 8) {
	                            	cm.Cameras.get(i).setTR(new Point3D(Double.parseDouble(theList.get(x-2)),
	                            			Double.parseDouble(theList.get(x-1)),
	                            			Double.parseDouble(theList.get(x))));
	                            }else if(x == 11) {
	                            	cm.Cameras.get(i).setBL(new Point3D(Double.parseDouble(theList.get(x-2)),
	                            			Double.parseDouble(theList.get(x-1)),
	                            			Double.parseDouble(theList.get(x))));
	                            }else if(x == 14) {
	                            	cm.Cameras.get(i).setBR(new Point3D(Double.parseDouble(theList.get(x-2)),
	                            			Double.parseDouble(theList.get(x-1)),
	                            			Double.parseDouble(theList.get(x))));
	                            }else if(x == 17) {
	                            	cm.Cameras.get(i).setCC(Double.parseDouble(theList.get(x-2)));
	                            	cm.Cameras.get(i).setExposure(Double.parseDouble(theList.get(x-1)));
	                            	cm.Cameras.get(i).VC.cap.set(Videoio.CAP_PROP_FRAME_WIDTH, (int)Double.parseDouble(theList.get(x)));
	                	    		cm.Cameras.get(i).VC.cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, (int)Double.parseDouble(theList.get(x))*0.66);
	                            }
	                        	}
	                		i++;
	                	}
	                	
	                    total += nRead;
	                    System.out.println(total);
	                }   

	                // Always close files.
	                inputStream.close();        

	                System.out.println("Read " + total + " bytes");
	            }
	            catch(FileNotFoundException ex) {
	                System.out.println(
	                    "Unable to open file '" + 
	                    fileName + "'");                
	            }
	            catch(IOException ex) {
	                System.out.println(
	                    "Error reading file '" 
	                    + fileName + "'");                  
	                // Or we could just do this: 
	                // ex.printStackTrace();
	            }
	    		
	    	}
	    }
	    class SaveActionListener implements ActionListener {
	    	public void actionPerformed(ActionEvent ev) {
	    		String fileName = File.getText();

	            try {
	                // Assume default encoding.
	                FileWriter fileWriter =
	                    new FileWriter(fileName);

	                // Always wrap FileWriter in BufferedWriter.
	                BufferedWriter bufferedWriter =
	                    new BufferedWriter(fileWriter);

	                // Note that write() does not automatically
	                // append a newline character.
	                int x = 1;
	                for(Camera c : cm.Cameras) {
	                bufferedWriter.write(Double.toString(c.getX())+","+
	                					Double.toString(c.getY())+","+
	                					Double.toString(c.getZ())+","+
	                					Double.toString(c.getTL().getX())+","+
	                					Double.toString(c.getTL().getY())+","+
	                					Double.toString(c.getTL().getZ())+","+
	                					Double.toString(c.getTR().getX())+","+
	                					Double.toString(c.getTR().getY())+","+
	                					Double.toString(c.getTR().getZ())+","+
	                					Double.toString(c.getBL().getX())+","+
	                					Double.toString(c.getBL().getY())+","+
	                					Double.toString(c.getBL().getZ())+","+
	                					Double.toString(c.getBR().getX())+","+
	                					Double.toString(c.getBR().getY())+","+
	                					Double.toString(c.getBR().getZ())+","+
	                					Double.toString(c.getCC())+","+
	                					Double.toString(c.getExposure())+","+
	                					Double.toString(c.getCapturedMat().width()));
	                if(x < cm.Cameras.size()) {
	                	bufferedWriter.write(";");
	                }
	                x++;
	                }

	                // Always close files.
	                bufferedWriter.close();
	            }
	            catch(IOException ex) {
	                System.out.println(
	                    "Error writing to file '"
	                    + fileName + "'");
	                // Or we could just do this:
	                // ex.printStackTrace();
	            }
	    		
	    		
	    	}
	    }
	    class TrackingModeListener implements ActionListener {
		      public void actionPerformed(ActionEvent ev) {
		     //   System.out.println(group.getSelection().getActionCommand());
		        if(cameraMode.getSelection().getActionCommand() == "0") {
		        	if(cm.Cameras.get(cm.currentCamera).TrackingMode != "AUTO") {
		        	cm.Cameras.get(cm.currentCamera).setTrackingStatus(false);
		        	}
		        	cm.Cameras.get(cm.currentCamera).TrackingMode = "AUTO";
		        } else if(cameraMode.getSelection().getActionCommand() == "1") {
		        	if(cm.Cameras.get(cm.currentCamera).TrackingMode != "MANUAL") {
			        	cm.Cameras.get(cm.currentCamera).setTrackingStatus(false);
			        	}
		        	cm.Cameras.get(cm.currentCamera).TrackingMode = "MANUAL";
		        } 
		        
		    }
	    }
	
}
