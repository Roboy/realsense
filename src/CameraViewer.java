import intel.rssdk.*;
import intel.rssdk.PXCMFaceData.Face;

import javax.imageio.ImageIO;
import javax.swing.*;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;

import java.awt.event.*;
import java.awt.image.*;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;
import java.awt.*;

public class CameraViewer
{    
	/*
	 * change "host" to the IP from your linux machine
	 * Enter: "ifconfig" in a terminal in linux and search for "eth0". Use the "inet addr"!!
	 */
	static String host = "10.183.16.47";
	static int cWidth  = 640;
	static int cHeight = 480;
	static int dWidth, dHeight;
	static boolean exit = false;
	static Ros ros;
	static PXCMRectI32 lastFaceRect;

	static boolean useROS = false;

	public static void main(String s[]) throws FileNotFoundException, IOException
	{
		// output where the "libpxcclr.jni64.dll" has to be copied to (at least in one of these paths)
		//System.out.println("Path: " + System.getProperty("java.library.path"));

		// load realsense library
		System.loadLibrary("libpxcclr.jni64");

		if(useROS)
		{
			// connect to ROSbridge
			System.out.println("Try to connect to ROSbridge");
			ros  = new Ros(host);
			ros.connect();
			System.out.println("Connection established.");

			// subscribe to a topic with name "/listener"
			subscribeTopic("/listener", "std_msgs/String");
		}
		else
			System.out.println("ROSbridge is turned off");

		// get a PXCMSenseManager which is used for the connection to the Realsense
		PXCMSenseManager senseMgr = PXCMSenseManager.CreateInstance(); 

		// enabling the different provided streams
		pxcmStatus sts = senseMgr.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_COLOR, cWidth, cHeight);
		sts = senseMgr.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_DEPTH);

		// enable face tracking
		pxcmStatus statusFace = senseMgr.EnableFace(null);
		if ( statusFace == pxcmStatus.PXCM_STATUS_NO_ERROR)
			System.out.println("Status Face is fine");
		else
			System.out.println("Status: " + statusFace.toString());

		PXCMFaceModule faceModule = senseMgr.QueryFace();

		PXCMFaceConfiguration faceConf = faceModule.CreateActiveConfiguration();
		faceConf.SetTrackingMode(PXCMFaceConfiguration.TrackingModeType.FACE_MODE_COLOR_PLUS_DEPTH);
		PXCMFaceConfiguration.RecognitionConfiguration recognitionConf = faceConf.QueryRecognition();

		// Enable face recognition
		recognitionConf.Enable();

		// Create a recognition database
		PXCMFaceConfiguration.RecognitionConfiguration.RecognitionStorageDesc desc = new PXCMFaceConfiguration.RecognitionConfiguration.RecognitionStorageDesc();
		desc.maxUsers = 10;
		pxcmStatus storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		
		// load database
		File databaseFile = new File("roboydb.txt");
		if (databaseFile.exists())
		{
			FileInputStream fis = new FileInputStream(databaseFile);
			byte[] buffer = new byte[fis.available()];
			fis.read(buffer);
			recognitionConf.SetDatabaseBuffer(buffer, buffer.length);
			System.out.println("Database loaded!");
		}
		else
		{
			System.out.println("Database could not be found!");
		}

		// Set the registration mode
		recognitionConf.SetRegistrationMode(PXCMFaceConfiguration.RecognitionConfiguration.RecognitionRegistrationMode.REGISTRATION_MODE_CONTINUOUS);

		faceConf.EnableAllAlerts();
		// Make it effective
		//faceConf.ApplyChanges();

		// initialize the manager
		sts = senseMgr.Init();
		
		System.out.println(sts);

		// retrieve the face tracking results
		PXCMFaceData faceData = faceModule.CreateOutput();

		// initialize the capturing of the streams
		PXCMCapture.Device device = senseMgr.QueryCaptureManager().QueryDevice();
		PXCMCapture.Device.StreamProfileSet profiles = new PXCMCapture.Device.StreamProfileSet();
		device.QueryStreamProfileSet(profiles);

		dWidth = profiles.depth.imageInfo.width;
		dHeight = profiles.depth.imageInfo.height;

		Listener listener = new Listener();

		// open a window for the color stream
		CameraViewer c_raw = new CameraViewer(); 
		DrawFrame c_df = new DrawFrame(cWidth, cHeight);
		JFrame cframe= new JFrame("Intel(R) RealSense(TM) SDK - Color Stream");	
		cframe.addWindowListener(listener);
		cframe.setSize(cWidth, cHeight); 
		cframe.add(c_df);
		cframe.setVisible(true); 

		// open a window for the depth stream
		CameraViewer d_raw = new CameraViewer(); 
		DrawFrame d_df=new DrawFrame(dWidth, dHeight);      
		JFrame dframe= new JFrame("Intel(R) RealSense(TM) SDK - Depth Stream"); 
		dframe.addWindowListener(listener);
		dframe.setSize(dWidth, dHeight); 
		dframe.add(d_df);
		dframe.setVisible(true); 

		if (sts == pxcmStatus.PXCM_STATUS_NO_ERROR)
		{
			int counter = 0;
			while (listener.exit == false)
			{
				counter++;				

				// aquire one frame
				sts = senseMgr.AcquireFrame(true);

				if (sts == pxcmStatus.PXCM_STATUS_NO_ERROR)
				{
					// get one sample from the camera
					PXCMCapture.Sample sample = senseMgr.QuerySample();

					faceModule=senseMgr.QueryFace();

					faceData.Update();
					if (faceModule != null)
					{							
						int nfaces = faceData.QueryNumberOfDetectedFaces();
						System.out.println("Faces: " + nfaces);
						if(nfaces > 0)
						{
							for (int i = 0 ; i < nfaces ; i++)
							{							   
								Face face = faceData.QueryFaceByIndex(0);
								PXCMFaceData.DetectionData detectData = face.QueryDetection(); 
								PXCMFaceData.RecognitionData recognitionData = face.QueryRecognition();

								System.out.println("recognitionData" + recognitionData.toString());

								// recognize the current face?
								int userId = recognitionData.QueryUserID();
								if (userId >= 0) {
									System.out.println("UserId: " + userId);
								}
								System.out.println("Id: " + userId);
								if(!recognitionData.IsRegistered())
								{	
									recognitionData.RegisterUser();
									System.out.println("New UserId: " + recognitionData.RegisterUser());
								}
								else
									System.out.println("User is already registered");

								if(recognitionData.IsRegistered())
								{	
									System.out.println("Is registered");
								}

								if (detectData != null)
								{
									lastFaceRect = new PXCMRectI32();
									boolean ret = detectData.QueryBoundingRect(lastFaceRect);
									if (ret) {
										System.out.println("");
										System.out.println ("Detection Rectangle at frame #" + counter); 
										System.out.println ("Top Left corner: (" + lastFaceRect.x + "," + lastFaceRect.y + ")" ); 
										System.out.println ("Height: " + lastFaceRect.h + " Width: " + lastFaceRect.w); 

										if(useROS)
										{
											if (counter > 10)
											{
												// publish a topic to "/echo"
												publishTopic("/echo", "std_msgs/String", "{\"data\": \"" + lastFaceRect.x + "," + lastFaceRect.y + "\"}");

												// calls a service from ROSbridge for adding two ints
												//callService("/add_two_ints", "rospy_tutorials/AddTwoInts", "{\"a\": 10, \"b\": 20}");

												counter = 0;
											}
										}
									}
								}
							}														
						}
						else
							lastFaceRect = null;
					}

					if (sample.color != null)
					{
						// get image data
						PXCMImage.ImageData cData = new PXCMImage.ImageData();                
						sts = sample.color.AcquireAccess(PXCMImage.Access.ACCESS_READ,PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, cData);
						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR) < 0)
						{
							System.out.println ("Failed to Acquire Access of color image data " + sts);
							System.exit(3);
						}

						int cBuff[] = new int[cData.pitches[0]/4 * cHeight];

						cData.ToIntArray(0, cBuff);
						c_df.image.setRGB (0, 0, cWidth, cHeight, cBuff, 0, cData.pitches[0]/4);

						drawFaceRect(lastFaceRect, c_df.image);

						//send image to linux
						if(useROS)
							sendImage2(c_df.image);

						c_df.repaint();  
						sts = sample.color.ReleaseAccess(cData);

						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0)
						{
							System.out.println ("Failed to Release Access of color image data");
							System.exit(3);
						}
					}

					if (sample.depth != null)
					{       
						// get depth data
						PXCMImage.ImageData dData = new PXCMImage.ImageData();
						sample.depth.AcquireAccess(PXCMImage.Access.ACCESS_READ,PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, dData);
						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0)
						{
							System.out.println ("Failed to Acquire Access of depth image data");
							System.exit(3);
						}

						int dBuff[] = new int[dData.pitches[0]/4 * dHeight];
						dData.ToIntArray(0, dBuff);
						d_df.image.setRGB (0, 0, dWidth, dHeight, dBuff, 0, dData.pitches[0]/4);
						d_df.repaint();
						sts = sample.depth.ReleaseAccess(dData);
						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0)
						{
							System.out.println ("Failed to Release Access of depth image data");
							System.exit(3);
						}
					}  
				}
				else
				{
					System.out.println("Failed to acquire frame: " + sts);
				}

				// release actual frame
				senseMgr.ReleaseFrame();
			}

			// closes connection to realsense
			senseMgr.Close();
			System.out.println("Done streaming");
		}
		else
		{
			System.out.println("Failed to initialize");
		}

		if(useROS)
		{
			// disconnets from ROS
			ros.disconnect();
		}
		
		PXCMFaceData recData = faceModule.CreateOutput();

		// allocate the buffer to save the database
		PXCMFaceData.RecognitionModuleData rmd=recData.QueryRecognitionModule();
		int nbytes=rmd.QueryDatabaseSize();
		byte[] buffer=new byte[nbytes];

		// retrieve the database buffer
		rmd.QueryDatabaseBuffer(buffer);

		// Save the buffer to a file
		FileOutputStream fos;
		try {
			File file = new File("roboydb.txt");
			fos = new FileOutputStream(file);
			fos.write(buffer);
			fos.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


		// Now load the database buffer back.
		recognitionConf.SetDatabaseBuffer(buffer,nbytes);
		
		cframe.dispose();
		dframe.dispose();
	}

	private static void drawFaceRect(PXCMRectI32 rect, BufferedImage image) {
		if(rect != null)
		{
			int x = rect.x;
			int y = rect.y;
			if(cHeight > y+rect.w && cWidth > x+rect.h)
			{
				for(int h = 0 ; h < rect.h; h++)
				{
					image.setRGB(x+h, y, 128);
					image.setRGB(x+h, y+rect.w, 128);
				}
				for(int w = 0 ; w < rect.w; w++)
				{
					image.setRGB(x, y+w, 128);
					image.setRGB(x+rect.h, y+w, 128);
				}
			}
		}
	}

	private static void sendImage() {
		BufferedImage bimg;
		int port = 6066;
		//String host = "10.183.20.136";
		try
		{
			System.out.println("Connecting to " + host
					+ " on port " + port);
			Socket client = new Socket(host, port);

			System.out.println("Just connected to "
					+ client.getRemoteSocketAddress());

			DataInputStream in=new DataInputStream(client.getInputStream());
			System.out.println(in.readUTF());
			System.out.println(in.readUTF());

			DataOutputStream out =
					new DataOutputStream(client.getOutputStream());

			out.writeUTF("Hello from "
					+ client.getLocalSocketAddress());
			out.writeUTF("client: hello to server");

			bimg = ImageIO.read(new File("C:\\Users\\Roboy\\Pictures\\Saved Pictures\\download.jpg"));

			ImageIO.write(bimg,"JPG",client.getOutputStream());
			System.out.println("Image sent!!!!");
			client.close();
		}catch(IOException e)
		{
			e.printStackTrace();
		}		
	}

	private static void sendImage2(BufferedImage bimg) {

		int port = 6066;
		//String host = "10.183.20.136";
		try
		{
			Socket client = new Socket(host, port);

			DataInputStream in=new DataInputStream(client.getInputStream());
			System.out.println(in.readUTF());
			System.out.println(in.readUTF());

			DataOutputStream out = new DataOutputStream(client.getOutputStream());

			out.writeUTF("Hello from " + client.getLocalSocketAddress());
			out.writeUTF("client: hello to server");

			if(bimg == null)
				System.out.println("Image is null");
			else
			{
				ImageIO.write(bimg,"JPG",client.getOutputStream());
				System.out.println("Image sent!!!!");
			}
			client.close();
		}catch(IOException e)
		{
			e.printStackTrace();
		}		
	}

	private static BufferedImage createImageFromBytes(byte[] imageData) {
		ByteArrayInputStream bais = new ByteArrayInputStream(imageData);
		try {
			return ImageIO.read(bais);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	private static void publishTopic(String topic, String type, String message)
	{
		// publish a topic to "topic"
		Topic echo = new Topic(ros, topic, type);
		Message toSend = new Message(message);
		echo.publish(toSend);
	}

	private static void subscribeTopic(String topic, String type)
	{
		// subscribe to a topic with name "topic"
		Topic echoBack = new Topic(ros, topic, type);
		echoBack.subscribe(new TopicCallback() {
			public void handleMessage(Message message) {
				System.out.println("From ROS: " + message.toString());
			}
		});
	}

	private static void callService(String serviceName, String destination, String requestMsg)
	{
		// calls a service from ROSbridge
		Service service = new Service(ros, serviceName, destination);

		ServiceRequest request = new ServiceRequest(requestMsg);
		ServiceResponse response = service.callServiceAndWait(request);
		System.out.println(response.toString());
	}

	public static void _main(String s[]) throws java.io.IOException
	{
		PXCMSenseManager senseMgr = PXCMSenseManager.CreateInstance();

		senseMgr.EnableFace(null);

		PXCMFaceModule faceModule = senseMgr.QueryFace();

		// Retrieve the input requirements
		pxcmStatus sts = pxcmStatus.PXCM_STATUS_DATA_UNAVAILABLE; 
		PXCMFaceConfiguration faceConfig = faceModule.CreateActiveConfiguration();
		faceConfig.SetTrackingMode(PXCMFaceConfiguration.TrackingModeType.FACE_MODE_COLOR);
		faceConfig.detection.isEnabled = true; 
		faceConfig.landmarks.isEnabled = true; 
		faceConfig.pose.isEnabled = true; 
		faceConfig.ApplyChanges();
		faceConfig.Update();

		senseMgr.Init();

		System.out.println("Start");
		PXCMFaceData faceData = faceModule.CreateOutput();

		for (int nframes=0; nframes<300000; nframes++)
		{
			senseMgr.AcquireFrame(true);

			PXCMCapture.Sample sample = senseMgr.QueryFaceSample();

			//faceData = faceModule.CreateOutput();
			faceData.Update();

			// Read and print data 
			for (int fidx=0; ; fidx++) {
				PXCMFaceData.Face face = faceData.QueryFaceByIndex(fidx);
				if (face==null) break;
				PXCMFaceData.DetectionData detectData = face.QueryDetection(); 

				if (detectData != null)
				{
					PXCMRectI32 rect = new PXCMRectI32();
					boolean ret = detectData.QueryBoundingRect(rect);
					if (ret) {
						System.out.println("");
						System.out.println ("Detection Rectangle at frame #" + nframes); 
						System.out.println ("Top Left corner: (" + rect.x + "," + rect.y + ")" ); 
						System.out.println ("Height: " + rect.h + " Width: " + rect.w); 
					}
				} else 
					break;

				PXCMFaceData.PoseData poseData = face.QueryPose();
				if (poseData != null)
				{
					PXCMFaceData.PoseEulerAngles pea = new PXCMFaceData.PoseEulerAngles();
					poseData.QueryPoseAngles(pea);
					System.out.println ("Pose Data at frame #" + nframes); 
					System.out.println ("(Roll, Yaw, Pitch) = (" + pea.roll + "," + pea.yaw + "," + pea.pitch + ")"); 
				}  
			}  

			//faceData.close();
			senseMgr.ReleaseFrame();
		}
		faceData.close();
		senseMgr.Close();
		System.exit(0);
		System.out.println("End");
	} 
}

class Listener extends WindowAdapter {
	public boolean exit = false;
	@Override public void windowClosing(WindowEvent e) {
		exit=true;
	}
}



class DrawFrame extends Component { 
	public BufferedImage image; 

	public DrawFrame(int width, int height) { 
		image=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);
	} 

	public void paint(Graphics g) { 
		((Graphics2D)g).drawImage(image,0,0,null); 
	}
}