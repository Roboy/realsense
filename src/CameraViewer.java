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
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import java.awt.*;

/**
 * This class provides everything to use the Realsense camera on the Windows machine and to connect to ROS via a ROSbridge
 * @author Lucas Weidner
 *
 */
public class CameraViewer
{    
	/**
	 * Cchange "host" to the IP from your linux machine
	 * Enter: "ifconfig" in a terminal in linux and search for "eth0". Use the "inet addr"!!
	 */
	public static String host = "10.183.16.47";
	
	/**
	 * width of the color stream 
	 */
	public static int cWidth  = 640;
	
	/**
	 * height of the color stream
	 */
	public static int cHeight = 480;
	
	/**
	 * vertical field of view of the realsense
	 */
	public static double vfov = 43;
	
	/**
	 * horizontal field of view of the realsense
	 */
	public static double hfov = 70;
	
	/**
	 * width and height of the depth stream
	 */
	public static int dWidth, dHeight;
	
	/**
	 * variable to check, if the program should end
	 */
	public static boolean exit = false;
	
	/**
	 * ROSbridge
	 */
	public static Ros ros;
	
	/**
	 * rectangular with the face data
	 */
	public static PXCMRectI32 faceRect;
	
	/**
	 * array of all faces from the actual frame
	 */
	public static ArrayList<PXCMRectI32> lastFaceRect;
	
	/**
	 * variable for checking if the actual face should get registered in the database
	 * (at the moment, face recognition does not work, so it is useless)
	 */
	public static boolean doRegister = true; 

	/**
	 * for debugging purposes: if true, the program will connect to linux via ROSbridge
	 */
	public static boolean useROS = false;

	/**
	 * Main method which will be executed and which holds all the code
	 */
	public static void main(String s[]) throws FileNotFoundException, IOException, InterruptedException
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

		if(senseMgr == null)
			System.out.println("is null");
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
		/*
		pxcmStatus storageStatus = recognitionConf.CreateStorage("RoboyDB.txt", desc);
		System.out.println("Status 1: " + storageStatus);
		storageStatus = recognitionConf.UseStorage("RoboyDB.txt");
		System.out.println("Status 2: " + storageStatus);
		 */
		/*
		// load database
		File databaseFile = new File("roboy.txt");
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
		 */


		// Set the registration mode
		recognitionConf.SetRegistrationMode(PXCMFaceConfiguration.RecognitionConfiguration.RecognitionRegistrationMode.REGISTRATION_MODE_ON_DEMAND);


		// Make it effective
		statusFace = faceConf.ApplyChanges();
		if ( statusFace == pxcmStatus.PXCM_STATUS_NO_ERROR)
			System.out.println("Apply changes worked");
		else
			System.out.println("Apply changes failed with: " + statusFace.toString());

		// initialize the manager
		sts = senseMgr.Init();

		// retrieve the face tracking results
		PXCMFaceData faceData = faceModule.CreateOutput();

		// initialize the capturing of the streams
		PXCMCapture.Device device = senseMgr.QueryCaptureManager().QueryDevice();
		PXCMCapture.Device.StreamProfileSet profiles = new PXCMCapture.Device.StreamProfileSet();
		device.QueryStreamProfileSet(profiles);

		//faceConf.close();
		//faceModule.close();
		
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

		lastFaceRect = new ArrayList<PXCMRectI32>();

		if (sts == pxcmStatus.PXCM_STATUS_NO_ERROR)
		{
			int counter = 0;
			while (listener.exit == false)
			{
				counter++;
				lastFaceRect.clear();

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
								Face face = faceData.QueryFaceByIndex(i);
								PXCMFaceData.DetectionData detectData = face.QueryDetection(); 
								PXCMFaceData.RecognitionData recognitionData = face.QueryRecognition();

								// recognize the current face?
								int userId = recognitionData.QueryUserID();
								System.out.println("Id: " + userId);

								if(!recognitionData.IsRegistered())
								{	
									if (doRegister)
									{
										recognitionData.RegisterUser();
										doRegister = false;
									}
									else
									{
										System.out.println("Unrecognized");
									}
								}
								else
									System.out.println("Is registered");

								if (detectData != null)
								{
									faceRect = new PXCMRectI32();
									boolean success = detectData.QueryBoundingRect(faceRect);

									if(i == 0)	// calculate relative position to first face
										calculateRelativePosition(faceRect);

									if (success) {
										System.out.println("");
										System.out.println ("Detection Rectangle at frame #" + counter); 
										System.out.println ("Top Left corner: (" + faceRect.x + "," + faceRect.y + ")" ); 
										System.out.println ("Height: " + faceRect.h + " Width: " + faceRect.w); 

										if(useROS)
										{
											if (counter > 10)
											{
												// publish a topic to "/echo"
												publishTopic("/echo", "std_msgs/String", "{\"data\": \"" + faceRect.x + "," + faceRect.y + "\"}");
												// calls a service from ROSbridge for adding two ints
												//callService("/add_two_ints", "rospy_tutorials/AddTwoInts", "{\"a\": 10, \"b\": 20}");

												counter = 0;
											}
										}
									}
									lastFaceRect.add(faceRect);
								}
							}														
						}
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

						if(lastFaceRect != null && !lastFaceRect.isEmpty())
							drawFaces(lastFaceRect, c_df.image);

						//send image to linux
						if(useROS)
							sendImage(c_df.image);

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

		cframe.dispose();
		dframe.dispose();
	}

	/**
	 * Draw faces inside the color stream window to show the detected faces
	 * @param faces array of all detected faces
	 * @param image color image from the stream
	 */
	public static void drawFaces(ArrayList<PXCMRectI32> faces, BufferedImage image) {
		if(faces != null)
		{
			/**
			 * for each face in the arraylist draw a rectangular around the face in the image
			 */
			for (PXCMRectI32 rect: faces) {
				int color = new Color(0,255,0).getRGB(); //-1189453;
				//int[] color2 = {255, 255, 255};
				int x = rect.x;
				int y = rect.y;
				if(cHeight > y+rect.w && cWidth > x+rect.h)
				{
					for(int h = 0 ; h < rect.h; h++)
					{
						image.setRGB(x+h, y, color);
						//image.setRGB(x+h, y+rect.w, color);
						image.setRGB(x+h, y+rect.w, color);
						//System.out.println("Color: " + image.getRGB(0, 0));
					}
					for(int w = 0 ; w < rect.w; w++)
					{
						image.setRGB(x, y+w, color);
						image.setRGB(x+rect.h, y+w, color);
					}
				}
				System.out.println("X: " + x + " Y: " + y);
			}
		}
	}

	/**
	 * Send image to ROS via the ROSbridge
	 * @param bimg Image to send to ROS
	 */
	public static void sendImage(BufferedImage bimg) {
		//port for IP
		int port = 6066;
		//String host = "10.183.20.136";
		try
		{
			// socket to Linux
			Socket client = new Socket(host, port);

			// input stream from Linux to receive data
			DataInputStream in=new DataInputStream(client.getInputStream());
			System.out.println(in.readUTF());
			System.out.println(in.readUTF());

			// output stream to Linux
			DataOutputStream out = new DataOutputStream(client.getOutputStream());

			out.writeUTF("Hello from " + client.getLocalSocketAddress());
			out.writeUTF("client: hello to server");

			if(bimg == null)
				System.out.println("Image is null");
			else
			{
				// send image via the output stream
				ImageIO.write(bimg,"JPG",client.getOutputStream());
				System.out.println("Image sent!!!!");
			}
			// close the stream
			client.close();
		}catch(IOException e)
		{
			e.printStackTrace();
		}		
	}

	/**
	 * Creates an image object out of an array of bytes
	 * @param imageData byte array with image data
	 * @return bufferedimage object
	 */
	public static BufferedImage createImageFromBytes(byte[] imageData) {
		ByteArrayInputStream bais = new ByteArrayInputStream(imageData);
		try {
			return ImageIO.read(bais);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	/**
	 * Calculate the relative position of the face (in 3D) to Roboy  
	 * @param faceBox the face position in the image
	 */
	public static void calculateRelativePosition(PXCMRectI32 faceBox) {

		double width = (double)cWidth;
		double height = (double)cHeight;
		double faceWidth = (double)faceBox.w;
		double faceHeight = (double)faceBox.h;
		double faceX = (double)faceBox.x;
		double faceY = (double)faceBox.y;
		double azimuth = (hfov/2) * (faceX + faceWidth/2 - width/2)/(width/2);
		double elevation = - (vfov/2) * (faceY + faceHeight/2 - height/2)/(height/2);

		System.out.println("Azimuth: "  + azimuth);
		System.out.println("Elevation: "  + elevation);
	}

	/**
	 * Publishs the message under the specified topic to the ROSbridge
	 * @param topic ROS topic
	 * @param type ROS type
	 * @param message message to send
	 */
	public static void publishTopic(String topic, String type, String message)
	{
		// publish a topic to "topic"
		Topic echo = new Topic(ros, topic, type);
		Message toSend = new Message(message);
		echo.publish(toSend);
	}
	
	/**
	 * Subscribes to topics from ROS
	 * It checks, if the GUI sends a command to register a face
	 * @param topic topic to subscribe to
	 * @param type type of the message
	 */
	public static void subscribeTopic(String topic, String type)
	{
		// subscribe to a topic with name "topic"
		Topic echoBack = new Topic(ros, topic, type);
		echoBack.subscribe(new TopicCallback() {
			public void handleMessage(Message message) {
				System.out.println("From ROS: " + message.toString());
				if(message.toJsonObject().containsKey("Register"))
				{
					doRegister = message.toJsonObject().getBoolean("Register");						
				}
			}
		});
	}

	/**
	 * Call ROS service
	 * @param serviceName
	 * @param destination
	 * @param requestMsg
	 */
	public static void callService(String serviceName, String destination, String requestMsg)
	{
		// calls a service from ROSbridge
		Service service = new Service(ros, serviceName, destination);

		ServiceRequest request = new ServiceRequest(requestMsg);
		ServiceResponse response = service.callServiceAndWait(request);
		System.out.println(response.toString());
	}
}

/**
 * Listener for external inputs if the program should stop
 * @author Lucas Weidner
 *
 */
class Listener extends WindowAdapter {
	public boolean exit = false;
	@Override public void windowClosing(WindowEvent e) {
		exit=true;
	}
}

/**
 * Frame to draw an image
 * @author Lucas Weidner
 *
 */
class DrawFrame extends Component { 
	public BufferedImage image; 

	public DrawFrame(int width, int height) { 
		image=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);
	} 

	public void paint(Graphics g) { 
		((Graphics2D)g).drawImage(image,0,0,null); 
	}
}