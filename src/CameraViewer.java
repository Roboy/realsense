import intel.rssdk.*;
import intel.rssdk.PXCMFaceData.Face;

import javax.swing.*;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;

import java.awt.event.*;
import java.awt.image.*;
import java.awt.*;

public class CameraViewer
{    
	/*
	 * change "host" to the IP from your linux machine
	 * Enter: "ifconfig" in a terminal in linux and search for "eth0". Use the "inet addr"!!
	 */
	static String host = "10.183.20.136";
	static int cWidth  = 640;
	static int cHeight = 480;
	static int dWidth, dHeight;
	static boolean exit = false;
	static Ros ros;

	public static void main(String s[])
	{
		// output where the "libpxcclr.jni64.dll" has to be copied to (at least in one of these paths)
		//System.out.println("Path: " + System.getProperty("java.library.path"));

		// load realsense library
		System.loadLibrary("libpxcclr.jni64");

		// connect to ROSbridge
		System.out.println("Try to connect to ROSbridge");
		ros  = new Ros(host);
		ros.connect();
		System.out.println("Connection established.");

		// subscribe to a topic with name "/listener"
		subscribeTopic("/listener", "std_msgs/String");

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
		/*
        // Retrieve the input requirements
        PXCMFaceConfiguration faceConfig = faceModule.CreateActiveConfiguration();
        faceConfig.SetTrackingMode(PXCMFaceConfiguration.TrackingModeType.FACE_MODE_COLOR);
        faceConfig.detection.isEnabled = true; 
        faceConfig.landmarks.isEnabled = true; 
        faceConfig.pose.isEnabled = true;
        //faceConfig.ApplyChanges();
        faceConfig.detection.isEnabled = true; 
        faceConfig.Update(); */

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
						System.out.println("Faces: " + faceData.QueryNumberOfDetectedFaces());
						if(faceData.QueryNumberOfDetectedFaces() > 0)
						{
							Face face = faceData.QueryFaceByIndex(0);
							PXCMFaceData.DetectionData detectData = face.QueryDetection(); 
				              
			                if (detectData != null)
			                {
			                    PXCMRectI32 rect = new PXCMRectI32();
			                    boolean ret = detectData.QueryBoundingRect(rect);
			                    if (ret) {
			                        System.out.println("");
			                        System.out.println ("Detection Rectangle at frame #" + counter); 
			                        System.out.println ("Top Left corner: (" + rect.x + "," + rect.y + ")" ); 
			                        System.out.println ("Height: " + rect.h + " Width: " + rect.w); 
			                        
			                        if (counter > 10)
			        				{
			        					// publish a topic to "/echo"
			        					publishTopic("/echo", "std_msgs/String", "{\"data\": \"" + rect.x + "," + rect.y + "\"}");

			        					// calls a service from ROSbridge for adding two ints
			        					//callService("/add_two_ints", "rospy_tutorials/AddTwoInts", "{\"a\": 10, \"b\": 20}");
			        					
			        					counter = 0;
			        				}
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
							System.out.println ("Failed to AcquireAccess of color image data");
							System.exit(3);
						}

						int cBuff[] = new int[cData.pitches[0]/4 * cHeight];

						cData.ToIntArray(0, cBuff);
						c_df.image.setRGB (0, 0, cWidth, cHeight, cBuff, 0, cData.pitches[0]/4);
						c_df.repaint();  
						sts = sample.color.ReleaseAccess(cData);

						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0)
						{
							System.out.println ("Failed to ReleaseAccess of color image data");
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
							System.out.println ("Failed to AcquireAccess of depth image data");
							System.exit(3);
						}

						int dBuff[] = new int[dData.pitches[0]/4 * dHeight];
						dData.ToIntArray(0, dBuff);
						d_df.image.setRGB (0, 0, dWidth, dHeight, dBuff, 0, dData.pitches[0]/4);
						d_df.repaint();
						sts = sample.depth.ReleaseAccess(dData);
						if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0)
						{
							System.out.println ("Failed to ReleaseAccess of depth image data");
							System.exit(3);
						}
					}  
				}
				else
				{
					System.out.println("Failed to acquire frame");
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

		// disconnets from ROS
		ros.disconnect();
		cframe.dispose();
		dframe.dispose();
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