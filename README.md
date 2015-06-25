See "WakeUpWithKinect" branch for the final project code.

Requirements to run the application:
Kinect ready machine
Visual Studio
Kinect SDK
Microsoft Speech SDK (both execution and runtime dlls required)
If solution does not build, might need to manually download other assemblies / dependencies like Fleck, newtonsoft.JSON, log4net (most of these can also be installed using Package Manager in Visual Studio)

Code Structure:
There are two main projects within the Wake UP with Kinect solution:
Kinect.Server: Runs a websocket server to serve the Kinect sensor data
Kinect.Client: Connects to the Kinect server with websockets, send alarm option information back to ther server, displays the users' joints by drawing them on canvas. Essentially the front-end that you see in the video.