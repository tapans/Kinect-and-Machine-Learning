//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Forms;
    using System.Media;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        private int debugCounter = 0;
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        // for clock timer
        System.Windows.Threading.DispatcherTimer Timer = new System.Windows.Threading.DispatcherTimer();

        // for alarm sounds
        SoundPlayer player = new SoundPlayer(@"C:\Users\tapan\Documents\BodyBasics-WPF\sounds\clockBuzzer.wav");
        private bool _ringing = false;

        // Timer config
        DateTime dt;
        System.Threading.Timer timer;

        // Hardcoded Jumping Jack Counter
        int exerciseState = 0;
        double threshold = 10;
        int exerciseCounter = 0;

        /* Training vars */
        // Count # of frames arrived 
        int frameCounter = -1; //uninitialized. -2 state means done.
        int num_frames = 60; // # of frames to user per training data
        int num_training_data = 1; // # of training data to record
        double[,] x_pos;
        double[,] y_pos;
        double[,] z_pos;
        StreamWriter streamWriter;  // Output Stream Writer variable

        public bool ringing
        {
            set { this._ringing = value; }
            get { return this._ringing; }            
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            //initialize clock
            this.initClock();

            // Initialize datetimepicker
            this.initDateTimePicker();

            //initialize arrays for training stuff
            x_pos = new double[num_frames, num_training_data];
            y_pos = new double[num_frames, num_training_data];
            z_pos = new double[num_frames, num_training_data];

        }

        private void initClock()
        {
            Timer.Tick += new EventHandler(Timer_Click);

            Timer.Interval = new TimeSpan(0, 0, 1);

            Timer.Start();
        }

        private void Timer_Click(object sender, EventArgs e)
        {
            DateTime d;

            d = DateTime.Now;

            clock.Content = d.Hour + " : " + d.Minute + " : " + d.Second;
        }

        private void initDateTimePicker()
        {
            dtpicker.Value = new DateTime(2015, 2, 22);
            dtpicker.ValueChanged += dtpicker_ValueChanged;
        }

        private void dtpicker_ValueChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
           this.dt = (System.DateTime) e.NewValue;
           setAlarm(dt - DateTime.Now);
        }

        private void setAlarm(TimeSpan ts)
        {
            if (this.timer != null) this.timer.Dispose();
            if (ts.TotalSeconds > 0) timer = new System.Threading.Timer(new System.Threading.TimerCallback(ring_alarm), null, ts, TimeSpan.FromHours(24));
        }

        private void ring_alarm(object state)
        {
            Console.Write("ring ring");            
            player.Play();
            this.ringing = true;
        }

        
        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {

                            // draws red rectangles on the boundary for clipping
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }


                           /* if (badCounter <= 60)
                            {
                                //Console.WriteLine(joints[JointType.HandRight].Position.X + "," + joints[JointType.HandRight].Position.Y + "," + joints[JointType.HandRight].Position.Z);
                                Console.WriteLine(jointPoints[JointType.HandRight].X + " " + jointPoints[JointType.HandRight].Y + " " + joints[JointType.HandRight].Position.Z);
                            }*/

                            /**********************************************Training Stuff********************************************************************************/
                            if (body.HandRightState == HandState.Open)
                            {
                                if (frameCounter == -1)
                                {
                                    Console.WriteLine("setting frame counter");
                                    frameCounter = 0;
                                }                    
                            }
                            if (frameCounter >= 0)
                            {
                                if (frameCounter < num_frames * num_training_data)
                                {
                                    int rowIndex = frameCounter % num_frames;
                                    int columnIndex = frameCounter / num_frames;
                                    x_pos[rowIndex, columnIndex] = jointPoints[JointType.HandRight].X;
                                    y_pos[rowIndex, columnIndex] = jointPoints[JointType.HandRight].Y;
                                    z_pos[rowIndex, columnIndex] = joints[JointType.HandRight].Position.Z;
                                    frameCounter++;
                                }
                                else
                                {
                                    //done training. show me the arrays
                                    print2DArray(x_pos);
                                    print2DArray(y_pos);
                                    print2DArray(z_pos);
                                    /*                                   
                                    print2DArray(x_pos, true, "circle_x.csv");
                                    print2DArray(y_pos, true, "circle_y.csv");
                                    print2DArray(z_pos, true, "circle_z.csv");
                                    frameCounter = -2; // done
                                     */
                                }
                            }
                            /**********************************************Training Stuff********************************************************************************/
                            

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            checkExercise(jointPoints);
                            //if (this.ringing) this.stopAlarm(body.HandLeftState, body.HandRightState);

                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        private void print2DArray(double[,] pos, bool writeToFile = false, String fileName="")
        {
            streamWriter = fileName != "" ? new StreamWriter("C:\\Users\\tapan\\Downloads\\gesture\\gesture\\data\\train\\" + fileName) : null;             
            for (int i = 0; i < pos.GetLength(0); i++)
            {
                String line = "";
                for (int j = 0; j < pos.GetLength(1); j++)
                {
                    if (j < pos.GetLength(1) - 1)
                    {
                        line += pos[i, j].ToString() + ",";
                    }
                    else
                    {
                        line += pos[i, j].ToString();
                    }
                }
                Console.WriteLine(line);
                if (writeToFile == true)
                {                    
                    streamWriter.WriteLine(line);
                }
            }
            if (streamWriter != null) streamWriter.Close();
        }

// **********************************************************************************************************

        
        public void checkExercise(Dictionary<JointType, Point> jointPoints)
        {
            Point leftElbowJoint = jointPoints[JointType.ElbowLeft];
            Point leftShoulderJoint = jointPoints[JointType.ShoulderLeft];
            Point spineShoulderJoint = jointPoints[JointType.SpineShoulder];
            Point rightShoulderJoint = jointPoints[JointType.ShoulderRight];
            Point rightElbowJoint = jointPoints[JointType.ElbowRight];

            switch (exerciseState)
            {
                case 0:
                    if (this.ringing && exerciseCounter == 5 )
                    {
                        stopAlarm();
                    }
                    if (this.isValidAngle(getAngleB(leftElbowJoint, leftShoulderJoint, spineShoulderJoint), 180) && this.isValidAngle(getAngleB(spineShoulderJoint, rightShoulderJoint, rightElbowJoint), 180))
                    {
                        exerciseState = 1;
                        Console.WriteLine("STATE 1");
                    }
                    break;
                case 1:
                    if (this.isValidAngle(getAngleB(leftElbowJoint, leftShoulderJoint, spineShoulderJoint), 115) && this.isValidAngle(getAngleB(spineShoulderJoint, rightShoulderJoint, rightElbowJoint), 115) && leftElbowJoint.Y < spineShoulderJoint.Y && rightElbowJoint.Y < spineShoulderJoint.Y)
                    {
                        exerciseState = 2;
                        Console.WriteLine("STATE 2");
                    }
                    break;
                case 2:
                    if (this.isValidAngle(getAngleB(leftElbowJoint, leftShoulderJoint, spineShoulderJoint), 115) && this.isValidAngle(getAngleB(spineShoulderJoint, rightShoulderJoint, rightElbowJoint), 115) && leftElbowJoint.Y > spineShoulderJoint.Y && rightElbowJoint.Y > spineShoulderJoint.Y)
                    {
                        exerciseState = 0;
                        exerciseCounter++;
                        Console.WriteLine("STATE 0");
                    }
                    break;
            }
        }

        public bool isValidAngle(double angle, int expected)
        {
            if (Math.Abs(angle - expected) < this.threshold)
            {
                return true;
            }
            return false;
        }

        public double getAngleB(Point A, Point B, Point C)
        {
            double a = distance(B, C);
            double b = distance(A, C);
            double c = distance(B, A);
            return (Math.Acos((Math.Pow(a, 2) + Math.Pow(c, 2) - Math.Pow(b, 2))/(2*a*c)))/Math.PI * 180;
        }

        public double distance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }
// **********************************************************************************************************
        
        private void stopAlarm(HandState leftHandState, HandState rightHandState)
        {
            if (leftHandState == HandState.Lasso || rightHandState == HandState.Lasso)
            {
                player.Stop();
                setAlarm(new TimeSpan(0, 0, 0, 5, 0));
            }
            else if (leftHandState == HandState.Closed || rightHandState == HandState.Closed)
            {
                player.Stop();
                setAlarm(new TimeSpan(0, 0, 0, 10, 0));
            }
        }

        private void stopAlarm()
        {
            player.Stop();
            this.ringing = false;
        }
        
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {

            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);                    
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

       

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

    }
}
