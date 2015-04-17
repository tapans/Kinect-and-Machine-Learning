// Accord.NET Sample Applications
// http://accord-framework.net
//
// Copyright © 2009-2014, César Souza
// All rights reserved. 3-BSD License:
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//      * Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//
//      * Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//
//      * Neither the name of the Accord.NET Framework authors nor the
//        names of its contributors may be used to endorse or promote products
//        derived from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 

using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;  
using Accord.Statistics.Distributions.Fitting;
using Accord.Statistics.Distributions.Multivariate;
using Accord.Statistics.Models.Fields;
using Accord.Statistics.Models.Fields.Functions;
using Accord.Statistics.Models.Fields.Learning;
using Accord.Statistics.Models.Markov;
using Accord.Statistics.Models.Markov.Learning;
using Accord.Statistics.Models.Markov.Topology;
using Gestures.Native;
using Microsoft.Kinect;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;

namespace Gestures.HMMs
{
    public partial class MainForm : Form
    {

        #region Kinect related vars
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
        private List<Tuple<JointType, JointType>> bones;/// <summary>     

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        Boolean checkingExercise = false;

        /// <summary>
        /// Stream for 32b-16b conversion.
        /// </summary>
        private KinectAudioStream convertStream = null;

        /// <summary>
        /// Speech recognition engine using audio data from Kinect.
        /// </summary>
        private SpeechRecognitionEngine speechEngine = null;

        private int frameCount = 0;
        //private int initialFrameCount = 90;
        #endregion


        private List<Database> databases;
        private String[] files;

        /* for num of frames comparison */
        private Dictionary<String, double> avgFramesPerLabel_Training;
        private int framesThreshold = 20;

        /* for num of matched joints comparison */
        private int numFramesWithinThreshold = 0;
        private List<int> numMatchedPoints = new List<int>();
        public static int matchedPointsOffset = 2; //can only compare with previous, start comparing from offset

        /* for exercise counting */
        private int excCount = 0;

        /* for debugging */
        Stopwatch stopWatch = new Stopwatch();
                
        private List<HiddenMarkovClassifier<MultivariateNormalDistribution>> hmms;

        public static int numJoints = Enum.GetNames(typeof(JointType)).Length;

        public MainForm()
        {
            #region Kinect initialization
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

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
            
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.KinectSensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();
            

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }

            // grab the audio stream
            IReadOnlyList<AudioBeam> audioBeamList = this.kinectSensor.AudioSource.AudioBeams;
            System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();

            // create the convert stream
            this.convertStream = new KinectAudioStream(audioStream);

            RecognizerInfo ri = TryGetKinectRecognizer();

            if (null != ri)
            {
                this.speechEngine = new SpeechRecognitionEngine(ri.Id);

                var directions = new Choices();
                directions.Add(new SemanticResultValue("start", "START"));
                directions.Add(new SemanticResultValue("stop", "STOP"));

                var gb = new GrammarBuilder { Culture = ri.Culture };
                gb.Append(directions);
                var g = new Grammar(gb);
                this.speechEngine.LoadGrammar(g);

                this.speechEngine.SpeechRecognized += this.SpeechRecognized;
                this.speechEngine.SpeechRecognitionRejected += this.SpeechRejected;

                // let the convertStream know speech is going active
                this.convertStream.SpeechActive = true;

                // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                // This will prevent recognition accuracy from degrading over time.
                ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);


                this.speechEngine.SetInputToAudioStream(
                    this.convertStream, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                this.speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
            else
            {
                //this.statusBarText.Text = Properties.Resources.NoSpeechRecognizer;
            }

            #endregion

            InitializeComponent();

            databases = new List<Database>();
            files = new String[numJoints];
            hmms = new List<HiddenMarkovClassifier<MultivariateNormalDistribution>>();
            for (int i = 0; i < numJoints; i++)
            {
                files[i] = ((JointType)i).ToString() + ".xml";
                hmms.Add(null);
                databases.Add(new Database());
            }
            avgFramesPerLabel_Training = new Dictionary<String, double>();
           
            /*
             * Since we have muliple joints/hmms/gestures corresponding to a single exercise, no sense in visualizing sequence for only 1 joint
             * 
            gridSamples.AutoGenerateColumns = false;
            cbClasses.DataSource = databases[0].Classes;
            gridSamples.DataSource = databases[0].Samples;
             */
             
            openDataDialog.InitialDirectory = Path.Combine(Application.StartupPath, "Resources");
        }

        private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            //throw new NotImplementedException();
        }

        /// <summary>
        /// Handler for recognized speech events.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            // Number of degrees in a right angle.
            const int DegreesInRightAngle = 90;

            // Number of pixels turtle should move forwards or backwards each time.
            const int DisplacementAmount = 60;

           // this.ClearRecognitionHighlights();

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "START":
                            //Console.WriteLine("start drawing");
                            inputKinect_startDrawing();
                            checkingExercise = true;
                        break;

                    case "STOP":
                            //Console.WriteLine("stop drawing");
                            inputKinect_stopDrawing();
                            checkingExercise = false;
                        break;                   
                }
            }
        }

        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            frameCount++;
           
            //if (frameCount % 2 == 0) {
            //    return;
            //}
            //Console.WriteLine("frame arrived. frame count atm is: " + frameCount);
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

            if (dataReceived){
                foreach (Body body in this.bodies)
                {
                    if (body.IsTracked)
                    {
                        

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
                            jointPoints[jointType] = new Point((int)depthSpacePoint.X, (int)depthSpacePoint.Y);
                        }
                        //if (body.HandRightState == HandState.Lasso && captureStarted == false)
                        //{
                        //    Console.WriteLine("start drawing");
                        //    inputKinect_startDrawing();
                        //    captureStarted = true;
                        //}

                        //else if (body.HandRightState == HandState.Open && captureStarted == true)
                        //{
                        //    Console.WriteLine("stop drawing");
                        //    inputKinect_stopDrawing();
                        //    captureStarted = false;
                        //}
                        if (checkingExercise == true)
                        {
                            //frameCount++; 
                            List<Point> pts = new List<Point>();
                            for (int i = 0; i < numJoints; i++)
                            {
                                pts.Add(jointPoints[((JointType)i)]);
                            }
                            inputKinect_Draw(pts);
                            //Console.WriteLine("start checking exercise, frame count atm is: " + frameCount);
                            checkForExercise();
                            //Console.WriteLine("done checking exercise, frame count atm is: " + frameCount);
                        }

                        //if (captureStarted == true && hmms[0]!=null)
                        //{
                            
                        //    //after n frames, check if current sequence matches any exercise
                        //    if (frameCount > initialFrameCount)
                        //    {
                        //        double[][][] inputs = new double[numJoints][][];
                        //        for (int i = 0; i < numJoints; i++)
                        //        {
                        //            inputs[i] = Sequence.Preprocess(canvas.GetSequence(i));
                        //        }

                        //        String[] output_labels = new String[inputs.Length];
                        //        double[][] probabilities = new double[inputs.Length][];
                        //        for (int i = 0; i < inputs.Length; i++)
                        //        {                                    
                        //            if (hmms[i] != null) 
                        //            {
                        //                int index = hmms[i].Compute(inputs[i]);
                        //                string label = (index >= 0) ? databases[i].Classes[index] : "NOT FOUND";
                        //                output_labels[i] = label;
                        //            }
                        //        }
                        //        String guessed_label;
                        //        if (output_labels.Any(x => x != "NOT FOUND"))
                        //        {
                        //            guessed_label = output_labels.Where(x => x != "NOT FOUND").GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;
                        //            //found exercise, stop drawing
                        //            //inputKinect_stopDrawing();
                        //            canvas.onHandStop();
                        //            canvas.Clear();
                        //            lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", guessed_label);
                        //            panelClassification.Visible = true;
                        //            panelUserLabeling.Visible = false;
                        //            cbClasses.SelectedItem = guessed_label;
                        //        }
                        //        else
                        //        {
                        //            //not found yet, continue
                        //        }
                        //    }
                        //}
                        
                        
                    }
                }
            }
        }

        /// <summary>
        /// Gets the metadata for the speech recognizer (acoustic model) most suitable to
        /// process audio from Kinect device.
        /// </summary>
        /// <returns>
        /// RecognizerInfo if found, <code>null</code> otherwise.
        /// </returns>
        private static RecognizerInfo TryGetKinectRecognizer()
        {
            IEnumerable<RecognizerInfo> recognizers;

            // This is required to catch the case when an expected recognizer is not installed.
            // By default - the x86 Speech Runtime is always expected. 
            try
            {
                recognizers = SpeechRecognitionEngine.InstalledRecognizers();
            }
            catch (COMException)
            {
                return null;
            }

            foreach (RecognizerInfo recognizer in recognizers)
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }

        private void KinectSensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        
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

        private void btnLearnHMM_Click(object sender, EventArgs e)
        {
            this.kinectSensor.Close();
            for (int i = 0; i < databases.Count; i++)
            {
                hmms[i] = this.learnHMM(databases[i]);
                Console.WriteLine("done learning hmm for joint: " + i + " : " + Enum.GetName(typeof(JointType), i));
            }

            //Get an estimate of whether the samples match the model or not 
            //naive algorithm: if most of the gestures/hmms/joints in a sample were successfully recognized (not rejected), then that sample/sequence is highlighted as green. if rejected, then highlighted as white
            //foreach (DataGridViewRow row in gridSamples.Rows)
            //{
            //    var sample = row.DataBoundItem as Sequence;
                               
            //    row.DefaultCellStyle.BackColor = (sample.RecognizedAs == sample.Output) ?
            //        Color.LightGreen : Color.White;
            //}

            this.kinectSensor.Open();
        }

        private HiddenMarkovClassifier<MultivariateNormalDistribution> learnHMM(Database database)
        {
            //TODO: add a check to ensure that data has been loaded
            //if (gridSamples.Rows.Count == 0)
            //{
            //    MessageBox.Show("Please load or insert some data first.");
            //    return null;
            //}

            BindingList<Sequence> samples = database.Samples;
            BindingList<String> classes = database.Classes;

            double[][][] inputs = new double[samples.Count][][];
            int[] outputs = new int[samples.Count];

            for (int i = 0; i < inputs.Length; i++)
            {
                inputs[i] = samples[i].Input;
                outputs[i] = samples[i].Output;
            }

            int states = 5;
            int iterations = 0;
            double tolerance = 0.1;
            bool rejection = true;


            HiddenMarkovClassifier<MultivariateNormalDistribution> hmm = new HiddenMarkovClassifier<MultivariateNormalDistribution>(classes.Count,
                new Forward(states), new MultivariateNormalDistribution(2), classes.ToArray());

            //hmm.Sensitivity = 1;

            // Create the learning algorithm for the ensemble classifier
            var teacher = new HiddenMarkovClassifierLearning<MultivariateNormalDistribution>(hmm,

                // Train each model using the selected convergence criteria
                i => new BaumWelchLearning<MultivariateNormalDistribution>(hmm.Models[i])
                {
                    Tolerance = tolerance,
                    Iterations = iterations,

                    FittingOptions = new NormalOptions()
                    {
                        Regularization = 1e-5
                    }
                }
            );

            teacher.Empirical = true;
            teacher.Rejection = rejection;


            // Run the learning algorithm
            double error = teacher.Run(inputs, outputs);

            // Classify all training instances
            foreach (var sample in database.Samples)
            {
                sample.RecognizedAs = hmm.Compute(sample.Input);
            }

            /*
             * ISSUE: only checks the last joint/hmm/gesture to determine if sample was recognized or not => incorrect measure of correspondence of sample with model!
             * potential soln: recognize if most joints/gestures/hmms fit the model / are not rejected?
             
            foreach (DataGridViewRow row in gridSamples.Rows)
            {
                var sample = row.DataBoundItem as Sequence;
                row.DefaultCellStyle.BackColor = (sample.RecognizedAs == sample.Output) ?
                    Color.LightGreen : Color.White;
            }
             
             */

            btnLearnHCRF.Enabled = true;
            return hmm;
        }

        // Load and save database methods
        private void openDataStripMenuItem_Click(object sender, EventArgs e)
        {
            openDataDialog.ShowDialog();
        }

        private void saveDataStripMenuItem_Click(object sender, EventArgs e)
        {
            saveDataDialog.ShowDialog();
        }

        private void openDataDialog_FileOk(object sender, System.ComponentModel.CancelEventArgs e)
        {
            for (int i = 0; i < numJoints; i++)
            {
                hmms[i] = null;
            }

            FolderBrowserDialog fbd = new FolderBrowserDialog();
            DialogResult result = fbd.ShowDialog();            

            for (int i = 0; i < files.Length; i++)
            {
                String path = Path.Combine(fbd.SelectedPath, Path.GetFileName(files[i]));
                databases[i].Load(new FileStream(path, FileMode.Open));      
            }

            //get avg frames for each label using one of the joint files
            avgFramesPerLabel_Training = databases[0].avgFramesPerLabel();

            btnLearnHMM.Enabled = true;
            btnLearnHCRF.Enabled = false;

            panelClassification.Visible = false;
            panelUserLabeling.Visible = false;
        }

        private void saveDataDialog_FileOk(object sender, CancelEventArgs e)
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            DialogResult result = fbd.ShowDialog();
            String dirPath = fbd.SelectedPath;
            String path;
            for (int i = 0; i < databases.Count; i++)
            {
                path = Path.Combine(dirPath, Path.GetFileName(files[i]));
                using (var stream = File.OpenWrite(path))
                    databases[i].Save(stream);
            }                
        }

        private void btnFile_MouseDown(object sender, MouseEventArgs e)
        {
            menuFile.Show(button4, button4.PointToClient(Cursor.Position));
        }



        // Top user interaction panel box events
        private void btnYes_Click(object sender, EventArgs e)
        {
            addExercise();
        }

        private void btnNo_Click(object sender, EventArgs e)
        {
            panelClassification.Visible = false;
            panelUserLabeling.Visible = true;
        }


        // Bottom user interaction panel box events
        private void btnClear_Click(object sender, EventArgs e)
        {
            canvas.Clear();
            panelUserLabeling.Visible = false;
        }

        private void btnInsert_Click(object sender, EventArgs e)
        {
            addExercise();
        }

        private void addExercise()
        {
            string selectedItem = cbClasses.SelectedItem as String;
            string classLabel = String.IsNullOrEmpty(selectedItem) ?
                cbClasses.Text : selectedItem;

            for (int i = 0; i < databases.Count; i++)
            {
                databases[i].Add(canvas.GetSequence(i), classLabel);
                //if (databases[i].Add(canvas.GetSequence(i), classLabel) != null)
                //{
                    

                //    if (databases[i].Classes.Count >= 1 &&
                //        databases[i].SamplesPerClass() >= 1)
                //        btnLearnHMM.Enabled = true;

                //    panelUserLabeling.Visible = false;
                //}
            }
            avgFramesPerLabel_Training = databases[0].avgFramesPerLabel();
            canvas.Clear();
        }

        //Kinect events
        private void inputKinect_stopDrawing()
        {
            canvas.onStop();
            
            double[][][] inputs = new double[numJoints][][];
            for (int i = 0; i < numJoints; i++)
            {
                inputs[i] = Sequence.Preprocess(canvas.GetSequence(i));
            }

            String[] output_labels = new String[inputs.Length];
            //double[][] probabilities = new double[inputs.Length][];
            for (int i = 0; i < inputs.Length; i++)
            {
                if (inputs[i].Length < 5)
                {
                    panelUserLabeling.Visible = false;
                    panelClassification.Visible = false;
                    return;
                }

                if (hmms[i] == null)
                {
                    panelUserLabeling.Visible = true;
                    panelClassification.Visible = false;
                }

                else
                {
                    int index = hmms[i].Compute(inputs[i]);
                    string label = (index >= 0) ? databases[i].Classes[index] : "NOT FOUND";
                    output_labels[i] = label;
                }
            }
            String guessed_label;
            if(output_labels.Any(x => x!="NOT FOUND")){
                guessed_label = output_labels.Where(x => x != "NOT FOUND").GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;
                Console.WriteLine("Most joints: "+guessed_label+" : " + output_labels.Where(x => x == guessed_label).Count());
            }
            else
            {
                guessed_label = "NOT FOUND";
                //guessed_label = output_labels.GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;                
            }

            //compare number of frames in sequence with avg # of frames in training cases for the predicted label
            //if (avgFramesPerLabel_Training.Count() > 0 && guessed_label != null)
            //{
            //    if (guessed_label != "NOT FOUND")
            //    {
            //        double framesDifference = Math.Abs(avgFramesPerLabel_Training[guessed_label] - canvas.GetSequence(0).Count());
            //        if (framesDifference > framesThreshold)
            //        {
            //            guessed_label = "REJECTED";
            //            Console.WriteLine("Frames difference: " + framesDifference);
            //        }
            //    }
            //}

            lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", guessed_label);
            panelClassification.Visible = true;
            panelUserLabeling.Visible = false;
            cbClasses.SelectedItem = guessed_label;


           //     else
           //     {
           //         probabilities[i] = hmms[i].Compute2(inputs[i]);
           //     }
           // }
           // //String guessed_label = output_labels.GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;
           //// List<double> probsum = new List<double>();

           // String guessed_label = "";
           // if (probabilities != null && probabilities[0] != null)
           // {
           //     var probsum = new Dictionary<double, string>();
           //     for (int i = 0; i < databases[0].Classes.Count; i++)
           //     {
           //         double tmp = 0;
           //         for (int j = 0; j < numJoints; j++)
           //         {
           //             tmp += probabilities[j][i];
           //         }
           //         probsum[tmp] = databases[0].Classes[i];
           //         Console.WriteLine("Prob for " + i + ": " + tmp);
           //     }
           //     if (probsum.Count > 0)
           //     {
           //         guessed_label = probsum[probsum.Keys.Max()];
           //     }
           // }
            
            
           // lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", guessed_label);
           // panelClassification.Visible = true;
           // panelUserLabeling.Visible = false;
           // //cbClasses.SelectedItem = guessed_label;
               
        }

        private void checkForExercise()
        {
            //canvas.onHandStop();
            if (excCount > 0)
            {
                Console.WriteLine("Execount GT 0");
            }
            double[][][] inputs = new double[numJoints][][];
            for (int i = 0; i < numJoints; i++)
            {
                inputs[i] = Sequence.Preprocess(canvas.GetSequence(i));
            }

            String[] output_labels = new String[inputs.Length];
            double[][] probabilities = new double[inputs.Length][];
            for (int i = 0; i < inputs.Length; i++)
            {
                if (inputs[i].Length < 5)
                {
                    //panelUserLabeling.Visible = false;
                    //panelClassification.Visible = false;
                    return;
                }

                if (hmms[i] == null)
                {
                    panelUserLabeling.Visible = true;
                    panelClassification.Visible = false;
                }

                else
                {
                    if (excCount > 0 && inputs[i].Length > 70)
                    {
                        //Console.WriteLine("debug the compute function? i guess?");
                        int fcxd;
                    }
                    int index = hmms[i].Compute(inputs[i]);
                    string label = (index >= 0) ? databases[i].Classes[index] : "NOT FOUND";
                    output_labels[i] = label;
                }
            }
            String guessed_label;
            if (output_labels.Any(x => x != "NOT FOUND"))
            {
                // return the most occuring exercise label by considering labels for all the joints
                guessed_label = output_labels.Where(x => x != "NOT FOUND").GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;
                int numOfGuessed = output_labels.Where(x => x == guessed_label).Count();
                Console.WriteLine("Most joints: " + guessed_label + " : " + numOfGuessed);
                if (numOfGuessed < 5)
                {
                    guessed_label = "NOT FOUND";
                }
            }
            else
            {
                guessed_label = "NOT FOUND";
                //guessed_label = output_labels.GroupBy(x => x).OrderByDescending(x => x.Count()).First().Key;                
            }
            Console.WriteLine("Finished computing! Guessed: "+ guessed_label + " num of seqs in canvas atm: " + canvas.GetSequence(0).Count());

            //compare number of frames in sequence with avg # of frames in training cases for the predicted label
            if (avgFramesPerLabel_Training.Count() > 0 && guessed_label != null) //only do below process for testing data, not training
            {
                if (guessed_label != "NOT FOUND")
                {
                    /* If within frame threshold:
                     *      keep track of num of matched joints until all previous x frames consecutively decrease
                     *      |_when this is the case:
                     *          stop exercise but remove the last x frames from the sequence and add them to the seq for next exc  
                     */
                    double frameCount = canvas.GetSequence(0).Count();
                    double framesDifference = avgFramesPerLabel_Training[guessed_label] - frameCount;
                    Console.WriteLine("Frame diff: " + framesDifference + ", avgFrames: " + avgFramesPerLabel_Training[guessed_label] + ", frames threshold: " + framesThreshold);
                    if ((framesDifference >= 0 && framesDifference <= framesThreshold) || (framesDifference < 0 && Math.Abs(framesDifference) <= framesThreshold + matchedPointsOffset))
                    {
                        Console.WriteLine("Frame diff accepted!");
                        numFramesWithinThreshold++;
                        numMatchedPoints.Add(output_labels.Where(x => x == guessed_label).Count());
                        if (numFramesWithinThreshold >= matchedPointsOffset)
                        {
                            if (numMatchedJointsDecreasing(numMatchedPoints, matchedPointsOffset))
                            {
                                //fix current exercise sequence by remove last offset # of points from seq, stop exc and append to new seq and start new exc
                                //List<List<Point>> lastOffsetSequences = canvas.returnLastOffsetSequences(matchedPointsOffset);
                                //checkingExercise = false;
                                //canvas.onStop();
                                excCount++;
                                stopWatch.Stop();
                                // Get the elapsed time as a TimeSpan value.
                                TimeSpan ts = stopWatch.Elapsed;

                                // Format and display the TimeSpan value. 
                                string elapsedTime = ts.TotalSeconds.ToString();
                                Console.WriteLine("RunTime " + elapsedTime + ", frame count: " + frameCount + ", expected time in seconds based on 30fps: " + frameCount / 30);

                                stopWatch.Reset();
                                stopWatch.Start();
                                Console.WriteLine("num exercises done so far: " + excCount);
                                //lbHaveYouDrawn.Text = String.Format("Number of exercises done so far: {0}?", excCount);
                                //panelClassification.Visible = true;
                                canvas.Clear();
                                //canvas.onStart();
                                //canvas.setSequences(lastOffsetSequences);
                                Console.WriteLine("Set the seq");
                                //checkingExercise = true;
                                //numFramesWithinThreshold = 0;
                            }
                        }

                    }
                }
            }

          //  lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", guessed_label);
            //panelClassification.Visible = true;
            //panelUserLabeling.Visible = false;
            //cbClasses.SelectedItem = guessed_label;


        }
        
        /*
         * Returns true iff lastN num of elements in numMatchedPoints list are sstrictly less than each other
         */
        private bool numMatchedJointsDecreasing(List<int> numMatchedPoints, int lastN)
        {
            int numElems = numMatchedPoints.Count();
            int max = numMatchedPoints[numElems - lastN];
            for (int i = numElems - lastN + 1; i < numElems; i++ ){
                if (numMatchedPoints[i] >= max)
                {
                    return false;
                }
                max = numMatchedPoints[i];
            }
            return true;
        }
        
        private void inputKinect_startDrawing()
        {
            canvas.onStart();

            lbIdle.Visible = false;

            excCount = 0;
            stopWatch.Restart();
        }

        private void inputKinect_Draw(List<Point> pts)
        {
            canvas.onDraw(pts);
        }

        // Aero Glass settings
        //
        private void MainForm_Load(object sender, EventArgs e)
        {
            // Perform special processing to enable aero
            if (SafeNativeMethods.IsAeroEnabled)
            {
                ThemeMargins margins = new ThemeMargins();
                margins.TopHeight = canvas.Top;
                margins.LeftWidth = canvas.Left;
                margins.RightWidth = ClientRectangle.Right - gridSamples.Right;
                margins.BottomHeight = ClientRectangle.Bottom - canvas.Bottom;

                // Extend the Frame into client area
                SafeNativeMethods.ExtendAeroGlassIntoClientArea(this, margins);
            }
        }

        /// <summary>
        ///   Paints the background of the control.
        /// </summary>
        protected override void OnPaintBackground(PaintEventArgs e)
        {
            base.OnPaintBackground(e);

            if (SafeNativeMethods.IsAeroEnabled)
            {
                // paint background black to enable include glass regions
                e.Graphics.Clear(Color.FromArgb(0, this.BackColor));
            }
        }
    }
}
