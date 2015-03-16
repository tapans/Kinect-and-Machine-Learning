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
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using System.Collections.Generic;
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

        Boolean captureStarted = false;
        #endregion

        
        private Database database;
        private HiddenMarkovClassifier<MultivariateNormalDistribution> hmm;
        private HiddenConditionalRandomField<double[]> hcrf;


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

            #endregion

            InitializeComponent();

            database = new Database();
            gridSamples.AutoGenerateColumns = false;
            cbClasses.DataSource = database.Classes;
            gridSamples.DataSource = database.Samples;
            openDataDialog.InitialDirectory = Path.Combine(Application.StartupPath, "Resources");
        }

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

                        if (body.HandRightState == HandState.Lasso && captureStarted == false)
                        {
                            Console.WriteLine("start drawing");
                            inputKinect_startDrawing();
                            captureStarted = true;
                        }

                        else if (body.HandRightState == HandState.Open && captureStarted == true)
                        {
                            Console.WriteLine("stop drawing");
                            inputKinect_stopDrawing();
                            captureStarted = false;
                        }

                        else if (captureStarted == true && body.HandRightState != HandState.Open)
                        {
                            Point p = jointPoints[JointType.HandRight];
                            Console.WriteLine("drawing to position: " + p.X + "," + p.Y);
                            inputKinect_Draw(p);
                        }
                    }
                }
            }
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
            if (gridSamples.Rows.Count == 0)
            {
                MessageBox.Show("Please load or insert some data first.");
                return;
            }

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
            double tolerance = 0.01;
            bool rejection = false;


            hmm = new HiddenMarkovClassifier<MultivariateNormalDistribution>(classes.Count,
                new Forward(states), new MultivariateNormalDistribution(2), classes.ToArray());


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

            foreach (DataGridViewRow row in gridSamples.Rows)
            {
                var sample = row.DataBoundItem as Sequence;
                row.DefaultCellStyle.BackColor = (sample.RecognizedAs == sample.Output) ?
                    Color.LightGreen : Color.White;
            }

            btnLearnHCRF.Enabled = true;
        }

        private void btnLearnHCRF_Click(object sender, EventArgs e)
        {
            if (gridSamples.Rows.Count == 0)
            {
                MessageBox.Show("Please load or insert some data first.");
                return;
            }

            var samples = database.Samples;
            var classes = database.Classes;

            double[][][] inputs = new double[samples.Count][][];
            int[] outputs = new int[samples.Count];

            for (int i = 0; i < inputs.Length; i++)
            {
                inputs[i] = samples[i].Input;
                outputs[i] = samples[i].Output;
            }

            int iterations = 100;
            double tolerance = 0.01;


            hcrf = new HiddenConditionalRandomField<double[]>(
                new MarkovMultivariateFunction(hmm));


            // Create the learning algorithm for the ensemble classifier
            var teacher = new HiddenResilientGradientLearning<double[]>(hcrf)
            {
                Iterations = iterations,
                Tolerance = tolerance
            };


            // Run the learning algorithm
            double error = teacher.Run(inputs, outputs);


            foreach (var sample in database.Samples)
            {
                sample.RecognizedAs = hcrf.Compute(sample.Input);
            }

            foreach (DataGridViewRow row in gridSamples.Rows)
            {
                var sample = row.DataBoundItem as Sequence;
                row.DefaultCellStyle.BackColor = (sample.RecognizedAs == sample.Output) ?
                    Color.LightGreen : Color.White;
            }
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
            hmm = null;
            hcrf = null;

            using (var stream = openDataDialog.OpenFile())
                database.Load(stream);

            btnLearnHMM.Enabled = true;
            btnLearnHCRF.Enabled = false;

            panelClassification.Visible = false;
            panelUserLabeling.Visible = false;
        }

        private void saveDataDialog_FileOk(object sender, CancelEventArgs e)
        {
            using (var stream = saveDataDialog.OpenFile())
                database.Save(stream);
        }

        private void btnFile_MouseDown(object sender, MouseEventArgs e)
        {
            menuFile.Show(button4, button4.PointToClient(Cursor.Position));
        }



        // Top user interaction panel box events
        private void btnYes_Click(object sender, EventArgs e)
        {
            addGesture();
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
            addGesture();
        }

        private void addGesture()
        {
            string selectedItem = cbClasses.SelectedItem as String;
            string classLabel = String.IsNullOrEmpty(selectedItem) ?
                cbClasses.Text : selectedItem;

            if (database.Add(canvas.GetSequence(), classLabel) != null)
            {
                canvas.Clear();

                if (database.Classes.Count >= 2 &&
                    database.SamplesPerClass() >= 3)
                    btnLearnHMM.Enabled = true;

                panelUserLabeling.Visible = false;
            }
        }


        // Canvas events
        private void inputCanvas_MouseUp(object sender, MouseEventArgs e)
        {
            double[][] input = Sequence.Preprocess(canvas.GetSequence());

            if (input.Length < 5)
            {
                panelUserLabeling.Visible = false;
                panelClassification.Visible = false;
                return;
            }

            if (hmm == null && hcrf == null)
            {
                panelUserLabeling.Visible = true;
                panelClassification.Visible = false;
            }

            else
            {
                int index = (hcrf != null) ?
                    hcrf.Compute(input) : hmm.Compute(input);

                string label = database.Classes[index];
                lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", label);
                panelClassification.Visible = true;
                panelUserLabeling.Visible = false;
                cbClasses.SelectedItem = label;
            }
        }

        private void canvas_MouseDown(object sender, MouseEventArgs e)
        {
            lbIdle.Visible = false;
        }

        //Kinect events
        private void inputKinect_stopDrawing()
        {
            canvas.onHandStop();

            double[][] input = Sequence.Preprocess(canvas.GetSequence());

            if (input.Length < 5)
            {
                panelUserLabeling.Visible = false;
                panelClassification.Visible = false;
                return;
            }

            if (hmm == null && hcrf == null)
            {
                panelUserLabeling.Visible = true;
                panelClassification.Visible = false;
            }

            else
            {
                int index = (hcrf != null) ?
                    hcrf.Compute(input) : hmm.Compute(input);

                string label = database.Classes[index];
                lbHaveYouDrawn.Text = String.Format("Have you drawn a {0}?", label);
                panelClassification.Visible = true;
                panelUserLabeling.Visible = false;
                cbClasses.SelectedItem = label;
            }
        }

        private void inputKinect_startDrawing()
        {
            canvas.onHandStart();

            lbIdle.Visible = false;
        }

        private void inputKinect_Draw(Point p)
        {
            canvas.onHandDraw(p);
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
