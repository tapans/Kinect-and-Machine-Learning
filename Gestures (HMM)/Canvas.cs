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

namespace Gestures.HMMs
{
    using System;
    using System.Collections.Generic;
    using System.Drawing;
    using System.Linq;
    using System.Windows.Forms;
    using System.Drawing.Drawing2D;

    public partial class Canvas : UserControl
    {
        private bool capturing;
        private List<List<Point>> sequences;

        public Canvas()
        {
            InitializeComponent();

            sequences = new List<List<Point>>();
            for (int i = 0; i < MainForm.numJoints; i++)
            {
                sequences.Add(new List<Point>());
            }
            this.DoubleBuffered = true;
        }

        public void setSequences(List<List<Point>> newExcSequences)
        {
            this.sequences = newExcSequences;
        }

        /*
         * returns last offset amount from current excercise for starting sequences for new exercise
        */
        public List<List<Point>> returnLastOffsetSequences(int n){

            //for each elem (list of points for corresponding joint) in this.seq, get last n points
            List<List<Point>> newExerciseSequences = new List<List<Point>>();
            int numPointsForCurrJoint = 0;
            for (int i = 0; i < MainForm.numJoints; i++)
            {
                numPointsForCurrJoint = this.sequences[i].Count();
                newExerciseSequences.Add(this.sequences[i].GetRange((numPointsForCurrJoint - n), n));  
                //this.sequences[i].RemoveRange((numPointsForCurrJoint - n), n);
            }
            return newExerciseSequences;
        }

        public Point[] GetSequence(int n)
        {
            return sequences[n].ToArray();
        }


        public void Clear()
        {
            for (int i = 0; i < sequences.Count; i++)
            {
                sequences[i].Clear();
            }
                
           this.Refresh();
        }

        protected override void OnPaint(PaintEventArgs e)
        {

            base.OnPaint(e);

            if (!this.DesignMode)
            {
                for (int ii = 0; ii < sequences.Count; ii++)
                {
                    if (sequences[ii].Count > 1)
                    {
                        List<Point> seq = sequences[ii];
                        using (Brush brush = new SolidBrush(Color.Blue))
                        {
                            using (Pen pen = new Pen(brush, 10))
                            {
                                e.Graphics.DrawEllipse(pen, sequences[ii][sequences[ii].Count - 1].X, sequences[ii][sequences[ii].Count - 1].Y, 5, 5);
                            }
                        }
                    }
                }
            }
        }

        public void onStart()
        {
            Clear();

            capturing = true;
        }

        public void onStop()
        {
            capturing = false;
        }

        public void onDraw(List<Point> pts)
        {
            if (capturing)
            {
                for (int i = 0; i < sequences.Count; i++)
                {
                    if (pts[i].X > 0 && pts[i].Y > 0)
                    {
                        sequences[i].Add(new Point(pts[i].X, pts[i].Y));
                        this.Refresh();
                    }
                }                    
            }
        }

    }
}
