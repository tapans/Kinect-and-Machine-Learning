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
        private List<List<Point>> sequence;

        public Canvas()
        {
            InitializeComponent();
            sequence = new List<List<Point>>();
            sequence.Add(new List<Point>());
            sequence.Add(new List<Point>());
            this.DoubleBuffered = true;
        }

        public Point[] GetSequence(int n)
        {

            return sequence[n].ToArray();
        }

        public void Clear()
        {
            for (int i = 0; i < sequence.Count; i++ )
            {
                sequence[i].Clear();
            }
            this.Refresh();
        }

        protected override void OnPaint(PaintEventArgs e)
        {

            base.OnPaint(e);

            if (!this.DesignMode)
            {
                for (int ii = 0; ii < sequence.Count; ii++)
                {
                    if (sequence[ii].Count > 1)
                    {
                        List<Point> seq = sequence[ii];
                        using (Brush brush = new SolidBrush(Color.Blue)) {
                            using (Pen pen = new Pen(brush, 10))
                            {
                                e.Graphics.DrawEllipse(pen, sequence[ii][sequence[ii].Count - 1].X, sequence[ii][sequence[ii].Count - 1].Y, 5, 5);
                            }
                        }
                        
                        //for (int i = 1; i < seq.Count; i++)
                        //{
                        //    int x = (int)seq[i].X;
                        //    int y = (int)seq[i].Y;
                        //    int p = (int)Accord.Math.Tools.Scale(0, seq.Count, 0, 255, i);

                        //    int prevX = (int)seq[i - 1].X;
                        //    int prevY = (int)seq[i - 1].Y;
                        //    int prevP = (int)Accord.Math.Tools.Scale(0, seq.Count, 0, 255, i - 1);

                        //    if (x == prevX && y == prevY)
                        //        continue;

                        //    Point start = new Point(prevX, prevY);
                        //    Point end = new Point(x, y);
                        //    Color colorStart = Color.FromArgb(255 - p, 0, p);
                        //    Color colorEnd = Color.FromArgb(255 - prevP, 0, prevP);

                        //    using (Brush brush = new LinearGradientBrush(start, end, colorStart, colorEnd))
                        //    using (Pen pen = new Pen(brush, 10))
                        //    {
                        //        pen.StartCap = LineCap.Round;
                        //        pen.EndCap = LineCap.Round;

                        //        e.Graphics.DrawLine(pen, prevX, prevY, x, y);
                        //    }
                        //}
                    }
                }
            }
        }

        public void onHandStart()
        {
            Clear();

            capturing = true;
        }

        public void onHandStop()
        {
            capturing = false;
        }

        public void onHandDraw(List<Point> pts)
        {
            if (capturing)
            {
                for (int i=0; i<sequence.Count; i++)
                {
                    if (pts[i].X > 0 && pts[i].Y > 0)
                    {
                        sequence[i].Add(new Point(pts[i].X, pts[i].Y));
                    }
                }
                this.Refresh();
            }
        }

    }
}
