using HelixToolkit.Wpf;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace MonkeyMotionControl
{
    /// <summary>
    /// Plot a trace in 3D space with marker, axes and bounding box.
    /// </summary>
    /// <remarks>
    /// This class utilizes the Helix Toolkit which is licensed under the MIT License.
    /// 
    /// The MIT License (MIT)
    /// Copyright(c) 2018 Helix Toolkit contributors
    /// 
    /// Permission is hereby granted, free of charge, to any person obtaining a
    /// copy of this software and associated documentation files (the
    /// "Software"), to deal in the Software without restriction, including
    /// without limitation the rights to use, copy, modify, merge, publish,
    /// distribute, sublicense, and/or sell copies of the Software, and to
    /// permit persons to whom the Software is furnished to do so, subject to
    /// the following conditions:
    /// 
    /// The above copyright notice and this permission notice shall be included
    /// in all copies or substantial portions of the Software.
    /// 
    /// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    /// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    /// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    /// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    /// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    /// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    /// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    /// </remarks>
    public class HelixPlot 
    {
        private TruncatedConeVisual3D marker;
        private BillboardTextVisual3D coords;
        private double labelOffset, minDistanceSquared;
        private string coordinateFormat;
        private List<LinesVisual3D> trace;
        private LinesVisual3D path;
        private Point3D point0;  // last point
        private Vector3D delta0;  // (dx,dy,dz)

        /// <summary>Current position marker.</summary>
        public bool EnableMarker { get; set; }

        /// <summary>Distance between ticks on the XY grid.</summary>
        public double TickSize { get; set; }

        /// <summary>A point closer than this distance from the previous point will not be plotted.</summary>
        public double MinDistance { get; set; }

        /// <summary>Number of decimal places for the marker coordinates.</summary>
        public int DecimalPlaces { get; set; }

        /// <summary>Brush used for the marker cone and coordinates.</summary>
        public SolidColorBrush MarkerBrush { get; set; }

        /// <summary>Reference of the parent HelixViewPort3D.</summary>
        private HelixViewport3D parentViewport { get; set; }

        /// <summary>Gets the current trace color.</summary>
        public Color TraceColor { get; set; }

        /// <summary>Gets the current trace thickness.</summary>
        public double TraceThickness { get; set; }

        /// <summary>Initializes a new instance of the <see cref="HelixPlot"/> class.</summary>
        public HelixPlot(HelixViewport3D viewport, bool enable_marker)
        {
            EnableMarker = enable_marker;
            TickSize = 10;
            MinDistance = 0.1;
            DecimalPlaces = 1;
            MarkerBrush = Brushes.Red;
            parentViewport = viewport;
            TraceColor = Colors.White;
            TraceThickness = 1;
            CreateElements();
        }

        /// <summary>Creates the plot elements.</summary>
        /// <remarks>Changes to the bounding box and other parameters will not take effect until this method is called.</remarks>
        public void CreateElements()
        {
            double lineThickness = 500 / 1000;
            labelOffset = lineThickness * 50;
            minDistanceSquared = MinDistance * MinDistance;

            if (EnableMarker)//Elements.HasFlag(EElements.Marker
            {
                marker = new TruncatedConeVisual3D();
                marker.Height = labelOffset;
                marker.BaseRadius = 0.0;
                marker.TopRadius = labelOffset / 5;
                marker.TopCap = true;
                marker.Origin = new Point3D(0.0, 0.0, 0.0);
                marker.Normal = new Vector3D(-1.0, -1.0, 1.0);
                marker.Fill = MarkerBrush;
                parentViewport.Children.Add(marker);

                coords = new BillboardTextVisual3D();
                coordinateFormat = string.Format("{{0:F{0}}}, {{1:F{0}}}, {{2:F{0}}}", DecimalPlaces, DecimalPlaces, DecimalPlaces);  // "{0:F2}, {1:F2}, {2:F2}"
                coords.Text = string.Format(coordinateFormat, 0.0, 0.0, 0.0);
                coords.Foreground = MarkerBrush;
                coords.Position = new Point3D(-labelOffset, -labelOffset, labelOffset);
                parentViewport.Children.Add(coords);
            }
            else
            {
                marker = null;
                coords = null;
            }

            if (trace != null)
            {
                foreach (LinesVisual3D p in trace)
                    parentViewport.Children.Add(p);

                if (trace.Count > 0)
                {
                    path = trace[trace.Count - 1];
                }
                
            }
        }

        /// <summary>Clears all traces.</summary>
        private void Clear()
        {
            trace = null;
            path = null;
            CreateElements();
        }

        public void Clear3D()
        {
            if (trace != null)
            {
                if (trace.Count > 0)
                {
                    foreach (LinesVisual3D p in trace)
                        parentViewport.Children.Remove(p);

                    parentViewport.Children.Remove(coords);
                    parentViewport.Children.Remove(marker);
                }
                Clear();
            }
        }

        /// <summary>
        /// Creates a new trace.
        /// </summary>
        /// <remarks>Existing traces will remain in the plot until <see cref="Clear"/> or <see cref="CreateElements"/> is called.</remarks>
        /// <param name="point">The (X,Y,Z) location.</param>
        /// <param name="color">The initial color.</param>
        /// <param name="thickness">The initial line thickness.</param>
        /// <returns>The trace count.</returns>
        /// <seealso cref="Clear"/>
        public void NewTrace(Point3D point, Color color, double thickness = 1)
        {
            path = new LinesVisual3D();
            path.Color = color;
            path.Thickness = thickness;
            trace = new List<LinesVisual3D>();
            trace.Add(path);
            parentViewport.Children.Add(path);
            point0 = point;
            delta0 = new Vector3D();

            if (marker != null)
            {
                marker.Origin = point;
                coords.Position = new Point3D(point.X - labelOffset, point.Y - labelOffset, point.Z + labelOffset);
                coords.Text = string.Format(coordinateFormat, point.X, point.Y, point.Z);
            }
        }

        /// <summary>
        /// Creates a new trace.
        /// </summary>
        /// <remarks>Existing traces will remain in the plot until <see cref="Clear"/> or <see cref="CreateElements"/> is called.</remarks>
        /// <param name="x">The initial X location.</param>
        /// <param name="y">The initial Y location.</param>
        /// <param name="z">The initial Z location.</param>
        /// <param name="color">The initial color.</param>
        /// <param name="thickness">The initial line thickness.</param>
        /// <returns>The trace count.</returns>
        /// <seealso cref="Clear"/>
        public void NewTrace(double x, double y, double z, Color color, double thickness = 1)
        {
            NewTrace(new Point3D(x, y, z), color, thickness);
        }

        /// <summary>
        /// Adds a point to the current trace with a specified color.
        /// </summary>
        /// <param name="point">The (X,Y,Z) location.</param>
        /// <param name="color">The color.</param>
        /// <param name="thickness">The line thickness (optional).</param>
        /// <seealso cref="AddPoint(double, double, double, Color, double)"/>
        public void AddPoint(Point3D point, Color color, double thickness = -1)
        {
            if (trace == null)
            {
                NewTrace(point, color, (thickness > 0) ? thickness : 1);
                return;
            }

            if ((point - point0).LengthSquared < minDistanceSquared) return;  // less than min distance from last point

            if (path.Color != color || (thickness > 0 && path.Thickness != thickness))
            {
                if (thickness <= 0)
                    thickness = path.Thickness;

                path = new LinesVisual3D();
                path.Color = color;
                path.Thickness = thickness;
                trace.Add(path);
                parentViewport.Children.Add(path);
            }

            // If line segments AB and BC have the same direction (small cross product) then remove point B.
            bool sameDir = false;
            var delta = new Vector3D(point.X - point0.X, point.Y - point0.Y, point.Z - point0.Z);
            delta.Normalize();  // use unit vectors (magnitude 1) for the cross product calculations
            if (path.Points.Count > 0)
            {
                double xp2 = Vector3D.CrossProduct(delta, delta0).LengthSquared;
                sameDir = (xp2 < 0.0005);  // approx 0.001 seems to be a reasonable threshold from logging xp2 values
                //if (!sameDir) Title = string.Format("xp2={0:F6}", xp2);
            }

            if (sameDir)  // extend the current line segment
            {
                path.Points[path.Points.Count - 1] = point;
                point0 = point;
                delta0 += delta;
            }
            else  // add a new line segment
            {
                path.Points.Add(point0);
                path.Points.Add(point);
                point0 = point;
                delta0 = delta;
            }

            if (marker != null)
            {
                marker.Origin = point;
                coords.Position = new Point3D(point.X - labelOffset, point.Y - labelOffset, point.Z + labelOffset);
                coords.Text = string.Format(coordinateFormat, point.X, point.Y, point.Z);
            }
        }

        /// <summary>
        /// Adds a point to the current trace.
        /// </summary>
        /// <param name="point">The (X,Y,Z) location.</param>
        /// <seealso cref="AddPoint(Point3D, Color, double)"/>
        public void AddPoint(Point3D point)
        {
            if (path == null)
            {
                NewTrace(point, Colors.Black, 1);
                return;
            }

            AddPoint(point, path.Color, path.Thickness);
        }

        /// <summary>
        /// Adds a point to the current trace with a specified color.
        /// </summary>
        /// <param name="x">The X location.</param>
        /// <param name="y">The Y location.</param>
        /// <param name="z">The Z location.</param>
        /// <param name="color">The color.</param>
        /// <param name="thickness">The line thickness (optional).</param>
        /// <seealso cref="AddPoint(Point3D, Color, double)"/>
        public void AddPoint(double x, double y, double z, Color color, double thickness = -1)
        {
            AddPoint(new Point3D(x, y, z), color, thickness);
        }

        /// <summary>
        /// Adds a point to the current trace.
        /// </summary>
        /// <param name="x">The X location.</param>
        /// <param name="y">The Y location.</param>
        /// <param name="z">The Z location.</param>
        /// <seealso cref="AddPoint(double, double, double, Color, double)"/>
        public void AddPoint(double x, double y, double z)
        {
            if (path == null) return;

            AddPoint(new Point3D(x, y, z), path.Color, path.Thickness);
        }
    }
}
