using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media.Media3D;

namespace MonkeyMotionControl
{

    class CustomSpline
    {
        //public static List<Point3D> pointsBezier;
        //public static void putPoints()
        //{
        //    if (pointsBezier == null)
        //        pointsBezier = new List<Point3D>();
        //    else
        //    {
        //        pointsBezier.Clear();
        //        pointsBezier = new List<Point3D>();
        //    }
        //    if (Helix3dSim.markers.Count == 0)
        //        return;
        //    for(int index=0; index<Helix3dSim.markers.Count; index++)
        //    {
        //        pointsBezier.Add(Helix3dSim.markers[index].point);
        //    }
        //}
        public static float spacing = 2.0f;
        public static Point3D GetBezierPoint(int index, float t)
        {
            if (t >= 1f)
                t = 1f;
            if (t < 0)
                t = 0;

            Point3D p1 = new Point3D();
            Point3D p2 = new Point3D();
            Point3D p3 = new Point3D();
            Point3D p4 = new Point3D();
            if(index==0)
                p1 = Helix3dSim.movePaths[index].ptCurve[0].point;
            else if(index==1)
                p1 = Helix3dSim.movePaths[index - 1].ptCurve[1].point;
            else
                p1 = Helix3dSim.movePaths[index - 1].ptCurve[0].point;
            p2 = Helix3dSim.movePaths[index].ptAdjust[0].point;
            p3 = Helix3dSim.movePaths[index].ptAdjust[1].point;
            if(index==0)
                p4 = Helix3dSim.movePaths[index].ptCurve[1].point;
            else
                p4 = Helix3dSim.movePaths[index].ptCurve[0].point;

            Point3D pt = new Point3D();
            pt = Bezier3D.GetPoint(p1, p2, p3, p4, t);
            return pt;
        }




        public static double GetDistance(Point3D point1, Point3D point2)
        {
            //pythagorean theorem dist^2 = a^2 + b^2 + c^2
            //thus dist = square root(a^2 + b^2 + c^2)
            double a = point2.X - point1.X;
            double b = point2.Y - point1.Y;
            double c = point2.Z - point1.Z;
            return Math.Sqrt(a * a + b * b + c * c);
        }

        public static List<Point3D> GetBezierPoint(int index)
        {
            float acc = 0.0001f;
            Point3D p1 = new Point3D();
            Point3D p2 = new Point3D();
            Point3D p3 = new Point3D();
            Point3D p4 = new Point3D();
            if (index == 0)
                p1 = Helix3dSim.movePaths[index].ptCurve[0].point;
            else if (index == 1)
                p1 = Helix3dSim.movePaths[index - 1].ptCurve[1].point;
            else
                p1 = Helix3dSim.movePaths[index - 1].ptCurve[0].point;
            p2 = Helix3dSim.movePaths[index].ptAdjust[0].point;
            p3 = Helix3dSim.movePaths[index].ptAdjust[1].point;
            if (index == 0)
                p4 = Helix3dSim.movePaths[index].ptCurve[1].point;
            else
                p4 = Helix3dSim.movePaths[index].ptCurve[0].point;

            Point3D last_spawn = Bezier3D.GetPoint(p1, p2, p3, p4, 0);

            List<Point3D> allPoints = new List<Point3D>();
            allPoints.Add(last_spawn);

            for (float t = acc; t <= 1.0f; t += acc)
            {
                Point3D trial = Bezier3D.GetPoint(p1, p2, p3, p4, t);
                if (GetDistance(trial, last_spawn) >= spacing)
                {
                    last_spawn = trial;
                    allPoints.Add(trial);
                }
            }
            return allPoints;
        }

        public Point3D GetVelocity(int index, float t)
        {
            return new Point3D();
            //return Bezier3D.GetFirstDerivative(
            //    Helix3dSim.pointsBezier[index-1][i], Helix3dSim.pointsBezier[index-1][i + 1], Helix3dSim.pointsBezier[index-1][i + 2], Helix3dSim.pointsBezier[index-1][i + 3], t);
        }
        public static Point3D GetSegmentPoint(int index, float t)
        {
            if (t >= 1f)
                t = 1f;
            if (t < 0)
                t = 0;
            Point3D p1 = new Point3D();
            Point3D p2 = new Point3D();
            Point3D pt = new Point3D();
            int i = 0;
            if (index == 0)
                i = 1;
            int k = 0;
            if (index == 1)
                k = 1;
            p1 = Helix3dSim.movePaths[index + i - 1].ptCurve[k].point;
            p2 = Helix3dSim.movePaths[index].ptCurve[i].point;



            pt.X = p1.X + Convert.ToInt32((p2.X - p1.X) * t);
            pt.Y = p1.Y + Convert.ToInt32((p2.Y - p1.Y) * t);
            pt.Z = p1.Z + Convert.ToInt32((p2.Z - p1.Z) * t);
            return pt;
        }

        public static List<Point3D> convertToPlane(Point3D p1, Point3D p2, Point3D p3)
        {
            Vector3D v1 = new Vector3D(p1.X, p1.Y, p1.Z);
            Vector3D v2 = new Vector3D(p2.X, p2.Y, p2.Z);
            Vector3D v3 = new Vector3D(p3.X, p3.Y, p3.Z);

            Vector3D p2top1 = v1 - v2;
            Vector3D p2top3 = v3 - v2;

            Vector3D circle_normal = new Vector3D();
            circle_normal = Vector3D.CrossProduct(p2top1, p2top3);
            circle_normal.Normalize();
            Vector3D xy_normal = new Vector3D(0, 0, 1);
            Vector3D rot_axis = Vector3D.CrossProduct(xy_normal, circle_normal);
            rot_axis.Normalize();
            double angle = Vector3D.AngleBetween(xy_normal, circle_normal);
            Quaternion rot = new Quaternion(rot_axis, -angle);

            Matrix3D m = Matrix3D.Identity;
            m.Rotate(rot);
            v1 = m.Transform(v1);
            v2 = m.Transform(v2);
            v3 = m.Transform(v3);

            List<Point3D> result = new List<Point3D>();
            Point3D pt = new Point3D(v1.X, v1.Y, v1.Z);
            result.Add(pt);
            pt = new Point3D(v2.X, v2.Y, v2.Z);
            result.Add(pt);
            pt = new Point3D(v3.X, v3.Y, v3.Z);
            result.Add(pt);

            return result;            
        }

        public static Point getCircleCenter(double x1,double y1, double x2, double y2,double radius)
        {
            double radsq = radius * radius;
            double q = Math.Sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
            double x3 = (x1 + x2) / 2 + Math.Sqrt(radsq - ((q / 2) * (q / 2))) * ((y1 - y2) / q);
            double y3 = (y1 + y2) / 2 + Math.Sqrt(radsq - ((q / 2) * (q / 2))) * ((x2 - x1) / q);
            return new Point(Convert.ToInt32(x3), Convert.ToInt32(y3));
        }

        public static Point getCircleCenter(double x1, double y1, double x2, double y2, double x3, double y3)
        {
            Point center = new Point();
            float ma = (float)((y2 - y1) / (x2 - x1));
            float mb = (float)((y3 - y2) / (x3 - x2));
            center.X = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
            center.Y = (-1 / ma) * (center.X - (x1 + x2) * 0.5) + (y1 + y2) * 0.5;
            return center;

        }

        public static Point3D getCircleCenter(int index)   // if bControl is true, it gets points on circle from given 3 points, else from given 2 points and radius
        {
            int radius = Helix3dSim.movePaths[index].radius;
            radius /= 2;
            radius += 100;
            Point3D pt1 = new Point3D();
            if (index == 0)
                pt1 = Helix3dSim.movePaths[0].ptCurve[0].point;
            else
            {
                if (index == 1)
                    pt1 = Helix3dSim.movePaths[index - 1].ptCurve[1].point;
                else
                    pt1 = Helix3dSim.movePaths[index - 1].ptCurve[0].point;
            }

            Point3D pt2 = new Point3D();
            if (index == 0)
                pt2 = Helix3dSim.movePaths[0].ptCurve[1].point;
            else
                pt2 = Helix3dSim.movePaths[index].ptCurve[0].point;

            Point3D pt3 = new Point3D();
            if (index == 0)
            {
                pt3.X = (pt1.X + pt2.X) / 2 + 300;
                pt3.Y = (pt1.Y + pt2.Y) / 2 + 300;
                pt3.Z = (pt1.Z + pt2.Z) / 2 + 300;
            }
            else
            {
                string shape = Helix3dSim.movePaths[index - 1].shape;
                if (shape == "bezier")
                    pt3 = Helix3dSim.movePaths[index - 1].ptAdjust[1].point;
                else if (shape == "segment")
                {
                    if (index == 1)
                        pt3 = Helix3dSim.movePaths[index - 1].ptCurve[0].point;
                    else
                        pt3 = Helix3dSim.movePaths[index - 2].ptCurve[0].point;
                }
                else if (shape == "arc")
                {
                    pt3 = Helix3dSim.movePaths[index - 1].ptAdjust[0].point;
                }
            }


            Vector3D v1 = new Vector3D(pt1.X, pt1.Y, pt1.Z);
            Vector3D v2 = new Vector3D(pt2.X, pt2.Y, pt2.Z);
            Vector3D v3 = new Vector3D(pt3.X, pt3.Y, pt3.Z);

            Vector3D p2top1 = v1 - v2;
            Vector3D p2top3 = v3 - v2;

            Vector3D circle_normal = new Vector3D();
            circle_normal = Vector3D.CrossProduct(p2top1, p2top3);
            circle_normal.Normalize();
            Vector3D xy_normal = new Vector3D(0, 0, 1);
            Vector3D rot_axis = Vector3D.CrossProduct(xy_normal, circle_normal);
            rot_axis.Normalize();
            double angle = Vector3D.AngleBetween(xy_normal, circle_normal);
            Quaternion rot = new Quaternion(rot_axis, -angle);

            Matrix3D m = Matrix3D.Identity;
            m.Rotate(rot);
            v1 = m.Transform(v1);
            v2 = m.Transform(v2);
            v3 = m.Transform(v3);

            List<Point3D> pt = new List<Point3D>();
            Point3D ptTemp = new Point3D(v1.X, v1.Y, v1.Z);
            pt.Add(ptTemp);
            ptTemp = new Point3D(v2.X, v2.Y, v2.Z);
            pt.Add(ptTemp);
            ptTemp = new Point3D(v3.X, v3.Y, v3.Z);
            pt.Add(ptTemp);

            Point center;
            center = getCircleCenter(pt[0].X, pt[0].Y, pt[1].X, pt[1].Y, pt[2].X, pt[2].Y);

            double r1 = (double)Math.Sqrt((pt[0].X - center.X) * (pt[0].X - center.X) + (pt[0].Y - center.Y) * (pt[0].Y - center.Y));
            double r2 = (double)Math.Sqrt((pt[1].X - center.X) * (pt[1].X - center.X) + (pt[1].Y - center.Y) * (pt[1].Y - center.Y));
            double r3 = (double)Math.Sqrt((pt[2].X - center.X) * (pt[2].X - center.X) + (pt[2].Y - center.Y) * (pt[2].Y - center.Y));

            CustomCurve curve = Helix3dSim.movePaths[index];
            curve.radius = (int)r1;
            Helix3dSim.movePaths[index] = curve;

            rot = new Quaternion(rot_axis, angle);
            m = Matrix3D.Identity;
            m.Rotate(rot);


            List<Point3D> ret = new List<Point3D>();
            Vector3D vector = new Vector3D();
            vector = m.Transform(new Vector3D(center.X, center.Y, pt[0].Z));

            return new Point3D(vector.X, vector.Y, vector.Z);
        }

        public static List<Point3D> GetArcPoints(int index, bool redraw)   
        {
            int i = 0, j = 0;
            if (index == 0)
                i = 1;
            if (index == 1)
                j = 1;
            int radius = 2 * (int)Math.Sqrt(Math.Pow(Helix3dSim.movePaths[index].ptCurve[i].point.X - Helix3dSim.movePaths[index + i - 1].ptCurve[j].point.X, 2)
                + Math.Pow(Helix3dSim.movePaths[index].ptCurve[i].point.Y - Helix3dSim.movePaths[index + i - 1].ptCurve[j].point.Y, 2)
                + Math.Pow(Helix3dSim.movePaths[index].ptCurve[i].point.Z - Helix3dSim.movePaths[index + i - 1].ptCurve[j].point.Z, 2));
            if (redraw)
                radius = Helix3dSim.movePaths[index].radius;

            Point3D pt1 = new Point3D();
            if (index == 0)
                pt1 = Helix3dSim.movePaths[0].ptCurve[0].point;
            else
            {
                if (index==1)
                    pt1 = Helix3dSim.movePaths[index - 1].ptCurve[1].point;
                else
                    pt1 = Helix3dSim.movePaths[index - 1].ptCurve[0].point;
            }

            Point3D pt3 = new Point3D();
            if (index == 0)
                pt3 = Helix3dSim.movePaths[0].ptCurve[1].point;
            else
                pt3 = Helix3dSim.movePaths[index].ptCurve[0].point;

            Point3D center = new Point3D();
            if(redraw)
            {
                center = Helix3dSim.movePaths[index].ptAdjust[0].point;   // center of arc
            }
            else
            {
                if(index>0)
                    center = Helix3dSim.movePaths[index - 1].ptAdjust[1].point;
                else
                {
                    Point3D p2 = Helix3dSim.movePaths[0].ptCurve[0].point;
                    p2.X += 300;
                    p2.Y += 300;
                    p2.Z += 300;
                    center = p2;
                }
            }


            Vector3D v1 = new Vector3D(pt1.X, pt1.Y, pt1.Z);
            Vector3D v2 = new Vector3D(center.X, center.Y, center.Z);
            Vector3D v3 = new Vector3D(pt3.X, pt3.Y, pt3.Z);

            Vector3D p2top1 = v1 - v2;
            Vector3D p2top3 = v3 - v2;

            Vector3D circle_normal = new Vector3D();
            circle_normal = Vector3D.CrossProduct(p2top1, p2top3);
            circle_normal.Normalize();
            Vector3D xy_normal = new Vector3D(0, 0, 1);
            Vector3D rot_axis = new Vector3D();
            if (circle_normal == xy_normal || circle_normal == -xy_normal)
                rot_axis = xy_normal;
            else
                rot_axis = Vector3D.CrossProduct(xy_normal, circle_normal);
            rot_axis.Normalize();
            double angle;
            if (circle_normal == xy_normal || circle_normal == -xy_normal)
                angle = 0;
            else
                angle = Vector3D.AngleBetween(xy_normal, circle_normal);
            Quaternion rot = new Quaternion(rot_axis, -angle);

            Matrix3D m = Matrix3D.Identity;
            m.Rotate(rot);
            v1 = m.Transform(v1);
            v2 = m.Transform(v2);
            v3 = m.Transform(v3);

            List<Point3D> pt = new List<Point3D>();
            Point3D ptTemp = new Point3D(v1.X, v1.Y, v1.Z);
            pt.Add(ptTemp);
            ptTemp = new Point3D(v2.X, v2.Y, v2.Z);
            pt.Add(ptTemp);
            ptTemp = new Point3D(v3.X, v3.Y, v3.Z);
            pt.Add(ptTemp);

            Point cent2D;
            if (redraw)
                cent2D = new Point(pt[1].X, pt[1].Y);
            else
                cent2D = getCircleCenter(pt[0].X, pt[0].Y, pt[2].X, pt[2].Y, radius);

            double r1 = (double)Math.Sqrt((pt[0].X - cent2D.X) * (pt[0].X - cent2D.X) + (pt[0].Y - cent2D.Y) * (pt[0].Y - cent2D.Y));
            double r2 = (double)Math.Sqrt((pt[1].X - cent2D.X) * (pt[1].X - cent2D.X) + (pt[1].Y - cent2D.Y) * (pt[1].Y - cent2D.Y));
            double r3 = (double)Math.Sqrt((pt[2].X - cent2D.X) * (pt[2].X - cent2D.X) + (pt[2].Y - cent2D.Y) * (pt[2].Y - cent2D.Y));
            double fai1 = Math.Atan2((double)(pt[0].Y - cent2D.Y) , (double)(pt[0].X - cent2D.X));
            double fai3 = Math.Atan2((double)(pt[1].Y - cent2D.Y), (double)(pt[1].X - cent2D.X));
            double fai2 = Math.Atan2((double)(pt[2].Y - cent2D.Y), (double)(pt[2].X - cent2D.X));

            if (fai1 > fai2)
            {
                double temp = fai2;
                fai2 = fai1;
                fai1 = temp;
            }

            //if (fai1 - fai2 > Math.PI)
            //{
            //    double temp1 = fai2;
            //    fai2 = 2 * Math.PI - fai1;
            //    fai1 = temp1;
            //}

            List<Point3D> result = new List<Point3D>();
            Point3D pp = new Point3D();
            pp.Z = pt[0].Z;
            for (double step=fai1;step<=fai2;step+=(double)spacing/(double)r1)
            {
                pp.X = cent2D.X + r1 * Math.Cos(step);
                pp.Y = cent2D.Y + r1 * Math.Sin(step);
                result.Add(pp);
            }

            rot = new Quaternion(rot_axis, angle);
            m = Matrix3D.Identity;
            m.Rotate(rot);


            List<Point3D> ret = new List<Point3D>();
            Vector3D vector = new Vector3D();
            for(i=0;i<result.Count;i++)
            {
                vector = m.Transform(new Vector3D(result[i].X, result[i].Y, result[i].Z));
                pp = new Point3D(vector.X, vector.Y, vector.Z);
                ret.Add(pp);
            }
            vector = m.Transform(new Vector3D(cent2D.X, cent2D.Y, pt[0].Z));
            center= new Point3D(vector.X, vector.Y, vector.Z);
            if(!redraw)
            {
                CustomCurve cur = Helix3dSim.movePaths[index];
                cur.ptAdjust[0].point = center;
                cur.radius = radius;
                Helix3dSim.movePaths[index] = cur;
            }

            return ret;
        }
    }
}
