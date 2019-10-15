using System.Collections.Generic;
using System.Windows.Media.Media3D;

namespace MonkeyMotionControl
{
    class Bezier3D
    {
        public List<Point3D> points;
        public void reset()
        {
            if(points!=null)
            {
                points.Clear();
                points = new List<Point3D>();
            }

        }
        Bezier3D(double[] b, int cpts, double[] p)
        {
            //int npts = (b.Length) / 2;
            //int icount, jcount;
            //double step, t;

            //// Calculate points on curve

            //icount = 0;
            //t = 0;
            //step = (double)1.0 / (cpts - 1);

            //for (int i1 = 0; i1 != cpts; i1++)
            //{
            //    if ((1.0 - t) < 5e-6)
            //        t = 1.0;

            //    jcount = 0;
            //    p[icount] = 0.0;
            //    p[icount + 1] = 0.0;
            //    for (int i = 0; i != npts; i++)
            //    {
            //        double basis = Bernstein(npts - 1, i, t);
            //        p[icount] += basis * b[jcount];
            //        p[icount + 1] += basis * b[jcount + 1];
            //        jcount = jcount + 2;
            //    }

            //    icount += 2;
            //    t += step;
            //}
        }

        public static Point3D GetPoint(Point3D p0, Point3D p1, Point3D p2, Point3D p3, float t)
        {
            if (t < 0)
                t = 0;
            else if (t > 1)
                t = 1;
            float oneMinusT = 1f - t;
            Point3D ret = new Point3D();
            ret.X = oneMinusT* oneMinusT *oneMinusT * p0.X + 3f * oneMinusT * oneMinusT * t * p1.X + 
                3f * oneMinusT * t * t * p2.X + t * t * t * p3.X;
            ret.Y = oneMinusT * oneMinusT * oneMinusT * p0.Y + 3f * oneMinusT * oneMinusT * t * p1.Y +
                3f * oneMinusT * t * t * p2.Y + t * t * t * p3.Y;
            ret.Z = oneMinusT * oneMinusT * oneMinusT * p0.Z + 3f * oneMinusT * oneMinusT * t * p1.Z +
                3f * oneMinusT * t * t * p2.Z + t * t * t * p3.Z;
            return ret;
        }

        public static Point3D GetFirstDerivative(Point3D p0, Point3D p1, Point3D p2, Point3D p3, float t)
        {
            if (t < 0)
                t = 0;
            else if (t > 1)
                t = 1;
            float oneMinusT = 1f - t;
            Point3D ret = new Point3D();
            ret.X = 3f * oneMinusT * oneMinusT * (p1.X - p0.X) + 6f * oneMinusT * t * (p2.X - p1.X) + 3f * t * t * (p3.X - p2.X);
            ret.Y = 3f * oneMinusT * oneMinusT * (p1.Y - p0.Y) + 6f * oneMinusT * t * (p2.Y - p1.Y) + 3f * t * t * (p3.Y - p2.Y);
            ret.Z = 3f * oneMinusT * oneMinusT * (p1.Z - p0.Z) + 6f * oneMinusT * t * (p2.Z - p1.Z) + 3f * t * t * (p3.Z - p2.Z);
            return ret;
        }
    }
}
