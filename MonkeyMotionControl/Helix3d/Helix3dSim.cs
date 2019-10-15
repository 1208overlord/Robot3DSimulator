using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;

namespace MonkeyMotionControl
{
    class Joint
    {
        public Model3D model { get; set; }
        public double angle = 0;
        public double angleMin = -180;
        public double angleMax = 180;
        public int rotPointX = 0;
        public int rotPointY = 0;
        public int rotPointZ = 0;
        public int rotAxisX = 0;
        public int rotAxisY = 0;
        public int rotAxisZ = 0;

        public Joint()
        {
            angle = 0;
            angleMax = 180;
            angleMin = -180;
            rotPointX = 0;
            rotPointY = 0;
            rotPointZ = 0;
            rotAxisX = 0;
            rotAxisY = 0;
            rotAxisZ = 0;
        }

        public Joint(Model3D pModel)
        {
            model = pModel;
        }
    }
	
	public class BezierMarkPoint : SphereVisual3D
    {
        public int number;
        public Point3D point;
        public string name;
        public BezierMarkPoint(int number,string name, Point3D point)
        {
            this.name = name;
            this.number = number;
            this.point = point;
        }
        public BezierMarkPoint()
        {
            new BezierMarkPoint(0, "point", new Point3D());
        }
    }

    public struct CustomCurve
    {
        public string shape;
        public List<BezierMarkPoint> ptCurve;
        public List<BezierMarkPoint> ptAdjust;
        public int radius;
        public List<Point3D> curvePath;
        public List<LinesVisual3D> ctrlLine;
    }

    public class Trajectory:LinesVisual3D
    {
        public int number;
        public Trajectory(int num)
        {
            this.number = num;
        }
    }

    public class PointTranslateManipulator : TranslateManipulator
    {
        protected override void OnMouseMove(System.Windows.Input.MouseEventArgs e)
        {
            base.OnMouseMove(e);
            if (this.IsMouseCaptured)
            {
                Point3D pt = new Point3D(this.Transform.Value.OffsetX, this.Transform.Value.OffsetY, this.Transform.Value.OffsetZ);
                //if (rr.VisualHit != null)
                if (Helix3dSim.bSelected)
                {
                    if(Helix3dSim.strSelected=="TargetPoint")
                    {
                        Helix3dSim.targetPoint.point = pt;
                        Helix3dSim.targetPoint.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));

                        Vector3D direction = new Vector3D(0, 0, 0);
                        direction.X = pt.X - Helix3dSim.arrow_PIP.Transform.Value.OffsetX;
                        direction.Y = pt.Y - Helix3dSim.arrow_PIP.Transform.Value.OffsetY;
                        direction.Z = pt.Z - Helix3dSim.arrow_PIP.Transform.Value.OffsetZ;
                        direction.Normalize();
                        Helix3dSim.arrow_PIP.Direction = direction;
                        Helix3dSim.UpdateTargetDistance();
                    }
                    else
                        Helix3dSim.onUpdataGraphics(pt);
                }
            }
        }
    }

    public class SimRobotStatus
    {
        //public static RobotStatus Instance = new RobotStatus();
        public double[] ToolCartesianPos = new double[] { 0, 0, 0, 0, 0, 0 }; // X, Y, Z, RX, RY, RZ
        public double[] JointPos = new double[] { 0, 0, 0, 0, 0, 0 }; // J1, J2, J3, J4, J5, J6
        public double[] HomeJointPos = new double[] { 0.0, -20.0, 0x87, 0.0, 0x41, 0.0 };
        public double[] HomeCartesianPos = new double[] { 706.17, 0.0, 274.32, -180.0, 0.0, -180.0 };
        public Point3D ToolOffset = new Point3D( 0, 0, 0 );

        public SimRobotStatus()
        {

        }

        public double X
        {
            get { return ToolCartesianPos[0]; }
            set { ToolCartesianPos[0] = value; }
        }
        public double Y
        {
            get { return ToolCartesianPos[1]; }
            set { ToolCartesianPos[1] = value; }
        }
        public double Z
        {
            get { return ToolCartesianPos[2]; }
            set { ToolCartesianPos[2] = value; }
        }
        public double RX
        {
            get { return ToolCartesianPos[3]; }
            set { ToolCartesianPos[3] = value; }
        }
        public double RY
        {
            get { return ToolCartesianPos[4]; }
            set { ToolCartesianPos[4] = value; }
        }
        public double RZ
        {
            get { return ToolCartesianPos[5]; }
            set { ToolCartesianPos[5] = value; }
        }
        public double J1
        {
            get { return JointPos[0]; }
            set { JointPos[0] = value; }
        }
        public double J2
        {
            get { return JointPos[1]; }
            set { JointPos[1] = value; }
        }
        public double J3
        {
            get { return JointPos[2]; }
            set { JointPos[2] = value; }
        }
        public double J4
        {
            get { return JointPos[3]; }
            set { JointPos[3] = value; }
        }
        public double J5
        {
            get { return JointPos[4]; }
            set { JointPos[4] = value; }
        }
        public double J6
        {
            get { return JointPos[5]; }
            set { JointPos[5] = value; }
        }

    }

    public class Helix3dSim
    {

        #region VARIABLES - 3D SIMULATOR

        private static MainWindow parentWindow { get; set; }

        public static HelixViewport3D viewport3d { get; set; }

        public struct TracePoint3D
        {
            public Point3D point;
            public Color color;
            public double thickness;
            public TracePoint3D(Point3D point, Color color, double thickness)
            {
                this.point = point;
                this.color = color;
                this.thickness = thickness;
            }
        }

        public bool IsPanControlEnabled { get; set; }

        public bool IsPathTracingEnabled { get; set; }

        private HelixPlot helixTracer;
        private Dictionary<SphereVisual3D, string> sequence_pointNames = new Dictionary<SphereVisual3D, string>();
        public List<SphereVisual3D> sequence_points = new List<SphereVisual3D>();
        private Dictionary<BillboardTextVisual3D, string> sequence_textNames = new Dictionary<BillboardTextVisual3D, string>();
        private List<BillboardTextVisual3D> sequence_pointsText = new List<BillboardTextVisual3D>();
        private List<Joint> joints = null;
        private Color oldColor = Colors.White;
        private static string debugPath = @"\x64\Debug";
        private static string basePath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + debugPath + "\\3D_Models\\";
        private GeometryModel3D oldSelectedModel = null;
        public static SphereVisual3D toolSphereVisual;
        private SphereVisual3D limitSphereVisual;
        public static BezierMarkPoint targetPoint;
        public static PointTranslateManipulator arrow_PIP;
        public LinesVisual3D trackLine;
        private ModelVisual3D RoboticArm;
        private GridLinesVisual3D gridLines;
        private Transform3DGroup F1;
        private Transform3DGroup F2;
        private Transform3DGroup F3;
        private Transform3DGroup F4;
        private Transform3DGroup F5;
        private Transform3DGroup F6;
        private RotateTransform3D R;
        private TranslateTransform3D T;
        private const string MODEL_PATH1 = "RX160L-HB_LINK1.stl";
        private const string MODEL_PATH2 = "RX160L-HB_LINK2.stl";
        private const string MODEL_PATH3 = "RX160L-HB_LINK3.stl";
        private const string MODEL_PATH4 = "RX160L-HB_LINK4.stl";
        private const string MODEL_PATH5 = "RX160L-HB_LINK5.stl";
        private const string MODEL_PATH6 = "RX160L-HB_LINK6.stl";
        private const string MODEL_PATH7 = "RX160L-HB_BASE.stl";
        private const string MODEL_PATH8 = "RX160L-HB_TOOL.stl";
        public SimRobotStatus simRobotStatus;
        private double LearningRate = 0.01;
        private double SamplingDistance = 0.15;
        private double DistanceThreshold = 20.0;
        public int count = 0;
        public int nCurve = 0;
        
        #endregion

        #region VARIABLES - MOTION PATH

        public List<TracePoint3D> tracepoints = new List<TracePoint3D>();       // list of trace points
        public static List<Trajectory> lineBezier = new List<Trajectory>();     // list of curves (list number is curve number)
        public static List<CustomCurve> movePaths = new List<CustomCurve>();    // list of curve points (list number is curve number)
        public static int nSelected = -1;                                       // number of selected curve
        public static Point3D ptSelected = new Point3D();                       // position of selected point
        public static bool bSelected = false;                                   // true when selecting point
        public static string strSelected = "";                                  // string of selected curve("point" or "control")
        public bool bCurveSelected = false;                                     // true when selecting curve
        public int nCurveSelected = -1;                                         // number of selected curve
        public static bool jointChanged = false;
        public static int nPoint = -1;
        public bool isDrag = false;
        public bool bMark = false;
        public List<Point3D> trackPts = new List<Point3D>();
        public static double limitSphereRadius = 0.0f;
        public static Point3D limitSphereCenterPoint = new Point3D();
        public static Point3D limitSpherePoint = new Point3D();
        public static bool bInSphere = true;
        public int oldCount = 0;
        public int totalCount = 0;
        public static int curPoint = -1;
        public float total_time_origin = 0.0f;
        public int play_step = 0;
        public bool bFirst = false;
        public bool bForward = true;
        public int timeCount = 0;
        public bool reachedPathEnd = false;
        public double totalDuration = 0;
        public double curPlayheadPos = 0;
        private static System.Windows.Forms.Timer timerCalc;
        private static System.Windows.Forms.Timer timerPlay;
        public bool wasSyncEnabled = false;
        private List<double[]> ptArray = new List<double[]>();
        public List<List<double[]>> trajJoints = new List<List<double[]>>();
        private static double targetDistance;
        private static Vector3D toolPosition;

        public static bool linkHandle
        {
            get
            {
                return parentWindow.link_handle_chk.IsChecked.Value;
            }
        }

        public static bool showHandle
        {
            get
            {
                return parentWindow.show_ctrl_chk.IsChecked.Value;
            }
        }

        public static bool showTracking
        {
            get
            {
                return parentWindow.show_tracking_line.IsChecked.Value;
            }
        }

        public static int curveCount
        {
            get
            {
                return movePaths.Count;
            }
        }

        #endregion

        /// <summary>
        /// Helix3dSim Constructor
        /// </summary>
        /// <param name="parent_window">Reference to the parent MainWindow</param>
        public Helix3dSim(MainWindow _parentwindow) {

            parentWindow = _parentwindow;
            viewport3d = _parentwindow.helix_viewport3d;
            IsPanControlEnabled = false;
            IsPathTracingEnabled = false;

            // 3D Viewport and Camera Settings
            viewport3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewport3d.Camera.LookDirection = new Vector3D(-3283, -5560, -584);
            viewport3d.Camera.UpDirection = new Vector3D(0.367, 0.619, 0.694);
            viewport3d.Camera.Position = new Point3D(3728, 5017, 1622);
            //viewport3d.Camera.LookDirection = new Vector3D(10, 0, 0);
            //viewport3d.Camera.UpDirection = new Vector3D(0, 0, 1);
            //viewport3d.Camera.Position = new Point3D(0, 10, 0);
            //viewport3d.Camera.FieldOfView = 45;

            // Gridlines
            gridLines = new GridLinesVisual3D();
            gridLines.Fill = Brushes.LightGray;
            gridLines.Width = 16000;
            gridLines.Length = 16000;
            gridLines.Thickness = 3;
            gridLines.MajorDistance = 500;
            gridLines.MinorDistance = 500;

            // Tracer
            helixTracer = new HelixPlot(viewport3d,false);
            
            // Robotic Arm Model
            List<string> modelsNames = new List<string>();
            modelsNames.Add(MODEL_PATH1);
            modelsNames.Add(MODEL_PATH2);
            modelsNames.Add(MODEL_PATH3);
            modelsNames.Add(MODEL_PATH4);
            modelsNames.Add(MODEL_PATH5);
            modelsNames.Add(MODEL_PATH6);
            modelsNames.Add(MODEL_PATH7);
            modelsNames.Add(MODEL_PATH8);
            RoboticArm = new ModelVisual3D();
            RoboticArm.Content = Initialize_Environment(modelsNames);

            // Tool location sphere (joint currently rotating to)
            toolSphereVisual = new SphereVisual3D()
            {
                Radius = 15,
                Center = new Point3D(0, 0, 0), //toolPosition.ToPoint3D(),
                Fill = Brushes.Yellow
            };

            // Center position of Tool sphere to Flange
            Rect3D b = joints[5].model.Bounds;
            Vector3D toolPosition = new Vector3D(b.X + b.SizeX / 2, b.Y + b.SizeY / 2, b.Z + b.SizeZ / 2);
            toolSphereVisual.Center = toolPosition.ToPoint3D();

            // PIP Arrow (Using Manipulator) 
            arrow_PIP = new PointTranslateManipulator();
            arrow_PIP.SetName("PIP_Manipulator");
            arrow_PIP.Direction = new Vector3D(1, 0, 0);
            arrow_PIP.Length = 200;
            arrow_PIP.Diameter = 10;
            arrow_PIP.Color = Colors.Red;
            arrow_PIP.Offset = new Vector3D(0, 0, 0);
            arrow_PIP.Transform = toolSphereVisual.Transform;
            arrow_PIP.TargetTransform = toolSphereVisual.Transform;
            arrow_PIP.Bind(toolSphereVisual);

            // PIP Tracking Line
            trackPts.Add(new Point3D(0, 0, 0));
            trackPts.Add(new Point3D(0, 0, 0));
            trackLine = new LinesVisual3D
            {
                Color = Colors.Yellow,
                Thickness = 2.0,
                Points = { new Point3D(0, 0, 0), new Point3D(0, 0, 0) }
            };

            // Target mark point
            targetPoint = new BezierMarkPoint(-1, "TargetPoint", new Point3D(1500, 0, 400));
            targetPoint.Radius = 25;
            targetPoint.Fill = Brushes.Red;
            targetPoint.Transform = new TranslateTransform3D(targetPoint.point.ToVector3D());

            // ADD items to the 3D Viewport
            viewport3d.Children.Add(new DefaultLights());
            viewport3d.Children.Add(gridLines);
            viewport3d.Children.Add(RoboticArm);
            viewport3d.Children.Add(targetPoint);
            viewport3d.Children.Add(toolSphereVisual);
            viewport3d.Children.Add(arrow_PIP);
            
            if (showTracking) viewport3d.Children.Add(trackLine);

            // Add Motion Boundary Limit Sphere
            limitSphereCenterPoint = new Point3D(joints[1].rotPointX, joints[1].rotPointY, joints[1].rotPointZ);
            limitSphereRadius = CustomSpline.GetDistance(limitSphereCenterPoint, new Point3D(2200, 0, 0));
            //addLimitSphere(limitSphereCenterPoint, limitSphereRadius);

            simRobotStatus = new SimRobotStatus();

            parentWindow.slider_simPos_J1.Minimum = joints[0].angleMin;
            parentWindow.slider_simPos_J1.Maximum = joints[0].angleMax;
            parentWindow.slider_simPos_J2.Minimum = joints[1].angleMin;
            parentWindow.slider_simPos_J2.Maximum = joints[1].angleMax;
            parentWindow.slider_simPos_J3.Minimum = joints[2].angleMin;
            parentWindow.slider_simPos_J3.Maximum = joints[2].angleMax;
            parentWindow.slider_simPos_J4.Minimum = joints[3].angleMin;
            parentWindow.slider_simPos_J4.Maximum = joints[3].angleMax;
            parentWindow.slider_simPos_J5.Minimum = joints[4].angleMin;
            parentWindow.slider_simPos_J5.Maximum = joints[4].angleMax;
            parentWindow.slider_simPos_J6.Minimum = joints[5].angleMin;
            parentWindow.slider_simPos_J6.Maximum = joints[5].angleMax;

            // Go to Initial Position
            goHomePosition();

        }

        private void addLimitSphere(Point3D center, double radius)
        {
            limitSphereVisual = new SphereVisual3D();
            limitSphereVisual.SetName("LimitSphere");
            limitSphereVisual.Radius = radius;
            limitSphereVisual.Center = center;
            //sphere.Fill = new SolidColorBrush(Colors.DarkBlue);
            DiffuseMaterial material = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(20, 255, 255, 255)));
            limitSphereVisual.Material = material;
            limitSphereVisual.BackMaterial = material;
            viewport3d.Children.Add(limitSphereVisual);
        }

        private void UpdateLimitSpherePosition(Vector3D point)
        {
            limitSphereVisual.Transform = new TranslateTransform3D(point);
        }

        private void addSphere(Point3D center, double radius)
        {
            SphereVisual3D sp = new SphereVisual3D();
            //sp.SetName("");
            sp.Radius = radius;
            sp.Center = center;
            //sp.Fill = Brushes.Red;
            DiffuseMaterial material = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(50, 255, 100, 100)));
            sp.Material = material;
            sp.BackMaterial = material;
            viewport3d.Children.Add(sp);
        }

        private Model3DGroup Initialize_Environment(List<string> modelsNames)
        {
            try
            {
                Model3DGroup RA = new Model3DGroup();
                ModelImporter importer = new ModelImporter();
                joints = new List<Joint>();
                foreach (string str in modelsNames)
                {
                    Color white = Colors.White;
                    MaterialGroup group = new MaterialGroup
                    {
                        Children = {
                            new EmissiveMaterial(new SolidColorBrush(white)),
                            new DiffuseMaterial(new SolidColorBrush(white)),
                            new SpecularMaterial(new SolidColorBrush(white), 200.0)
                        }
                    };
                    Model3DGroup pModel = importer.Load("./3D_Models/" + str, null, false);
                    GeometryModel3D modeld = pModel.Children[0] as GeometryModel3D;
                    modeld.Material = group;
                    modeld.BackMaterial = group;
                    joints.Add(new Joint(pModel));
                }

                RA.Children.Add(joints[0].model);
                RA.Children.Add(joints[1].model);
                RA.Children.Add(joints[2].model);
                RA.Children.Add(joints[3].model);
                RA.Children.Add(joints[4].model);
                RA.Children.Add(joints[5].model);
                RA.Children.Add(joints[6].model);
                RA.Children.Add(joints[7].model);

                Color robotColor = Color.FromArgb(0xff, 150, 150, 150);
                Color robotFlangeColor = Color.FromArgb(0xff, 240, 240, 240);
                Color toolColor = Color.FromArgb(0x00, 60, 60, 60);
                changeModelColor(joints[0], robotColor);
                changeModelColor(joints[1], robotColor);
                changeModelColor(joints[2], robotColor);
                changeModelColor(joints[3], robotColor);
                changeModelColor(joints[4], robotColor);
                changeModelColor(joints[5], robotFlangeColor);
                changeModelColor(joints[6], robotColor);
                changeModelColor(joints[7], toolColor);

                joints[0].angleMin = -160.0;
                joints[0].angleMax = 160.0;
                joints[0].rotAxisX = 0;
                joints[0].rotAxisY = 0;
                joints[0].rotAxisZ = 1;
                joints[0].rotPointX = 0;
                joints[0].rotPointY = 0;
                joints[0].rotPointZ = 0;
                //addSphere(new Point3D(joints[0].rotPointX, joints[0].rotPointY, joints[0].rotPointZ), 20);

                joints[1].angleMin = -137.5;
                joints[1].angleMax = 137.5;
                joints[1].rotAxisX = 0;
                joints[1].rotAxisY = 1;
                joints[1].rotAxisZ = 0;
                joints[1].rotPointX = 150;
                joints[1].rotPointY = 172;
                joints[1].rotPointZ = 550;
                //addSphere(new Point3D(joints[1].rotPointX, joints[1].rotPointY, joints[1].rotPointZ), 20);

                joints[2].angleMin = -150.0;
                joints[2].angleMax = 150.0;
                joints[2].rotAxisX = 0;
                joints[2].rotAxisY = 1;
                joints[2].rotAxisZ = 0;
                joints[2].rotPointX = 150;
                joints[2].rotPointY = 171;
                joints[2].rotPointZ = 1375;
                //addSphere(new Point3D(joints[2].rotPointX, joints[2].rotPointY, joints[2].rotPointZ), 20);

                joints[3].angleMin = -270.0;
                joints[3].angleMax = 270.0;
                joints[3].rotAxisX = 0;
                joints[3].rotAxisY = 0;
                joints[3].rotAxisZ = 1;
                joints[3].rotPointX = 150;
                joints[3].rotPointY = 0;
                joints[3].rotPointZ = 1508;
                //addSphere(new Point3D(joints[3].rotPointX, joints[3].rotPointY, joints[3].rotPointZ), 20);

                joints[4].angleMin = -105.0;
                joints[4].angleMax = 120.0;
                joints[4].rotAxisX = 0;
                joints[4].rotAxisY = 1;
                joints[4].rotAxisZ = 0;
                joints[4].rotPointX = 150;
                joints[4].rotPointY = -1;
                joints[4].rotPointZ = 2300;
                //addSphere(new Point3D(joints[4].rotPointX, joints[4].rotPointY, joints[4].rotPointZ), 20);

                joints[5].angleMin = -270.0;
                joints[5].angleMax = 270.0;
                joints[5].rotAxisX = 0;
                joints[5].rotAxisY = 0;
                joints[5].rotAxisZ = 1;
                joints[5].rotPointX = 150;
                joints[5].rotPointY = 0;
                joints[5].rotPointZ = 2396;
                //addSphere(new Point3D(joints[5].rotPointX, joints[5].rotPointY, joints[5].rotPointZ), 20);

                return RA;
            }
            catch (Exception exception)
            {
                Console.WriteLine("Helix3DSim Initialize Exception Error:" + exception.StackTrace);
                return null;
            }
        }

        private Color changeModelColor(Joint pJoint, Color newColor)
        {
            Model3DGroup model = (Model3DGroup)pJoint.model;
            return changeModelColor(model.Children[0] as GeometryModel3D, newColor);
        }

        private Color changeModelColor(GeometryModel3D pModel, Color newColor)
        {
            if (pModel == null)
                return oldColor;

            Color previousColor = Colors.Red;

            try
            {
                //Console.WriteLine("Model Material: " + pModel.Material.GetType());
                if (pModel.Material is MaterialGroup)
                {
                    MaterialGroup mg = (MaterialGroup)pModel.Material;
                    if (mg.Children.Count > 0)
                    {
                        previousColor = ((EmissiveMaterial)mg.Children[0]).Color;
                        ((EmissiveMaterial)mg.Children[0]).Color = newColor;
                        ((DiffuseMaterial)mg.Children[1]).Color = newColor;
                    }
                }
                else
                {
                    DiffuseMaterial dm = (DiffuseMaterial)pModel.Material;
                    previousColor = dm.Color;
                    dm.Color = newColor;
                }
            }
            catch (Exception exc)
            {
                previousColor = oldColor;
                Console.WriteLine("Change Color Exception Error:" + exc.StackTrace);
            }
            return previousColor;
        }

        private void selectModel(Model3D pModel)
        {
            try
            {
                Model3DGroup group = (Model3DGroup)pModel;
                oldSelectedModel = group.Children[0] as GeometryModel3D;
            }
            catch (Exception exception)
            {
                oldSelectedModel = (GeometryModel3D)pModel;
                Console.WriteLine("Select Model Exception Error:" + exception.StackTrace);
            }
            oldColor = changeModelColor(oldSelectedModel, ColorHelper.HexToColor("#ff3333"));
            CombinedManipulator manipulator = new CombinedManipulator();
        }

        private void unselectModel()
        {
            changeModelColor(oldSelectedModel, oldColor);
        }

        public Model3DGroup Load(string path)
        {
            Model3DGroup group2;
            if (ReferenceEquals(path, null))
            {
                group2 = null;
            }
            else
            {
                Model3DGroup group = null;
                string str2 = Path.GetExtension(path).ToLower();
                if (str2 == ".3ds")
                {
                    group = new StudioReader(null).Read(path);
                }
                else if (str2 == ".lwo")
                {
                    group = new LwoReader(null).Read(path);
                }
                else if (str2 == ".obj")
                {
                    group = new ObjReader(null).Read(path);
                }
                else if (str2 == ".objz")
                {
                    group = new ObjReader(null).ReadZ(path);
                }
                else if (str2 == ".stl")
                {
                    group = new StLReader(null).Read(path);
                }
                else
                {
                    if (str2 != ".off")
                    {
                        throw new InvalidOperationException("File format not supported.");
                    }
                    group = new OffReader(null).Read(path);
                }
                group2 = group;
            }
            return group2;
        }

        public void OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (bCurveSelected)
                lineBezier[nCurveSelected].Color = Colors.Green;

            Point mousePos = e.GetPosition(viewport3d);

            //Console.WriteLine($"2-Point X: {mousePos.X}, Y:{mousePos.Y}");

            Point3D? nullable = viewport3d.FindNearestPoint(e.GetPosition(viewport3d));
            Visual3D visual = viewport3d.FindNearestVisual(e.GetPosition(viewport3d));
            HitTestResult rslt = VisualTreeHelper.HitTest(viewport3d, mousePos);
            PointHitTestParameters hitParams = new PointHitTestParameters(mousePos);
            if (visual == null)
            {
                // Select point
                bSelected = false;
                nSelected = -1;

                // Select curve
                //if(bCurveSelected)
                //    lineBezier[nCurveSelected].Color = Colors.Green;

                bCurveSelected = false;
                nCurveSelected = -1;

                parentWindow.btn_Add_Arc.IsEnabled = false;
                parentWindow.btn_Add_Linear.IsEnabled = false;
            }
            else if (rslt.VisualHit.GetType() == typeof(BezierMarkPoint))
            {
                bSelected = true;
                BezierMarkPoint visualHit = (BezierMarkPoint)rslt.VisualHit;
                Point3D point = visualHit.point;
                strSelected = visualHit.name;
                ptSelected = visualHit.point;
                nSelected = visualHit.number;
                if (strSelected == "point")
                {
                    int num = 0;
                    while (true)
                    {
                        if (num < movePaths[nSelected].ptCurve.Count)
                        {
                            if (!(ptSelected == movePaths[nSelected].ptCurve[num].point))
                            {
                                num++;
                                continue;
                            }
                            nPoint = num;
                        }
                        break;
                    }
                }
                else if (strSelected == "control")
                {
                    int num2 = 0;
                    while (true)
                    {
                        if (num2 < movePaths[nSelected].ptAdjust.Count)
                        {
                            if (!(ptSelected == movePaths[nSelected].ptAdjust[num2].point))
                            {
                                num2++;
                                continue;
                            }
                            nPoint = num2;
                        }
                        break;
                    }
                }
                ///////////  Display info on UI  ////////////
                if (strSelected == "point")
                {
                    Point3D tt = new Point3D();
                    tt.X = targetPoint.point.X - ptSelected.X;
                    tt.Y = targetPoint.point.Y - ptSelected.Y;
                    tt.Z = targetPoint.point.Z - ptSelected.Z;
                    double radius = Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y + tt.Z * tt.Z);
                    double theta = Math.Atan2(tt.Y, tt.X) * 57.3;
                    double fai = Math.Atan2(Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y), tt.Z) * 57.3;
                    parentWindow.tb_sim_pointdata_id.Text = (string)Convert.ToString(nSelected + 1);
                    if (nSelected == 0 && nPoint == 0)
                        parentWindow.tb_sim_pointdata_id.Text = (string)Convert.ToString(nSelected);
                    parentWindow.tb_sim_selpoint_x.Text = (string)Convert.ToString((int)ptSelected.X);
                    parentWindow.tb_sim_selpoint_y.Text = (string)Convert.ToString((int)ptSelected.Y);
                    parentWindow.tb_sim_selpoint_z.Text = (string)Convert.ToString((int)ptSelected.Z);
                    //parentWindow.tb_sim_selpoint_PIPDistance.Text = (string)Convert.ToString((int)CustomSpline.GetDistance(ptSelected, targetPoint.point));
                    parentWindow.tb_sim_selpoint_rx.Text = (string)Convert.ToString((int)radius);
                    parentWindow.tb_sim_selpoint_ry.Text = (string)Convert.ToString((int)theta);
                    parentWindow.tb_sim_selpoint_rz.Text = (string)Convert.ToString((int)fai);
                }
            }
            else if (rslt.VisualHit.GetType() == typeof(Trajectory))
            {
                bSelected = false;
                bCurveSelected = true;
                Trajectory visualHit = (Trajectory)rslt.VisualHit;
                nCurveSelected = visualHit.number;

                lineBezier[nCurveSelected].Color = Colors.Red;
                parentWindow.btn_Add_Arc.IsEnabled = true;
                parentWindow.btn_Add_Linear.IsEnabled = true;
            }
            ////////////////////////////////////////////////////////

            if (visual != null)
            {
                if (rslt.VisualHit.GetType() == typeof(BezierMarkPoint))
                {
                    Visual3DCollection visualGroup = viewport3d.Children;
                    int count = visualGroup.Count;
                    for (int i = count - 1; i >= 0; i--)
                    {
                        String name = visualGroup[i].GetName();
                        if (name == "PIP_Manipulator")
                            continue;
                        if (visualGroup[i].GetType() == typeof(PointTranslateManipulator))
                            viewport3d.Children.RemoveAt(i);
                    }
                    if (visualGroup.Contains(trackLine) && strSelected == "TargetPoint")
                        viewport3d.Children.Remove(trackLine);
                    bMark = true;

                    bSelected = true;
                    BezierMarkPoint mVisual3D = (BezierMarkPoint)rslt.VisualHit;
                    Point3D center = mVisual3D.Center;
                    Console.WriteLine($"BezierMarkPoint center: X:{center.X}, Y:{center.Y}, Z:{center.Z}");

                    PointTranslateManipulator man_x = new PointTranslateManipulator();
                    PointTranslateManipulator man_y = new PointTranslateManipulator();
                    PointTranslateManipulator man_z = new PointTranslateManipulator();
                    if (strSelected == "control" && movePaths[nSelected].shape == "arc")
                    {
                        Vector3D directRadius = new Vector3D();
                        int i = 0, j = 0;
                        if (nSelected == 0)
                            i = 1;
                        if (nSelected == 1)
                            j = 1;
                        directRadius.X = (movePaths[nSelected].ptCurve[i].point.X + movePaths[nSelected + i - 1].ptCurve[j].point.X) / 2 -
                            ptSelected.X;
                        directRadius.Y = (movePaths[nSelected].ptCurve[i].point.Y + movePaths[nSelected + i - 1].ptCurve[j].point.Y) / 2 -
                            ptSelected.Y;
                        directRadius.Z = (movePaths[nSelected].ptCurve[i].point.Z + movePaths[nSelected + i - 1].ptCurve[j].point.Z) / 2 -
                            ptSelected.Z;
                        directRadius.Normalize();

                        man_x.Direction = directRadius;
                        man_x.Length = 200;
                        man_x.Diameter = 20;
                        man_x.Color = Colors.Red;
                        man_x.Offset = new Vector3D(30, 0, 0);
                        man_x.Transform = mVisual3D.Transform;
                        man_x.TargetTransform = mVisual3D.Transform;
                        man_x.Bind(mVisual3D);
                        viewport3d.Children.Add(man_x);

                        Vector3D dd = new Vector3D();
                        dd.X = movePaths[nSelected].ptCurve[i].point.X - movePaths[nSelected + i - 1].ptCurve[j].point.X;
                        dd.Y = movePaths[nSelected].ptCurve[i].point.Y - movePaths[nSelected + i - 1].ptCurve[j].point.Y;
                        dd.Z = movePaths[nSelected].ptCurve[i].point.Z - movePaths[nSelected + i - 1].ptCurve[j].point.Z;
                        directRadius = Vector3D.CrossProduct(directRadius, dd);
                        directRadius.Normalize();
                        man_y.Direction = directRadius;
                        man_y.Length = 200;
                        man_y.Diameter = 20;
                        man_y.Color = Colors.Blue;
                        man_y.Offset = new Vector3D(30, 0, 0);
                        man_y.Transform = mVisual3D.Transform;
                        man_y.TargetTransform = mVisual3D.Transform;
                        man_y.Bind(mVisual3D);
                        viewport3d.Children.Add(man_y);
                    }
                    else
                    {
                        man_x.Direction = new Vector3D(1, 0, 0);
                        man_x.Length = 200;
                        man_x.Diameter = 20;
                        man_x.Color = Colors.Red;
                        man_x.Offset = new Vector3D(30, 0, 0);
                        man_x.Transform = mVisual3D.Transform;
                        man_x.TargetTransform = mVisual3D.Transform;
                        man_x.Bind(mVisual3D);

                        man_y.Direction = new Vector3D(0, 1, 0);
                        man_y.Length = 200;
                        man_y.Diameter = 20;
                        man_y.Color = Colors.Green;
                        man_y.Offset = new Vector3D(0, 30, 0);
                        man_y.Position = mVisual3D.Center;
                        man_y.Transform = mVisual3D.Transform;
                        man_y.TargetTransform = mVisual3D.Transform;
                        man_y.Bind(mVisual3D);

                        man_z.Direction = new Vector3D(0, 0, 1);
                        man_z.Length = 200;
                        man_z.Diameter = 20;
                        man_z.Color = Colors.Blue;
                        man_z.Offset = new Vector3D(mVisual3D.Center.X, mVisual3D.Center.Y, mVisual3D.Center.Z);
                        man_z.Position = mVisual3D.Center;
                        man_z.Transform = mVisual3D.Transform;
                        man_z.TargetTransform = mVisual3D.Transform;
                        man_z.Bind(mVisual3D);

                        viewport3d.Children.Add(man_x);
                        viewport3d.Children.Add(man_y);
                        viewport3d.Children.Add(man_z);
                    }

                    nSelected = mVisual3D.number;
                    Console.WriteLine($"nSelected: {nSelected}");
                }
            }
            else
            {
                bSelected = false;
                Visual3DCollection visualGroup = viewport3d.Children;
                int count = visualGroup.Count;
                for (int i = count - 1; i >= 0; i--)
                {
                    String name = visualGroup[i].GetName();
                    if (name == "PIP_Manipulator")
                        continue;
                    if (visualGroup[i].GetType() == typeof(PointTranslateManipulator))
                        viewport3d.Children.RemoveAt(i);
                }
                bMark = true;
                nSelected = -1;
            }
        }

        public void OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            //bSelected = false;
        }

        public void OnMouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
        }

        private HitTestResultBehavior HitResultCallback(HitTestResult result)
        {
            RayHitTestResult result2 = result as RayHitTestResult;
            if ((result2 != null) && (result2 is RayMeshGeometry3DHitTestResult))
            {
            }
            return HitTestResultBehavior.Continue;
        }

        public static void onUpdataGraphics(Point3D pt)
        {
            double dist = CustomSpline.GetDistance(pt, limitSphereCenterPoint);
            if(dist >= limitSphereRadius && bInSphere)
            {
                bInSphere = false;
                limitSpherePoint.X = pt.X * 0.9;
                limitSpherePoint.Y = pt.Y * 0.9;
                limitSpherePoint.Z = pt.Z * 0.9;
            }
            if (dist < limitSphereRadius)
            {
                bInSphere = true;
                limitSpherePoint = pt;
            }
            jointChanged = true;

            parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            parentWindow.btn_sim_PointNext.IsEnabled = false;
            
            //Ray3D ray = viewport3d.Viewport.Point2DtoRay3D(e.GetPosition(viewport3d));
            Point3D tempPt = new Point3D();  // Track the position of mouse
            //if (ray != null)
            //{
            //    tempPt = ray.PlaneIntersection(ptSelected, ray.Direction).Value;
            //}
            tempPt = limitSpherePoint;

            if (strSelected == "point")
            {
                movePaths[nSelected].ptCurve[nPoint].point = tempPt;
                movePaths[nSelected].ptCurve[nPoint].Transform = new TranslateTransform3D(new Vector3D(tempPt.X, tempPt.Y, tempPt.Z));
            }
            else if (strSelected == "control")
            {
                movePaths[nSelected].ptAdjust[nPoint].Transform = new TranslateTransform3D(new Vector3D(tempPt.X, tempPt.Y, tempPt.Z));
                movePaths[nSelected].ptAdjust[nPoint].point = tempPt;
            }

            if (curveCount == 1 && movePaths[0].ptCurve.Count < 2)
                return;

            viewport3d.Children.Remove(lineBezier[nSelected]);
            lineBezier[nSelected] = null;
            movePaths[nSelected].curvePath.Clear();
            if (movePaths[nSelected].shape == "bezier")
            {
                if (nSelected == 0)
                {
                    viewport3d.Children.Remove(movePaths[nSelected].ctrlLine[nPoint]);
                    movePaths[nSelected].ctrlLine[nPoint] = null;
                }
                else if (strSelected == "point")
                {
                    viewport3d.Children.Remove(movePaths[nSelected].ctrlLine[1]);
                    movePaths[nSelected].ctrlLine[1] = null;
                }
                else
                {
                    viewport3d.Children.Remove(movePaths[nSelected].ctrlLine[nPoint]);
                    movePaths[nSelected].ctrlLine[nPoint] = null;
                }
            }

            if (nSelected == 0)
            {
                if ((movePaths[nSelected].shape == "bezier") && ((nPoint == 1) && (curveCount > (nSelected + 1))))
                {
                    if ((strSelected == "point") || linkHandle)
                    {
                        if (movePaths[nSelected + 1].shape == "bezier")
                        {
                            viewport3d.Children.Remove(movePaths[nSelected + 1].ctrlLine[0]);
                            movePaths[nSelected + 1].ctrlLine[0] = null;
                        }
                        viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                        lineBezier[nSelected + 1] = null;
                        movePaths[nSelected + 1].curvePath.Clear();
                    }
                    if (linkHandle && (movePaths[nSelected + 1].shape == "bezier"))
                    {
                        double[] numArray = new double[] { (2.0 * movePaths[nSelected].ptCurve[1].point.X) - movePaths[nSelected].ptAdjust[1].point.X, (2.0 * movePaths[nSelected].ptCurve[1].point.Y) - movePaths[nSelected].ptAdjust[1].point.Y, (2.0 * movePaths[nSelected].ptCurve[1].point.Z) - movePaths[nSelected].ptAdjust[1].point.Z };
                        movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(numArray[0], numArray[1], numArray[2]));
                        movePaths[nSelected + 1].ptAdjust[0].point = new Point3D(numArray[0], numArray[1], numArray[2]);
                    }
                    if (movePaths[nSelected + 1].shape == "arc" && strSelected == "point")
                    {
                        Point3D pp = CustomSpline.getCircleCenter(nSelected + 1);
                        movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                        movePaths[nSelected + 1].ptAdjust[0].point = pp;
                    }
                }
                else if (movePaths[nSelected].shape == "segment" && nPoint == 1 && (curveCount > (nSelected + 1)))
                {
                    viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                    lineBezier[nSelected + 1] = null;
                    movePaths[nSelected + 1].curvePath.Clear();
                    if (movePaths[nSelected + 1].shape == "arc")
                    {
                        Point3D pp = CustomSpline.getCircleCenter(nSelected + 1);
                        movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                        movePaths[nSelected + 1].ptAdjust[0].point = pp;
                    }
                }
                else if (movePaths[nSelected].shape == "arc" && strSelected == "point")
                {
                    Point3D pp = CustomSpline.getCircleCenter(0);
                    movePaths[0].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                    movePaths[0].ptAdjust[0].point = pp;

                    if (nPoint == 1 && curveCount > nSelected + 1)
                    {
                        viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                        lineBezier[nSelected + 1] = null;
                        movePaths[nSelected + 1].curvePath.Clear();
                        if (movePaths[nSelected + 1].shape == "bezier")
                        {
                            viewport3d.Children.Remove(movePaths[nSelected + 1].ctrlLine[0]);
                            movePaths[nSelected + 1].ctrlLine[0] = null;
                        }
                        else if (movePaths[nSelected + 1].shape == "arc")
                        {
                            pp = CustomSpline.getCircleCenter(1);
                            movePaths[1].ptAdjust[1].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                            movePaths[1].ptAdjust[1].point = pp;
                        }
                    }
                }
            }
            else
            {
                if (movePaths[nSelected].shape == "bezier")
                {
                    if ((nPoint == 1) || (strSelected == "point"))
                    {
                        if (curveCount > (nSelected + 1))
                        {
                            if ((strSelected == "point") || linkHandle)
                            {
                                if (movePaths[nSelected + 1].shape == "bezier")
                                {
                                    viewport3d.Children.Remove(movePaths[nSelected + 1].ctrlLine[0]);
                                    movePaths[nSelected + 1].ctrlLine[0] = null;
                                    if (linkHandle)
                                    {
                                        double[] numArray2 = new double[] { (2.0 * movePaths[nSelected].ptCurve[0].point.X) - movePaths[nSelected].ptAdjust[1].point.X,
                                            (2.0 * movePaths[nSelected].ptCurve[0].point.Y) - movePaths[nSelected].ptAdjust[1].point.Y,
                                            (2.0 * movePaths[nSelected].ptCurve[0].point.Z) - movePaths[nSelected].ptAdjust[1].point.Z };
                                        movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(numArray2[0], numArray2[1], numArray2[2]));
                                        movePaths[nSelected + 1].ptAdjust[0].point = new Point3D(numArray2[0], numArray2[1], numArray2[2]);
                                    }
                                }
                                viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                                lineBezier[nSelected + 1] = null;
                                movePaths[nSelected + 1].curvePath.Clear();
                            }
                            if (strSelected == "point" && movePaths[nSelected + 1].shape == "arc")
                            {
                                Point3D pp = CustomSpline.getCircleCenter(nSelected + 1);
                                movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                                movePaths[nSelected + 1].ptAdjust[0].point = pp;
                            }
                        }
                    }
                    if ((nPoint == 0) && (strSelected == "control") && (movePaths[nSelected - 1].shape == "bezier") && linkHandle)
                    {
                        viewport3d.Children.Remove(movePaths[nSelected - 1].ctrlLine[1]);
                        movePaths[nSelected - 1].ctrlLine[1] = null;
                        viewport3d.Children.Remove(lineBezier[nSelected - 1]);
                        lineBezier[nSelected - 1] = null;
                        movePaths[nSelected - 1].curvePath.Clear();
                        double[] numArray3 = new double[3];
                        if (nSelected == 1)
                        {
                            numArray3[0] = (2.0 * movePaths[nSelected - 1].ptCurve[1].point.X) - movePaths[nSelected].ptAdjust[0].point.X;
                            numArray3[1] = (2.0 * movePaths[nSelected - 1].ptCurve[1].point.Y) - movePaths[nSelected].ptAdjust[0].point.Y;
                            numArray3[2] = (2.0 * movePaths[nSelected - 1].ptCurve[1].point.Z) - movePaths[nSelected].ptAdjust[0].point.Z;
                        }
                        else
                        {
                            numArray3[0] = (2.0 * movePaths[nSelected - 1].ptCurve[0].point.X) - movePaths[nSelected].ptAdjust[0].point.X;
                            numArray3[1] = (2.0 * movePaths[nSelected - 1].ptCurve[0].point.Y) - movePaths[nSelected].ptAdjust[0].point.Y;
                            numArray3[2] = (2.0 * movePaths[nSelected - 1].ptCurve[0].point.Z) - movePaths[nSelected].ptAdjust[0].point.Z;
                        }
                        movePaths[nSelected - 1].ptAdjust[1].Transform = new TranslateTransform3D(new Vector3D(numArray3[0], numArray3[1], numArray3[2]));
                        movePaths[nSelected - 1].ptAdjust[1].point = new Point3D(numArray3[0], numArray3[1], numArray3[2]);
                    }
                }
                else if (movePaths[nSelected].shape == "segment")
                {
                    if (curveCount > (nSelected + 1))
                    {
                        viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                        lineBezier[nSelected + 1] = null;
                        movePaths[nSelected + 1].curvePath.Clear();
                        if (movePaths[nSelected + 1].shape == "bezier")
                        {
                            viewport3d.Children.Remove(movePaths[nSelected + 1].ctrlLine[0]);
                            movePaths[nSelected + 1].ctrlLine[0] = null;
                        }
                        if (movePaths[nSelected + 1].shape == "arc")
                        {
                            Point3D pp = CustomSpline.getCircleCenter(nSelected + 1);
                            movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                            movePaths[nSelected + 1].ptAdjust[0].point = pp;
                        }
                    }

                }
                else if (movePaths[nSelected].shape == "arc")
                {
                    if (strSelected == "point")
                    {
                        Point3D pp = CustomSpline.getCircleCenter(nSelected);
                        movePaths[nSelected].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                        movePaths[nSelected].ptAdjust[0].point = pp;

                        if (curveCount > (nSelected + 1))
                        {
                            viewport3d.Children.Remove(lineBezier[nSelected + 1]);
                            lineBezier[nSelected + 1] = null;
                            movePaths[nSelected + 1].curvePath.Clear();
                            if (movePaths[nSelected + 1].shape == "bezier")
                            {
                                viewport3d.Children.Remove(movePaths[nSelected + 1].ctrlLine[0]);
                                movePaths[nSelected + 1].ctrlLine[0] = null;
                            }
                            if (movePaths[nSelected + 1].shape == "arc")
                            {
                                pp = CustomSpline.getCircleCenter(nSelected + 1);
                                movePaths[nSelected + 1].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(pp.X, pp.Y, pp.Z));
                                movePaths[nSelected + 1].ptAdjust[0].point = pp;
                            }
                        }
                    }
                }
            }

            redraw();

            int cc = 0;
            if (movePaths.Count == 1)
                cc = 1;
            double dist1 = CustomSpline.GetDistance(movePaths[0].ptCurve[0].point, movePaths[movePaths.Count - 1].ptCurve[cc].point);

            parentWindow.lbl_total_distance.Content = dist.ToString();
            
        }

        public void makeDefaultBezier(int index, bool redraw)
        {
            double[] temp = { 0, 0, 0 };
            int i = 0;
            if (index == 0)
                i = 1;
            int j = 0;
            if (index == 1)
                j = 1;
            temp[0] = movePaths[index + i - 1].ptCurve[j].point.X +
                (int)(movePaths[index].ptCurve[i].point.X - movePaths[index + i - 1].ptCurve[j].point.X) * .25;
            temp[1] = movePaths[index + i - 1].ptCurve[j].point.Y +
                (int)(movePaths[index].ptCurve[i].point.Y - movePaths[index + i - 1].ptCurve[j].point.Y) * .25;
            temp[2] = movePaths[index + i - 1].ptCurve[j].point.Z +
                (int)(movePaths[index].ptCurve[i].point.Z - movePaths[index + i - 1].ptCurve[j].point.Z) * .25;
            if (redraw)
            {
                if(!bSelected)  // not the case when delete point
                {
                    if (movePaths[index].shape == "bezier")
                    {
                        viewport3d.Children.Remove(movePaths[index].ptAdjust[0]);
                        viewport3d.Children.Remove(movePaths[index].ptAdjust[1]);
                        viewport3d.Children.Remove(movePaths[index].ctrlLine[0]);
                        viewport3d.Children.Remove(movePaths[index].ctrlLine[1]);
                    }
                    else if (movePaths[index].shape == "arc")
                        viewport3d.Children.Remove(movePaths[index].ptAdjust[0]);
                    viewport3d.Children.Remove(lineBezier[index]);
                }
                CustomCurve curve = movePaths[index];
                curve.shape = "bezier";
                movePaths[index] = curve;
                movePaths[index].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(temp[0], temp[1], temp[2]));
                movePaths[index].ptAdjust[0].point = new Point3D(temp[0], temp[1], temp[2]);
                viewport3d.Children.Add(movePaths[index].ptAdjust[0]);
            }
            else
                addSeqPoint(index, "bezier", temp, "control");

            temp[0] = movePaths[index + i - 1].ptCurve[j].point.X +
                (int)(movePaths[index].ptCurve[i].point.X - movePaths[index + i - 1].ptCurve[j].point.X) * .75;
            temp[1] = movePaths[index + i - 1].ptCurve[j].point.Y +
                (int)(movePaths[index].ptCurve[i].point.Y - movePaths[index + i - 1].ptCurve[j].point.Y) * .75;
            temp[2] = movePaths[index + i - 1].ptCurve[j].point.Z +
                (int)(movePaths[index].ptCurve[i].point.Z - movePaths[index + i - 1].ptCurve[j].point.Z) * .75;
            if (redraw)
            {
                movePaths[index].ptAdjust[1].Transform = new TranslateTransform3D(new Vector3D(temp[0], temp[1], temp[2]));
                movePaths[index].ptAdjust[1].point = new Point3D(temp[0], temp[1], temp[2]);
                viewport3d.Children.Add(movePaths[index].ptAdjust[1]);

                drawBezierCurveSegment(index, true);
                drawControlCurveSegment(index, true);
            }
            else
                addSeqPoint(index, "bezier", temp, "control");
        }

        public static void makeArc(int index, bool redraw)
        {
            if (movePaths[index].shape == "bezier")
            {
                viewport3d.Children.Remove(movePaths[index].ptAdjust[0]);
                viewport3d.Children.Remove(movePaths[index].ptAdjust[1]);
                viewport3d.Children.Remove(movePaths[index].ctrlLine[0]);
                viewport3d.Children.Remove(movePaths[index].ctrlLine[1]);
            }
            else if (movePaths[index].shape == "arc" && !redraw)
                return;
            if(!redraw)
            {
                viewport3d.Children.Remove(lineBezier[index]);
                lineBezier[index] = null;
            }
            lineBezier[index] = new Trajectory(index);
            Point3D item = new Point3D();
            List<Point3D> arcPoints = CustomSpline.GetArcPoints(index, redraw);
            CustomCurve curve = movePaths[index];
            curve.shape = "arc";
            curve.curvePath = arcPoints;
            if(!redraw)
            {
                Point3D center = movePaths[index].ptAdjust[0].point;

                movePaths[index].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(center.X, center.Y, center.Z));
                movePaths[index].ptAdjust[0].point = center;

            }

            movePaths[index] = curve;
            for (int i = 0; i < movePaths[index].curvePath.Count - 1; i++)
            {
                lineBezier[index].Color = Colors.Green;
                lineBezier[index].Thickness = 2.0;
                item = movePaths[index].curvePath[i];
                lineBezier[index].Points.Add(item);
                item = movePaths[index].curvePath[i + 1];
                lineBezier[index].Points.Add(item);
            }
            viewport3d.Children.Add(lineBezier[index]);
            if (showHandle && !bSelected)
                viewport3d.Children.Add(movePaths[index].ptAdjust[0]);
        }

        public void drawArc(int index)
        {
            lineBezier.Add(new Trajectory(index));
            Point3D item = new Point3D();
            List<Point3D> arcPoints = CustomSpline.GetArcPoints(index, true);
            CustomCurve curve = movePaths[index];
            curve.shape = "arc";
            curve.curvePath = arcPoints;
            //if (!redraw)
            //{
            //    Point3D center = movePaths[index].ptAdjust[0].point;

            //    movePaths[index].ptAdjust[0].Transform = new TranslateTransform3D(new Vector3D(center.X, center.Y, center.Z));
            //    movePaths[index].ptAdjust[0].point = center;

            //}

            movePaths[index] = curve;
            for (int i = 0; i < movePaths[index].curvePath.Count - 1; i++)
            {
                lineBezier[index].Color = Colors.Green;
                lineBezier[index].Thickness = 2.0;
                item = movePaths[index].curvePath[i];
                lineBezier[index].Points.Add(item);
                item = movePaths[index].curvePath[i + 1];
                lineBezier[index].Points.Add(item);
            }
            viewport3d.Children.Add(lineBezier[index]);
        }

        public static void makeLine(int index)
        {
            if (movePaths[index].shape == "bezier")
            {
                viewport3d.Children.Remove(movePaths[index].ptAdjust[0]);
                viewport3d.Children.Remove(movePaths[index].ptAdjust[1]);
                viewport3d.Children.Remove(movePaths[index].ctrlLine[0]);
                viewport3d.Children.Remove(movePaths[index].ctrlLine[1]);
            }
            else if(movePaths[index].shape=="arc")
                viewport3d.Children.Remove(movePaths[index].ptAdjust[0]);
            viewport3d.Children.Remove(lineBezier[index]);
            lineBezier[index] = null;
            CustomCurve curve = movePaths[index];
            curve.shape = "segment";
            curve.curvePath = new List<Point3D>();
            movePaths[index] = curve;
            lineBezier[index] = new Trajectory(index);
            Point3D item = new Point3D();
            float t = 0f;
            int i = 0, j = 0;
            if (index == 0)
                i = 1;
            if (index == 1)
                j = 1;
            float dist = (float)CustomSpline.GetDistance(movePaths[index].ptCurve[i].point, movePaths[index + i - 1].ptCurve[j].point);
            float step = CustomSpline.spacing / dist;
            while (true)
            {
                if (t > 1f)
                {
                    break;
                }
                item = CustomSpline.GetSegmentPoint(index, t);
                movePaths[index].curvePath.Add(item);
                t += step;
            }
            for (i = 0; i < movePaths[index].curvePath.Count - 1; i++)
            {
                lineBezier[index].Color = Colors.Green;
                lineBezier[index].Thickness = 2.0;
                item = movePaths[index].curvePath[i];
                lineBezier[index].Points.Add(item);
                item = movePaths[index].curvePath[i + 1];
                lineBezier[index].Points.Add(item);
            }
            viewport3d.Children.Add(lineBezier[index]);
        }

        public void drawSegment(int index)
        {
            lineBezier.Add(new Trajectory(index));
            CustomCurve curve = movePaths[index];
            curve.shape = "segment";
            curve.curvePath = new List<Point3D>();
            movePaths[index] = curve;
            lineBezier[index] = new Trajectory(index);
            Point3D item = new Point3D();
            float t = 0f;
            int i = 0, j = 0;
            if (index == 0)
                i = 1;
            if (index == 1)
                j = 1;
            float dist = (float)CustomSpline.GetDistance(movePaths[index].ptCurve[i].point, movePaths[index + i - 1].ptCurve[j].point);
            float step = CustomSpline.spacing / dist;
            while (true)
            {
                if (t > 1f)
                {
                    break;
                }
                item = CustomSpline.GetSegmentPoint(index, t);
                movePaths[index].curvePath.Add(item);
                t += step;
            }
            for (i = 0; i < movePaths[index].curvePath.Count - 1; i++)
            {
                lineBezier[index].Color = Colors.Green;
                lineBezier[index].Thickness = 2.0;
                item = movePaths[index].curvePath[i];
                lineBezier[index].Points.Add(item);
                item = movePaths[index].curvePath[i + 1];
                lineBezier[index].Points.Add(item);
            }
            viewport3d.Children.Add(lineBezier[index]);
        }

        public static void drawBezierCurveSegment(int index, bool redraw)
        {
            CustomCurve curve = movePaths[index];
            curve.curvePath = new List<Point3D>();
            movePaths[index] = curve;
            if (redraw)
            {
                lineBezier[index] = new Trajectory(index);
            }
            else
            {
                lineBezier.Add(new Trajectory(index));      
            }
            Point3D item = new Point3D();
            List<Point3D> pts = CustomSpline.GetBezierPoint(index);
            CustomCurve cur = movePaths[index];
            cur.curvePath = pts;
            movePaths[index] = cur;
            int num2 = 0;
            while (true)
            {
                if (num2 >= (movePaths[index].curvePath.Count - 1))
                {
                    viewport3d.Children.Add(lineBezier[index]);
                    return;
                }
                lineBezier[index].Color = Colors.Green;
                lineBezier[index].Thickness = 2.0;
                item = movePaths[index].curvePath[num2];
                lineBezier[index].Points.Add(item);
                item = movePaths[index].curvePath[num2 + 1];
                lineBezier[index].Points.Add(item);
                num2++;
            }
        }

        public static void drawControlCurveSegment(int index, bool redraw)
        {
            if (movePaths[index].ptAdjust.Count == 2)
            {
                if (redraw)
                {
                    int num = 0;
                    while (true)
                    {
                        if (num >= 2)
                        {
                            break;
                        }
                        if (movePaths[index].ctrlLine != null && movePaths[index].ctrlLine[num] != null)
                        {
                            if(viewport3d.Children.Contains(movePaths[index].ctrlLine[num]))
                                viewport3d.Children.Remove(movePaths[index].ctrlLine[num]);
                            movePaths[index].ctrlLine[num] = null;
                        }
                        num++;
                    }
                }
                CustomCurve curve = new CustomCurve();
                curve = movePaths[index];
                curve.ctrlLine = new List<LinesVisual3D>();
                LinesVisual3D item = new LinesVisual3D {
                    Color = Colors.Blue,
                    Thickness = 2.0,
                    Points = { movePaths[index].ptAdjust[0].point }
                };
                if (index == 0)
                {
                    item.Points.Add(movePaths[index].ptCurve[0].point);
                }
                else if (index == 1)
                {
                    item.Points.Add(movePaths[index - 1].ptCurve[1].point);
                }
                else
                {
                    item.Points.Add(movePaths[index - 1].ptCurve[0].point);
                }
                curve.ctrlLine.Add(item);
                item = null;
                item = new LinesVisual3D {
                    Color = Colors.Blue,
                    Thickness = 2.0,
                    Points = { movePaths[index].ptAdjust[1].point }
                };
                if (index == 0)
                {
                    item.Points.Add(movePaths[index].ptCurve[1].point);
                }
                else
                {
                    item.Points.Add(movePaths[index].ptCurve[0].point);
                }
                curve.ctrlLine.Add(item);
                if (showHandle)
                {
                    viewport3d.Children.Add(curve.ctrlLine[0]);
                    viewport3d.Children.Add(curve.ctrlLine[1]);
                }
                movePaths[index] = curve;
            }
            if(movePaths[index].shape=="arc")
            {
                lineBezier[index] = new Trajectory(index);
                CustomCurve curve = new CustomCurve();
                List<Point3D> arcPoints = new List<Point3D>();
                arcPoints = CustomSpline.GetArcPoints(index, true);
                curve = movePaths[index];
                curve.curvePath = arcPoints;
                movePaths[index] = curve;

                Point3D pt = new Point3D();
                for (int i = 0; i < movePaths[index].curvePath.Count - 1; i++)
                {
                    lineBezier[index].Color = Colors.Green;
                    lineBezier[index].Thickness = 2;
                    pt = movePaths[index].curvePath[i];
                    lineBezier[index].Points.Add(pt);
                    pt = movePaths[index].curvePath[i + 1];
                    lineBezier[index].Points.Add(pt);
                }
                viewport3d.Children.Add(lineBezier[index]);
            }
        }

        public void addSeqPoint(int id, string curveShape, double[] p, string name)
        {
            CustomCurve curve;
            Color mainColor = new Color();
            if (name == "point")
                mainColor = Colors.LimeGreen;
            else if (name == "control")
                mainColor = Colors.Blue;
            var materialGroup = new MaterialGroup();
            materialGroup.Children.Add(new EmissiveMaterial(new SolidColorBrush(mainColor)));
            materialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(mainColor)));
            materialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(mainColor), 200));


            BezierMarkPoint item = new BezierMarkPoint(id, name, new Point3D(p[0], p[1], p[2]))
            {
                Center = new Point3D(0, 0, 0),
                Radius = 15,
                Material = materialGroup
            };


            int text_offset = 30;
            var textvis3d = new BillboardTextVisual3D()
            {
                Position = new Point3D(p[0] + text_offset, p[1] + text_offset, p[2] + (text_offset * 2)),
                Foreground = Brushes.White,
                Text = string.Format("{0}", id) // p[0], p[1], p[2],
            };

            item.Transform = new TranslateTransform3D(new Vector3D(p[0], p[1], p[2]));

            if (curveCount != 0)
            {
                if ((curveCount != 1) || (id != 0))
                {
                    if (name == "point")
                    {
                        curve = new CustomCurve {
                            shape = curveShape,
                            ptCurve = new List<BezierMarkPoint>()
                        };
                        curve.ptCurve.Add(item);
                        movePaths.Add(curve);
                    }
                    if (name == "control")
                    {
                        curve = movePaths[id];
                        if (ReferenceEquals(curve.ptAdjust, null))
                        {
                            curve.ptAdjust = new List<BezierMarkPoint>();
                        }
                        curve.ptAdjust.Add(item);
                        movePaths[id] = curve;
                    }
                }
                else
                {
                    if (name == "point")
                    {
                        movePaths[0].ptCurve.Add(item);
                    }
                    if (name == "control")
                    {
                        curve = movePaths[0];
                        if (ReferenceEquals(curve.ptAdjust, null))
                        {
                            curve.ptAdjust = new List<BezierMarkPoint>();
                        }
                        curve.ptAdjust.Add(item);
                        movePaths[0] = curve;
                    }
                }
            }
            else
            {
                curve = new CustomCurve
                {
                    shape = curveShape,
                    ptCurve = new List<BezierMarkPoint>()
                };
                curve.ptCurve.Add(item);
                movePaths.Add(curve);
            }
            if (showHandle || name == "point")
            {
                viewport3d.Children.Add(item);
            }
            bool bFlag = movePaths[id].ptAdjust != null && movePaths[id].ptAdjust.Count == 2;
            if (bFlag)
            {
                drawBezierCurveSegment(id, false);
                drawControlCurveSegment(id, false);
            }
        }

        public void clearSeqPoints()
        {
            foreach (SphereVisual3D visuald in sequence_points)
            {
                viewport3d.Children.Remove(visuald);
            }
            foreach (BillboardTextVisual3D visuald2 in sequence_pointsText)
            {
                viewport3d.Children.Remove(visuald2);
            }
            sequence_points.Clear();
            sequence_pointsText.Clear();
        }

        public double DistanceFromTarget(Vector3D target, double[] angles)
        {
            Vector3D vectord = ForwardKinematics(angles);
            return Math.Sqrt((Math.Pow(vectord.X - target.X, 2.0) + Math.Pow(vectord.Y - target.Y, 2.0)) + Math.Pow(vectord.Z - target.Z, 2.0));
        }

        private bool checkAngles(double[] oldAngles, double[] angles)
        {
            int index = 0;
            while (true)
            {
                bool flag2;
                if (index > 5)
                {
                    flag2 = true;
                }
                else
                {
                    if (oldAngles[index] == angles[index])
                    {
                        index++;
                        continue;
                    }
                    flag2 = false;
                }
                return flag2;
            }
        }

        private static T Clamp<T>(T value, T min, T max) where T: IComparable<T>
        {
            T local = value;
            if (value.CompareTo(max) > 0)
            {
                local = max;
            }
            if (value.CompareTo(min) < 0)
            {
                local = min;
            }
            return local;
        }

        /// <summary>
        /// Forward Kinematic Function
        /// </summary>
        /// <param name="angles">Joint angles</param>
        /// <returns>Tool Cartesian Position</returns>
        public Vector3D ForwardKinematics(double[] angles)
        {
            // TODO: Check new joint angles within range limits
            // Update Joints Angles
            joints[0].angle = angles[0];
            joints[1].angle = angles[1];
            joints[2].angle = angles[2];
            joints[3].angle = angles[3];
            joints[4].angle = angles[4];
            joints[5].angle = angles[5];
            
            // Setup 3D Model Transformations
            F1 = new Transform3DGroup();
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[0].rotAxisX, (double) joints[0].rotAxisY, (double) joints[0].rotAxisZ), angles[0]), new Point3D((double) joints[0].rotPointX, (double) joints[0].rotPointY, (double) joints[0].rotPointZ));
            F1.Children.Add(R);
            F2 = new Transform3DGroup();
            T = new TranslateTransform3D(0.0, 0.0, 0.0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[1].rotAxisX, (double) joints[1].rotAxisY, (double) joints[1].rotAxisZ), angles[1]), new Point3D((double) joints[1].rotPointX, (double) joints[1].rotPointY, (double) joints[1].rotPointZ));
            F2.Children.Add(T);
            F2.Children.Add(R);
            F2.Children.Add(F1);
            F3 = new Transform3DGroup();
            T = new TranslateTransform3D(0.0, 0.0, 0.0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[2].rotAxisX, (double) joints[2].rotAxisY, (double) joints[2].rotAxisZ), angles[2]), new Point3D((double) joints[2].rotPointX, (double) joints[2].rotPointY, (double) joints[2].rotPointZ));
            F3.Children.Add(T);
            F3.Children.Add(R);
            F3.Children.Add(F2);
            F4 = new Transform3DGroup();
            T = new TranslateTransform3D(0.0, 0.0, 0.0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[3].rotAxisX, (double) joints[3].rotAxisY, (double) joints[3].rotAxisZ), angles[3]), new Point3D((double) joints[3].rotPointX, (double) joints[3].rotPointY, (double) joints[3].rotPointZ));
            F4.Children.Add(T);
            F4.Children.Add(R);
            F4.Children.Add(F3);
            F5 = new Transform3DGroup();
            T = new TranslateTransform3D(0.0, 0.0, 0.0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[4].rotAxisX, (double) joints[4].rotAxisY, (double) joints[4].rotAxisZ), angles[4]), new Point3D((double) joints[4].rotPointX, (double) joints[4].rotPointY, (double) joints[4].rotPointZ));
            F5.Children.Add(T);
            F5.Children.Add(R);
            F5.Children.Add(F4);
            F6 = new Transform3DGroup();
            T = new TranslateTransform3D(0.0, 0.0, 0.0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D((double) joints[5].rotAxisX, (double) joints[5].rotAxisY, (double) joints[5].rotAxisZ), angles[5]), new Point3D((double) joints[5].rotPointX, (double) joints[5].rotPointY, (double) joints[5].rotPointZ));
            F6.Children.Add(T);
            F6.Children.Add(R);
            F6.Children.Add(F5);
            
            // Apply 3D Model Transformations
            joints[0].model.Transform = F1;
            joints[1].model.Transform = F2;
            joints[2].model.Transform = F3;
            joints[3].model.Transform = F4;
            joints[4].model.Transform = F5;
            joints[5].model.Transform = F6;
            joints[7].model.Transform = F6;

            // TODO: Apply Tool Position Offset
            toolSphereVisual.Model.Transform = F6;
            
            // TODO: Return X,Y,Z,RX,RY,RZ
            return UpdateToolCurrentCoord();
        }

        public Vector3D UpdateToolCurrentCoord()
        {
            if (bMark)
            {
                bMark = false;
                if (!viewport3d.Children.Contains(arrow_PIP))
                    viewport3d.Children.Add(arrow_PIP);
            }

            Rect3D b = toolSphereVisual.Model.Bounds; //joints[5].model.Bounds;

            // Update tool position global variable
            toolPosition = new Vector3D(b.X + b.SizeX / 2, b.Y + b.SizeY / 2, b.Z + b.SizeZ / 2);

            // Calculate direction between tool and target points
            Vector3D direction = new Vector3D(0, 0, 0);
            direction.X = targetPoint.point.X - toolPosition.X;
            direction.Y = targetPoint.point.Y - toolPosition.Y;
            direction.Z = targetPoint.point.Z - toolPosition.Z;
            direction.Normalize();

            // Update arrow direction and position
            arrow_PIP.Direction = direction;
            arrow_PIP.Position = toolPosition.ToPoint3D();

            // Update track line between tool and target
            trackLine.Points[0] = targetPoint.point;
            trackLine.Points[1] = toolPosition.ToPoint3D();

            UpdateTargetDistance();

            return toolPosition;
        }

        public static void UpdateTargetDistance()
        {
            targetDistance = CustomSpline.GetDistance(toolPosition.ToPoint3D(), targetPoint.point);
            parentWindow.tb_sim_TargetDistance.Text = Math.Round(targetDistance, 2).ToString();
        }

        // TODO: Create Cartesian type Overload
        public void UpdateRoboticArm(double j1, double j2, double j3, double j4, double j5, double j6)
        {
            Vector3D pos = ForwardKinematics(new double[] { j1, j2, j3, j4, j5, j6 });
            if (IsPathTracingEnabled)
            {
                addTracePoint3D(new TracePoint3D(new Point3D(toolPosition.X, toolPosition.Y, toolPosition.Z), Color.FromArgb(200, 0xff, 0, 0), 1.0));
            }
            UpdateUICoordinates();
        }

        public void UpdateUICoordinates()
        {
            IntPtr outFK = Marshal.AllocHGlobal(6 * Marshal.SizeOf<double>());
            MainWindow.rlForwardKinematics(joints[0].angle,
                joints[1].angle,
                joints[2].angle,
                joints[3].angle,
                joints[4].angle,
                joints[5].angle,
                outFK);
            double[] arrFK = new double[6];
            Marshal.Copy(outFK, arrFK, 0, 6);
            Marshal.FreeHGlobal(outFK);

            parentWindow.slider_simPos_x.Value = arrFK[0];
            parentWindow.slider_simPos_y.Value = arrFK[1];
            parentWindow.slider_simPos_z.Value = arrFK[2];
            parentWindow.slider_simPos_rx.Value = arrFK[3];
            parentWindow.slider_simPos_ry.Value = arrFK[4];
            parentWindow.slider_simPos_rz.Value = arrFK[5];

            parentWindow.slider_simPos_J1.Value = joints[0].angle;
            parentWindow.slider_simPos_J2.Value = joints[1].angle;
            parentWindow.slider_simPos_J3.Value = joints[2].angle;
            parentWindow.slider_simPos_J4.Value = joints[3].angle;
            parentWindow.slider_simPos_J5.Value = joints[4].angle;
            parentWindow.slider_simPos_J6.Value = joints[5].angle;
        }

        public double[] getAngleFromPosition(Vector3D target, double[] angles)
        {
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
            {
                //movements = 0;
                return angles;
            }

            double[] oldAngles = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            int index = 0;
            while (true)
            {
                index++;
                angles.CopyTo(oldAngles, 0);
                for (int i = 0; i <= 5; i++)
                {
                    // Gradient descent
                    // Update : Solution -= LearningRate * Gradient
                    double gradient = PartialGradient(target, angles, i);
                    angles[i] -= LearningRate * gradient;

                    // Clamp
                    angles[i] = Clamp(angles[i], joints[i].angleMin, joints[i].angleMax);

                    Vector3D dd = ForwardKinematics(angles);
                    // Early termination
                    if (DistanceFromTarget(target, angles) < DistanceThreshold || checkAngles(oldAngles, angles))
                    {
                        //movements = 0;
                        return angles;
                    }

                }
                if (count != movePaths.Count - 1)
                    if (index == 8000)
                        return angles;
            }
        }

        private double[] InverseKinematics(Vector3D target, double[] angles)
        {
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
            {
                //movements = 0;
                return angles;
            }

            double[] oldAngles = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            angles.CopyTo(oldAngles, 0);
            for (int i = 0; i <= 5; i++)
            {
                // Gradient descent
                // Update : Solution -= LearningRate * Gradient
                double gradient = PartialGradient(target, angles, i);
                angles[i] -= LearningRate * gradient;

                // Clamp
                angles[i] = Clamp(angles[i], joints[i].angleMin, joints[i].angleMax);

                // Early termination
                if (DistanceFromTarget(target, angles) < DistanceThreshold || checkAngles(oldAngles, angles))
                {
                    //movements = 0;
                    return angles;
                }
            }

            return angles;
        }

        private double PartialGradient(Vector3D target, double[] angles, int i)
        {
            // Saves the angle,
            // it will be restored later
            double angle = angles[i];

            // Gradient : [F(x+SamplingDistance) - F(x)] / h
            double f_x = DistanceFromTarget(target, angles);

            angles[i] += SamplingDistance;
            double f_x_plus_d = DistanceFromTarget(target, angles);

            double gradient = (f_x_plus_d - f_x) / SamplingDistance;

            // Restores
            angles[i] = angle;

            return gradient;
        }

        /// <summary>
        /// Forward Kinematics Robotics Library DLL Function Call
        /// </summary>
        public double[] ForwardKinematics(double j1, double j2, double j3, double j4, double j5, double j6)
        {
            double[] retArr = new double[6];
            IntPtr outPtr = Marshal.AllocHGlobal(6 * Marshal.SizeOf<double>());
            MainWindow.rlForwardKinematics(j1, j2, j3, j4, j5, j6, outPtr);
            Marshal.Copy(outPtr, retArr, 0, 6);
            Marshal.FreeHGlobal(outPtr);
            return retArr;
            /*
            X.Text = Math.Round(retArr[0], 3, MidpointRounding.ToEven).ToString(); 
            Y.Text = Math.Round(retArr[1], 3, MidpointRounding.ToEven).ToString();
            Z.Text = Math.Round(retArr[2], 3, MidpointRounding.ToEven).ToString();
            A.Text = Math.Round(retArr[3], 3, MidpointRounding.ToEven).ToString();
            B.Text = Math.Round(retArr[4], 3, MidpointRounding.ToEven).ToString();
            C.Text = Math.Round(retArr[5], 3, MidpointRounding.ToEven).ToString();
            */
        }

        /// <summary>
        /// Inverse Kinematics Robotics Library DLL Function Call
        /// </summary>
        public double[] InverseKinematics(double x, double y, double z, double rx, double ry, double rz)
        {
            double[] jointAngles = new double[6] { joints[0].angle, joints[1].angle, joints[2].angle,
                joints[3].angle, joints[4].angle, joints[5].angle };
            int size = Marshal.SizeOf(jointAngles[0]) * jointAngles.Length;
            IntPtr intPtr = Marshal.AllocHGlobal(size);
            Marshal.Copy(jointAngles, 0, intPtr, jointAngles.Length);

            double[] retArr = new double[6];
            IntPtr outPtr = Marshal.AllocHGlobal(6 * Marshal.SizeOf<double>());
            MainWindow.rlInverseKinematics(intPtr, x, y, z, rx, ry, rz, outPtr);
            Marshal.Copy(outPtr, retArr, 0, 6);
            Marshal.FreeHGlobal(outPtr);
            Marshal.FreeHGlobal(intPtr);
            return retArr;
            /*
            J1.Text = Math.Round(retArr[0], 3, MidpointRounding.ToEven).ToString(); 
            J2.Text = Math.Round(retArr[1], 3, MidpointRounding.ToEven).ToString();
            J3.Text = Math.Round(retArr[2], 3, MidpointRounding.ToEven).ToString();
            J4.Text = Math.Round(retArr[3], 3, MidpointRounding.ToEven).ToString();
            J5.Text = Math.Round(retArr[4], 3, MidpointRounding.ToEven).ToString();
            J6.Text = Math.Round(retArr[5], 3, MidpointRounding.ToEven).ToString();
            */
        }

        public double[] GetJoints() => new double[] { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle };

        public void goHomePosition()
        {
            UpdateRoboticArm(   simRobotStatus.HomeJointPos[0],
                                simRobotStatus.HomeJointPos[1],
                                simRobotStatus.HomeJointPos[2],
                                simRobotStatus.HomeJointPos[3],
                                simRobotStatus.HomeJointPos[4],
                                simRobotStatus.HomeJointPos[5] ); 
        }

        public void goFirstPathPoint()
        {
            if (jointChanged)
            {
                double[] angles = GetJoints();
                Vector3D vector = new Vector3D(movePaths[0].ptCurve[0].point.X, movePaths[0].ptCurve[0].point.Y, movePaths[0].ptCurve[0].point.Z);
                angles = getAngleFromPosition(vector, angles);
                ptArray[0] = angles;
            }
            UpdateRoboticArm(ptArray[0][0], ptArray[0][1], ptArray[0][2], ptArray[0][3], ptArray[0][4], ptArray[0][5]);
            curPoint = 0;
            parentWindow.btn_sim_PointFirst.IsEnabled = false;
            parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            parentWindow.btn_sim_PointNext.IsEnabled = true;
            parentWindow.btn_sim_PointLast.IsEnabled = true;
        }

        public void goLastPathPoint()
        {
            if (jointChanged)
            {
                double[] angles = GetJoints();
                Vector3D vector = new Vector3D(movePaths[0].ptCurve[0].point.X, movePaths[0].ptCurve[0].point.Y, movePaths[0].ptCurve[0].point.Z);
                angles = getAngleFromPosition(vector, angles);
                ptArray[0] = angles;
            }
            UpdateRoboticArm(ptArray[0][0], ptArray[0][1], ptArray[0][2], ptArray[0][3], ptArray[0][4], ptArray[0][5]);
            curPoint = 0;
            parentWindow.btn_sim_PointFirst.IsEnabled = true;
            parentWindow.btn_sim_PointPrevious.IsEnabled = true;
            parentWindow.btn_sim_PointNext.IsEnabled = false;
            parentWindow.btn_sim_PointLast.IsEnabled = false;
        }

        public void goFirstPathPoint2() // REMOVE?
        {
            resetHelixPlayback();
            double[] angles = { trajJoints[nCurve][count][0],
                                trajJoints[nCurve][count][1],
                                trajJoints[nCurve][count][2],
                                trajJoints[nCurve][count][3],
                                trajJoints[nCurve][count][4],
                                trajJoints[nCurve][count][5] };

            UpdateRoboticArm(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
        }

        #region CALCULATE TRAJECTORY
        public void Calculate()
        {
            parentWindow.UI_SimCalculateBeforeState();
            
            // Initialize the variable for storing the trajectory info and move robot arm to the origin
            if (jointChanged)
            {
                ptArray.Clear();
                ptArray = new List<double[]>();
                double[] angles = GetJoints();
                Vector3D vector = new Vector3D(movePaths[0].ptCurve[0].point.X, movePaths[0].ptCurve[0].point.Y, movePaths[0].ptCurve[0].point.Z);
                angles = getAngleFromPosition(vector, angles);
                // TODO angles = InverseKinematics(vector, angles);
                ptArray.Add(angles);
            }
            if (trajJoints.Count > 0)
                trajJoints.Clear();
            trajJoints.Add(new List<double[]>());
            trajJoints[0].Add(ptArray[0]);
            nCurve = 0;
            count = 0;
            UpdateRoboticArm(ptArray[0][0], ptArray[0][1], ptArray[0][2], ptArray[0][3], ptArray[0][4], ptArray[0][5]);
            ptArray[0] = trajJoints[0][0];

            // Set the timer to control calculating speed
            timerCalc = new System.Windows.Forms.Timer();
            timerCalc.Interval = 100;
            timerCalc.Tick += new System.EventHandler(timer_Calculate);
            timerCalc.Enabled = true;
        }

        private void timer_Calculate(object sender, EventArgs e)
        {
            double[] angles = calculateTrajectory();
            if (angles == null)
                return;
            if (trajJoints.Count - 1 < nCurve)
                trajJoints.Add(new List<double[]>());
            trajJoints[nCurve].Add(angles);
            //if (count == movePaths[nCurve].curvePath.Count - 1)
            //{
            //    ptArray.Add(angles);
            //}
            parentWindow.slider_simPos_J1.Value = angles[0];
            parentWindow.slider_simPos_J2.Value = angles[1];
            parentWindow.slider_simPos_J3.Value = angles[2];
            parentWindow.slider_simPos_J4.Value = angles[3];
            parentWindow.slider_simPos_J5.Value = angles[4];
            parentWindow.slider_simPos_J6.Value = angles[5];
            totalCount++;
        }

        public double[] calculateTrajectory()
        {
            count++;
            if (count == movePaths[nCurve].curvePath.Count)
            {
                count = 1;
                nCurve++;
            }
            if (nCurve > movePaths.Count - 1)
            {
                jointChanged = false;
                timerCalc.Enabled = false;
                timerCalc = null;
                //Console.WriteLine($"Count: {count}, nCurve: {nCurve}, curPoint: {curPoint}");
                count = 0;
                nCurve = 0;
                curPoint = 0;
                // Reset position to first point in Path
                UpdateRoboticArm(ptArray[0][0], ptArray[0][1], ptArray[0][2], ptArray[0][3], ptArray[0][4], ptArray[0][5]);
                parentWindow.UI_SimCalculateAfterState();
                return null;
            }

            // Total Path Length Calculation Step
            double tempDist = CustomSpline.GetDistance(movePaths[nCurve].curvePath[count], movePaths[nCurve].curvePath[count - 1]);
            double preLength = Convert.ToInt32(parentWindow.lbl_total_length.Content);
            preLength += (int)tempDist;
            parentWindow.lbl_total_length.Content = Convert.ToString(preLength);

            Vector3D temp_target = new Vector3D();
            temp_target.X = movePaths[nCurve].curvePath[count].X; //Current Point on Path X Value 
            temp_target.Y = movePaths[nCurve].curvePath[count].Y; //Current Point on Path Y Value
            temp_target.Z = movePaths[nCurve].curvePath[count].Z; //Current Point on Path Z Value

            //Console.WriteLine($"{temp_target.X} {temp_target.Y} {temp_target.Z}");

            // Inverse Kinematics to get new position Joint angles
            double[] old_angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle };
            //double[] angles = getAngleFromPosition(temp_target, old_angles);
            Vector3D direction = arrow_PIP.Direction;
            double rotY = (Math.PI / 2 - Math.Acos(direction.Z)) * 180 / Math.PI;
            double rotZ = -(Math.PI - Math.Atan(direction.Y / direction.X)) * 180 / Math.PI;
            double[] angles = InverseKinematics(temp_target.X, temp_target.Y, temp_target.Z, 180, rotY, rotZ);


            //Rect3D b = toolSphereVisual.Model.Bounds; //joints[5].model.Bounds;
            //// Update tool position global variable
            //toolPosition = new Vector3D(b.X + b.SizeX / 2, b.Y + b.SizeY / 2, b.Z + b.SizeZ / 2);
            //// Calculate direction between tool and target points
            //Vector3D direction = new Vector3D(0, 0, 0);
            //direction.X = targetPoint.point.X - toolPosition.X;
            //direction.Y = targetPoint.point.Y - toolPosition.Y;
            //direction.Z = targetPoint.point.Z - toolPosition.Z;
            //direction.Normalize();
            //double angleY = Math.Acos(direction.X);


            //double[] angles = InverseKinematics(temp_target.X, temp_target.Y, temp_target.Z, 180, angleY*180/3.14, 180);
            // double[] angles = InverseKinematics(temp_target, old_angles);

            // Move Robotic Arm to current point in Path
            UpdateRoboticArm(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            
            if (count == movePaths[nCurve].curvePath.Count - 1)
            {
                if (jointChanged)
                    ptArray.Add(angles);
                else
                    ptArray[nCurve + 1] = angles;
            }

            return angles;
        }
        #endregion

        public void reset()
        {
            removeVisual();
            lineBezier.Clear();
            movePaths.Clear();
            ptArray.Clear();
            trajJoints.Clear();
        }

        public void removeVisual()
        {
            if (movePaths.Count == 0)
                return;
            else if (movePaths.Count == 1 && movePaths[0].ptCurve.Count == 1)
            {
                viewport3d.Children.Remove(movePaths[0].ptCurve[0]);
                return;
            }

            for (int i=0;i<movePaths.Count;i++)
            {
                viewport3d.Children.Remove(lineBezier[i]);
                viewport3d.Children.Remove(movePaths[i].ptCurve[0]);
                if (movePaths[i].shape=="bezier")
                {
                    viewport3d.Children.Remove(movePaths[i].ptAdjust[0]);
                    viewport3d.Children.Remove(movePaths[i].ptAdjust[1]);
                    if(i==0)
                        viewport3d.Children.Remove(movePaths[i].ptCurve[1]);
                    viewport3d.Children.Remove(movePaths[i].ctrlLine[0]);
                    viewport3d.Children.Remove(movePaths[i].ctrlLine[1]);
                }
                else if(movePaths[i].shape=="arc")
                {
                    viewport3d.Children.Remove(movePaths[i].ptAdjust[0]);
                }
            }
        }

        public void addVisual()
        {
            for (int i = oldCount; i < movePaths.Count; i++)
            {
                viewport3d.Children.Add(movePaths[i].ptCurve[0]);
                if (i == 0)
                    viewport3d.Children.Add(movePaths[i].ptCurve[1]);
                if (oldCount > 0 && i == oldCount)
                    continue;
                if(showHandle)
                {
                    if (movePaths[i].shape == "bezier")
                    {
                        viewport3d.Children.Add(movePaths[i].ptAdjust[0]);
                        viewport3d.Children.Add(movePaths[i].ptAdjust[1]);
                    }
                    else if (movePaths[i].shape == "arc")
                    {
                        viewport3d.Children.Add(movePaths[i].ptAdjust[0]);
                    }
                }
            }
        }

        public void deleteAndUpdate()
        {
            if (curveCount == 1)
            {
                reset();
                return;
            }

            CustomCurve curve = new CustomCurve();
            BezierMarkPoint tempPt = new BezierMarkPoint();

            for(int i=0;i<2;i++)
            {
                viewport3d.Children.Remove(lineBezier[nSelected + i]);
                if (movePaths[nSelected + i].shape == "bezier")
                {
                    viewport3d.Children.Remove(movePaths[nSelected + i].ctrlLine[0]);
                    viewport3d.Children.Remove(movePaths[nSelected + i].ctrlLine[1]);
                    viewport3d.Children.Remove(movePaths[nSelected + i].ptAdjust[0]);
                    viewport3d.Children.Remove(movePaths[nSelected + i].ptAdjust[1]);
                }
                else if (movePaths[nSelected + i].shape == "arc")
                {
                    viewport3d.Children.Remove(movePaths[nSelected + i].ptAdjust[0]);
                }

                if (nSelected + 1 == curveCount || (nSelected==0 && nPoint==0))
                    break;
            }

            if (nSelected==0 && nPoint==0)
            {
                tempPt = movePaths[nSelected].ptCurve[1 - nPoint];

                viewport3d.Children.Remove(movePaths[nSelected].ptCurve[0]);
                viewport3d.Children.Remove(movePaths[nSelected].ptCurve[1]);

                if (nPoint==0)
                {
                    curve = movePaths[nSelected + 1];
                    curve.ptCurve.Add(tempPt);
                    tempPt = curve.ptCurve[0];
                    curve.ptCurve[0] = curve.ptCurve[1];
                    curve.ptCurve[1] = tempPt;
                    viewport3d.Children.Add(curve.ptCurve[0]);
                    movePaths[nSelected + 1] = curve;

                    for(int i=0;i<curveCount - 1;i++)
                    {
                        movePaths[i] = movePaths[i + 1];
                        movePaths[i].ptAdjust[0].number = i;
                        movePaths[i].ptAdjust[1].number = i;
                        movePaths[i].ptCurve[0].number = i;
                        if(i==0)
                            movePaths[i].ptCurve[1].number = i;
                        lineBezier[i] = lineBezier[i + 1];
                        lineBezier[i].number = i;
                    }

                    for (int i = 0; i < ptArray.Count - 1; i++)
                        ptArray[i] = ptArray[i + 1];
                }
            }
            else if(nSelected==curveCount-1)
            {
                viewport3d.Children.Remove(movePaths[nSelected].ptCurve[0]);
            }
            else
            {
                int k = 0;
                if (nSelected == 0)
                    k = 1;
                viewport3d.Children.Remove(movePaths[nSelected].ptCurve[k]);
                curve = movePaths[nSelected];
                curve.ptCurve[k] = movePaths[nSelected + 1].ptCurve[0];
                
                movePaths[nSelected] = curve;
                movePaths[nSelected].ptCurve[0].number = nSelected;
                makeDefaultBezier(nSelected, true);
                for(int i=nSelected;i<curveCount-1;i++)
                {
                    if(i!=nSelected)
                    {
                        movePaths[i] = movePaths[i + 1];
                        lineBezier[i] = lineBezier[i + 1];
                    }
                    movePaths[i].ptAdjust[0].number = i;
                    movePaths[i].ptAdjust[1].number = i;
                    movePaths[i].ptCurve[0].number = i;
                    if (i == 0)
                        movePaths[i].ptCurve[1].number = i;
                    lineBezier[i].number = i;
                }
                for (int i = nSelected + 1; i < ptArray.Count - 1; i++)
                    ptArray[i] = ptArray[i + 1];
            }
            ptArray.RemoveAt(ptArray.Count - 1);
            lineBezier.RemoveAt(curveCount - 1);
            movePaths.RemoveAt(curveCount - 1);
        }

        public static void redraw()
        {
            for (int i = 0; i < curveCount; i++)
            {
                string shape = movePaths[i].shape;
                if (movePaths[i].curvePath.Count == 0)
                {
                    
                    if (shape == "bezier")
                    {
                        drawBezierCurveSegment(i, true);
                        for (int num2 = 0; num2 < 2; num2++)
                        {
                            if (movePaths[i].ctrlLine[num2] == null)
                            {
                                drawControlCurveSegment(i, true);
                            }
                        }
                    }
                    if (shape == "arc")
                        makeArc(i, true);
                    if (shape == "segment")
                        makeLine(i);

                }
            }
        }

        public void AddBezierDefault()
        {
            if (bCurveSelected)
            {
                bCurveSelected = false;
                makeDefaultBezier(nCurveSelected, true);
            }
            else
            {
                double[] angles = { parentWindow.slider_simPos_J1.Value, parentWindow.slider_simPos_J2.Value, parentWindow.slider_simPos_J3.Value, parentWindow.slider_simPos_J4.Value, parentWindow.slider_simPos_J5.Value, parentWindow.slider_simPos_J6.Value };
                if (ptArray.Count > 1)
                    if (angles == ptArray[ptArray.Count - 1])
                        return;
                ptArray.Add(angles);
                //Vector3D vector = ForwardKinematics(angles);
                double[] retVal = ForwardKinematics(parentWindow.slider_simPos_J1.Value, parentWindow.slider_simPos_J2.Value,
                    parentWindow.slider_simPos_J3.Value, parentWindow.slider_simPos_J4.Value, 
                    parentWindow.slider_simPos_J5.Value, parentWindow.slider_simPos_J6.Value);
                Random random = new Random();
                //int curveCount = Helix3dSim.curveCount;
                //double[] temp = { vector.X, vector.Y, vector.Z };
                double[] temp = { retVal[0], retVal[1], retVal[2] };
                if (ptArray.Count == 1 || ptArray.Count == 2)
                {
                    addSeqPoint(0, "bezier", temp, "point");
                }
                else
                    addSeqPoint(curveCount, "bezier", temp, "point");
                if (ptArray.Count > 1)
                {
                    makeDefaultBezier(ptArray.Count - 2, false);
                    parentWindow.btn_Calculate.IsEnabled = true;
                    parentWindow.btn_save.IsEnabled = true;
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointFirst.IsEnabled = true;

                    int cc = 0;
                    if (movePaths.Count == 1)
                        cc = 1;
                    double dist = CustomSpline.GetDistance(movePaths[0].ptCurve[0].point, movePaths[movePaths.Count - 1].ptCurve[cc].point);
                    parentWindow.lbl_total_distance.Content = dist.ToString();
                    curPoint = ptArray.Count - 1;
                }
                parentWindow.btn_reset.IsEnabled = true;
            }
        }

        #region TRACER

        private void addTracePoint3D(TracePoint3D point)
        {
            tracepoints.Add(point);
            PlotData();
        }

        public void TracerClear()
        {
            helixTracer.Clear3D();
        }

        private void PlotData()
        {
            //Console.WriteLine($"Trace Points Count: {tracepoints.Count}");
            if (tracepoints.Count == 1)
            {
                TracePoint3D point;
                lock (tracepoints)
                {
                    point = tracepoints[0];
                    tracepoints.Clear();
                }
                helixTracer.AddPoint(point.point, point.color, point.thickness);
            }
            else
            {
                TracePoint3D[] pointsArray;
                lock (tracepoints)
                {
                    pointsArray = tracepoints.ToArray();
                    tracepoints.Clear();
                }
                foreach (TracePoint3D point in pointsArray)
                    helixTracer.AddPoint(point.point, point.color, point.thickness);
            }
        }

        #endregion

        #region PLAYBACK

        public void goPreviousPathPoint()
        {
            if (ptArray.Count < 1)
                return;
            if (!parentWindow.btn_sim_PointNext.IsEnabled)
                parentWindow.btn_sim_PointNext.IsEnabled = true;
            curPoint--;
            if (curPoint < 0)
            {
                curPoint = 0;
                parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            }
            else
            {
                UpdateRoboticArm( ptArray[curPoint][0], ptArray[curPoint][1], ptArray[curPoint][2], ptArray[curPoint][3], ptArray[curPoint][4], ptArray[curPoint][5]);

                int k = 0, j = 0;
                if (curPoint == 0) k = 1;
                if (curPoint == 1) j = 1;
                parentWindow.tb_sim_pointdata_id.Text = (string)Convert.ToString(curPoint);
                parentWindow.tb_sim_selpoint_x.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.X);
                parentWindow.tb_sim_selpoint_y.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.Y);
                parentWindow.tb_sim_selpoint_z.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.Z);
                //parentWindow.tb_sim_selpoint_PIPDistance.Text = (string)Convert.ToString((int)CustomSpline.GetDistance(movePaths[curPoint + k - 1].ptCurve[k].point, targetPoint.point));

                Point3D tt = new Point3D();
                tt.X = targetPoint.point.X - movePaths[curPoint + k - 1].ptCurve[j].point.X;
                tt.Y = targetPoint.point.Y - movePaths[curPoint + k - 1].ptCurve[j].point.Y;
                tt.Z = targetPoint.point.Z - movePaths[curPoint + k - 1].ptCurve[j].point.Z;
                double radius = Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y + tt.Z * tt.Z);
                double theta = Math.Atan2(tt.Y, tt.X) * 57.3;
                double fai = Math.Atan2(Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y), tt.Z) * 57.3;
                parentWindow.tb_sim_selpoint_rx.Text = (string)Convert.ToString((int)radius);
                parentWindow.tb_sim_selpoint_ry.Text = (string)Convert.ToString((int)theta);
                parentWindow.tb_sim_selpoint_rz.Text = (string)Convert.ToString((int)fai);
            }
            if (curPoint <= 0)
            {
                curPoint = 0;
                parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            }
        }

        public void goNextPathPoint()
        {
            if (ptArray.Count < 1)
                return;
            if (!parentWindow.btn_sim_PointPrevious.IsEnabled)
                parentWindow.btn_sim_PointPrevious.IsEnabled = true;
            curPoint++;
            if (curPoint > ptArray.Count - 1)
            {
                curPoint = ptArray.Count - 1;
                parentWindow.btn_sim_PointNext.IsEnabled = false;
            }
            else
            {
                UpdateRoboticArm(ptArray[curPoint][0], ptArray[curPoint][1], ptArray[curPoint][2], ptArray[curPoint][3], ptArray[curPoint][4], ptArray[curPoint][5]);

                int k = 0, j = 0;
                if (curPoint == 0)
                    k = 1;
                if (curPoint == 1)
                    j = 1;
                parentWindow.tb_sim_pointdata_id.Text = (string)Convert.ToString(curPoint);
                parentWindow.tb_sim_selpoint_x.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.X);
                parentWindow.tb_sim_selpoint_y.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.Y);
                parentWindow.tb_sim_selpoint_z.Text = (string)Convert.ToString((int)movePaths[curPoint + k - 1].ptCurve[j].point.Z);
                
                //parentWindow.tb_sim_selpoint_PIPDistance.Text = (string)Convert.ToString((int)CustomSpline.GetDistance(movePaths[curPoint + k - 1].ptCurve[k].point, targetPoint.point));

                Point3D tt = new Point3D();
                tt.X = targetPoint.point.X - movePaths[curPoint + k - 1].ptCurve[j].point.X;
                tt.Y = targetPoint.point.Y - movePaths[curPoint + k - 1].ptCurve[j].point.Y;
                tt.Z = targetPoint.point.Z - movePaths[curPoint + k - 1].ptCurve[j].point.Z;
                double radius = Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y + tt.Z * tt.Z);
                double theta = Math.Atan2(tt.Y, tt.X) * 57.3;
                double fai = Math.Atan2(Math.Sqrt(tt.X * tt.X + tt.Y * tt.Y), tt.Z) * 57.3;
                parentWindow.tb_sim_selpoint_rx.Text = (string)Convert.ToString((int)radius);
                parentWindow.tb_sim_selpoint_ry.Text = (string)Convert.ToString((int)theta);
                parentWindow.tb_sim_selpoint_rz.Text = (string)Convert.ToString((int)fai);
            }

            if (curPoint >= ptArray.Count - 1)
            {
                curPoint = ptArray.Count - 1;
                parentWindow.btn_sim_PointNext.IsEnabled = false;
            }
        }

        public void PlaybackStop()
        {
            if (timerPlay == null)
                return;

            parentWindow.btn_sim_player_Stop.IsEnabled = false;
            parentWindow.btn_sim_player_Pause.IsEnabled = false;
            parentWindow.btn_sim_PointPrevious.IsEnabled = true;
            parentWindow.btn_sim_PointNext.IsEnabled = true;
            parentWindow.btn_sim_PointFirst.IsEnabled = true;
            parentWindow.btn_sim_PointLast.IsEnabled = true;
            parentWindow.btn_Add_Default.IsEnabled = true;

            curPoint = nCurve;

            timerPlay.Stop();
            timerPlay.Tick -= new System.EventHandler(timer_play);
            timerPlay = null;

            parentWindow.btn_sim_player_PlayForward.Content = "Play >";
            parentWindow.btn_sim_player_PlayReverse.Content = "< Play";
            parentWindow.btn_sim_player_PlayReverse.IsEnabled = true;
            parentWindow.btn_sim_player_StepForward.IsEnabled = true;
            parentWindow.btn_sim_player_StepReverse.IsEnabled = true;
        }

        public void PlaybackPause()
        {
            if (timerPlay == null)
                return;
            parentWindow.btn_sim_player_Pause.IsEnabled = false;

            if (timerPlay.Enabled)
            {
                timerPlay.Enabled = false;
                if (bForward)
                    parentWindow.btn_sim_player_PlayForward.Content = "Resume >";
                else
                    parentWindow.btn_sim_player_PlayReverse.Content = "< Resume";
                parentWindow.btn_sim_player_Pause.IsEnabled = false;
            }
        }

        public void PlaybackPlayForward()
        {
            //totalDuration = Convert.ToDouble(parentWindow.tb_sim_duration.Text);

            if (totalDuration == 0)
                return;
            bForward = true;

            parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            parentWindow.btn_sim_PointNext.IsEnabled = false;
            parentWindow.btn_sim_PointFirst.IsEnabled = false;
            parentWindow.btn_sim_PointLast.IsEnabled = false;
            parentWindow.btn_Add_Default.IsEnabled = false;
            parentWindow.btn_sim_player_StepForward.IsEnabled = false;
            parentWindow.btn_sim_player_StepReverse.IsEnabled = false;
            parentWindow.btn_sim_player_PlayReverse.IsEnabled = false;
            parentWindow.btn_sim_player_Pause.IsEnabled = true;
            parentWindow.btn_sim_player_Stop.IsEnabled = true;

            if (timerPlay != null)
            {
                if (!timerPlay.Enabled)
                {
                    timerPlay.Enabled = true;
                    parentWindow.btn_sim_player_PlayForward.Content = "Playing";
                }
            }
            else
            {
                bFirst = true;
                timerPlay = new System.Windows.Forms.Timer();
                int timer_step = 33;
                if (timer_step <= 0)
                {
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointNext.IsEnabled = true;
                    parentWindow.btn_sim_PointFirst.IsEnabled = true;
                    parentWindow.btn_sim_PointLast.IsEnabled = true;
                    parentWindow.btn_Add_Default.IsEnabled = true;
                    parentWindow.btn_sim_player_StepForward.IsEnabled = true;
                    parentWindow.btn_sim_player_StepReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_PlayReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_Pause.IsEnabled = false;
                    parentWindow.btn_sim_player_Stop.IsEnabled = false;
                    return;
                }
                timerPlay.Interval = timer_step;
                timerPlay.Tick += new EventHandler(timer_play);
                timerPlay.Enabled = true;
                parentWindow.btn_sim_player_PlayForward.Content = "Playing";
                curPlayheadPos = 0;
                parentWindow.lbl_sim_player_CurrentPlayPos.Content = TimeSpan.FromSeconds(curPlayheadPos).ToString(@"hh\:mm\:ss\.fff");
                total_time_origin = (float)(timer_step * (totalCount - 1)) * 0.001f;
                play_step = (int)(Math.Round(total_time_origin / totalDuration));
                if (total_time_origin / totalDuration < 1)
                {
                    timerPlay.Interval = (int)(timer_step * (float)(totalDuration / total_time_origin));
                    play_step = 1;
                }
            }
        }

        public void PlaybackPlayReverse()
        {
            //totalDuration = Convert.ToDouble(parentWindow.tb_sim_duration.Text);

            if (totalDuration == 0)
                return;
            bForward = false;

            parentWindow.btn_sim_PointPrevious.IsEnabled = false;
            parentWindow.btn_sim_PointNext.IsEnabled = false;
            parentWindow.btn_sim_PointFirst.IsEnabled = false;
            parentWindow.btn_sim_PointLast.IsEnabled = false;
            parentWindow.btn_Add_Default.IsEnabled = false;
            parentWindow.btn_sim_player_StepForward.IsEnabled = false;
            parentWindow.btn_sim_player_StepReverse.IsEnabled = false;
            parentWindow.btn_sim_player_PlayForward.IsEnabled = false;
            parentWindow.btn_sim_player_Pause.IsEnabled = true;
            parentWindow.btn_sim_player_Stop.IsEnabled = true;

            if (timerPlay != null)
            {
                if (!timerPlay.Enabled)
                {
                    timerPlay.Enabled = true;
                    parentWindow.btn_sim_player_PlayReverse.Content = "Playing";
                }
            }
            else
            {
                bFirst = true;
                timerPlay = new System.Windows.Forms.Timer();
                //////////
                int timer_step = 33;
                if (timer_step <= 0)
                {
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointNext.IsEnabled = true;
                    parentWindow.btn_sim_PointFirst.IsEnabled = true;
                    parentWindow.btn_sim_PointLast.IsEnabled = true;
                    parentWindow.btn_Add_Default.IsEnabled = true;
                    parentWindow.btn_sim_player_StepForward.IsEnabled = true;
                    parentWindow.btn_sim_player_StepReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_PlayReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_Pause.IsEnabled = false;
                    parentWindow.btn_sim_player_Stop.IsEnabled = false;
                    return;
                }
                timerPlay.Interval = timer_step;
                timerPlay.Tick += new EventHandler(timer_play);
                timerPlay.Enabled = true;
                parentWindow.btn_sim_player_PlayReverse.Content = "Playing";
                curPlayheadPos = 0;
                parentWindow.lbl_sim_player_CurrentPlayPos.Content = TimeSpan.FromSeconds(curPlayheadPos).ToString(@"hh\:mm\:ss\.fff");
                total_time_origin = (float)(timer_step * (totalCount - 1)) * 0.001f;
                play_step = (int)(Math.Round(total_time_origin / totalDuration));
                if (total_time_origin / totalDuration < 1)
                {
                    timerPlay.Interval = (int)(timer_step * (float)(totalDuration / total_time_origin));
                    play_step = 1;
                }

            }
        }

        private void timer_play(object sender, EventArgs e)
        {
            curPlayheadPos += (double)timerPlay.Interval / 1000;
            parentWindow.lbl_sim_player_CurrentPlayPos.Content = TimeSpan.FromSeconds(curPlayheadPos).ToString(@"hh\:mm\:ss\.fff");

            if (bForward)
            {
                timeCount++;
                count += play_step;
                if (count > trajJoints[nCurve].Count - 1)
                {
                    count = 0;
                    nCurve++;
                }
                if (nCurve > trajJoints.Count - 1)
                {
                    count = 0;
                    nCurve = 0;
                    timerPlay.Enabled = false;
                    timerPlay = null;
                    parentWindow.btn_sim_player_PlayForward.Content = "Play >";
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointNext.IsEnabled = true;
                    parentWindow.btn_sim_PointFirst.IsEnabled = true;
                    parentWindow.btn_sim_PointLast.IsEnabled = true;
                    parentWindow.btn_Add_Default.IsEnabled = true;
                    parentWindow.btn_sim_player_StepForward.IsEnabled = true;
                    parentWindow.btn_sim_player_StepReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_PlayReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_Pause.IsEnabled = false;
                    parentWindow.btn_sim_player_Stop.IsEnabled = false;
                    parentWindow.slider_sim_player_TimeSlider.Value = 0.0;
                    timeCount = 0;
                }
            }
            else
            {
                timeCount--;
                count -= play_step;
                if (count < 0)
                {
                    nCurve--;
                    if (nCurve >= 0)
                        count = trajJoints[nCurve].Count - 1;
                }
                if (nCurve < 0)
                {
                    nCurve = trajJoints.Count - 1;
                    count = trajJoints[nCurve].Count - 1;
                    timerPlay.Enabled = false;
                    timerPlay = null;
                    parentWindow.btn_sim_player_PlayReverse.Content = "< Play";
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointNext.IsEnabled = true;
                    parentWindow.btn_sim_PointFirst.IsEnabled = true;
                    parentWindow.btn_sim_PointLast.IsEnabled = true;
                    parentWindow.btn_Add_Default.IsEnabled = true;
                    parentWindow.btn_sim_player_StepForward.IsEnabled = true;
                    parentWindow.btn_sim_player_StepReverse.IsEnabled = true;
                    parentWindow.btn_sim_player_PlayForward.IsEnabled = true;
                    parentWindow.btn_sim_player_Pause.IsEnabled = false;
                    parentWindow.btn_sim_player_Stop.IsEnabled = false;
                    parentWindow.slider_sim_player_TimeSlider.Value = 100.0;
                    timeCount = totalCount;
                }
            }

            UpdateRoboticArm(trajJoints[nCurve][count][0],
                             trajJoints[nCurve][count][1], 
                             trajJoints[nCurve][count][2],
                             trajJoints[nCurve][count][3], 
                             trajJoints[nCurve][count][4], 
                             trajJoints[nCurve][count][5]);
            
            parentWindow.slider_sim_player_TimeSlider.Value = (double)timeCount / totalCount * 100;
        }

        private void resetHelixPlayback()
        {
            timeCount = 0;
            count = 0;
            nCurve = 0;
            reachedPathEnd = false;
        }

        public void PlaybackStepForward()
        {
            timeCount += Convert.ToInt32(parentWindow.tb_sim_player_StepSize.Text);
            count += Convert.ToInt32(parentWindow.tb_sim_player_StepSize.Text);
            if (count > trajJoints[nCurve].Count - 1)
            {
                count -= trajJoints[nCurve].Count;
                nCurve++;
            }
            if (nCurve > trajJoints.Count - 1)
            {
                nCurve = trajJoints.Count - 1;
                count = trajJoints[nCurve].Count - 1;
                parentWindow.btn_sim_player_StepForward.IsEnabled = false;
            }

            double[] angles = {trajJoints[nCurve][count][0],
                trajJoints[nCurve][count][1],
                trajJoints[nCurve][count][2],
                trajJoints[nCurve][count][3],
                trajJoints[nCurve][count][4],
                trajJoints[nCurve][count][5]};

            UpdateRoboticArm(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            
            parentWindow.slider_simPos_J1.Value = angles[0];
            parentWindow.slider_simPos_J2.Value = angles[1];
            parentWindow.slider_simPos_J3.Value = angles[2];
            parentWindow.slider_simPos_J4.Value = angles[3];
            parentWindow.slider_simPos_J5.Value = angles[4];
            parentWindow.slider_simPos_J6.Value = angles[5];

            parentWindow.slider_sim_player_TimeSlider.Value = timeCount * 100 / totalCount;
        }

        public void PlaybackStepReverse()
        {
            timeCount -= Convert.ToInt32(parentWindow.tb_sim_player_StepSize.Text);
            count -= Convert.ToInt32(parentWindow.tb_sim_player_StepSize.Text);
            if (count < 0)
            {
                nCurve--;
                if (nCurve >= 0)
                    count = trajJoints[nCurve].Count + count;
            }
            if (nCurve < 0)
            {
                nCurve = 0;
                count = 0;
                parentWindow.btn_sim_player_StepReverse.IsEnabled = false;
            }

            double[] angles = {trajJoints[nCurve][count][0],
                trajJoints[nCurve][count][1],
                trajJoints[nCurve][count][2],
                trajJoints[nCurve][count][3],
                trajJoints[nCurve][count][4],
                trajJoints[nCurve][count][5]};

            UpdateRoboticArm(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            
            parentWindow.slider_simPos_J1.Value = angles[0];
            parentWindow.slider_simPos_J2.Value = angles[1];
            parentWindow.slider_simPos_J3.Value = angles[2];
            parentWindow.slider_simPos_J4.Value = angles[3];
            parentWindow.slider_simPos_J5.Value = angles[4];
            parentWindow.slider_simPos_J6.Value = angles[5];
            parentWindow.slider_sim_player_TimeSlider.Value = timeCount * 100 / totalCount;
        }

        #endregion

        #region FILE HANDLING

        public void updateOpenData()
        {
            addVisual();
            //lineBezier = new List<Trajectory>();
            for (int i = oldCount; i < movePaths.Count; i++)
            {
                if (i != 0 && i == oldCount)
                {
                    makeDefaultBezier(i, false);
                    continue;
                }
                if (movePaths[i].shape == "bezier")
                {
                    drawBezierCurveSegment(i, false);
                    drawControlCurveSegment(i, false);
                }
                else if (movePaths[i].shape == "arc")
                    drawArc(i);
                else if (movePaths[i].shape == "segment")
                    drawSegment(i);
            }
        }

        public void OpenFile()
        {
            CustomCurve curve = new CustomCurve();
            CustomCurve curve0 = new CustomCurve();

            OpenFileDialog openFileDialog = new OpenFileDialog();
            string fileName;
            if (openFileDialog.ShowDialog() == true)
            {
                if (movePaths.Count == 1 && movePaths[0].ptCurve.Count == 1)
                    reset();
                oldCount = movePaths.Count;

                fileName = openFileDialog.FileName;

                System.Xml.Linq.XDocument xDoc = System.Xml.Linq.XDocument.Load(fileName);
                var items = xDoc.Descendants("PathData");
                int i = oldCount;
                if (oldCount > 0)
                    i++;
                //ptArray = new List<double[]>();
                //movePaths = new List<CustomCurve>();

                var ptMain = new MaterialGroup();
                ptMain.Children.Add(new EmissiveMaterial(new SolidColorBrush(Colors.LimeGreen)));
                ptMain.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.LimeGreen)));
                ptMain.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.LimeGreen), 200));

                var ptControl = new MaterialGroup();
                ptControl.Children.Add(new EmissiveMaterial(new SolidColorBrush(Colors.Blue)));
                ptControl.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.Blue)));
                ptControl.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.Blue), 200));
                foreach (var item in items)
                {
                    ///////////  shape of curve  //////////////
                    curve.shape = item.Element("shape").Value;
                    if (oldCount > 0)
                        curve0.shape = item.Element("shape").Value;

                    ////////// pt1 ////////////
                    string ptStr = item.Element("pt1").Value;
                    string[] ptsStr = ptStr.Split(',');
                    Point3D pt = new Point3D(Convert.ToInt32(Convert.ToDouble(ptsStr[0])), Convert.ToInt32(Convert.ToDouble(ptsStr[1])), Convert.ToInt32(Convert.ToDouble(ptsStr[2])));
                    BezierMarkPoint mark = new BezierMarkPoint(i, "point", pt)
                    {
                        Center = new Point3D(0, 0, 0),
                        Radius = 15,
                        Material = ptMain
                    };
                    mark.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));
                    curve.ptCurve = new List<BezierMarkPoint>();
                    curve.ptCurve.Add(mark);
                    if (oldCount > 0 && i == oldCount + 1)
                    {
                        curve0.ptCurve = new List<BezierMarkPoint>();
                        mark.number -= 1;
                        curve0.ptCurve.Add(mark);
                    }

                    ////////// pt2 //////////
                    if (i == 0)
                    {
                        ptStr = item.Element("pt2").Value;
                        ptsStr = ptStr.Split(',');
                        pt = new Point3D(Convert.ToInt32(Convert.ToDouble(ptsStr[0])), Convert.ToInt32(Convert.ToDouble(ptsStr[1])), Convert.ToInt32(Convert.ToDouble(ptsStr[2])));
                        {
                            mark = new BezierMarkPoint(i, "point", pt)
                            {
                                Center = new Point3D(0, 0, 0),
                                Radius = 15,
                                Material = ptMain
                            };
                            mark.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));
                        }
                        curve.ptCurve.Add(mark);
                    }
                    if (oldCount > 0 && i == oldCount + 1)
                    {
                        ptStr = item.Element("pt2").Value;
                        ptsStr = ptStr.Split(',');
                        pt = new Point3D(Convert.ToInt32(Convert.ToDouble(ptsStr[0])), Convert.ToInt32(Convert.ToDouble(ptsStr[1])), Convert.ToInt32(Convert.ToDouble(ptsStr[2])));
                        {
                            mark = new BezierMarkPoint(i, "point", pt)
                            {
                                Center = new Point3D(0, 0, 0),
                                Radius = 15,
                                Material = ptMain
                            };
                            mark.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));
                        }
                        curve.ptCurve[0] = mark;
                    }

                    ///////// pt3 ///////////
                    ptStr = item.Element("pt3").Value;
                    ptsStr = ptStr.Split(',');
                    pt = new Point3D(Convert.ToInt32(Convert.ToDouble(ptsStr[0])), Convert.ToInt32(Convert.ToDouble(ptsStr[1])), Convert.ToInt32(Convert.ToDouble(ptsStr[2])));
                    {
                        mark = new BezierMarkPoint(i, "control", pt)
                        {
                            Center = new Point3D(0, 0, 0),
                            Radius = 15,
                            Material = ptControl
                        };
                        mark.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));
                        curve.ptAdjust = new List<BezierMarkPoint>();
                        curve.ptAdjust.Add(mark);
                    }

                    ////////// pt4 ///////////
                    ptStr = item.Element("pt4").Value;
                    ptsStr = ptStr.Split(',');
                    pt = new Point3D(Convert.ToInt32(Convert.ToDouble(ptsStr[0])), Convert.ToInt32(Convert.ToDouble(ptsStr[1])), Convert.ToInt32(Convert.ToDouble(ptsStr[2])));
                    {
                        mark = new BezierMarkPoint(i, "control", pt)
                        {
                            Center = new Point3D(0, 0, 0),
                            Radius = 15,
                            Material = ptControl
                        };
                        mark.Transform = new TranslateTransform3D(new Vector3D(pt.X, pt.Y, pt.Z));
                        curve.ptAdjust.Add(mark);
                    }


                    if (oldCount != 0 && i == oldCount + 1)
                    {
                        movePaths.Add(curve0);
                    }
                    movePaths.Add(curve);

                    /////////// joint data ///////////
                    ptStr = item.Element("jointChanged").Value;
                    jointChanged &= Convert.ToBoolean(ptStr);
                    double[] temp = { 0, 0, 0, 0, 0, 0 };
                    if (i == 0 || (oldCount > 0 && i == oldCount + 1))
                    {
                        ptStr = item.Element("joint0").Value;
                        ptsStr = ptStr.Split(',');
                        for (int k = 0; k < 6; k++)
                            temp[k] = Convert.ToDouble(ptsStr[k]);
                        ptArray.Add(temp);
                    }

                    double[] tempp = { 0, 0, 0, 0, 0, 0 };
                    ptStr = item.Element("joint1").Value;
                    ptsStr = ptStr.Split(',');
                    for (int k = 0; k < 6; k++)
                        tempp[k] = Convert.ToDouble(ptsStr[k]);
                    ptArray.Add(tempp);

                    i++;
                }

                updateOpenData();

                parentWindow.btn_sim_PointFirst.IsEnabled = true;
                parentWindow.btn_sim_PointLast.IsEnabled = true;
                parentWindow.btn_Calculate.IsEnabled = true;
                parentWindow.btn_reset.IsEnabled = true;
                if (!jointChanged)
                {
                    parentWindow.btn_sim_PointPrevious.IsEnabled = true;
                    parentWindow.btn_sim_PointNext.IsEnabled = true;
                }

            }
        }

        public void SaveFile()
        {
            if (movePaths.Count == 0)
                return;
            SaveFileDialog saveFileDialog = new SaveFileDialog();
            saveFileDialog.Filter = "Path file (*.xml)|*.txt|C# file (*.cs)|*.cs";
            string fileName;
            if (saveFileDialog.ShowDialog() == true)
            {
                fileName = saveFileDialog.FileName;
                // ... add items...
                System.Data.DataTable dt = new System.Data.DataTable();
                dt.TableName = "PathData";
                dt.Columns.Add("shape");
                dt.Columns.Add("pt1");
                dt.Columns.Add("pt2");
                dt.Columns.Add("pt3");
                dt.Columns.Add("pt4");
                dt.Columns.Add("jointChanged");
                dt.Columns.Add("joint0");
                dt.Columns.Add("joint1");
                string tempp = "";
                for (int i = 0; i < movePaths.Count; i++)
                {
                    ///////////  point data  /////////////
                    dt.Rows.Add();
                    dt.Rows[dt.Rows.Count - 1]["shape"] = movePaths[i].shape;
                    dt.Rows[dt.Rows.Count - 1]["pt1"] = movePaths[i].ptCurve[0].point;
                    if (i == 0)
                        dt.Rows[dt.Rows.Count - 1]["pt2"] = movePaths[i].ptCurve[1].point;
                    dt.Rows[dt.Rows.Count - 1]["pt3"] = movePaths[i].ptAdjust[0].point;
                    dt.Rows[dt.Rows.Count - 1]["pt4"] = movePaths[i].ptAdjust[1].point;

                    ///////////  joint data  ////////////
                    dt.Rows[dt.Rows.Count - 1]["jointChanged"] = jointChanged;
                    if (i == 0)
                    {
                        tempp = Convert.ToString(ptArray[0][0]);
                        for (int k = 1; k < 6; k++)
                            tempp += ("," + Convert.ToString(ptArray[0][k]));
                        dt.Rows[dt.Rows.Count - 1]["joint0"] = tempp;
                    }
                    tempp = Convert.ToString(ptArray[i + 1][0]);
                    for (int k = 1; k < 6; k++)
                        tempp += ("," + Convert.ToString(ptArray[i + 1][k]));
                    dt.Rows[dt.Rows.Count - 1]["joint1"] = tempp;
                }

                dt.WriteXml(fileName);

            }
        }

        #endregion

        /* REMOVED STANDALONE ANIMATION TIMER
        private int movements = 10000;
        timer = new System.Windows.Forms.Timer();
        timer.Interval = 1;
        timer.Tick += new System.EventHandler(timer_Tick);
        timer.Enabled = false;
        private void timer_Tick(object sender, EventArgs e)
        {
            double[] angles = new double[] { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle };
            angles = InverseKinematics(ToolTargetCoord, angles);
            joints[0].angle = angles[0];
            joints[1].angle = angles[1];
            joints[2].angle = angles[2];
            joints[3].angle = angles[3];
            joints[4].angle = angles[4];
            joints[5].angle = angles[5];
            movements = movements - 1;
            if (movements <= 0)
            {
                //IsAnimating = false;
                //timer.Stop();
            }
            UpdateTool CurrentCoord();
            if (IsPathTracingEnabled)
            {
                addTracePoint3D(new TracePoint3D(new Point3D(ToolCurrentCoord.X, ToolCurrentCoord.Y, ToolCurrentCoord.Z), Color.FromArgb(200, 0, 0xff, 0), 1.0));
            }
        }
        */

        public bool TogglePanControl()
        {
            bool flag2;
            if (!IsPanControlEnabled)
            {
                viewport3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
                IsPanControlEnabled = true;
                flag2 = true;
            }
            else
            {
                viewport3d.PanGesture = new MouseGesture(MouseAction.None);
                IsPanControlEnabled = false;
                flag2 = false;
            }
            return flag2;
        }

        public void toggleShowControlPoint(bool bShow)
        {
            if (bShow)
            {
                for (int i = 0; i < movePaths.Count; i++)
                {
                    if (movePaths[i].shape == "bezier")
                        for (int j = 0; j < 2; j++)
                        {
                            viewport3d.Children.Add(movePaths[i].ctrlLine[j]);
                            viewport3d.Children.Add(movePaths[i].ptAdjust[j]);
                        }
                    else if (movePaths[i].shape == "arc")
                        viewport3d.Children.Add(movePaths[i].ptAdjust[0]);
                }

            }
            else
            {
                for (int i = 0; i < movePaths.Count; i++)
                {
                    if (movePaths[i].shape == "bezier")
                        for (int j = 0; j < 2; j++)
                        {
                            viewport3d.Children.Remove(movePaths[i].ctrlLine[j]);
                            viewport3d.Children.Remove(movePaths[i].ptAdjust[j]);
                        }
                    else if (movePaths[i].shape == "arc")
                        viewport3d.Children.Remove(movePaths[i].ptAdjust[0]);
                }
            }
        }

        public void toggleShowTrackingLine(bool bShow)
        {
            if(bShow)
            {
                if (!viewport3d.Children.Contains(trackLine))
                    viewport3d.Children.Add(trackLine);
            }
            else
            {
                if (viewport3d.Children.Contains(trackLine))
                    viewport3d.Children.Remove(trackLine);
            }

        }

        public void toggleShowRobotArm(bool flag)
        {
            if(flag)
            {
                if (!viewport3d.Children.Contains(RoboticArm))
                    viewport3d.Children.Add(RoboticArm);
            }
            else
            {
                if (viewport3d.Children.Contains(RoboticArm))
                    viewport3d.Children.Remove(RoboticArm);
            }
        }

    }
}
