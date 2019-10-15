using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.ComponentModel;
using System.Data;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Threading;
using System.Management;
using Microsoft.Win32;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using HelixToolkit.Wpf;
//using HelixPoint = System.Windows.Point;

namespace MonkeyMotionControl
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        #region CONSTANTS

        private const bool OPTION_VIDEO_AUTOSTART = false;
        private const int DEFAULT_LIVE_STEP_SIZE = 1;
        private const int MAX_LIVE_STEP_SIZE = 20;
        private const int LIVE_MAX_SPEED = 5;

        #endregion

        #region VARIABLES

        private DispatcherTimer mainTimer;

        public bool sync3D_isEnabled = false;
        private bool liveControl_isEnabled = false;
        private bool joypad_isEnabled = false;
        private bool joypad_isConnected = false;
        private bool isSimSlider_ValueChanged = false;

        private bool allowFK = false;

        private DataTable datatable_Sequence;
        private int datatable_CurPointCounter = 1;
        private bool isDatatableInitialized = false;

        private bool video_preview_mode;
        private bool video_record_mode;
        private string video_capture_format;

        public event PropertyChangedEventHandler PropertyChanged;

        private bool liveControl_isStreaming = false;
        private volatile int streamCommand = 0;

        private double _duration = 0;

        public double[] prevJointAngles = new double[] { 0, 0, 0, 0, 0, 0};

        #endregion

        #region CONSTRUCTOR

        public MainWindow()
        {
            InitializeComponent();
            Title = "Monkey Motion Control v0.90";
        }

        private void mainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //UIDisableAppFunctions();
            
            Initialize3DSimulator();
            
            InitializeMainTimer();
            
        }

        private void mainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (true) //this.isDataSaved
            {
                if (showMessageBoxYESNO("Close Application", "Close without saving?"))
                {
                    //RobotControllerDisconnect();
                    
                }
                else
                {
                    e.Cancel = true; // If user doesn't want to close, cancel closure
                }
            }
        }

        #endregion

        #region INTERFACE TO C++ DLL
        [DllImport("DHKinematics.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void rlForwardKinematics(double j1, double j2, double j3, double j4, double j5, double j6, IntPtr ret);

        [DllImport("DHKinematics.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void rlInverseKinematics(IntPtr prev, double x, double y, double z, double rotX, double rotY, double rotZ, IntPtr ret);
        #endregion

        #region MAIN TIMER

        private void InitializeMainTimer()
        {
            mainTimer = new System.Windows.Threading.DispatcherTimer(System.Windows.Threading.DispatcherPriority.Normal);
            mainTimer.Tick += new EventHandler(mainTimer_Tick);
            mainTimer.Interval = new TimeSpan(0, 0, 0, 0, 100); // 100ms
            mainTimer.IsEnabled = true;
            //mainTimer.Start();
        }

        private void mainTimer_Tick(object sender, EventArgs e)
        {
            try
            {
                //toggleIndicatorMainTimer();

                // Check for video input signal / or Phantom signal
                // if avaialble then start video feed screen. 
                //if (phantomCamera.CameraOnline && OPTION_VIDEO_AUTOSTART)
                //{
                //    VideoCapture_StartPreview();
                //}

                //timerRobotUpdate();
                /*
                if (sync3D_isEnabled)
                {
                    // Update simulated 3D robot with synced real robot angles
                    helix3dsim.UpdateRoboticArm( robotController.robotStatus.curJointPos[0], 
                                                 robotController.robotStatus.curJointPos[1], 
                                                 robotController.robotStatus.curJointPos[2],
                                                 robotController.robotStatus.curJointPos[3], 
                                                 robotController.robotStatus.curJointPos[4], 
                                                 robotController.robotStatus.curJointPos[5]);
                }
                else
                {*/
                if (isSimSlider_ValueChanged)
                {
                    // TODO: Lock when updating new values to prevent repeat calls
                    
                    helix3dsim.UpdateRoboticArm(slider_simPos_J1.Value,
                        slider_simPos_J2.Value, 
                        slider_simPos_J3.Value, 
                        slider_simPos_J4.Value, 
                        slider_simPos_J5.Value, 
                        slider_simPos_J6.Value);
                    isSimSlider_ValueChanged = false;
                }
                
                //}

                // Update simulator slider values
                //double[] joints = helix3dsim.GetJoints();
                //slider_simPos_J1.Value = joints[0];
                //slider_simPos_J2.Value = joints[1];
                //slider_simPos_J3.Value = joints[2];
                //slider_simPos_J4.Value = joints[3];
                //slider_simPos_J5.Value = joints[4];
                //slider_simPos_J6.Value = joints[5];
                //tb_simPos_x.Text = ;
                //tb_simPos_y.Text = ;
                //tb_simPos_x.Text = ;
                //Tx_Target.Text = ;
                //Ty_Target.Text = ;
                //Tz_Target.Text = ;
                //TestFK();

                // GAMEPAD ENABLE
                /*if (joypad_isEnabled)
                {
                    updateJoypadStatus();
                    if (!joypad_isConnected)
                    {
                        addLog("Joypad Disconnected.");
                        disableJoypad();
                    }
                }*/

            }
            catch (Exception l_e)
            {
                MessageBox.Show(l_e.Message);
            }
        }

        

        private bool showMessageBoxYESNO(string title, string message)
        {
            MessageBoxResult rsltMessageBox = MessageBox.Show(message, title, MessageBoxButton.YesNo, MessageBoxImage.Exclamation);
            if (rsltMessageBox == MessageBoxResult.Yes) return true;
            else return false;
        }

        #endregion

        #region 3D SIMULATOR

        private Helix3dSim helix3dsim;
        public Helix3dSim getHelix() { return helix3dsim; }
        //public HelixPoint mouseSelectionStartPoint = new HelixPoint();
        //public HelixPoint mouseSelectionEndPoint = new HelixPoint();
        //protected Point3D? MouseDownPoint3D { get; set; }

        private void Initialize3DSimulator()
        {
            helix3dsim = new Helix3dSim(this);
            
            show_ctrl_chk.IsChecked = true;
            link_handle_chk.IsChecked = false;
            btn_Add_Arc.IsEnabled = false;
            btn_Add_Linear.IsEnabled = false;

            btn_sim_PointPrevious.IsEnabled = false;
            btn_sim_PointNext.IsEnabled = false;
            btn_sim_PointFirst.IsEnabled = false;
            btn_sim_PointLast.IsEnabled = false;
            btn_Calculate.IsEnabled = false;
            btn_sim_player_PlayForward.IsEnabled = false;
            btn_sim_player_PlayReverse.IsEnabled = false;
            btn_sim_player_StepForward.IsEnabled = false;
            btn_sim_player_StepReverse.IsEnabled = false;
            btn_sim_player_Pause.IsEnabled = false;
            btn_sim_player_Stop.IsEnabled = false;
            btn_save.IsEnabled = false;
            btn_reset.IsEnabled = false;
            tb_sim_player_StepSize.Text = "1";
            lbl_sim_player_CurrentPlayPos.Content = FormatTimeString(0);
            tb_sim_duration.Text = FormatTimeString(5);
            //tb_sim_duration.Text = FormatTimeString(MotionGraph.Duration);
            tb_sim_selpoint_x.Text = "0";
            tb_sim_selpoint_y.Text = "0";
            tb_sim_selpoint_z.Text = "0";
            slider_sim_player_TimeSlider.Minimum = 0;
            slider_sim_player_TimeSlider.Maximum = 100;

            prevJointAngles[0] = slider_simPos_J1.Value;
            prevJointAngles[1] = slider_simPos_J2.Value;
            prevJointAngles[2] = slider_simPos_J3.Value;
            prevJointAngles[3] = slider_simPos_J4.Value;
            prevJointAngles[4] = slider_simPos_J5.Value;
            prevJointAngles[5] = slider_simPos_J6.Value;
        }

        public void SimJoint_Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            //isSimSlider_ValueChanged = true;
            if (!allowFK)
                return;
            if(helix3dsim != null)
            {
                helix3dsim.ForwardKinematics(new double[] { slider_simPos_J1.Value,
                    slider_simPos_J2.Value,
                    slider_simPos_J3.Value,
                    slider_simPos_J4.Value,
                    slider_simPos_J5.Value,
                    slider_simPos_J6.Value});
            }
            double[] retArr = new double[6];
            IntPtr outPtr = Marshal.AllocHGlobal(6 * Marshal.SizeOf<double>());
            rlForwardKinematics(slider_simPos_J1.Value,
                slider_simPos_J2.Value,
                slider_simPos_J3.Value,
                slider_simPos_J4.Value,
                slider_simPos_J5.Value,
                slider_simPos_J6.Value,
                outPtr);
            Marshal.Copy(outPtr, retArr, 0, 6);
            Marshal.FreeHGlobal(outPtr);

            slider_simPos_x.Value = retArr[0];
            slider_simPos_y.Value = retArr[1];
            slider_simPos_z.Value = retArr[2];
            slider_simPos_rx.Value = retArr[3];
            slider_simPos_ry.Value = retArr[4];
            slider_simPos_rz.Value = retArr[5];

        }

        public void SimCartesian_Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            //isSimSlider_ValueChanged = true;
            if (allowFK)
                return;

            prevJointAngles = new double[] { slider_simPos_J1.Value,
                slider_simPos_J2.Value,
                slider_simPos_J3.Value,
                slider_simPos_J4.Value,
                slider_simPos_J5.Value,
                slider_simPos_J6.Value };

            int size = Marshal.SizeOf(prevJointAngles[0]) * prevJointAngles.Length;
            IntPtr pnt = Marshal.AllocHGlobal(size);
            Marshal.Copy(prevJointAngles, 0, pnt, prevJointAngles.Length);

            IntPtr outIK = Marshal.AllocHGlobal(6 * Marshal.SizeOf<double>());
            rlInverseKinematics(pnt,
                Convert.ToDouble(slider_simPos_x.Value),
                Convert.ToDouble(slider_simPos_y.Value),
                Convert.ToDouble(slider_simPos_z.Value),
                Convert.ToDouble(slider_simPos_rx.Value),
                Convert.ToDouble(slider_simPos_ry.Value),
                Convert.ToDouble(slider_simPos_rz.Value),
                outIK);
            double[] arrIK = new double[6];
            Marshal.Copy(outIK, arrIK, 0, 6);
            Marshal.FreeHGlobal(outIK);
            Marshal.FreeHGlobal(pnt);

            if(helix3dsim!=null)
                helix3dsim.ForwardKinematics(new double[] { arrIK[0],arrIK[1],arrIK[2],
                    arrIK[3],arrIK[4],arrIK[5]});

            slider_simPos_J1.Value = arrIK[0];
            slider_simPos_J2.Value = arrIK[1];
            slider_simPos_J3.Value = arrIK[2];
            slider_simPos_J4.Value = arrIK[3];
            slider_simPos_J5.Value = arrIK[4];
            slider_simPos_J6.Value = arrIK[5];

        }

        private void ViewPort3D_OnKeyDown(object sender, KeyboardEventArgs k)
        {
            if (Keyboard.IsKeyDown(Key.LeftShift) || Keyboard.IsKeyDown(Key.RightShift))
            {
                Helix3dSim.viewport3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
                toggle3DPanControl();
            }
        }

        private void ViewPort3D_OnKeyUp(object sender, KeyboardEventArgs k)
        {
            if (Keyboard.IsKeyDown(Key.LeftShift) || Keyboard.IsKeyDown(Key.RightShift))
            {
                Helix3dSim.viewport3d.PanGesture = new MouseGesture(MouseAction.None);
                //toggle3DPanControl();
            }
        }

        private void ViewPort3D_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            //HelixPoint position = e.GetPosition(helix_viewport3d);
            helix3dsim.OnMouseLeftButtonDown(sender, e);
            //mouseSelectionStartPoint = e.GetPosition(helix_viewport3d);
            //Point3D pt = MouseDownPoint3D.Value;
            //Console.WriteLine($"MouseDownPoint3D X: {pt.X}, Y:{pt.Y}, Z{pt.Z}");
        }

        private void ViewPort3D_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            helix3dsim.OnMouseLeftButtonUp(sender, e);
            //mouseSelectionEndPoint = e.GetPosition(helix_viewport3d);
        }

        private void viewPort3D_OnMouseMove(object sender, MouseEventArgs e)
        {
            helix3dsim.OnMouseMove(sender, e);
            //Point position = e.GetPosition(helix_viewport3d);
            //helix_viewport3d.Camera.
            if (e.LeftButton == MouseButtonState.Pressed)
            {
            }
        }

        private void OnDragOver(object sender, DragEventArgs e)
        {
            Console.WriteLine("dfdfd");
        }

        private void ViewPort3D_ClearSeqPoints()
        {
            helix3dsim.clearSeqPoints();
        }

        private void ViewPort3D_AddSeqPoint(int id, double[] point, String name)
        {
            //helix3dsim.addSeqPoint(id, point, name);
        }

        private void toggle3DPanControl()
        {
            if (helix3dsim.TogglePanControl())
            {
                btn_3D_EnablePan.Background = Brushes.LightSalmon;
                btn_3D_EnablePan.Text = "Pan ON";
            }
            else
            {
                btn_3D_EnablePan.Background = Brushes.Gray;
                btn_3D_EnablePan.Text = "Pan OFF";
            }
        }

        private void btn_3D_EnablePan_Click(object sender, RoutedEventArgs e)
        {
            toggle3DPanControl();
        }

        private void btn_3DTracerClear_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.TracerClear();
        }

        private void btn_3DTracerToggle_Click(object sender, RoutedEventArgs e)
        {
            if (!helix3dsim.IsPathTracingEnabled)
            {
                helix3dsim.IsPathTracingEnabled = true;
                btn_3DTracerToggle.Content = "Tracer ON";
            }
            else
            {
                helix3dsim.IsPathTracingEnabled = false;
                btn_3DTracerToggle.Content = "Tracer OFF";
            }
        }

        private void btn_Calculate_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.Calculate();
        }

        public void UI_SimCalculateBeforeState()
        {
            // Disable buttons unnecessary during calculating the trajectory info
            btn_sim_PointPrevious.IsEnabled = false;
            btn_sim_PointNext.IsEnabled = false;
            btn_sim_PointFirst.IsEnabled = false;
            btn_sim_PointLast.IsEnabled = false;
            btn_Add_Default.IsEnabled = false;
            btn_sim_player_PlayForward.IsEnabled = false;
            btn_sim_player_PlayReverse.IsEnabled = false;
            btn_sim_player_StepForward.IsEnabled = false;
            btn_sim_player_StepReverse.IsEnabled = false;
            btn_sim_player_Pause.IsEnabled = false;
            if (sync3D_isEnabled)
            {
                helix3dsim.wasSyncEnabled = true;
                disableSync3DSim();
            }
            btn_Calculate.Content = "Calc...";
            lbl_total_length.Content = Convert.ToString(0);
        }

        public void UI_SimCalculateAfterState()
        {
            btn_sim_PointPrevious.IsEnabled = true;
            btn_sim_PointNext.IsEnabled = true;
            btn_sim_PointFirst.IsEnabled = true;
            btn_sim_PointLast.IsEnabled = true;
            btn_Add_Default.IsEnabled = true;
            btn_sim_player_PlayForward.IsEnabled = true;
            btn_sim_player_PlayReverse.IsEnabled = true;
            btn_sim_player_StepForward.IsEnabled = true;
            btn_sim_player_StepReverse.IsEnabled = true;
            btn_Calculate.Content = "Calculate";
            
            if (helix3dsim.wasSyncEnabled)
            {
                enableSync3DSim();
            }
        }

        private void btn_Add_Default_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.AddBezierDefault();
        }

        private void btn_Go_Home_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.goHomePosition();
        }

        private void btn_sim_player_Stop_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackStop();
        }

        private void btn_Add_Bezier_Curve_Click(object sender, RoutedEventArgs e)
        {
            //AddBezierCurve addWindow = new AddBezierCurve();
            //addWindow.Show();
        }

        private void btn_SimPointPrevious_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.goPreviousPathPoint();
        }

        private void btn_SimPointNext_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.goNextPathPoint();
        }

        private void btn_SimPointFirst_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.goFirstPathPoint();
        }

        private void btn_SimPointLast_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.goLastPathPoint();
        }

        public void show_ctrl_chk_Click(object sender, RoutedEventArgs e)
        {
            if (show_ctrl_chk.IsChecked.Value)
            {
                helix3dsim.toggleShowControlPoint(true);
            }
            else
            {
                helix3dsim.toggleShowControlPoint(false);
            }
        }

        private void btn_PlayForward_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackPlayForward();
        }

        private void btn_PlayReverse_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackPlayReverse();
        }

        private void btn_Pause_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackPause();
        }
        
        private void btn_Step_Forward_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackStepForward();
        }

        private void btn_Step_Reverse_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.PlaybackStepReverse();
        }

        private void btn_Add_Linear_Click(object sender, RoutedEventArgs e)
        {
            if (helix3dsim.bCurveSelected)
            {
                helix3dsim.bCurveSelected = false;
                Helix3dSim.makeLine(helix3dsim.nCurveSelected);
            }
        }

        private void btn_Add_Arc_Click(object sender, RoutedEventArgs e)
        {
            if (helix3dsim.bCurveSelected)
            {
                helix3dsim.bCurveSelected = false;
                Helix3dSim.makeArc(helix3dsim.nCurveSelected, false);
            }
        }

        private void btn_delete_Click(object sender, RoutedEventArgs e)
        {
            if (Helix3dSim.bSelected)
            {
                helix3dsim.deleteAndUpdate();
            }
        }

        private void btn_open_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.OpenFile();
        }

        private void btn_save_Click(object sender, RoutedEventArgs e)
        {
            helix3dsim.SaveFile();
        }

        private void btn_reset_Click(object sender, RoutedEventArgs e)
        {
            btn_sim_PointPrevious.IsEnabled = false;
            btn_sim_PointNext.IsEnabled = false;
            btn_sim_PointFirst.IsEnabled = false;
            btn_sim_PointLast.IsEnabled = false;
            btn_Calculate.IsEnabled = false;
            btn_sim_player_PlayForward.IsEnabled = false;
            btn_sim_player_PlayReverse.IsEnabled = false;
            btn_sim_player_StepReverse.IsEnabled = false;
            btn_sim_player_StepForward.IsEnabled = false;
            btn_sim_player_Stop.IsEnabled = false;
            btn_save.IsEnabled = false;
            btn_reset.IsEnabled = false;
            slider_sim_player_TimeSlider.Value = 0;
            lbl_total_length.Content = "0";
            tb_sim_pointdata_id.Text = "0";
            tb_sim_selpoint_x.Text = "0";
            tb_sim_selpoint_y.Text = "0";
            tb_sim_selpoint_z.Text = "0";
            tb_sim_selpoint_rx.Text = "0";
            tb_sim_selpoint_ry.Text = "0";
            tb_sim_selpoint_rz.Text = "0";
            tb_sim_TargetDistance.Text = "0";
            helix3dsim.reset();
        }

        private void tb_SimDuration_TextChanged(object sender, TextChangedEventArgs e)
        {
            TimeSpan time;
            if (TimeSpan.TryParseExact(tb_sim_duration.Text, @"hh\:mm\:ss\.fff", null, out time))
            {
                if (time.TotalSeconds <= 0)
                {
                    tb_sim_duration.Text = FormatTimeString(1); //Default 1 Second
                }
                else
                {
                    if(helix3dsim != null)
                    {
                        helix3dsim.totalDuration = time.TotalSeconds;
                    }
                }
            }
            else
            {
                MessageBox.Show("Invalid Time Duration Format. (hh:mm:ss.fff)","Error",MessageBoxButton.OK,MessageBoxImage.Error,MessageBoxResult.OK,MessageBoxOptions.None);
                if (helix3dsim != null)
                {
                    tb_sim_duration.Text = FormatTimeString(helix3dsim.totalDuration);
                }
            }
        }

        private void bias_Changed(object sender, TextChangedEventArgs e)
        {
            /*
            if (Helix3dSim.bSelected)
            {
                if (tb_sim_selpoint_x.Text == "")
                    tb_sim_selpoint_x.Text = "0";
                if (tb_sim_selpoint_y.Text == "")
                    tb_sim_selpoint_y.Text = "0";
                if (tb_sim_selpoint_z.Text == "")
                    tb_sim_selpoint_z.Text = "0";
                Point3D ptOrigin = new Point3D();
                if (Helix3dSim.strSelected == "point")
                    ptOrigin = Helix3dSim.movePaths[Helix3dSim.nSelected].ptCurve[Helix3dSim.nPoint].point;
                else if (Helix3dSim.strSelected == "control")
                    ptOrigin = Helix3dSim.movePaths[Helix3dSim.nSelected].ptAdjust[Helix3dSim.nPoint].point;
                ptOrigin.X += Convert.ToInt32(tb_sim_selpoint_x.Text);
                ptOrigin.Y += Convert.ToInt32(tb_sim_selpoint_y.Text);
                ptOrigin.Z += Convert.ToInt32(tb_sim_selpoint_z.Text);
                Helix3dSim.onUpdataGraphics(ptOrigin);
            }
            */
        }

        private void btn_Sync3DSim_Click(object sender, RoutedEventArgs e)
        {
            if (sync3D_isEnabled)
            {
                disableSync3DSim();
            }
            else
            {
                enableSync3DSim();
            }
        }

        public void enableSync3DSim()
        {
            sync3D_isEnabled = true;
            btn_Sync3DSim.Background = Brushes.LimeGreen;
        }

        public void disableSync3DSim()
        {
            sync3D_isEnabled = false;
            btn_Sync3DSim.Background = Brushes.Red;
        }

        private void show_tracking_line_Checked(object sender, RoutedEventArgs e)
        {
            if (show_tracking_line.IsChecked.Value)
            {
                helix3dsim.toggleShowTrackingLine(true);
            }
            else
            {
                helix3dsim.toggleShowTrackingLine(false);
            }
        }
        
        private void show_robot_arm_Click(object sender, RoutedEventArgs e)
        {
            if (helix3dsim != null)
            {
                if (show_robot_arm.IsChecked.Value)
                {
                    helix3dsim.toggleShowRobotArm(true);
                }
                else
                {
                    helix3dsim.toggleShowRobotArm(false);
                }
            }
        }

        private String FormatTimeString(double time)
        {
            return TimeSpan.FromSeconds(time).ToString(@"hh\:mm\:ss\.fff");
        }


        private static double ConvertRadiansToDegrees(double radians)
        {
            double degrees = (180 / Math.PI) * radians;
            return (degrees);
        }

        private void Load3DModel(object sender, RoutedEventArgs e)
        {
            Model3DGroup modelgroup = null;
            try
            {
                //Adding a gesture here
                //viewPort3d.RotateGesture = new MouseGesture(MouseAction.LeftClick);

                //Import 3D model file
                HelixToolkit.Wpf.ModelImporter import = new HelixToolkit.Wpf.ModelImporter();

                //Load the 3D model file
                OpenFileDialog openFileDialog = new OpenFileDialog();
                openFileDialog.Filter = "3D files (*.3ds)|*.3ds|All files (*.*)|*.*";
                if (openFileDialog.ShowDialog() == true)
                {
                    modelgroup = import.Load(openFileDialog.FileName);
                    ModelVisual3D model = new ModelVisual3D();
                    model.Content = modelgroup;

                    Vector3D position = new Vector3D(600, 0, 0);
                    Vector3D axis = new Vector3D(1, 0, 0); //In case you want to rotate it about the x-axis
                    Matrix3D transformationMatrix = model.Content.Transform.Value; //Gets the matrix indicating the current transformation value
                    transformationMatrix.Rotate(new Quaternion(axis, 0)); //Makes a rotation transformation over this matrix
                    //transformationMatrix.RotateAt(new Quaternion(axis, angle), modelCenter); //modelCenter is the Point3D variable indicating the center
                    transformationMatrix.Translate(position);
                    model.Content.Transform = new MatrixTransform3D(transformationMatrix); //Applies the transformation to your model

                    helix_viewport3d.Children.Add(model);

                    Rect3D bounds = model.Content.Bounds;

                    /*
                    HelixToolkit.Wpf.CombinedManipulator manipulator = new HelixToolkit.Wpf.CombinedManipulator();
                    manipulator.CanTranslateX = true;
                    manipulator.CanTranslateY = true;
                    manipulator.CanTranslateZ = true;
                    manipulator.CanRotateX = true;
                    manipulator.CanRotateY = true;
                    manipulator.CanRotateZ = true;
                    manipulator.Diameter = Math.Max(bounds.SizeX, Math.Max(bounds.SizeY, bounds.SizeZ)) * 1.2;
                    manipulator.Position = new Point3D(position.X,position.Y,position.Z);
                    manipulator.Pivot = new Point3D(position.X, position.Y, position.Z);
                    manipulator.TargetTransform = model.Transform;
                    manipulator.Transform = model.Transform;
                    manipulator.Bind(model);
                    helix_viewport3d.Children.Add(manipulator);
                    */
                    HelixToolkit.Wpf.TranslateManipulator man_x = new HelixToolkit.Wpf.TranslateManipulator();
                    HelixToolkit.Wpf.TranslateManipulator man_y = new HelixToolkit.Wpf.TranslateManipulator();
                    HelixToolkit.Wpf.TranslateManipulator man_z = new HelixToolkit.Wpf.TranslateManipulator();

                    man_x.Direction = new Vector3D(1, 0, 0);
                    man_x.Length = 200;
                    man_x.Diameter = 20;
                    man_x.Color = Colors.Red;
                    man_x.Offset = new Vector3D(30, 0, 0);
                    man_x.Position = new Point3D(position.X, position.Y, position.Z);
                    man_x.Transform = model.Transform;
                    man_x.TargetTransform = model.Transform;
                    man_x.Bind(model);

                    man_y.Direction = new Vector3D(0, 1, 0);
                    man_y.Length = 200;
                    man_y.Diameter = 20;
                    man_y.Color = Colors.Green;
                    man_y.Offset = new Vector3D(0, 30, 0);
                    man_y.Position = new Point3D(position.X, position.Y, position.Z);
                    man_y.Transform = model.Transform;
                    man_y.TargetTransform = model.Transform;
                    man_y.Bind(model);

                    man_z.Direction = new Vector3D(0, 0, 1);
                    man_z.Length = 200;
                    man_z.Diameter = 20;
                    man_z.Color = Colors.Blue;
                    man_z.Offset = new Vector3D(0, 0, 30);
                    man_z.Position = new Point3D(position.X, position.Y, position.Z);
                    man_z.Transform = model.Transform;
                    man_z.TargetTransform = model.Transform;
                    man_z.Bind(model);

                    helix_viewport3d.Children.Add(man_x);
                    helix_viewport3d.Children.Add(man_y);
                    helix_viewport3d.Children.Add(man_z);

                }
            }
            catch (Exception ex)
            {
                // Handle exception in case can not find the 3D model file
                MessageBox.Show("Exception Error : " + ex.StackTrace);
            }

        }

        #endregion

        private void switchFKandIK(object sender, RoutedEventArgs e)
        {
            allowFK = !allowFK;
            if (allowFK) btn_allow_FK.Content = "Allow FK";
            else btn_allow_FK.Content = "Allow IK";
        }
    }

}
