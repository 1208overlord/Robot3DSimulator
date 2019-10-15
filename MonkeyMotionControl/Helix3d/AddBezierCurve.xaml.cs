using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Monkey3DHelix
{
    /// <summary>
    /// Interaction logic for AddBezierCurve.xaml
    /// </summary>
    public partial class AddBezierCurve : Window
    {
        public AddBezierCurve()
        {
            InitializeComponent();
        }

        private void btn_Add_Point_Click(object sender, RoutedEventArgs e)
        {

        }

        private void btn_Close_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private void joint_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            MainWindow parentWindow = Window.GetWindow(this) as MainWindow;
            parentWindow.getHelix().UpdateRoboticArm(slider_3Djoint1.Value, slider_3Djoint2.Value, slider_3Djoint3.Value, slider_3Djoint4.Value, slider_3Djoint5.Value, slider_3Djoint6.Value);
        }
    }
}
