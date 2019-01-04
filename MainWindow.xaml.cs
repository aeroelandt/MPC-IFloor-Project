//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

using System.Collections.Generic;
using System.Windows.Controls;
using System.Windows.Media.Animation;
using WpfAnimatedGif;

namespace MPC_IFloor_Project
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Runtime.InteropServices;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        /// <summary>
        /// The DPI.
        /// </summary>
        private readonly double DPI = 96.0;

        /// <summary>
        /// Bytes per pixel.
        /// </summary>
        private readonly int BYTES_PER_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] _bodies = null;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush _handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush _handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private readonly DrawingGroup _drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private readonly DrawingImage _imageSource;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private readonly int _displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private readonly int _displayHeight;

        private readonly int _colorWidth;

        private readonly int _colorHeight;

        /// <summary>
        /// The RGB pixel values used for the background removal (green-screen) effect.
        /// </summary>
        private readonly byte[] _displayPixels = null;

        /// <summary>
        /// The color points used for the background removal (green-screen) effect.
        /// </summary>
        private readonly ColorSpacePoint[] _colorPoints = null;

        /// <summary>
        /// The depth values.
        /// </summary>
        private readonly ushort[] _depthData = null;

        /// <summary>
        /// The body index values.
        /// </summary>
        private readonly byte[] _bodyData = null;

        /// <summary>
        /// The RGB pixel values.
        /// </summary>
        private readonly byte[] _colorData = null;

        // DEFAULT VANAF HIER

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int _bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor _kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private readonly CoordinateMapper _coordinateMapper = null;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader _multiFrameSourceReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private readonly WriteableBitmap _bitmap = null;

        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private readonly uint _bitmapBackBufferSize = 0;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private readonly DepthSpacePoint[] _colorMappedToDepthPoints = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string _statusText = null;

        private string _bodyCoords = null;
        private string _bodyIndexCoords = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private readonly List<Tuple<JointType, JointType>> _bones;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush _trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush _inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen _inferredBonePen = new Pen(Brushes.Gray, 1);

        private bool _isBitmapLocked;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            _kinectSensor = KinectSensor.GetDefault();

            _multiFrameSourceReader =
                _kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color |
                                                         FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            _multiFrameSourceReader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            // Create the drawing group we'll use for drawing
            _drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            _imageSource = new DrawingImage(_drawingGroup);

            _coordinateMapper = _kinectSensor.CoordinateMapper;

            var depthFrameDescription = _kinectSensor.DepthFrameSource.FrameDescription;

            _displayWidth = depthFrameDescription.Width;
            _displayHeight = depthFrameDescription.Height;


            var colorFrameDescription = _kinectSensor.ColorFrameSource.FrameDescription;

            _colorWidth = colorFrameDescription.Width;
            _colorHeight = colorFrameDescription.Height;

            _colorMappedToDepthPoints = new DepthSpacePoint[_colorWidth * _colorHeight];

            _bitmap = new WriteableBitmap(_colorWidth, _colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            _depthData = new ushort[_displayWidth * _displayHeight];
            _bodyData = new byte[_displayWidth * _displayHeight];
            _colorData = new byte[_colorWidth * _colorHeight * BYTES_PER_PIXEL];
            _displayPixels = new byte[_displayWidth * _displayHeight * BYTES_PER_PIXEL];
            _colorPoints = new ColorSpacePoint[_displayWidth * _displayHeight];

            // Calculate the WriteableBitmap back buffer size
            _bitmapBackBufferSize = (uint)((_bitmap.BackBufferStride * (_bitmap.PixelHeight - 1)) +
                                           (_bitmap.PixelWidth * _bytesPerPixel));

            _kinectSensor.IsAvailableChanged += Sensor_IsAvailableChanged;

            _kinectSensor.Open();

            StatusText = Properties.Resources.OpenHands;

            #region DrawBody

            // a bone defined as a line between two joints
            _bones = new List<Tuple<JointType, JointType>>();

            // Torso
            _bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            _bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            _bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            _bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            _bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            _bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            _bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            _bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            #endregion

            DataContext = this;
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext, string hand)
        {
            switch (handState)
            {
                case HandState.Open:
                    drawingContext.DrawEllipse(_handOpenBrush, null, handPosition, HandSize, HandSize);
                    Image image = new Image
                    {
                        Width = 100,
                        Uid = flowerCanvas.Children.Count.ToString()
                        //Name = "X" + handPosition.X
                    };
                    var bmImage = new BitmapImage();
                    bmImage.BeginInit();
                    bmImage.UriSource = new Uri("Images/growingFlower.gif", UriKind.Relative);
                    bmImage.EndInit();
                    image.Source = bmImage;
                    ImageBehavior.SetAnimatedSource(image, bmImage);
                    ImageBehavior.SetRepeatBehavior(image, new RepeatBehavior(1));

                    flowerCanvas.Children.Add(image);
                    if (flowerCanvas.Children.Count > 20)
                    {
                        var element = flowerCanvas.Children[flowerCanvas.Children.Count - 21];
                        flowerCanvas.Children.Remove(element);
                    }
                    Canvas.SetTop(image, handPosition.Y * 2.5);
                    Canvas.SetLeft(image, handPosition.X * 2.5);
                    break;
                case HandState.Closed:
                    drawingContext.DrawEllipse(_handClosedBrush, null, handPosition, HandSize, HandSize);
                    //var enume = flowerCanvas.Children.GetEnumerator();
                    //if (enume.MoveNext())
                    //    BodyCoords = enume.Current.ToString();
                    break;
            }
        }

        #region DrawBody

        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
            DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in _bones)
            {
                DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (var jointType in joints.Keys)
            {
                Brush drawBrush = null;

                var trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = _trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = _inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
            JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            var joint0 = joints[jointType0];
            var joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            var drawPen = _inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        #endregion

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource => _bitmap;

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get { return _statusText; }

            set
            {
                if (_statusText != value)
                {
                    _statusText = value;

                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("StatusText"));
                }
            }
        }

        public string BodyCoords
        {
            get { return _bodyCoords; }

            set
            {
                if (_bodyCoords != value)
                {
                    _bodyCoords = value;

                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("BodyCoords"));
                }
            }
        }

        public string BodyIndexCoords
        {
            get { return _bodyIndexCoords; }

            set
            {
                if (_bodyIndexCoords != value)
                {
                    _bodyIndexCoords = value;

                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("BodyIndexCoords"));
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (_multiFrameSourceReader != null)
            {
                // MultiSourceFrameReder is IDisposable
                _multiFrameSourceReader.Dispose();
                _multiFrameSourceReader = null;
            }

            if (_kinectSensor == null) return;
            _kinectSensor.Close();
            _kinectSensor = null;
        }

        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyIndexFrame bodyIndexFrame = null;
            BodyFrame bodyFrame = null;

            _isBitmapLocked = false;

            var multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null) || bodyFrame == null)
                {
                    return;
                }

                ProcessBackgroundOld(depthFrame, colorFrame, bodyIndexFrame);

                ProcessBody(bodyFrame, false);
            }
            finally
            {
                if (_isBitmapLocked)
                {
                    _bitmap.Unlock();
                }

                depthFrame?.Dispose();

                colorFrame?.Dispose();

                bodyIndexFrame?.Dispose();

                bodyFrame?.Dispose();
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            StatusText = _kinectSensor.IsAvailable
                ? Properties.Resources.RunningStatusText
                : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSourceHands => _imageSource;

        public void ProcessBackgroundOld(DepthFrame depthFrame, ColorFrame colorFrame, BodyIndexFrame bodyIndexFrame)
        {
            // Process Depth
            var depthWidth = _displayWidth;
            var depthHeight = _displayHeight;

            // Access the depth frame data directly via LockImageBuffer to avoid making a copy
            using (var depthFrameData = depthFrame.LockImageBuffer())
            {
                _coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                    depthFrameData.UnderlyingBuffer,
                    depthFrameData.Size,
                    _colorMappedToDepthPoints);
            }

            // We're done with the DepthFrame 
            depthFrame.Dispose();

            // Lock the bitmap for writing
            _bitmap.Lock();
            _isBitmapLocked = true;

            colorFrame.CopyConvertedFrameDataToIntPtr(_bitmap.BackBuffer, _bitmapBackBufferSize, ColorImageFormat.Bgra);

            // We're done with the ColorFrame 
            colorFrame.Dispose();

            // We'll access the body index data directly to avoid a copy
            using (var bodyIndexData = bodyIndexFrame.LockImageBuffer())
            {
                unsafe
                {
                    var bodyIndexDataPointer = (byte*)bodyIndexData.UnderlyingBuffer;

                    var colorMappedToDepthPointCount = _colorMappedToDepthPoints.Length;

                    fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = _colorMappedToDepthPoints)
                    {
                        // Treat the color data as 4-byte pixels
                        var bitmapPixelsPointer = (uint*)_bitmap.BackBuffer;

                        // Loop over each row and column of the color image
                        // Zero out any pixels that don't correspond to a body index
                        for (var colorIndex = 0; colorIndex < colorMappedToDepthPointCount; ++colorIndex)
                        {
                            var colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                            var colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                            // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                            if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                                !float.IsNegativeInfinity(colorMappedToDepthY))
                            {
                                // Make sure the depth pixel maps to a valid point in color space
                                var depthX = (int)(colorMappedToDepthX);
                                var depthY = (int)(colorMappedToDepthY);

                                // If the point is not valid, there is no body index there.
                                if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                                {
                                    var depthIndex = (depthY * depthWidth) + depthX;

                                    // If we are tracking a body for the current pixel, do not zero out the pixel
                                    if (bodyIndexDataPointer[depthIndex] != 0xff)
                                    {
                                        continue;
                                    }
                                }
                            }

                            bitmapPixelsPointer[colorIndex] = 0;
                        }
                    }
                    _bitmap.AddDirtyRect(new Int32Rect(0, 0, _bitmap.PixelWidth, _bitmap.PixelHeight));
                }
            }
        }

        public void ProcessBackgroundNew(DepthFrame depthFrame, ColorFrame colorFrame, BodyIndexFrame bodyIndexFrame)
        {
            var depthWidth = _displayWidth;
            var depthHeight = _displayHeight;

            var bodyIndexWidth = bodyIndexFrame.FrameDescription.Width;
            var bodyIndexHeight = bodyIndexFrame.FrameDescription.Height;

            if (((depthWidth * depthHeight) == _depthData.Length) && ((_colorWidth * _colorHeight * BYTES_PER_PIXEL) == _colorData.Length) && ((bodyIndexWidth * bodyIndexHeight) == _bodyData.Length))
            {
                depthFrame.CopyFrameDataToArray(_depthData);

                if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    colorFrame.CopyRawFrameDataToArray(_colorData);
                }
                else
                {
                    colorFrame.CopyConvertedFrameDataToArray(_colorData, ColorImageFormat.Bgra);
                }

                bodyIndexFrame.CopyFrameDataToArray(_bodyData);

                _coordinateMapper.MapDepthFrameToColorSpace(_depthData, _colorPoints);

                Array.Clear(_displayPixels, 0, _displayPixels.Length);

                for (var y = 0; y < depthHeight; ++y)
                {
                    for (var x = 0; x < depthWidth; ++x)
                    {
                        var depthIndex = (y * depthWidth) + x;

                        var player = _bodyData[depthIndex];

                        if (player == 0xff) continue;
                        var colorPoint = _colorPoints[depthIndex];

                        var colorX = (int)Math.Floor(colorPoint.X + 0.5);
                        var colorY = (int)Math.Floor(colorPoint.Y + 0.5);

                        if ((colorX >= 0) && (colorX < _colorWidth) && (colorY >= 0) && (colorY < _colorHeight))
                        {
                            var colorIndex = ((colorY * _colorWidth) + colorX) * BYTES_PER_PIXEL;
                            var displayIndex = depthIndex * BYTES_PER_PIXEL;

                            _displayPixels[displayIndex + 0] = _colorData[colorIndex];
                            _displayPixels[displayIndex + 1] = _colorData[colorIndex + 1];
                            _displayPixels[displayIndex + 2] = _colorData[colorIndex + 2];
                            _displayPixels[displayIndex + 3] = 0xff;
                        }
                    }
                }

                _bitmap.Lock();

                Marshal.Copy(_displayPixels, 0, _bitmap.BackBuffer, _displayPixels.Length);
                _bitmap.AddDirtyRect(new Int32Rect(0, 0, depthWidth, depthHeight));

                _bitmap.Unlock();
            }

        }

        public void ProcessBody(BodyFrame bodyFrame, bool showSkeleton)
        {
            // Process handstate
            if (_bodies == null)
            {
                _bodies = new Body[bodyFrame.BodyCount];
            }

            // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
            // As long as those body objects are not disposed and not set to null in the array,
            // those body objects will be re-used.
            bodyFrame.GetAndRefreshBodyData(_bodies);

            using (var dc = _drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(new SolidColorBrush(Color.FromArgb(0, 0, 0, 0)), null,
                    new Rect(0.0, 0.0, _displayWidth, _displayHeight));

                foreach (var body in _bodies)
                {
                    if (!body.IsTracked) continue;
                    var joints = body.Joints;

                    // convert the joint points to depth (display) space
                    var jointPoints = new Dictionary<JointType, Point>();

                    foreach (var jointType in joints.Keys)
                    {
                        // sometimes the depth(Z) of an inferred joint may show as negative
                        // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                        var position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }

                        var depthSpacePoint =
                            _coordinateMapper.MapCameraPointToDepthSpace(position);

                        var x = depthSpacePoint.X + ((_displayWidth - depthSpacePoint.X) * ((_displayWidth - depthSpacePoint.X) / 5000));
                        var y = depthSpacePoint.Y * 0.675;

                        var point = new Point(x, y);
                        jointPoints[jointType] = point;
                    }

                    var drawPen = new Pen(Brushes.Red, 6); //red for hands

                    if (showSkeleton)
                        DrawBody(joints, jointPoints, dc, drawPen);

                    DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc, "left");
                    DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc, "right");
                }

                // prevent drawing outside of our render area
                _drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _displayWidth, _displayHeight));
            }
        }
    }
}
