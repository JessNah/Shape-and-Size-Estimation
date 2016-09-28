//------------------------------------------------------------------------------

//Code first saves default view (View of empty belt)
//Everytime there's been a movement followed by 1.5 seconds of no movement at all (object is steadily placed or been removed or any other event occurs) (STATUS gets updated only following a movement event)
//The code will compare the current view with the default view of nothing on the belt. If there is a big difference and if the difference indicates a nearer depth (something come into view, rather than something having left the view)
//The code will then check to see if this difference corresponds to a depth change of pixels being closely together (one big object rather than alot of noise all over or small things come into view of the sensor here and there that adds up to alot of depth change)
//If so, the code will then indicate that an object/bag is on the belt.
//The code will average 3 subsequent images of this object to remove noise and have a solid image of the object alone.

//note: #pixels in a Kinect frame = 217,088  with 512w and 424h

// CTRL F "CODE TO VIEW GRIDSIZE" to find codes to uncomment to run and view gridsize.
// Set j = 0 initialized to print the image difference once.

//Testing and development code has been left and is commented.

//------------------------------------------------------------------------------


namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Timers;
    using System.Collections.Generic;
    using System.Linq;    /// <summary>
                          /// Interaction logic for MainWindow
                          /// </summary>
    public partial class Window : System.Windows.Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;  
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;
            
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;
        private byte[] printPixels = null;
        private byte[] prevDepthPixels = null; 
        private byte[] defaultb4MovDepthPixels = null;
        private ushort[] actualDepth = null;
        private int[,] twoD = null;
        private int[,] tempTwoD = null;
        private int[,] toPrint = null;
        private int[,] toPrint1 = null;
        private int[,] toPrint2 = null;

        private System.Timers.Timer noMoveTimer;
        private System.Timers.Timer caliTimer;

        private int k = 0; //flag variable, used for first time obtaining of "prevFrame" and for one time calibration.
        private int count = 0; //counts number of pixels that observe depth change
        private List<int> list = new List<int>(); //used to hold counts during calibration
        private int standardError = 0;
        private int movement = 0;  //bool holds the value for if a recent movement occured
        private int frameSize = 0; //initialized
        private int stateCount = 0;
        private int stateCount2 = 0;
        private int pixelProximity = 0;
        private int maxpp = 0;
        private List<int> pixelDiffs = new List<int>();
        private List<int> leftArray = new List<int>();
        private List<int> rightArray = new List<int>();
        private List<int> botArray = new List<int>();
        private List<int> topArray = new List<int>();
        private List<int> tempDiffs = new List<int>();
        private int xxx = 0;
        private int yyy = 0;
        private int j = 0;             
        private int printCount = 0;
        private int defaultTemp = 0;
       //private int yyyy = 0;  //CODE TO VIEW GRIDSIZE

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 


        public Window()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            
            //additional space allocated for saving different versions of the views received, used by various features of the code (default view with no movement, 3 versions of the image used to average out noise, etc.)
            this.prevDepthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.defaultb4MovDepthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.printPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.actualDepth = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.twoD = new int[this.depthFrameDescription.Height, this.depthFrameDescription.Width];
            this.tempTwoD = new int[this.depthFrameDescription.Height, this.depthFrameDescription.Width];
            this.toPrint = new int[this.depthFrameDescription.Height, this.depthFrameDescription.Width];
            this.toPrint1 = new int[this.depthFrameDescription.Height, this.depthFrameDescription.Width];
            this.toPrint2 = new int[this.depthFrameDescription.Height, this.depthFrameDescription.Width];
            
            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        } //end Window

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.depthBitmap;
            }
        } //end ImageSource

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
        } //end StatusText

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        } //end MainWindow_Closing

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.depthBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                }
            }//end if
        }//end ScreenshotButton_Click

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;
                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //maxDepth = depthFrame.DepthMaxReliableDistance;

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }//end using buffer
                }//end if
            }
            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }//end Reader_FrameArrived

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// The code first calibrates to get a view of the default image, with no object.
        /// The code also calibrates the amount of natural noise in the variable count, so that it can distinguish actual movement from noise.
        /// The function then upon movement, starts a timer which launches the event which differentiates if an object is indeed there and receives its information.
        /// If movement occurs, the timer is cancelled (Object is still in motion and not properly settled/placed)
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            frameSize = (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel);

            if (k == 0)  //k is a flag variable. @ k = 0 for the very first frame, dont compare to "previous"
            {
                for (int i = 0; i < frameSize; ++i)
                {
                    // Get the depth for this pixel
                    ushort depth = frameData[i];

                    // To convert to a byte, we're mapping the depth value to the byte range.
                    // Values outside the reliable depth range are mapped to 0 (black).
                    this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                    this.actualDepth[i] = (ushort)(depth >= minDepth && depth <= maxDepth ? (depth) : 0); // Depth not mapped to byte range (Actual Depth value)
                    this.prevDepthPixels[i] = this.depthPixels[i];

                }
                k = 1;
            }
            else
            { 
                // convert depth to a visual representation
                for (int i = 0; i < frameSize; ++i)
                {
                    // Get the depth for this pixel
                    ushort depth = frameData[i];

                    // To convert to a byte, we're mapping the depth value to the byte range.
                    // Values outside the reliable depth range are mapped to 0 (black).
                    this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                    this.actualDepth[i] = (ushort)(depth >= minDepth && depth <= maxDepth ? (depth) : 0);

                      // CODE TO VIEW GRIDSIZE for testing   (30 by 30 square)
                     /*
                    int q = 167656; 
                    for (int w = 0; w < 15; w++)
                    {
                        for (int e = 1; e <=15; e++)
                        {
                        this.depthPixels[q + (512 * w) + (e - 1)] = (byte) 250; 
                        }
                    }  */

                    if (Math.Abs(this.depthPixels[i] - this.prevDepthPixels[i]) > 5) //if depth color difference greater than 5  (max is 256, black is 0)
                    {
                        count++; //count keeps track of number of pixels that have a notable change in depth
                    }
                    this.prevDepthPixels[i] = this.depthPixels[i]; ////  save the last version of the frame  ***
                }
                if (k == 1)   //@ k = 1 calibrate the standard changes in pixel depths (noise/error)
                {
                    calibrateTimer();
                    if (caliTimer != null) { list.Add(count); }
                }
                else if (k == 2) //Saves the default view (Without objects placed)
                {
                    for (int i = 0; i < frameSize; ++i)
                    {
                        this.defaultb4MovDepthPixels[i] = this.prevDepthPixels[i];
                    }
                    k = 3;
                    
                } // default set, move to cycling check stage v
                else if (k == 3)
                {
                    if ((count < standardError + (frameSize * 0.0025)) && (movement == 0)) //after movement stops 
                    {
                        stateCount++;
                        if (stateCount >= 60)
                        {
                            if (noMoveTimer != null) { noMoveTimer.Enabled = false; } //cancel any running instance of timer
                            noMovementTimer(); //run timer to check if no movement for 1 second, then check if there is now a change in view (an object)
                            movement = 1;
                            stateCount = 0;

                            //this.StatusText = actualDepth[146600].ToString(); //test location to find out distance amount and this specified location in mm
                        }
                    } //end if
                    if (count > (standardError + (frameSize * 0.0025)) && (movement == 1)) //if movement happens, cancel timer and reset it
                    {
                        stateCount2++;
                        if (stateCount2 >= 30)
                        {
                            if (noMoveTimer != null) { noMoveTimer.Enabled = false; } //cancel any running instance of timer
                            movement = 0;
                            stateCount2 = 0;
                        }
                    } //end if
                }
                count = 0;
            } //end else
        }//end ProcessDepthFrameData

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
            // CODE TO VIEW GRIDSIZE   - Code used duiring testing/developtment, to get an idea of how much of the view, a square of pixels takes up.
           /* yyyy = yyyy + 1;
            if(yyyy == 990000000)
            {
                int k = 0;  // Pause
            }*/
        } //end RenderDepthPixels

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }//end Sensor_IsAvailableChanged

        /// <summary>
        /// Calibrate timer which launches the calibrate event after 1 second
        /// </summary>
        private void calibrateTimer()
        {
            // Create a timer with a 1 second calibrate interval.
            caliTimer = new System.Timers.Timer(1000);
            // Hook up the Elapsed event for the timer. 
            caliTimer.Elapsed += OnCalibrateEvent;
            caliTimer.AutoReset = false;
            caliTimer.Enabled = true;
        }//end CalibrateTimer

        /// <summary>
        /// Calibrate event calculates the standard error noise amount so that we can differentiate this from actual movement.
        /// </summary>
        private void OnCalibrateEvent(Object source, ElapsedEventArgs e)
        {
            int nonZeroCount = 0;
            for (int i = 0; i < list.Count - 1; i++)
            {
                if (list[i] != 0)
                {
                    standardError = standardError + list[i];
                    nonZeroCount++;
                }
            }
            standardError = (standardError / nonZeroCount);
            k = 2;
            caliTimer.Stop();
            caliTimer.Enabled = false;
        }//end CalibrateEvent

        /// <summary>
        /// noMovementTimer launches the twoSecCheck event, is launched after a movement occurs
        /// </summary>
        private void noMovementTimer()  // if this event gets to occur, it means possible object and check will occur.
        {
            // Create a timer with a 1.5 second interval.
            noMoveTimer = new System.Timers.Timer(1500);
            // Hook up the Elapsed event for the timer. 
            noMoveTimer.Elapsed += noMovementCheck;
            noMoveTimer.AutoReset = true;
            noMoveTimer.Enabled = true;
        } //end noMovementTimer

        /// <summary>
        /// twoSecCheck event occurs after 1500 seconds of there being no movement having taken place 
        /// (movement occurerd (object got placed), movement stopped (object is settled in view)
        /// The code first checks if there is a notable difference having taken place in the view from the default view
        /// If so, it transfers the current view into a 2D array where 1 denotes an area of depth change and 0 denotes a pixel of no depth change
        /// Then the code iterates through the 2d array, saving the largest collection of pixels for which a depth change occurered (to differentiate larger areas of noise from an actual object)
        /// The top left location where this large collection of pixels occured is noted to mark the location of the object
        /// The code is run through 3 times, to save 3 different timed images of the object (collection of pixels) so that they can be averaged to cancel out noise and get a more accurate image.
        /// A black and white image of this object's shape is then saved to the computer's pictures folder.
        /// </summary>
        private void noMovementCheck(Object source, ElapsedEventArgs e)
        {
            int difference = 0;
            maxpp = 0;
            for (int i = 0; i < frameSize; ++i)
            {   //calculate if there is a notable difference in depth changes from the default to the current view
                difference = difference + (this.defaultb4MovDepthPixels[i] - this.depthPixels[i]);
            }
            if (difference > 4000) //If there is a notable difference
            {
                pixelProximity = 0;
                pixelDiffs.Clear();
                for (int i = 0; i < frameSize; ++i)
                {  
                    difference = (Math.Abs(this.defaultb4MovDepthPixels[i] - this.depthPixels[i]));
                    if (difference > 3)  // if there is a substantial difference in depths, save the index that this was observed at. Otherwise save 0 in pixelDiffs list.
                    {
                        pixelDiffs.Add(1); //store the indexes of the pixels, if a pixel has not much difference, store 0
                        tempDiffs.Add(0); //initialize to 0
                    }
                    else
                    {
                        pixelDiffs.Add(0);
                    }
                }

                //transfers the image differences onto a 2d array
                int xx = 0;
                int yy = 0;
                for (int z = 0; z < pixelDiffs.Count; z++)
                {
                    twoD[xx, yy] = pixelDiffs[z];
                    yy++;
                    if ((z + 1) % 512 == 0)
                    {
                        xx++;
                        yy = 0;
                    }
                }
                
                //twoD holds original 2d array of differences
                //tempTwoD holds the largest blob version of the image
                for (int m = 0; m < twoD.GetLength(0); m++)
                {
                    for (int n = 0; n < twoD.GetLength(1); n++)
                    {
                        if (twoD[m, n] != 0) //if there is a difference observed while iterating through twoD array
                        {
                            floodFill(m, n); //use the floodFill algorithm function to note the size and shape of the blob that this pixel is part of

                            if (pixelProximity > maxpp) //save location where max difference was found
                            {
                                maxpp = pixelProximity; //oixelProximity variable is used to keep track of size of blobs for comparison
                                xxx = m;       //top left coordinates
                                yyy = n;
                                // save different image versions of the blob, while observed at different times.
                                if (printCount == 0)
                                {
                                    Array.Clear(toPrint, 0, toPrint.Length);
                                }
                                else if (printCount == 1)
                                {
                                    Array.Clear(toPrint1, 0, toPrint1.Length);
                                }
                                else if (printCount == 2)
                                {
                                    Array.Clear(toPrint2, 0, toPrint2.Length);
                                }
                                for (int a = 0; a < tempTwoD.GetLength(0); a++)
                                {
                                    for (int s = 0; s < tempTwoD.GetLength(1); s++)
                                    {
                                        if (tempTwoD[a, s] != 1)
                                        {
                                            tempTwoD[a, s] = 0;
                                        }
                                        if (printCount == 0)
                                        {
                                            toPrint[a, s] = tempTwoD[a, s];   //save the image of differences that had the greatest diff
                                        }
                                        else if (printCount == 1)
                                        {
                                            toPrint1[a, s] = tempTwoD[a, s];   //save the image of differences that had the greatest diff
                                        }
                                        else if (printCount == 2)
                                        {
                                            toPrint2[a, s] = tempTwoD[a, s];   //save the image of differences that had the greatest diff
                                        }
                                    }
                                }//end for
                            } // end if

                            Array.Clear(tempTwoD, 0, tempTwoD.Length);
                            pixelProximity = 0;
                        }
                    }
                } // end for loop
                
                printCount++; //increment so that the next time this function is called, another array of the blob image is saved instead
                string objState = null;
                objState = maxpp.ToString();
                //this.StatusText = objState + "   " + xxx.ToString() + "   " + yyy.ToString();

                if (printCount == 3) //after the 3 different images are acquired.
                {
                    //averages the 3 different images to get a more accurate image with less noise
                    for (int m = 0; m < toPrint.GetLength(0); m++)
                    {
                        for (int n = 0; n < toPrint.GetLength(1); n++)
                        {
                            if (toPrint[m, n] + toPrint1[m, n] + toPrint2[m, n] >= 2)
                            {
                                toPrint[m, n] = 1;
                            }
                            else
                            {
                                toPrint[m, n] = 0;
                            }
                        }
                    }//end for
                    

                    int wTest = 0;
                    int wCount = 0;
                    int wFinal = 0;
                    int hTest = 0;
                    int hCount = 0;
                    int hFinal = 0;

                    int left = toPrint.GetLength(1);
                    int right = 0;
                    int top = toPrint.GetLength(0);
                    int bot = 0;

                    for (int m = 0; m < toPrint.GetLength(0); m++)
                    {
                        left = toPrint.GetLength(1);
                        right = 0;
                        for (int n = 0; n < toPrint.GetLength(1); n++)
                        {
                            // gets the left and right most pixel indexes of the object
                            if (m + 2 < toPrint.GetLength(0) && m - 2 > 0)
                            {
                                if (n < left && toPrint[m, n] == 1 && toPrint[m + 1, n] == 1 && toPrint[m - 1, n] == 1 && toPrint[m + 2, n] == 1 && toPrint[m - 2, n] == 1)
                                {
                                    left = n;
                                }
                                if (n > right && toPrint[m, n] == 1 && toPrint[m + 1, n] == 1 && toPrint[m - 1, n] == 1 && toPrint[m + 2, n] == 1 && toPrint[m - 2, n] == 1)
                                {
                                    right = n;
                                }
                            }
                            // Ensures all non 1 values are set to 0 and prints the image to the output console.
                            if (toPrint[m, n] != 1)
                            {
                                toPrint[m, n] = 0;
                            }
                            if(toPrint[m, n] == 1)  //used to calculate center of mass coordinates
                            {
                                wTest = wTest + n;
                                wCount++;
                                hTest = hTest + m;
                                hCount++;
                            }
                        }
                        if (left != toPrint.GetLength(1))
                        {
                            leftArray.Add(left);
                        }
                        if (right != 0)
                        {
                            rightArray.Add(right);
                        }
                    }//end for

                    for (int n = 0; n < toPrint.GetLength(1); n++)
                    {
                        top = toPrint.GetLength(0);
                        bot = 0;
                        for (int m = 0; m < toPrint.GetLength(0); m++)
                        {
                            //gets the bottom and top most pixel indexes of the object
                            if (n + 2 < toPrint.GetLength(1) && n - 2 > 0)
                            {
                                if (m > bot && toPrint[m, n] == 1 && toPrint[m, n + 1] == 1 && toPrint[m, n - 1] == 1 && toPrint[m, n + 2] == 1 && toPrint[m, n - 2] == 1)
                                {
                                    bot = m;
                                }
                                if (m < top && toPrint[m, n] == 1 && toPrint[m, n + 1] == 1 && toPrint[m, n - 1] == 1 && toPrint[m, n + 2] == 1 && toPrint[m, n - 2] == 1)
                                {
                                    top = m;
                                }
                            }
                        }
                        if (top != toPrint.GetLength(0))
                        {
                            topArray.Add(top);
                        }
                        if (bot != 0)
                        {
                            botArray.Add(bot);
                        }
                    }//end for

                    //  First attempted method for finding edges -> ignore the corners and average the coordinates for the side.  This method is better if we manage to get an overage better image of the item with less noise
                    left = 0;
                    right = 0;
                    top = 0;
                    bot = 0;
                    int errorFixL = (int)Math.Round(leftArray.Count * 0.07,0);
                    int errorFixR = (int)Math.Round(rightArray.Count * 0.07,0);
                    int errorFixT = (int)Math.Round(topArray.Count * 0.07, 0);
                    int errorFixB = (int)Math.Round(botArray.Count * 0.07, 0);
                    for (int z = errorFixL; z < leftArray.Count - errorFixL; z++)
                    {
                        left = left + leftArray[z];
                    }
                    left = left / (leftArray.Count - (2 * errorFixL));

                    for (int z = errorFixR; z < rightArray.Count - errorFixR; z++)
                    {
                        right = right + rightArray[z];
                    }
                    right = right / (rightArray.Count - (2*errorFixR));

                    for (int z = errorFixT; z < topArray.Count - errorFixT; z++)
                    {
                        top = top + topArray[z];
                    }
                    top = top / (topArray.Count - (2 * errorFixT));

                    for (int z = errorFixB; z < botArray.Count - errorFixB; z++)
                    {
                        bot = bot + botArray[z];
                    }
                    bot = bot / (botArray.Count - (2 * errorFixB));
                    

                    //Second attempted method for finding edges -> find the most frequently occuring coordinate value for each side.
                    int left2 = leftArray.GroupBy(i => i).OrderByDescending(grp => grp.Count())
                    .Select(grp => grp.Key).First();
                    int right2 = rightArray.GroupBy(i => i).OrderByDescending(grp => grp.Count())
                    .Select(grp => grp.Key).First();
                    int bot2 = botArray.GroupBy(i => i).OrderByDescending(grp => grp.Count())
                    .Select(grp => grp.Key).First();
                    int top2 = topArray.GroupBy(i => i).OrderByDescending(grp => grp.Count())
                    .Select(grp => grp.Key).First();


                    //averaging the 2 methods above with a greater weight on the second method(currently preferred)
                    left = ((left*1) + (left2 * 9)) / 10;
                    right = ((right * 1) + (right2 * 9)) / 10;
                    bot = ((bot * 1) + (bot2 * 9)) / 10;
                    top = ((top * 1) + (top2 * 9)) / 10;

                    // Comments just to note calculations for trying to convert pixel image scale of object to real world scale to get object's measurements.
                    //280 mm = 160  pixels  @ 600 mm
                    //280 mm = 140 pixels  @ 800 mm
                    //280 mm = 130 pixels  @ 900 mm
                    //280 mm = 120 pixels  @ 1000 mm

                    //0.0014(depth) + 0.8701 = multiplier
                    //multiplier * pixelsWidth = actual distance in mm


                    //for Testing, creates a mark on the center of the image
                    wFinal = wTest / wCount;
                    hFinal = hTest / hCount;
                    for (int n = 1; n < 4; n++)
                    {
                        toPrint[hFinal - n, wFinal] = 2;
                        toPrint[hFinal, wFinal + n] = 2;
                        toPrint[hFinal + n, wFinal] = 2;
                        toPrint[hFinal, wFinal - n] = 2;
                    }
                    
                    //for visual testing on image saved
                    for (int f = 0; f < toPrint.GetLength(0); f++)
                    {
                        toPrint[f, left] = 2;
                    }

                    for (int f = 0; f < toPrint.GetLength(0); f++)
                    {
                        toPrint[f, right] = 2;
                    }
                    for (int f = 0; f < toPrint.GetLength(1); f++)
                    {
                        toPrint[top,f] = 2;
                    }

                    for (int f = 0; f < toPrint.GetLength(1); f++)
                    {
                        toPrint[bot, f] = 2;
                    }

                    //testing text information on object
                    int width = right - left;
                    int height = bot - top;
                    Console.WriteLine("Width of object(pixels): " + width);
                    Console.WriteLine("Height of object(pixels): " + height);
                    int locationDepth = (512 * hFinal) + wFinal;
                    Console.WriteLine("Depth of object: " + actualDepth[locationDepth]);  //actualDepth[167656]
                    double doubWidthOfObject = ((0.0014* actualDepth[locationDepth]) + 0.8701) * width;
                    int widthOfObject = (int)doubWidthOfObject;
                    double doubHeightOfObject = ((0.0014 * actualDepth[locationDepth]) + 0.8701) * height;
                    int heightOfObject = (int)doubHeightOfObject;
                    Console.WriteLine("Actual Width(mm) of object: " + (widthOfObject));
                    Console.WriteLine("Actual Width(inch) of object: " + (widthOfObject * 0.0393701));
                    Console.WriteLine("Actual Height(mm) of object: " + (heightOfObject));
                    Console.WriteLine("Actual Height(inch) of object: " + (heightOfObject * 0.0393701));
                    this.StatusText = "Width: " + widthOfObject + "mm / " + (widthOfObject * 0.0393701) + "in   |   Height: " + heightOfObject + "mm / " + (heightOfObject * 0.0393701) + "in";


                    //saves a screenshot of the difference image in the print folder.
                    if (j == 0)
                    {
                        this.Dispatcher.Invoke((Action)(() =>
                        {
                            byte val = 0;
                            xx = 0;
                            yy = 0;
                            val = 0;
                            for (int z = 0; z < frameSize; z++) //Copies the image back into a byte array from the 2D array
                            {
                                if((toPrint[xx, yy]) == 2) //marks the center on the image
                                {
                                    val = 120;
                                }
                                else if ((toPrint[xx, yy]) == 1) //blackens the difference image object
                                { val = 0;}
                                else { val = 250; } //makes everything else white
                                this.printPixels[z] = val;
                                yy++;
                                if ((z + 1) % 512 == 0)
                                {
                                    xx++;
                                    yy = 0;
                                }
                            }//end for


                            //Renders the image into a bitmap and saves it in the pictures folder
                            this.depthBitmap.WritePixels(
                            new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                            this.printPixels,
                            this.depthBitmap.PixelWidth,
                            0);

                            if (this.depthBitmap != null)
                            {
                                // create a png bitmap encoder which knows how to save a .png file
                                BitmapEncoder encoder = new PngBitmapEncoder();
                                // create frame from the writable bitmap and add to encoder
                                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));
                                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
                                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                                string path = Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

                                // write the new file to disk
                                try
                                {
                                    // FileStream is IDisposable
                                    using (FileStream fs = new FileStream(path, FileMode.Create))
                                    {
                                        encoder.Save(fs);
                                    }
                                    //this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                                }
                                catch (IOException)
                                {
                                    //this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                                }
                            }
                        })); //end dispatcher
                    } //end if
                    j = 1; //change so that the image is only saved once.
                }//end if printcount==3
                movement = 0;
                noMoveTimer.Stop();
                noMoveTimer.Enabled = false;
            }
        } //end noMovementCheck

        /// <summary>
        /// flood fill algorithm 
        /// </summary>
        private void floodFill(int x, int y)
        {
            if (twoD[x, y] != 0)
            {
                pixelProximity++; //increase counter that keeps relative track of size
                if ((x - 1) >= 0) { twoD[x - 1, y] = 0; }  // set to 0, to keep track of squares already visited
                if ((x + 1) < twoD.GetLength(0)) { twoD[x + 1, y] = 0; }
                if ((y - 1) >= 0) { twoD[x, y - 1] = 0; }
                if ((y + 1) < twoD.GetLength(1)) { twoD[x, y + 1] = 0; }
                if ((x - 1) >= 0 && (y - 1) >= 0) { twoD[x - 1, y - 1] = 0; }
                if ((x - 1) >= 0 && (y + 1) < twoD.GetLength(1)) { twoD[x - 1, y + 1] = 0; }
                if ((x + 1) < twoD.GetLength(0) && (y - 1) >= 0) { twoD[x + 1, y - 1] = 0; }
                if ((x + 1) < twoD.GetLength(0) && (y + 1) < twoD.GetLength(1)) { twoD[x + 1, y + 1] = 0; }
                twoD[x, y] = 0;
                if ((x - 1) >= 0) { tempTwoD[x - 1, y] = 1; }  //mark squares to keep a record of the image with blob difference
                if ((x + 1) < tempTwoD.GetLength(0)) { tempTwoD[x + 1, y] = 1; }
                if ((y - 1) >= 0) { tempTwoD[x, y - 1] = 1; }
                if ((y + 1) < tempTwoD.GetLength(1)) { tempTwoD[x, y + 1] = 1; }
                if ((x - 1) >= 0 && (y - 1) >= 0) { tempTwoD[x - 1, y - 1] = 1; } 
                if ((x - 1) >= 0 && (y + 1) < tempTwoD.GetLength(1)) { tempTwoD[x - 1, y + 1] = 1; }
                if ((x + 1) < tempTwoD.GetLength(0) && (y - 1) >= 0) { tempTwoD[x + 1, y - 1] = 1; }
                if ((x + 1) < tempTwoD.GetLength(0) && (y + 1) < tempTwoD.GetLength(1)) { tempTwoD[x + 1, y + 1] = 1; }
                tempTwoD[x, y] = 1;
                // iterate over neighbour pixels and check them
                for (int f = -3; f <= 3; f = f + 3)
                {
                    for (int d = -3; d <= 3; d = d + 3)
                    {
                        if ((x + f) < twoD.GetLength(0) && (x + f) >= 0 && (y + d) < twoD.GetLength(1) && (y + d) >= 0)
                        {
                            floodFill(x + f, y + d);
                        }
                    }
                }//end for
            }//end if
            else { return; }
        } //end DFS
        
    } //end class
}