using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace TagTracking {
    static class CameraCalibrate {
        public static bool onCalibration;
        public static bool startCalibrating = false;
        public static int cameraToUse = 0;
        public static int markersLength = 18;
        public static int framesSaved = 0;
        public static bool removeBadFrames = false;
        public static double lastError = 99999999999;
        public static int lastFrameCount = 0;
        static Dictionary dict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);
        static CharucoBoard chboard;
        static DetectorParameters arucoParams;
        static List<VectorOfPointF> charucoCorners = new List<VectorOfPointF>();
        static List<VectorOfInt> charucoIds = new List<VectorOfInt>();
        static Size imageSize = new(800, 600);
        static int lastSaved = 0;
        static bool calculatingPreview = false;
        public static void Init() {
            onCalibration = true;
            startCalibrating = false;
            cameraToUse = 0;
            removeBadFrames = false;
            framesSaved = 0;
            lastError = 99999999999;
            imageSize = new(800, 600);
            charucoCorners = new List<VectorOfPointF>();
            charucoIds = new List<VectorOfInt>();
            ChooseCamera();
        }

        private static void ChooseCamera() {
            if (Tag.cameras.Length > 1) {
                Console.WriteLine($"Choose a camera [0 to {Tag.cameras.Length - 1}] (ID is indicated in 'config.txt')");
                try {
                    cameraToUse = int.Parse(Console.ReadLine());
                } catch { cameraToUse = -1; }
                if (cameraToUse < 0 || cameraToUse >= Tag.cameras.Length) {
                    Console.WriteLine("Camera ID not available\nCancelling...");
                    onCalibration = false;
                    return;
                }
            }
            lastError = 99999999999;
            Console.WriteLine($"Starting calibration using camera {cameraToUse}");
            //Console.WriteLine("Press [3] to remove frame when error goes above 1.0");

            if (chboard == null) GenerateCharucoBoard();
            dict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);
            arucoParams = DetectorParameters.GetDefault();
            startCalibrating = true;
        }

        public static void GetFrame(Mat frame) {
            if (frame.IsEmpty) return;
            int thisTime = (int)(Program.timer.ElapsedMilliseconds / 1000);
            imageSize = frame.Size;
            VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
            VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
            ArucoInvoke.DetectMarkers(frame, dict, corners, ids, arucoParams);
            ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(0, 255, 0));
            VectorOfPointF charucoCornersTemp = new();
            VectorOfInt charucoIdsTemp = new();
            if (ids.Size > 0) {
                ArucoInvoke.InterpolateCornersCharuco(corners, ids, frame, chboard, charucoCornersTemp, charucoIdsTemp);
                if (charucoCornersTemp.Size > 10 && thisTime > lastSaved && startCalibrating) {
                    lastSaved = thisTime;
                    charucoCorners.Add(charucoCornersTemp);
                    charucoIds.Add(charucoIdsTemp);
                    Console.WriteLine($"Saved frame {framesSaved}");
                    framesSaved++;
                    if (!calculatingPreview) {
                        Task.Run(() => CalculatePreview());
                    }
                }
            }
            CvInvoke.Imshow($"calib", frame);
            CvInvoke.WaitKey(1);
        }
        public static void CalculatePreview() {
            calculatingPreview = true;
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Matrix<double> cameraMatrix = new Matrix<double>(3, 3);
            Matrix<double> distortionCoefficients = new Matrix<double>(5, 1);
            MCvTermCriteria terminationCriteria = new MCvTermCriteria(30, 0.001);
            //int thisFrameCount = charucoIds.Count-1;
            double error = ArucoInvoke.CalibrateCameraCharuco(new VectorOfVectorOfPointF(charucoCorners.ToArray()), new VectorOfVectorOfInt(charucoIds.ToArray()), chboard, imageSize, cameraMatrix, distortionCoefficients, rvecs, tvecs, CalibType.Default, terminationCriteria);
            Console.WriteLine("Current error: " + error);
            //if (error > lastError) {
            //    for (int i = 0; i < thisFrameCount - lastFrameCount; i++) {
            //        int index = thisFrameCount - i;
            //        charucoIds[index] = charucoIds[lastFrameCount];
            //        charucoCorners[index] = charucoCorners[lastFrameCount];
            //        Console.WriteLine($"Removed: {index}");
            //    }
            //}
            //lastError = error;
            //lastFrameCount = thisFrameCount;
            calculatingPreview = false;
        }
        public static void StartCalibration() {
            startCalibrating = false;
            Mat rvecs = new Mat(); // rotation vector
            Mat tvecs = new Mat(); // translation vector

            Matrix<double> cameraMatrix = new Matrix<double>(3, 3);
            Matrix<double> distortionCoefficients = new Matrix<double>(5, 1);
            MCvTermCriteria terminationCriteria = new MCvTermCriteria(30, 0.001);
            Console.WriteLine("Processing...");
            double error = ArucoInvoke.CalibrateCameraCharuco(new VectorOfVectorOfPointF(charucoCorners.ToArray()), new VectorOfVectorOfInt(charucoIds.ToArray()), chboard, imageSize, cameraMatrix, distortionCoefficients, rvecs, tvecs, CalibType.Default, terminationCriteria);
            Console.WriteLine($"Finished with an error of: {error} (lower the better)");
            string file = $"calibration_parameters_{DateTime.Now.ToString("yy-MM-dd-hh-mm-ss")}.xml";
            FileStorage fileStorage = new FileStorage(file, FileStorage.Mode.Write);
            fileStorage.Write($"\"{DateTime.Now}\"", "framesCount");
            fileStorage.Write(framesSaved, "framesCount");
            fileStorage.Write($"CameraRes", "cameraResolution");
            fileStorage.Write(cameraMatrix.Mat, "cameraMatrix");
            fileStorage.Write(distortionCoefficients.Mat, "dist_coeffs");
            fileStorage.Write(error, "avg_reprojection_error");
            fileStorage.ReleaseAndGetString();
            string xml = File.ReadAllText(file); //fuck it
            xml = xml.Replace("<cameraResolution>CameraRes</cameraResolution>", $"<cameraResolution>{imageSize.Width} {imageSize.Height}</cameraResolution>");
            File.WriteAllText(file, xml);
            onCalibration = false;

            Console.WriteLine("Apply camera parameters on configuration file? (Y/N)");
            var a = Console.ReadKey();
            if (a.Key == ConsoleKey.Y) {
                string[] lines = File.ReadAllLines("config.txt");
                for (int i = 0; i < lines.Length; i++) {
                    string[] split = lines[i].Split("=");
                    if (split[0].Equals($"camera{cameraToUse}File")) {
                        lines[i] = $"camera{cameraToUse}File={file}";
                        Console.WriteLine($"Saved for camera {cameraToUse}");

                        break;
                    }
                }
                File.WriteAllLines("config.txt", lines);
            }
            Console.WriteLine("\nFinishied Calibration!");
        }
        static void GenerateCharucoBoard() {
            int sizeMult = 8;
            int tagW = 8;
            int tagH = 6;
            chboard = new CharucoBoard(tagW, tagH, 10 * sizeMult, 6 * sizeMult, dict);
        }
        public static void DrawBoard() {
            Mat boardImage = new Mat();
            int sizeMult = 10;
            int tagW = 8;
            int tagH = 6;
            if (chboard == null) GenerateCharucoBoard();
            Size imageSize = new Size(10 * tagW * sizeMult, 10 * tagH * sizeMult);
            chboard.Draw(imageSize, boardImage);
            boardImage.Save("charuco.png");
        }
        public static void Cancel() {
            Console.WriteLine("Calibration cancelled");
            onCalibration = false;
            startCalibrating = false;
        }
    }
}
