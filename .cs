using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;

namespace KinectFaceDetection
{
    class Program
    {
        static void Main(string[] args)
        {
            using (var kinectSensor = KinectSensor.GetDefault())
            {
                kinectSensor.Open();

                using (var frameReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Infrared | FrameSourceTypes.Body))
                {
                    var faceFrameSources = new FaceFrameSource[6];
                    var faceFrameReaders = new FaceFrameReader[6];

                    for (int i = 0; i < 6; i++)
                    {
                        faceFrameSources[i] = new FaceFrameSource(kinectSensor, 0, FaceFrameFeatures.BoundingBoxInColorSpace);
                        faceFrameReaders[i] = faceFrameSources[i].OpenReader();
                    }

                    frameReader.MultiSourceFrameArrived += (s, e) =>
                    {
                        using (var multiSourceFrame = e.FrameReference.AcquireFrame())
                        {
                            if (multiSourceFrame == null)
                            {
                                return;
                            }

                            using (var colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                            using (var infraredFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
                            using (var bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                            {
                                if (colorFrame == null || infraredFrame == null || bodyFrame == null)
                                {
                                    return;
                                }

                                var colorFrameDescription = colorFrame.FrameDescription;
                                var colorPixels = new byte[colorFrameDescription.Width * colorFrameDescription.Height * 4];
                                colorFrame.CopyConvertedFrameDataToArray(colorPixels, ColorImageFormat.Bgra);

                                var infraredFrameDescription = infraredFrame.FrameDescription;
                                var infraredPixels = new ushort[infraredFrameDescription.Width * infraredFrameDescription.Height];
                                infraredFrame.CopyFrameDataToArray(infraredPixels);

                                var bodies = new Body[bodyFrame.BodyCount];
                                bodyFrame.GetAndRefreshBodyData(bodies);

                                for (int i = 0; i < 6; i++)
                                {
                                    if (faceFrameSources[i].IsTrackingIdValid)
                                    {
                                        using (var faceFrame = faceFrameReaders[i].AcquireLatestFrame())
                                        {
                                            if (faceFrame != null)
                                            {
                                                var faceBoundingBoxInColorSpace = faceFrame.FaceBoundingBoxInColorSpace;
                                                var x = (int)faceBoundingBoxInColorSpace.Left;
                                                var y = (int)faceBoundingBoxInColorSpace.Top;
                                                var width = (int)faceBoundingBoxInColorSpace.Width;
                                                var height = (int)faceBoundingBoxInColorSpace.Height;

                                                if (x >= 0 && y >= 0 && width > 0 && height > 0 && x + width < colorFrameDescription.Width && y + height < colorFrameDescription.Height)
                                                {
                                                    // Draw rectangle around face
                                                    for (int j = 0; j < width; j++)
                                                    {
                                                        var index1 = ((y * colorFrameDescription.Width) + (x + j)) * 4;
                                                        var index2 = (((y + height - 1) * colorFrameDescription.Width) + (x + j)) * 4;
                                                        colorPixels[index1] = 0;
                                                        colorPixels[index1 + 1] = 255;
                                                        colorPixels[index1 + 2] = 0;
                                                        colorPixels[index2]
