using System;
using System.Collections;
using OpenCvSharp;
namespace csharp {
    class Program {
        static void Main (string[] args) {
            Mat src = new Mat ("tag25.png", ImreadModes.Color);
            Apriltag ap = new Apriltag ("canny", true, "tag25h9");
            var result = ap.detect (src);
            Console.WriteLine ("Detector: " + result.Count);
        }
    }
}