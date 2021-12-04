/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class CameraInfoPublisher : UnityPublisher<MessageTypes.Sensor.CameraInfo>
    {
        public Camera ImageCamera;
        public string FrameId = "local_map_camera";
        public uint resolutionWidth = 512;
        public uint resolutionHeight = 512;

        private MessageTypes.Sensor.CameraInfo message;
        private double[] K;
        private double[] D;
        private double[] R;
        private double[] P;
        private float ax, ay, x0, y0;

        protected override void Start()
        {
            base.Start();
            (ax, ay, x0, y0) = GetCameraIntrinsics();
            K = new double[] {ax, 0.0, x0, 0.0, ay, y0, 0.0, 0.0, 1.0};
            D = new double[] {0.0, 0.0, 0.0, 0.0, 0.0};
            R = new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            P = new double[] {ax, 0.0, x0, 0.0, 0.0, ay, y0, 0.0, 0.0, 0.0, 1.0, 0.0};
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.CameraInfo();
            message.header.frame_id = FrameId;
            message.height = resolutionHeight;
            message.width = resolutionWidth;
            message.K = K;
            message.D = D;
            message.R = R;
            message.P = P;
            message.distortion_model = "plumb_bob";
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void UpdateMessage()
        {
            message.header.Update();
            Publish(message);
        }

        public (float, float, float, float) GetCameraIntrinsics()
        {
            /*
            Sets physical camera parameters based on the calibration matrix K.
            Reference: https://forum.unity.com/threads/how-to-use-opencv-camera-calibration-to-set-physical-camera-parameters.704120/
            */
            float f = 35.0f; // f can be arbitrary, as long as sensor_size is resized to make ax,ay consistient
            float ax, ay, sizeX, sizeY;
            float x0, y0, shiftX, shiftY;
    
            ax = resolutionWidth; //554.3826904296875;
            ay = resolutionHeight; //554.3826904296875;
            x0 = resolutionWidth / 2.0f; //320.0;
            y0 = resolutionHeight / 2.0f; //240.0;
    
            sizeX = f * resolutionWidth / ax;
            sizeY = f * resolutionHeight / ay;
    
            //PlayerSettings.defaultScreenWidth = resolutionWidth;
            //PlayerSettings.defaultScreenHeight = height;
    
            shiftX = -(x0 - resolutionWidth / 2.0f) / resolutionWidth;
            shiftY = (y0 - resolutionHeight / 2.0f) / resolutionHeight;
    
            GetComponent<Camera>().sensorSize = new Vector2(sizeX, sizeY);     // in mm, mx = 1000/x, my = 1000/y
            GetComponent<Camera>().focalLength = f;                            // in mm, ax = f * mx, ay = f * my
            GetComponent<Camera>().lensShift = new Vector2(shiftX, shiftY);    // W/2,H/w for (0,0), 1.0 shift in full W/H in image plane

            // Debug.Log("Sensor size [mm]: "+GetComponent<Camera>().sensorSize);
            // Debug.Log("Focal length [mm]: "+GetComponent<Camera>().focalLength);
            // Debug.Log("Lens shift: "+GetComponent<Camera>().lensShift);
            // Debug.Log("Width: "+resolutionWidth+", Height: "+resolutionHeight);

            return (ax, ay, x0, y0);
        }

    }
}
