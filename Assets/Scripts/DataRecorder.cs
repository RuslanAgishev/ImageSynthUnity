using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using Newtonsoft.Json;


public class OdometryData
{
    public int frame_number { get; set; }     // frame number
    public string pose_m { get; set; }
    public string vel_m_sec { get; set; }
    public string acc_m_sec_sec { get; set;}
    public string orient_quat { get; set; }
    public string ang_vel_deg_sec { get; set; }
    public string ang_acc_deg_sec_sec { get; set;}
}

public class DetectionData
{
    public int frame_number { get; set; }     // frame number
    public string tags {get; set;}            // objects tags: Pedestrian or TrafficCar
    public string poses_m { get; set; }       // poses in the world coord frame
    public string orients_quat { get; set; }  // orientations in the world coord frame
    public string bbox_sizes_m { get; set; }  // bounding boxes sizes
}

public class DataRecorder : MonoBehaviour
{
    public string dataPath = "dataset/";
    public float screenCaptureInterval = 0.1f; // in seconds
    public int width = 640;
    public int height = 480;
    public bool saveData = false;
    public int specificPass = -1;
    public float detectionLookupRadius = 50.0f;
    private string fileName = "data";

    private int framesCounter = 1;
    private int FPS;
    // Odometry attributes
    private Vector3 velocity = new Vector3(0f, 0f, 0f);
    private Vector3 acceleration = new Vector3(0f, 0f, 0f);
    private Vector3 orientation;
    private Vector3 angular_velocity = new Vector3(0f, 0f, 0f);
    private Vector3 angular_acceleration = new Vector3(0f, 0f, 0f);
    private Vector3 lastPosition;
    private Vector3 lastVelocity = new Vector3(0f, 0f, 0f);
    private Vector3 lastOrientation;
    private Vector3 lastAngularVelocity = new Vector3(0f, 0f, 0f);
    // Detections attributes
    private string tags;
    private string poses_m;
    private string orients_quat;
    private string bbox_sizes_m;

    // Start is called before the first frame update
    void Start()
    {
        lastPosition = transform.position;
        lastOrientation = new Vector3(transform.rotation.normalized.eulerAngles.x, transform.rotation.normalized.eulerAngles.y, transform.rotation.normalized.eulerAngles.z);

        // write string to file
        if (saveData)
        {
            Debug.Log("Recording the data...");
            File.WriteAllText(dataPath+"odometry_"+fileName+".json", "[");
            File.WriteAllText(dataPath+"detections_"+fileName+".json", "[");
        }
        ChangeCameraIntrinsics();
    }

    // Update is called once per frame
    void Update()
    {
        // calculate FPS
        FPS = (int)(1f / Time.deltaTime);
        // calculate odometry and detections
        GetOdometryData();
        GetDetectionsData();
    }

    

    void LateUpdate()
    {
        // if there are frames available
        if ((int)(screenCaptureInterval*FPS) > 0 && saveData)
        {
            // save data every [screenCaptureInterval] seconds
            if (Time.frameCount % (int)(screenCaptureInterval*FPS) == 0 ) {
                // Save camera frames
                // NOTE: due to per-camera / per-object motion being calculated late in the frame and after Update()
                // capturing is moved into LateUpdate (see ImageSynthesis.cs Known Issues)
                GetComponent<ImageSynthesis>().Save(fileName+"_"+framesCounter, width, height, dataPath+"/images/", specificPass);

                // Write odometry data
                WriteOdometryData();
                // Write detections data
                WriteDetectionData();
                
                // Update the label
                framesCounter++;
            }
        }
    }

    public void ChangeCameraIntrinsics()
    {
        /*
        Sets physical camera parameters based on the calibration matrix K.
        Reference: https://forum.unity.com/threads/how-to-use-opencv-camera-calibration-to-set-physical-camera-parameters.704120/
        */
        float f = 35.0f; // f can be arbitrary, as long as sensor_size is resized to make ax,ay consistient
        float ax, ay, sizeX, sizeY;
        float x0, y0, shiftX, shiftY;
 
        ax = 615.7320556640625f;
        ay = 615.7219848632812f;
        x0 = 312.435302734375f;
        y0 = 244.60476684570312f;
 
        sizeX = f * width / ax;
        sizeY = f * height / ay;
 
        //PlayerSettings.defaultScreenWidth = width;
        //PlayerSettings.defaultScreenHeight = height;
 
        shiftX = -(x0 - width / 2.0f) / width;
        shiftY = (y0 - height / 2.0f) / height;
 
        Camera.main.sensorSize = new Vector2(sizeX, sizeY);     // in mm, mx = 1000/x, my = 1000/y
        Camera.main.focalLength = f;                            // in mm, ax = f * mx, ay = f * my
        Camera.main.lensShift = new Vector2(shiftX, shiftY);    // W/2,H/w for (0,0), 1.0 shift in full W/H in image plane

        // Debug.Log("Sensor size [mm]: "+Camera.main.sensorSize);
        // Debug.Log("Focal length [mm]: "+Camera.main.focalLength);
        // Debug.Log("Lens shift: "+Camera.main.lensShift);
        // Debug.Log("Width: "+width+", Height: "+height);
    }

    private void WriteOdometryData()
    {
        OdometryData odom_data = new OdometryData()
        {
            frame_number = framesCounter,
            pose_m = transform.position.ToString(),
            vel_m_sec = velocity.ToString(),
            acc_m_sec_sec = acceleration.ToString(),
            orient_quat = QuatToString(transform.rotation.normalized),
            ang_vel_deg_sec = angular_velocity.ToString(),
            ang_acc_deg_sec_sec = angular_acceleration.ToString(),
        };
        string json_odom = JsonConvert.SerializeObject(odom_data, Formatting.Indented);
        File.AppendAllText(dataPath+"/odometry/"+fileName+".json", json_odom);
        File.AppendAllText(dataPath+"/odometry/"+fileName+".json", "\n,\n");
    }


    private void WriteDetectionData()
    {
        DetectionData detection_data = new DetectionData()
        {
            frame_number = framesCounter,
            tags = tags,
            poses_m = poses_m,
            orients_quat = orients_quat,
            bbox_sizes_m = bbox_sizes_m
        };
        string json_detection = JsonConvert.SerializeObject(detection_data, Formatting.Indented);
        File.AppendAllText(dataPath+"/detections/"+fileName+".json", json_detection);
        File.AppendAllText(dataPath+"/detections/"+fileName+".json", "\n,\n");
    }

    void OnDestroy()
    {
        void DeleteLastLine(string filepath)
        {
            string[] data = File.ReadAllLines(filepath);
            var lines = new List<string>(data);
            File.WriteAllLines(filepath, lines.GetRange(0, lines.Count - 1).ToArray());
        }
        if (saveData)
        {
            // Remove last line, which is simply a redundant coma (",")
            DeleteLastLine(dataPath+"odometry_"+fileName+".json");
            File.AppendAllText(dataPath+"odometry_"+fileName+".json", "]");

            // Remove last line, which is simply a redundant coma (",")
            DeleteLastLine(dataPath+"detections_"+fileName+".json");
            File.AppendAllText(dataPath+"detections_"+fileName+".json", "]");
        }
    }

    float NormQuat(Quaternion q)
    {
        return Mathf.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    }

    float Norm(Vector3 v)
    {
        return Mathf.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }

    string QuatToString(Quaternion q)
    {
        float norm = NormQuat(q);
        string q_norm = "("+(q[0]/norm)+", "+(q[1]/norm)+", "+(q[2]/norm)+", "+(q[3]/norm)+")";
        return q_norm;
    }

    void GetOdometryData()
    {
        // position
        float x = transform.position.x;
        float y = transform.position.y;
        float z = transform.position.z;

        // calculate velocity
        velocity = (transform.position - lastPosition) / Time.deltaTime;
        lastPosition = transform.position;

        // calculate acceleration (noisy linear method)
        acceleration = (velocity - lastVelocity) / Time.deltaTime;
        lastVelocity = velocity;

        // orientation
        float p = transform.rotation.normalized.eulerAngles.x; // rotation.normalized around X axis
        float q = transform.rotation.normalized.eulerAngles.y; // rotation.normalized around Y axis
        float r = transform.rotation.normalized.eulerAngles.z; // rotation.normalized around Z axis
        orientation = new Vector3(p, q, r);

        // angular velocity
        angular_velocity = (orientation - lastOrientation) / Time.deltaTime;
        lastOrientation = orientation;

        // angular acceleration
        angular_acceleration = (angular_velocity - lastAngularVelocity) / Time.deltaTime;
        lastAngularVelocity = angular_velocity;

        // Debug.Log("Position: "+transform.position);
        // Debug.Log("Euler angles orientation: "+orientation);
        // Debug.Log("Velocity: "+Norm(velocity));
        // Debug.Log("Acceleration: "+Norm(acceleration));
        // Debug.Log("Angular velocity: "+Norm(angular_velocity));
        // Debug.Log("Angular acceleration: "+Norm(angular_acceleration));
    }

    void GetDetectionsData()
    {
        GameObject[] cars = GameObject.FindGameObjectsWithTag("TrafficCar");
        GameObject[] pedestrians = GameObject.FindGameObjectsWithTag("Pedestrian");

        float[] dists_to_cars = new float[cars.Length];
        float[] dists_to_pedestrians = new float[pedestrians.Length];
        tags = "";
        poses_m = "";
        orients_quat = "";
        bbox_sizes_m = "";

        // Collect cars bbox-data
        for (int i=0; i<cars.Length; i++)
        {
            dists_to_cars[i] = Norm(cars[i].transform.position - transform.position);
            if (dists_to_cars[i]>0.1f && dists_to_cars[i]<detectionLookupRadius)
            {
                // Write car data to a file
                tags += "TrafficCar, ";
                poses_m += cars[i].transform.position.ToString() + ", ";
                orients_quat += QuatToString(cars[i].transform.rotation.normalized) + ", ";
                bbox_sizes_m += cars[i].GetComponent<DimBoxes.BoundBox>().meshBound.size.ToString() + ", ";
            }
        }
        // Collect pedestrians bbox-data
        for (int i=0; i<pedestrians.Length; i++)
        {
            dists_to_pedestrians[i] = Norm(pedestrians[i].transform.position - transform.position);
            if (dists_to_pedestrians[i]>0.1f && dists_to_pedestrians[i]<detectionLookupRadius)
            {
                // Write car data to a file
                tags += "Pedestrian, ";
                poses_m += (pedestrians[i].transform.position).ToString() + ", ";
                orients_quat += QuatToString(pedestrians[i].transform.rotation.normalized) + ", ";
                bbox_sizes_m += pedestrians[i].GetComponent<DimBoxes.BoundBoxHuman>().bound.size.ToString() + ", ";
            }
        }
        if (tags.Length > 2)
        {
            // remove redundant ", " at the end of each string
            tags = tags.Substring(0,tags.Length-2);
            poses_m = poses_m.Substring(0,poses_m.Length-2);
            orients_quat = orients_quat.Substring(0,orients_quat.Length-2);
            bbox_sizes_m = bbox_sizes_m.Substring(0,bbox_sizes_m.Length-2);
        }
        
        // if (bbox_sizes_m.ToArray().Length>0) { Debug.Log(bbox_sizes_m[bbox_sizes_m.ToArray().Length-1]); };
    }
}
