using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;

namespace RosSharp.RosBridgeClient
{
    public class ImageSynthPublisher : UnityPublisher<MessageTypes.Sensor.CompressedImage>
    {
        // pass configuration
        private CapturePass[] capturePasses = new CapturePass[] {
            new CapturePass() { name = "_img" },
            new CapturePass() { name = "_id", supportsAntialiasing = false },
            new CapturePass() { name = "_layer", supportsAntialiasing = false },
            new CapturePass() { name = "_depth" },
            new CapturePass() { name = "_normals" },
            new CapturePass() { name = "_flow", supportsAntialiasing = false, needsRescale = true } // (see issue with Motion Vectors in @KNOWN ISSUES)
        };
        struct CapturePass {
            // configuration
            public string name;
            public bool supportsAntialiasing;
            public bool needsRescale;
            public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }

            // impl
            public Camera camera;
        };

        public Shader uberReplacementShader;
        // whether semantic segmentation is labeled as grayscale image
	    public bool grayscaleSegmentation = false;

        public Camera ImageCamera;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;

        private MessageTypes.Sensor.CompressedImage message;
        private Texture2D texture2D;
        private Rect rect;

        protected override void Start()
        {
            // default fallbacks, if shaders are unspecified
            if (!uberReplacementShader)
                uberReplacementShader = Shader.Find("Hidden/UberReplacement");

            // use real camera to capture final image
            capturePasses[0].camera = this.ImageCamera;
            for (int q = 1; q < capturePasses.Length; q++)
                capturePasses[q].camera = CreateHiddenCamera (capturePasses[q].name);

            OnCameraChange();
		    OnSceneChange();

            base.Start();
            InitializeGameObject();
            InitializeMessage();
            Camera.onPostRender += UpdateImage;
        }

        private void UpdateImage(Camera _camera)
        {
            if (texture2D != null && _camera == capturePasses[3].camera)
                UpdateMessage();
        }

        private void InitializeGameObject()
        {
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            capturePasses[3].camera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.CompressedImage();
            message.header.frame_id = FrameId;
            message.format = "jpeg";
        }

        private void UpdateMessage()
        {
            #if UNITY_EDITOR
            if (DetectPotentialSceneChangeInEditor())
                OnSceneChange();
            #endif // UNITY_EDITOR

            // @TODO: detect if camera properties actually changed
            OnCameraChange();

            message.header.Update();
            texture2D.ReadPixels(rect, 0, 0);
            message.data = texture2D.EncodeToJPG(qualityLevel);
            Publish(message);
        }

        private Camera CreateHiddenCamera(string name)
        {
            var go = new GameObject (name, typeof (Camera));
            go.hideFlags = HideFlags.HideAndDontSave;
            go.transform.parent = transform;

            var newCamera = go.GetComponent<Camera>();
            return newCamera;
        }

        static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode)
        {
            SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
        }

        static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode, Color clearColor)
        {
            var cb = new CommandBuffer();
            cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
            cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
            cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
            cam.SetReplacementShader(shader, "");
            cam.backgroundColor = clearColor;
            cam.clearFlags = CameraClearFlags.SolidColor;
        }

        static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
        {
            var cb = new CommandBuffer();
            cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
            cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
            cam.depthTextureMode = depthTextureMode;
        }

        enum ReplacelementModes {
            ObjectId 			= 0,
            CatergoryId			= 1,
            DepthCompressed		= 2,
            DepthMultichannel	= 3,
            Normals				= 4
        };

        public void OnCameraChange()
        {
            int targetDisplay = 1;
            var mainCamera = this.ImageCamera;
            foreach (var pass in capturePasses)
            {
                if (pass.camera == mainCamera)
                    continue;

                // cleanup capturing camera
                pass.camera.RemoveAllCommandBuffers();

                // copy all "main" camera parameters into capturing camera
                pass.camera.CopyFrom(mainCamera);

                // set targetDisplay here since it gets overriden by CopyFrom()
                pass.camera.targetDisplay = targetDisplay++;
            }

            // setup command buffers and replacement shaders
            // SetupCameraWithReplacementShader(capturePasses[1].camera, uberReplacementShader, ReplacelementModes.ObjectId);
            SetupCameraWithReplacementShader(capturePasses[2].camera, uberReplacementShader, ReplacelementModes.CatergoryId);
            SetupCameraWithReplacementShader(capturePasses[3].camera, uberReplacementShader, ReplacelementModes.DepthCompressed, Color.white);
            // SetupCameraWithReplacementShader(capturePasses[4].camera, uberReplacementShader, ReplacelementModes.Normals);
            // SetupCameraWithPostShader(capturePasses[5].camera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);
        }

        public void OnSceneChange()
        {
            var renderers = Object.FindObjectsOfType<Renderer>();
            var mpb = new MaterialPropertyBlock();
            foreach (var r in renderers)
            {
                var id = r.gameObject.GetInstanceID();
                var layer = r.gameObject.layer;
                var tag = r.gameObject.tag;

                // mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
                mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer, grayscaleSegmentation));
                r.SetPropertyBlock(mpb);
            }
        }

        #if UNITY_EDITOR
        private GameObject lastSelectedGO;
        private int lastSelectedGOLayer = -1;
        private string lastSelectedGOTag = "unknown";
        private bool DetectPotentialSceneChangeInEditor()
        {
            bool change = false;
            // there is no callback in Unity Editor to automatically detect changes in scene objects
            // as a workaround lets track selected objects and check, if properties that are 
            // interesting for us (layer or tag) did not change since the last frame
            if (UnityEditor.Selection.transforms.Length > 1)
            {
                // multiple objects are selected, all bets are off!
                // we have to assume these objects are being edited
                change = true;
                lastSelectedGO = null;
            }
            else if (UnityEditor.Selection.activeGameObject)
            {
                var go = UnityEditor.Selection.activeGameObject;
                // check if layer or tag of a selected object have changed since the last frame
                var potentialChangeHappened = lastSelectedGOLayer != go.layer || lastSelectedGOTag != go.tag;
                if (go == lastSelectedGO && potentialChangeHappened)
                    change = true;

                lastSelectedGO = go;
                lastSelectedGOLayer = go.layer;
                lastSelectedGOTag = go.tag;
            }

            return change;
        }
        #endif // UNITY_EDITOR
    }
}
