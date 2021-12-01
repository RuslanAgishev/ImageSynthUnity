using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class OdometryPublisher : UnityPublisher<MessageTypes.Nav.Odometry>
    {
        public Transform PublishedTransform;
        public string FrameId = "Unity";

        private MessageTypes.Nav.Odometry message;

        // Odometry attributes
        private Vector3 velocity = new Vector3(0f, 0f, 0f);
        private Vector3 orientation;
        private Vector3 angular_velocity = new Vector3(0f, 0f, 0f);
        private Vector3 lastPosition;
        private Vector3 lastVelocity = new Vector3(0f, 0f, 0f);
        private Vector3 lastOrientation;
        private Vector3 lastAngularVelocity = new Vector3(0f, 0f, 0f);

        protected override void Start()
        {
            base.Start();
            InitializeMessage();

            lastPosition = PublishedTransform.position;
            lastOrientation = new Vector3(PublishedTransform.rotation.normalized.eulerAngles.x, PublishedTransform.rotation.normalized.eulerAngles.y, PublishedTransform.rotation.normalized.eulerAngles.z);
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Nav.Odometry
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = FrameId
                }
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.pose.position = GetGeometryPoint(PublishedTransform.position.Unity2Ros());
            message.pose.pose.orientation = GetGeometryQuaternion(PublishedTransform.rotation.Unity2Ros());

            Vector3 velocity = GetLinearVelocity();
            Vector3 angular_velocity = GetAngularVelocity();

            message.twist.twist.linear.x = velocity[0];
            message.twist.twist.linear.y = velocity[1];
            message.twist.twist.linear.z = velocity[2];
            message.twist.twist.angular.x = angular_velocity[0];
            message.twist.twist.angular.y = angular_velocity[1];
            message.twist.twist.angular.z = angular_velocity[2];

            Publish(message);
        }

        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            MessageTypes.Geometry.Quaternion geometryQuaternion = new MessageTypes.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

        private Vector3 GetLinearVelocity()
        {
            // calculate velocity
            velocity = (PublishedTransform.position - lastPosition) / Time.deltaTime;
            lastPosition = PublishedTransform.position;
            return velocity;
        }

        private Vector3 GetAngularVelocity()
        {
            // orientation
            float p = PublishedTransform.rotation.normalized.eulerAngles.x; // rotation.normalized around X axis
            float q = PublishedTransform.rotation.normalized.eulerAngles.y; // rotation.normalized around Y axis
            float r = PublishedTransform.rotation.normalized.eulerAngles.z; // rotation.normalized around Z axis
            orientation = new Vector3(p, q, r);

            // angular velocity
            angular_velocity = (orientation - lastOrientation) / Time.deltaTime;
            lastOrientation = orientation;
            return angular_velocity;
        }

    }
}
