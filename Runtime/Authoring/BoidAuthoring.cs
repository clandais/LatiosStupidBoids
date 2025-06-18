using System;
using Latios;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Boids.Authoring
{
    [AddComponentMenu("Boids/Boid Authoring")]
    public class BoidAuthoring : MonoBehaviour
    {
        [SerializeField] BoidSettings boidSettings;


        void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position, boidSettings.centeringRadius);
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, boidSettings.avoidanceRadius);
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(transform.position, boidSettings.alignmentRadius);
        }

        class BoidAuthoringBaker : Baker<BoidAuthoring>
        {
            public override void Bake(BoidAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<BoidTag>(entity);
                AddBuffer<BoidNeighbor>(entity);
                AddComponent(entity, authoring.boidSettings);

                AddComponent(entity, new BoidForces
                {
                    AlignmentForce = float3.zero,
                    AvoidanceForce = float3.zero,
                    CenteringForce = float3.zero,
                    FollowForce    = float3.zero
                });

                AddComponent(entity, new BoidGoal
                {
                    GoalPosition = authoring.transform.position
                });
            }
        }
    }


    public struct BoidTag : IComponentData { }

    public struct BoidGoal : IComponentData
    {
        public float3 GoalPosition;
    }


    [Serializable]
    public struct BoidSettings : IComponentData
    {
        public float agentRadius;
        public float maxSpeed;
        public float maxForce;


        [Header("Forces")] [FormerlySerializedAs("neighborRadius")]
        public float centeringRadius;

        public float centeringStrength;

        public float avoidanceRadius;
        public float avoidanceStrength;

        public float alignmentRadius;
        public float alignmentStrength;

        public float followStrength;
    }


    public struct BoidForces : IComponentData
    {
        public float3 Acceleration;
        public float3 Velocity;
        public float3 AvoidanceForce;
        public float3 AlignmentForce;
        public float3 CenteringForce;
        public float3 FollowForce;
    }

    public struct BoidNeighbor : IBufferElementData
    {
        public EntityWith<BoidTag> Neighbor;
    }
}