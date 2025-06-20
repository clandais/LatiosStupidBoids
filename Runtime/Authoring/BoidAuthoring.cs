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
        [SerializeField] bool         boidEnabled = true;

        void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position, boidSettings.cohesionRadius);
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, boidSettings.separationRadius);
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(transform.position, boidSettings.alignmentRadius);
        }

        class BoidAuthoringBaker : Baker<BoidAuthoring>
        {
            public override void Bake(BoidAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<BoidTag>(entity);


                AddComponent<BoidEnabledTag>(entity);
                SetComponentEnabled<BoidEnabledTag>(entity, authoring.boidEnabled);

                AddBuffer<BoidNeighbor>(entity);
                AddComponent(entity, authoring.boidSettings);



                AddComponent(entity, new BoidForces
                {
                    Alignment  = float3.zero,
                    Separation = float3.zero,
                    Cohesion   = float3.zero,
                    Follow     = float3.zero
                });

                AddComponent(entity, new BoidGoal
                {
                    GoalPosition = authoring.transform.position
                });
            }
        }
    }


    public struct BoidTag : IComponentData { }

    public struct BoidEnabledTag : IComponentData, IEnableableComponent { }

    public struct BoidGoal : IComponentData
    {
        public float3 GoalPosition;
    }


    [Serializable]
    public struct BoidSettings : IComponentData
    {
        public float maxSpeed;
        public float maxForce;


        [FormerlySerializedAs("centeringRadius")] [Header("Forces")] [FormerlySerializedAs("neighborRadius")]
        public float cohesionRadius;

        [FormerlySerializedAs("centeringStrength")]
        public float cohesionWeight;

        [FormerlySerializedAs("avoidanceRadius")]
        public float separationRadius;

        [FormerlySerializedAs("avoidanceStrength")]
        public float separationWeight;

        public float alignmentRadius;

        [FormerlySerializedAs("alignmentStrength")]
        public float alignmentWeight;

        public float followStrength;
        public float slowingRadius;
        public float agentWeight;
    }


    public struct BoidForces : IComponentData
    {
        public float3 Velocity;

        public float3 Separation;
        public float3 Alignment;
        public float3 Cohesion;
        public float3 Follow;
    }

    public struct BoidNeighbor : IBufferElementData
    {
        public EntityWith<BoidTag> Neighbor;
    }
}