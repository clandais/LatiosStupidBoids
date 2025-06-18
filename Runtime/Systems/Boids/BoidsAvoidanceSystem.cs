using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    public partial struct BoidsAvoidanceSystem : ISystem
    {
        EntityQuery m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_query = state.Fluent()
                .WithAspect<BoidAspect>()
                .Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency = new AvoidJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true)
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }


        [BurstCompile]
        internal partial struct AvoidJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<WorldTransform> WorldTransformLookup;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int _,
                BoidAspect boidAspect)
            {
                if (boidAspect.NeighborCount == 0) return;

                var desiredSeparation = boidAspect.Settings.avoidanceRadius;


                var position = WorldTransformLookup[entity].position;
                var avoidance = float3.zero;
                var neighborCount = 0;

                foreach (var neighbor in boidAspect.Neighbors)
                    if (WorldTransformLookup.HasComponent(neighbor.Neighbor.entity))
                    {
                        var neighborTransform = WorldTransformLookup[neighbor.Neighbor.entity];
                        var distance = math.distance(position, neighborTransform.position);
                        if (distance < desiredSeparation)
                        {
                            var diff = position - neighborTransform.position;
                            // Normalize the difference vector to get the direction
                            if (math.lengthsq(diff) < float.Epsilon)
                                continue; // Avoid division by zero

                            diff      =  math.normalize(diff) * (1f / distance);
                            avoidance += diff;
                            neighborCount++;
                        }
                    }


                avoidance = math.normalize(avoidance);
                if (math.any(math.isnan(avoidance)) || neighborCount == 0) return;
                // Set the avoidance force
                boidAspect.ApplyForce(avoidance * boidAspect.Settings.avoidanceStrength);
            }
        }
    }
}