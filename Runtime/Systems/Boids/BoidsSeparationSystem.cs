using Boids.Authoring;
using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    internal partial struct BoidsSeparationSystem : ISystem
    {
        EntityQuery m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_query = state.Fluent()
                .WithAspect<BoidAspect>()
                .WithEnabled<BoidEnabledTag>()
                .Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency = new SeparationJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true)
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }


        [BurstCompile]
        internal partial struct SeparationJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<WorldTransform> WorldTransformLookup;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int _,
                BoidAspect boidAspect)
            {
                if (boidAspect.NeighborCount == 0)
                {
                    boidAspect.SeparationForce = float3.zero;
                    return;
                }

                var position = WorldTransformLookup[entity].position;
                var totalForce = float3.zero;


                foreach (var neighbor in boidAspect.Neighbors)
                {
                    var neighborPosition = WorldTransformLookup[neighbor.Neighbor.entity].position;
                    var distance = math.distance(neighborPosition, position);

                    if (math.distance(position, neighborPosition) > boidAspect.Settings.separationRadius)
                        continue;

                    var pushForce = position - neighborPosition;
                    totalForce += math.normalize(pushForce) / distance;
                }

                if (math.any(math.isnan(totalForce)))
                {
                    boidAspect.SeparationForce = float3.zero;
                    return;
                }

                boidAspect.SeparationForce = totalForce;
            }
        }
    }
}