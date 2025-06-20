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
    internal partial struct BoidsCohesionSystem : ISystem
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
            state.Dependency = new CohesionJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true)
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        partial struct CohesionJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<WorldTransform> WorldTransformLookup;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int entityIndex,
                BoidAspect boidAspect)
            {
                if (boidAspect.NeighborCount == 0)
                {
                    boidAspect.CohesionForce = float3.zero;
                    return;
                }

                var position = WorldTransformLookup[entity].position;
                var centerOfMass = position;
                var neighborCount = 0;
                foreach (var neighbor in boidAspect.Neighbors)
                {
                    var neighborPosition = WorldTransformLookup[neighbor.Neighbor.entity].position;

                    if (math.distance(position, neighborPosition) > boidAspect.Settings.cohesionRadius)
                        continue;

                    centerOfMass += neighborPosition;
                    neighborCount++;
                }

                var force = float3.zero;
                if (neighborCount > 0)
                {
                    centerOfMass /= neighborCount;
                    force        =  math.normalize(centerOfMass);
                }


                if (math.any(math.isnan(force)))
                {
                    boidAspect.CohesionForce = float3.zero;
                    return;
                }

                boidAspect.CohesionForce = force;
            }
        }
    }
}