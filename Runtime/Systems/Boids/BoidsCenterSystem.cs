using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    public partial struct BoidsCenterSystem : ISystem
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
            state.Dependency = new CenterJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true)
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        partial struct CenterJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<WorldTransform> WorldTransformLookup;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int entityIndex,
                BoidAspect boidAspect)
            {
                if (boidAspect.NeighborCount == 0) return;

                var position = WorldTransformLookup[entity].position;
                var centering = float3.zero;
                var neighborCount = 0;

                foreach (var neighbor in boidAspect.Neighbors)
                    if (WorldTransformLookup.HasComponent(neighbor.Neighbor.entity))
                    {
                        var neighborPosition = WorldTransformLookup[neighbor.Neighbor.entity].position;
                        var distance = math.distance(neighborPosition, position);
                        if (distance < boidAspect.Settings.centeringRadius)
                        {
                            centering += neighborPosition - position;
                            neighborCount++;
                        }
                    }

                if (math.length(centering) > float.Epsilon && neighborCount > 0)
                {
                    centering = math.normalize(centering);
                    boidAspect.ApplyForce(centering * boidAspect.Settings.centeringStrength);
                }
            }
        }
    }
}