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
    public partial struct BoidsAlignmentSystem : ISystem
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
            state.Dependency = new AlignJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true),
                BoidForces           = SystemAPI.GetComponentLookup<BoidForces>()
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }


        [BurstCompile]
        partial struct AlignJob : IJobEntity
        {
            [ReadOnly]                            public ComponentLookup<WorldTransform> WorldTransformLookup;
            [NativeDisableParallelForRestriction] public ComponentLookup<BoidForces>     BoidForces;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int entityIndex,
                in DynamicBuffer<BoidNeighbor> neighbors,
                in BoidSettings settings)
            {
                if (neighbors.Length == 0) return;

                var alignment = float3.zero;
                var neighborCount = 0;
                var position = WorldTransformLookup[entity].position;

                foreach (var neighbor in neighbors)
                    if (WorldTransformLookup.HasComponent(neighbor.Neighbor.entity) &&
                        BoidForces.HasComponent(neighbor.Neighbor.entity))
                    {
                        var neighborTransform = WorldTransformLookup[neighbor.Neighbor.entity];
                        var distance = math.distance(position, neighborTransform.position);
                        if (distance < settings.avoidanceRadius)
                        {
                            // Only add the velocity of the neighbor if it's within the alignment radius
                            alignment += BoidForces[neighbor.Neighbor.entity].Velocity;
                            neighborCount++;
                        }
                    }

                if (math.lengthsq(alignment) > 0f && neighborCount > 0)
                {
                    // Average the alignment vector
                    alignment /= neighborCount;
                    alignment =  math.normalize(alignment);

                    if (math.any(math.isnan(alignment))) return;
                    var forces = BoidForces[entity];
                    forces.Acceleration += alignment * settings.alignmentStrength;

                    // boidAspect.ApplyForce(alignment * boidAspect.Settings.alignmentStrength);
                }
            }
        }
    }
}