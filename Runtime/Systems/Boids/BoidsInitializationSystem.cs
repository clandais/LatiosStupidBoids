using Boids.Authoring;
using Boids.Components;
using Latios;
using Latios.Psyshock;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    [RequireMatchingQueriesForUpdate]
    internal partial struct BoidsInitializationSystem : ISystem
    {
        EntityQuery m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_query = state.Fluent()
                .WithAspect<TransformAspect>()
                .WithAspect<BoidAspect>()
                .WithEnabled<BoidEnabledTag>()
                .Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var bodies = new NativeArray<ColliderBody>(m_query.CalculateEntityCount(), Allocator.TempJob);
            state.Dependency = new ClearNeighborBufferJob
            {
                Bodies = bodies
            }.ScheduleParallel(m_query, state.Dependency);

            // TODO : Add PhysicsSettings
            state.Dependency = Physics.BuildCollisionLayer(bodies)
                .ScheduleParallel(out var layer, state.WorldUpdateAllocator, state.Dependency);

            var findNeighbors = new FindNeighbors
            {
                BoidNeighborLookup = SystemAPI.GetBufferLookup<BoidNeighbor>()
            };

            state.Dependency = Physics.FindPairs(layer, findNeighbors).ScheduleParallel(state.Dependency);
            state.Dependency = bodies.Dispose(state.Dependency);
        }

        [BurstCompile]
        partial struct ClearNeighborBufferJob : IJobEntity
        {
            [NativeDisableParallelForRestriction] public NativeArray<ColliderBody> Bodies;

            void Execute(Entity entity, [EntityIndexInQuery] int idx, TransformAspect transformAspect,
                BoidAspect boidAspect)
            {
                boidAspect.ClearNeighbors(idx, entity, Bodies, transformAspect.worldTransform);
                if (math.any(math.isnan(boidAspect.GoalPosition)))
                    boidAspect.SetGoal(transformAspect.worldPosition);
            }
        }

        [BurstCompile]
        struct FindNeighbors : IFindPairsProcessor
        {
            public PhysicsBufferLookup<BoidNeighbor> BoidNeighborLookup;

            public void Execute(in FindPairsResult result)
            {
                if (Physics.DistanceBetween(result.colliderA, result.transformA, result.colliderB, result.transformB,
                        0f, out _))
                {
                    BoidNeighborLookup[result.entityA].Add(new BoidNeighbor
                    {
                        Neighbor = new EntityWith<BoidTag>
                        {
                            entity = result.entityB
                        }
                    });

                    BoidNeighborLookup[result.entityB].Add(new BoidNeighbor
                    {
                        Neighbor = new EntityWith<BoidTag>
                        {
                            entity = result.entityA
                        }
                    });
                }
            }
        }
    }
}