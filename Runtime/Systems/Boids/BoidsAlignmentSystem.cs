﻿using Boids.Authoring;
using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    internal partial struct BoidsAlignmentSystem : ISystem
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
            state.Dependency = new AlignJob
            {
                WorldTransformLookup = SystemAPI.GetComponentLookup<WorldTransform>(true)
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }


        [BurstCompile]
        partial struct AlignJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<WorldTransform> WorldTransformLookup;

            void Execute(
                Entity entity,
                [EntityIndexInQuery] int entityIndex,
                BoidAspect boidAspect)
            {
                if (boidAspect.NeighborCount == 0)
                {
                    boidAspect.AlignmentForce = float3.zero;
                    return;
                }

                var transform = WorldTransformLookup[entity];
                var avgHeading = transform.forwardDirection;
                var count = 0;

                foreach (var neighbor in boidAspect.Neighbors)
                {
                    var neighborTransform = WorldTransformLookup[neighbor.Neighbor.entity];

                    if (math.distance(transform.position, neighborTransform.position) >
                        boidAspect.Settings.separationRadius)
                        continue;

                    avgHeading += neighborTransform.forwardDirection;
                    count++;
                }

                if (count > 0)
                {
                    avgHeading /= count;
                    avgHeading -= transform.forwardDirection; // Remove the boid's own heading
                }

                if (math.any(math.isnan(avgHeading)))
                {
                    boidAspect.AlignmentForce = float3.zero;
                    return;
                }

                boidAspect.AlignmentForce = avgHeading;
            }
        }
    }
}