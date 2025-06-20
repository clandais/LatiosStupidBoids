using Boids.Systems.Boids;
using Latios;
using Latios.Transforms.Systems;
using Unity.Entities;

namespace Boids.Systems
{
    [DisableAutoCreation]
    [UpdateInGroup(typeof(PreTransformSuperSystem))]
    public partial class BoidSuperSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            GetOrCreateAndAddUnmanagedSystem<BoidsInitializationSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsCohesionSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsSeparationSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsAlignmentSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsFollowGoalSystem>();
        }
    }
}