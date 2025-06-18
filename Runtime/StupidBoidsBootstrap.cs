using Boids.Systems;
using Latios;
using Unity.Entities;

namespace Boids
{
    public static class StupidBoidsBootstrap
    {
        public static void InstallBoids(World world)
        {
            BootstrapTools.InjectSystem(TypeManager.GetSystemTypeIndex<BoidSuperSystem>(), world);
        }
    }
}