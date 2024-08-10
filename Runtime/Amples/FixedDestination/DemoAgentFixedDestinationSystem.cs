using System;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Xacce.BlobActor.Runtime;
using Random = Unity.Mathematics.Random;

namespace Xacce.Susanin.Runtime.Amples.FixedDestination
{
    public partial struct DemoAgentFixedDestination : IComponentData
    {
        public float3 point;
    }

    [BurstCompile]
    public partial struct DemoAgentFixedDestinationSystem : ISystem
    {
        private Random _rnd;

        [BurstDiscard]
        public void OnCreate(ref SystemState state)
        {
            _rnd = Random.CreateFromIndex(Core.Runtime.Utility.GetUniqueUIntFromInt(DateTime.Now.Millisecond));
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var buff = SystemAPI.GetSingletonBuffer<NavMeshSearchPath>();
            foreach (var (p, agent, d, ltw, entity) in SystemAPI.Query<RefRW<BlobActorPath>, RefRW<BlobActorFlags>, RefRW<DemoAgentFixedDestination>, RefRO<LocalToWorld>>()
                         .WithEntityAccess())
            {
                var agentRo = agent.ValueRO;
                agentRo.Set(BlobActorFlags.Flag.ForceRepath);
                ref var prw = ref p.ValueRW;
                prw.destination = d.ValueRO.point;
                agent.ValueRW = agentRo;
            }

            state.Enabled = false;
        }
    }
}