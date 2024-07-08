using System;
using BlobActor.Runtime;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Random = Unity.Mathematics.Random;
namespace Amples.FixedDestination
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
            foreach (var (agent, d, ltw, entity) in SystemAPI.Query<RefRW<ActorRuntime>, RefRW<DemoAgentFixedDestination>, RefRO<LocalToWorld>>().WithEntityAccess())
            {
                var agentRo = agent.ValueRO;
                agentRo.Set(ActorRuntime.Flag.ForceRepath);
                agentRo.destination = d.ValueRO.point;
                agent.ValueRW = agentRo;
                // buff.Add(
                // new NavMeshSearchPath()
                // {
                // from = ltw.ValueRO.Position,
                // to = d.ValueRO.point,
                // agentTypeId = 0,
                // extents = new float3(1f, 1f, 1f),
                // areaMask = -1,
                // response = entity,
                // });
            }
            state.Enabled = false;
        }
    }
}