using System;
using BlobActor.Runtime;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Random = Unity.Mathematics.Random;

namespace Amples.RandomAgents
{
    public partial struct DemoAgentRandomDestinations : IComponentData
    {
        public float radius;
        public float delay;
        public float currentDelay;
    }

    [BurstCompile]
    public partial struct RndSystem : ISystem
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
            foreach (var (agent, d, ltw, entity) in SystemAPI.Query<RefRO<ActorRuntime>, RefRW<DemoAgentRandomDestinations>, RefRO<LocalToWorld>>().WithEntityAccess())
            {
                var dRo = d.ValueRO;
                dRo.currentDelay -= SystemAPI.Time.DeltaTime;
                if (dRo.currentDelay <= 0)
                {
                    dRo.currentDelay = dRo.delay;
                    var dir = _rnd.NextFloat3Direction();
                    dir.y = 0f;
                    buff.Add(
                        new NavMeshSearchPath()
                        {
                            from = ltw.ValueRO.Position,
                            to = ltw.ValueRO.Position + dir * _rnd.NextFloat(1, dRo.radius),
                            agentTypeId = 0,
                            extents = new float3(1f, 1f, 1f),
                            areaMask = -1,
                            response = entity,
                        });
                }
                d.ValueRW = dRo;
            }
        }
    }
}