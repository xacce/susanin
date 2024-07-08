using BlobActor.Runtime;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace Jobs
{
    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct SeeekerJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public float deltaTime;
        public Entity pathRequestEntity;
        public EntityCommandBuffer.ParallelWriter ecb;
        private bool _cached;
        [ReadOnly] public NavMeshQuery query;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<NavMeshLocation> _straight;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<float> _sides;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<StraightPathFlags> _flags;

        private int _index;


        [BurstCompile]
        private void Execute(ref ActorRuntime actorRuntime, ref SusaninActor susaninActor, in Actor actor, DynamicBuffer<NavMeshPathElement> path, ref LocalTransform transform,
            Entity entity)
        {
            ref var agentBlob = ref actor.blob.Value;
            bool located = false;
            if (!path.IsEmpty && (actorRuntime.flags & ActorRuntime.Flag.PathSeeekerEnabled) != 0)
            {

                #region Progress path

                if (susaninActor.currentLocation.polygon.IsNull()) //Fresh actor, need set base location
                {
                    susaninActor.currentLocation = query.CreateLocation(transform.Position, path[0].polygonId);

                }
                //trying to detect progress on path
                susaninActor.currentLocation = query.MoveLocation(susaninActor.currentLocation, transform.Position, agentBlob.areaMask);
                bool correct = false;
                for (int i = 0; i < path.Length; i++)
                {
                    var element = path[i];

                    if (element.polygonId.Equals(susaninActor.currentLocation.polygon))
                    {
                        actorRuntime.currentPathIndex = i;
                        correct = true;
                    }

                }
                if (!correct) // current position not found in current path-push position at start of nodes
                {
                    actorRuntime.currentPathIndex = 0;
                    path.Insert(
                        0,
                        new NavMeshPathElement()
                        {
                            polygonId = susaninActor.currentLocation.polygon
                        });
                }
                located = true;

                #endregion

                if (actorRuntime.currentPathIndex == path.Length - 1 && math.distancesq(transform.Position, actorRuntime.destination) <= agentBlob.stoppingDistanceSq)
                {
                    actorRuntime.Set(ActorRuntime.Flag.Reached);
                    actorRuntime.currentPathIndex = 0;
                    path.Clear();
                    actorRuntime.velocity = float3.zero;
                }
                else
                {
                    actorRuntime.Unset(ActorRuntime.Flag.Reached);

                    #region Smooth path

                    var ppath = path.Reinterpret<PolygonId>().AsNativeArray();
                    var slice = new NativeSlice<PolygonId>(ppath, actorRuntime.currentPathIndex);
                    int straightCount = 0;
                    StraightPath.FindStraightPath(
                        ref query,
                        transform.Position,
                        actorRuntime.destination,
                        slice,
                        path.Length - actorRuntime.currentPathIndex,
                        ref _straight,
                        ref _flags,
                        ref _sides,
                        4,
                        ref straightCount,
                        out var result);

                    if (!result)
                    {
                        Debug.Log($"Cant build straight path: {entity.Index}");
                        path.Clear();
                    }
                    else
                    {
                        var newDirection = math.normalizesafe((float3)_straight[1].position - transform.Position) * actorRuntime.maxSpeed;
                        actorRuntime.velocity = math.lerp(actorRuntime.velocity, newDirection, agentBlob.acceleration * deltaTime);
                    }

                    #endregion

                }
            }
            else
            {
                actorRuntime.currentPathIndex = 0;
            }
            if (!located || susaninActor.currentLocation.polygon.IsNull())
                susaninActor.currentLocation = query.MoveLocation(susaninActor.currentLocation, transform.Position);

            if (!susaninActor.currentLocation.polygon.IsNull()) transform.Position = susaninActor.currentLocation.position;
            if (actorRuntime.isReached && !actorRuntime.requireRepath) return;
            if (agentBlob.repathRate > 0) actorRuntime.repath -= deltaTime;
            if (actorRuntime.requireRepath || (actorRuntime.autoRepath && !actorRuntime.isReached && agentBlob.repathRate > 0 && actorRuntime.repath <= 0))
            {
                actorRuntime.repath = agentBlob.repathRate;
                actorRuntime.Unset(ActorRuntime.Flag.ForceRepath);
                actorRuntime.Unset(ActorRuntime.Flag.Reached);
                NavMeshSearchPath.Create(entity, transform.Position, actorRuntime.destination, agentBlob.extents, agentBlob.agentTypeId, agentBlob.areaMask, out var request);
                ecb.AppendToBuffer(_index, pathRequestEntity, request);
            }
        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            if (!_cached)
            {
                _straight = new NativeArray<NavMeshLocation>(4, Allocator.Temp);
                _sides = new NativeArray<float>(4, Allocator.Temp);
                _flags = new NativeArray<StraightPathFlags>(4, Allocator.Temp);
                _cached = true;
            }
            _index = unfilteredChunkIndex;
            return true;
        }


        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
        {
        }
    }
}