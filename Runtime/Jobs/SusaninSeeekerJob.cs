using Core.Runtime;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Experimental.AI;
using Xacce.BlobActor.Runtime;

namespace Xacce.Susanin.Runtime.Jobs
{
    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct SusaninSeeekerJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public float deltaTime;
        public Entity pathRequestEntity;
        public EntityCommandBuffer.ParallelWriter ecb;

        private bool _cached;
        private int _index;
        
        [ReadOnly] public NavMeshQuery query;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<NavMeshLocation> _straight;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<float> _sides;

        [NativeDisableContainerSafetyRestriction]
        private NativeArray<StraightPathFlags> _flags;


        [BurstCompile]
        private void Execute(
            ref DynamicObjectVelocity dynamicObjectVelocity,
            in BlobActorLimits limits,
            ref BlobActorPath path,
            ref BlobActorFlags blobActorFlags,
            ref SusaninActor susaninActor,
            in LocalTransform transform,
            in BlobActor.Runtime.BlobActor blobActor,
            DynamicBuffer<NavMeshPathElement> pathElements,
            Entity entity)
        {
            ref var agentBlob = ref blobActor.blob.Value;
            bool located = false;
            if (!pathElements.IsEmpty && (blobActorFlags.flags & BlobActorFlags.Flag.PathSeeekerEnabled) != 0)
            {
                #region Progress path

                if (susaninActor.currentLocation.polygon.IsNull()) //Fresh actor, need set base location
                {
                    susaninActor.currentLocation = query.CreateLocation(transform.Position, pathElements[0].polygonId);
                }

                //trying to detect progress on path
                susaninActor.currentLocation = query.MoveLocation(susaninActor.currentLocation, transform.Position, agentBlob.areaMask);
                bool correct = false;
                for (int i = 0; i < pathElements.Length; i++)
                {
                    var element = pathElements[i];

                    if (element.polygonId.Equals(susaninActor.currentLocation.polygon))
                    {
                        path.currentPathIndex = i;
                        correct = true;
                    }
                }

                if (!correct) // current position not found in current path-push position at start of nodes
                {
                    path.currentPathIndex = 0;
                    pathElements.Insert(
                        0,
                        new NavMeshPathElement()
                        {
                            polygonId = susaninActor.currentLocation.polygon
                        });
                }

                located = true;

                #endregion

                if (path.currentPathIndex == pathElements.Length - 1 && math.distancesq(transform.Position, path.destination) <= agentBlob.stoppingDistanceSq)
                {
                    blobActorFlags.Set(BlobActorFlags.Flag.Reached);
                    path.currentPathIndex = 0;
                    pathElements.Clear();
                    dynamicObjectVelocity.velocity = float3.zero;
                }
                else
                {
                    blobActorFlags.Unset(BlobActorFlags.Flag.Reached);

                    #region Smooth path

                    var ppath = pathElements.Reinterpret<PolygonId>().AsNativeArray();
                    var slice = new NativeSlice<PolygonId>(ppath, path.currentPathIndex);
                    int straightCount = 0;
                    StraightPath.FindStraightPath(
                        ref query,
                        transform.Position,
                        path.destination,
                        slice,
                        pathElements.Length - path.currentPathIndex,
                        ref _straight,
                        ref _flags,
                        ref _sides,
                        4,
                        ref straightCount,
                        out var result);

                    if (!result)
                    {
                        Debug.Log($"Cant build straight path: {entity.Index}");
                        pathElements.Clear();
                    }
                    else
                    {
                        var newDirection = math.normalizesafe((float3)_straight[1].position - transform.Position) * limits.maxSpeed;
                        dynamicObjectVelocity.velocity = math.lerp(dynamicObjectVelocity.velocity, newDirection, agentBlob.acceleration * deltaTime);
                    }

                    #endregion
                }
            }
            else
            {
                path.currentPathIndex = 0;
            }

            if (!located || susaninActor.currentLocation.polygon.IsNull())
                susaninActor.currentLocation = query.MoveLocation(susaninActor.currentLocation, transform.Position);

            // if (!susaninActor.currentLocation.polygon.IsNull()) transform.Position = susaninActor.currentLocation.position;
            if (blobActorFlags.isReached && !blobActorFlags.requireRepath) return;
            if (agentBlob.repathRate > 0) path.repath -= deltaTime;
            if (blobActorFlags.requireRepath || (blobActorFlags.autoRepath && !blobActorFlags.isReached && agentBlob.repathRate > 0 && path.repath <= 0))
            {
                path.repath = agentBlob.repathRate;
                blobActorFlags.Unset(BlobActorFlags.Flag.ForceRepath);
                blobActorFlags.Unset(BlobActorFlags.Flag.Reached);
                NavMeshSearchPath.Create(entity, transform.Position, path.destination, agentBlob.extents, agentBlob.agentTypeId, agentBlob.areaMask, out var request);
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