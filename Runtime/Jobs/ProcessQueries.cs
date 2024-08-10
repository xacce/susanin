using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine.Experimental.AI;

namespace Xacce.Susanin.Runtime.Jobs
{
    [BurstCompile]
    public partial struct SusaninProcessQueriesJob : IJob
    {
        [NativeDisableContainerSafetyRestriction]
        public EntityCommandBuffer.ParallelWriter ecb;

        [NativeDisableContainerSafetyRestriction]
        public NativeQueue<int>.ParallelWriter pool;

        // [NativeDisableContainerSafetyRestriction] 
        public NavMeshQuery query;
        public int queryIndex;
        [NativeSetThreadIndex] public int index;

        [NativeDisableContainerSafetyRestriction]
        public NativeList<NavMeshQueryData> data;

        public int max;

        [BurstCompile]
        public void Execute()
        {
            var queryData = data[queryIndex];
            switch (queryData._status)
            {
                case NavMeshQueryData._Status.Idle:
                    return;
                case NavMeshQueryData._Status.Fresh:
                {
                    var locationFrom = query.MapLocation(queryData.request.from, queryData.request.extents, queryData.request.agentTypeId, queryData.request.areaMask);
                    var locationTo = query.MapLocation(queryData.request.to, queryData.request.extents, queryData.request.agentTypeId, queryData.request.areaMask);
                    if (locationFrom.polygon.IsNull() || locationTo.polygon.IsNull())
                    {
                        NavMeshQueryData.Reset(ref queryData);
                        pool.Enqueue(queryIndex);
                    }
                    else
                    {
                        queryData.status = query.BeginFindPath(locationFrom, locationTo, queryData.request.areaMask);
                        queryData._status = NavMeshQueryData._Status.Wait;
                    }
                    break;
                }
                case NavMeshQueryData._Status.Wait:
                {
                    queryData.status = query.UpdateFindPath(max, out var iterations);
                    queryData.iterations = iterations;
                    if ((queryData.status & PathQueryStatus.Success) != 0 || (queryData.status & PathQueryStatus.PartialResult) != 0)
                    {
                        if ((query.EndFindPath(out var pathSize) & PathQueryStatus.Success) != 0)
                        {
                            if (!queryData.request.response.Equals(Entity.Null))
                            {
                                var pl = new NativeArray<PolygonId>(pathSize, Allocator.Temp);
                                var buffer = ecb.SetBuffer<NavMeshPathElement>(index, queryData.request.response);
                                query.GetPathResult(pl);
                                for (int j = 0; j < pl.Length; j++)
                                {
                                    buffer.Add(
                                        new NavMeshPathElement()
                                        {
                                            polygonId = pl[j]
                                        });
                                }
                            }
                        }
                        NavMeshQueryData.Reset(ref queryData);
                        pool.Enqueue(queryIndex);
                    }
                    else
                    {
                        NavMeshQueryData.Reset(ref queryData);
                        pool.Enqueue(queryIndex);
                    }
                    break;
                }
            }
            data[queryIndex] = queryData;

        }
    }
}