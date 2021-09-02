// dllmain.cpp : Defines the entry point for the DLL application.
#include "Recast.h"
#include "framework.h"
#include <cmath>
#include <cstring>

#ifdef _WIN32
BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}
#endif

rcContext *context = nullptr;

rcPolyMesh *currentMesh = nullptr;

rcConfig config;

float bmin[3];
float bmax[3];

DllExport [[maybe_unused]] void
SetNavmeshBuildParams(float cs, float ch, float walkableSlopeAngle, float walkableHeight, float walkableClimb,
                      float walkableRadius, int minRegionArea) {
    memset(&config, 0, sizeof(config));
    config.cs = cs;
    config.ch = ch;
    config.walkableSlopeAngle = walkableSlopeAngle;
    config.walkableHeight = (int) ceilf(walkableHeight / ch);
    config.walkableClimb = (int) floorf(walkableClimb / ch);
    config.walkableRadius = (int) ceilf(walkableRadius / cs);
    config.maxEdgeLen = (int) 12;
    config.maxSimplificationError = 1.3f;
    config.minRegionArea = (int) rcSqr(minRegionArea); // Note: area = size*size
    config.mergeRegionArea = (int) rcSqr(20); // Note: area = size*size
    config.maxVertsPerPoly = 3;
    config.detailSampleDist = 6;
    config.detailSampleMaxError = 1;
}

// Navmesh building based off of demo:
// https://github.com/recastnavigation/recastnavigation/blob/master/RecastDemo/Source/Sample_SoloMesh.cpp
DllExport [[maybe_unused]] bool BuildNavmeshForMesh(float *verts, int vcount, int *indices, int icount) {
    if (context == nullptr) {
        context = new rcContext();
    }

    // Calculate bounding box
    for (int i = 0; i < vcount; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == 0) {
                bmin[j] = verts[j];
                bmax[j] = verts[j];
            } else {
                float v = verts[i * 3 + j];
                if (v < bmin[j]) {
                    bmin[j] = v;
                }
                if (v > bmax[j]) {
                    bmax[j] = v;
                }
            }
        }
    }

    rcVcopy(config.bmin, bmin);
    rcVcopy(config.bmax, bmax);
    rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

    // Step 2: rasterization
    rcHeightfield *heightfield = rcAllocHeightfield();
    if (!heightfield) {
        return false;
    }
    if (!rcCreateHeightfield(context, *heightfield, config.width, config.height, config.bmin, config.bmax, config.cs,
                             config.ch)) {
        return false;
    }

    auto *triareas = new unsigned char[icount / 3];
    memset(triareas, 0, (icount / 3) * sizeof(unsigned char));
    rcMarkWalkableTriangles(context, config.walkableSlopeAngle, verts, vcount, indices, icount / 3, triareas);
    if (!rcRasterizeTriangles(context, verts, vcount, indices, triareas, icount / 3, *heightfield,
                              config.walkableClimb)) {
        return false;
    }

    delete[] triareas;

    // Step 3: Filter walkable surfaces

    // Step 4: Partitioning
    rcCompactHeightfield *cheightfield = rcAllocCompactHeightfield();
    if (!cheightfield) {
        return false;
    }
    if (!rcBuildCompactHeightfield(context, config.walkableHeight, config.walkableClimb, *heightfield, *cheightfield)) {
        return false;
    }
    rcFreeHeightField(heightfield);

    if (!rcErodeWalkableArea(context, config.walkableRadius, *cheightfield)) {
        return false;
    }

    // Use watershed partitioning
    if (!rcBuildDistanceField(context, *cheightfield)) {
        return false;
    }

    if (!rcBuildRegions(context, *cheightfield, 0, config.minRegionArea, config.mergeRegionArea)) {
        return false;
    }

    // Step 5: Contours
    rcContourSet *cset = rcAllocContourSet();
    if (!cset) {
        return false;
    }
    if (!rcBuildContours(context, *cheightfield, config.maxSimplificationError, config.maxEdgeLen, *cset)) {
        return false;
    }

    // Step 6: poly mesh
    if (currentMesh != nullptr) {
        rcFreePolyMesh(currentMesh);
    }
    currentMesh = rcAllocPolyMesh();
    if (!currentMesh) {
        return false;
    }
    if (!rcBuildPolyMesh(context, *cset, config.maxVertsPerPoly, *currentMesh)) {
        return false;
    }

    return true;
}

DllExport [[maybe_unused]] int GetMeshVertCount() {
    if (currentMesh != nullptr) {
        return currentMesh->nverts;
    }
    return -1;
}

DllExport [[maybe_unused]] int GetMeshTriCount() {
    if (currentMesh != nullptr) {
        return currentMesh->npolys;
    }
    return -1;
}

DllExport [[maybe_unused]] void GetMeshVerts(void *buffer) {
    if (currentMesh != nullptr) {
        memcpy(buffer, currentMesh->verts, currentMesh->nverts * sizeof(short) * 3);
    }
}

DllExport [[maybe_unused]] void GetMeshTris(void *buffer) {
    if (currentMesh != nullptr) {
        memcpy(buffer, currentMesh->polys, currentMesh->npolys * sizeof(short) * 2 * currentMesh->nvp);
    }
}

DllExport [[maybe_unused]] void GetBoundingBox(float *buffer) {
    buffer[0] = bmin[0];
    buffer[1] = bmin[1];
    buffer[2] = bmin[2];
    buffer[3] = bmax[0];
    buffer[4] = bmax[1];
    buffer[5] = bmax[2];
}