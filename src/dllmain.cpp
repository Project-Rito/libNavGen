// dllmain.cpp : Defines the entry point for the DLL application.
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "ChunkyTriMesh.h"
#include "framework.h"
#include <cmath>
#include <cstring>
#include <vector>


inline unsigned int nextPow2(unsigned int v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

inline unsigned int ilog2(unsigned int v)
{
    unsigned int r;
    unsigned int shift;
    r = (v > 0xffff) << 4; v >>= r;
    shift = (v > 0xff) << 3; v >>= shift; r |= shift;
    shift = (v > 0xf) << 2; v >>= shift; r |= shift;
    shift = (v > 0x3) << 1; v >>= shift; r |= shift;
    r |= (v >> 1);
    return r;
}

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

#define TILE_WIDTH 250
#define TILE_HEIGHT 250

rcContext* context = nullptr;
dtNavMesh* navmesh = nullptr;

rcConfig config;
DllExport [[maybe_unused]] void
SetNavmeshBuildParams(float cs, float ch, float walkableSlopeAngle, float walkableHeight, float walkableClimb,
    float walkableRadius, int minRegionArea, float detailSampleDist, float detailSampleMaxError) {

    memset(&config, 0, sizeof(config));
    config.cs = cs;
    config.ch = ch;
    config.walkableSlopeAngle = walkableSlopeAngle;
    config.walkableHeight = (int)ceilf(walkableHeight / ch);
    config.walkableClimb = (int)floorf(walkableClimb / ch);
    config.walkableRadius = (int)ceilf(walkableRadius / cs);
    config.maxEdgeLen = (int)12;
    config.maxSimplificationError = 1.3f;
    config.minRegionArea = (int)rcSqr(minRegionArea); // Note: area = size*size
    config.mergeRegionArea = (int)rcSqr(20); // Note: area = size*size
    config.maxVertsPerPoly = 3;
    config.tileSize = TILE_WIDTH / config.cs; // NAVGEN_TODO: Make param
    config.borderSize = config.walkableRadius + 3; // Idk the demo does it
    config.width = (config.tileSize + config.borderSize*2); // Demo does this too
    config.height = (config.tileSize + config.borderSize*2); // And this
    config.detailSampleDist = detailSampleDist;
    config.detailSampleMaxError = detailSampleMaxError;
}

DllExport [[maybe_unused]] bool BuildTileForMesh(float* verts, int vcount, int* indices, int icount, int tx, int ty, float orig[3], bool field) {
    if (context == nullptr) {
        context = new rcContext();
    }
    if (navmesh == nullptr) {
        dtNavMeshParams params;
        memset(&params, 0, sizeof(params));
        params.orig[0] = 0.f;
        params.orig[1] = 0.f;
        params.orig[2] = 0.f;
        params.tileHeight = TILE_HEIGHT;
        params.tileWidth = TILE_WIDTH;

        // Max tiles and max polys affect how the tile IDs are caculated.
        // There are 22 bits available for identifying a tile and a polygon.
        int tileBits = rcMin((int)ilog2(nextPow2(1000.f / TILE_WIDTH * 1000.f / TILE_HEIGHT)), 14);
        if (tileBits > 14) tileBits = 14;
        int polyBits = 22 - tileBits;
        params.maxTiles = 1 << tileBits;
        params.maxPolys = 1 << polyBits;

        navmesh = dtAllocNavMesh();
        navmesh->init(&params);
    }


    float bmin[3];
    float bmax[3];

    // Calculate bounding box
    for (int i = 0; i < vcount; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == 0) {
                bmin[j] = verts[j];
                bmax[j] = verts[j];
            }
            else {
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

    if (field) {
        bmin[0] = orig[0];// -((TILE_WIDTH)*config.cs);
        bmin[2] = orig[2];// -((TILE_WIDTH)*config.cs);
        bmax[0] = orig[0] + ((TILE_WIDTH));
        bmax[2] = orig[2] + ((TILE_WIDTH));
    }


    rcVcopy(config.bmin, bmin);
    rcVcopy(config.bmax, bmax);
    //rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

    config.bmin[0] -= config.borderSize;
    config.bmin[2] -= config.borderSize;
    config.bmax[0] += config.borderSize;
    config.bmax[2] += config.borderSize;

    // Step 2: rasterization
    rcHeightfield* heightfield = rcAllocHeightfield();
    if (!heightfield) {
        return false;
    }
    if (!rcCreateHeightfield(context, *heightfield, config.width, config.height, config.bmin, config.bmax, config.cs,
        config.ch)) {
        return false;
    }

    rcChunkyTriMesh* chunkyMesh = new rcChunkyTriMesh;
    rcCreateChunkyTriMesh(verts, indices, icount / 3, 256, chunkyMesh);

    float tbmin[2], tbmax[2];
    tbmin[0] = config.bmin[0];
    tbmin[1] = config.bmin[2];
    tbmax[0] = config.bmax[0];
    tbmax[1] = config.bmax[2];
    int cid[8192];// TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 8192);
    if (!ncid)
        return 0;

    auto* triareas = new unsigned char[icount / 3];
    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* ctris = &chunkyMesh->tris[node.i * 3];
        const int nctris = node.n;

        memset(triareas, 0, nctris * sizeof(unsigned char));
        rcMarkWalkableTriangles(context, config.walkableSlopeAngle,
            verts, vcount, ctris, nctris, triareas);

        if (!rcRasterizeTriangles(context, verts, vcount, ctris, triareas, nctris, *heightfield, config.walkableClimb))
            return false;
    }

    delete[] triareas;
    triareas = 0;

    // Step 3: Filter walkable surfaces

    // Step 4: Partitioning
    rcCompactHeightfield* cheightfield = rcAllocCompactHeightfield();
    if (!cheightfield) {
        return false;
    }
    if (!rcBuildCompactHeightfield(context, config.walkableHeight, config.walkableClimb, *heightfield, *cheightfield)) {
        return false;
    }
    rcFreeHeightField(heightfield);

    //if (!rcErodeWalkableArea(context, config.walkableRadius, *cheightfield)) {
    //    return false;
    //}

    // Use watershed partitioning
    if (!rcBuildDistanceField(context, *cheightfield)) {
        return false;
    }

    if (!rcBuildRegions(context, *cheightfield, config.borderSize, config.minRegionArea, config.mergeRegionArea)) {
        return false;
    }

    // Step 5: Contours
    rcContourSet* cset = rcAllocContourSet();
    if (!cset) {
        return false;
    }
    if (!rcBuildContours(context, *cheightfield, config.maxSimplificationError, config.maxEdgeLen, *cset)) {
        return false;
    }

    if (cset->nconts == 0)
        return false;

    // Step 6: poly mesh
    rcPolyMesh* pmesh;
    pmesh = rcAllocPolyMesh();
    if (!pmesh) {
        return false;
    }
    if (!rcBuildPolyMesh(context, *cset, config.maxVertsPerPoly, *pmesh)) {
        return false;
    }

    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    if (!rcBuildPolyMeshDetail(context, *pmesh, *cheightfield, config.detailSampleDist, config.detailSampleMaxError, *dmesh))
        return false;

    rcFreeCompactHeightfield(cheightfield);
    cheightfield = 0;
    rcFreeContourSet(cset);
    cset = 0;

    dtNavMeshCreateParams createParams;
    memset(&createParams, 0, sizeof(createParams));
    createParams.verts = pmesh->verts;
    createParams.vertCount = pmesh->nverts;
    createParams.polys = pmesh->polys;
    createParams.polyAreas = pmesh->areas;
    createParams.polyFlags = pmesh->flags;
    createParams.polyCount = pmesh->npolys;
    createParams.nvp = pmesh->nvp;
    createParams.detailMeshes = dmesh->meshes;
    createParams.detailVerts = dmesh->verts;
    createParams.detailVertsCount = dmesh->nverts;
    createParams.detailTris = dmesh->tris;
    createParams.detailTriCount = dmesh->ntris;
    //createParams.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
    //createParams.offMeshConRad = m_geom->getOffMeshConnectionRads();
    //createParams.offMeshConDir = m_geom->getOffMeshConnectionDirs();
    //createParams.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
    //createParams.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
    //createParams.offMeshConUserID = m_geom->getOffMeshConnectionId();
    //createParams.offMeshConCount = m_geom->getOffMeshConnectionCount();
    createParams.walkableHeight = config.walkableHeight;
    createParams.walkableRadius = config.walkableRadius;
    createParams.walkableClimb = config.walkableClimb;
    createParams.tileX = tx;
    createParams.tileY = ty;
    createParams.tileLayer = 0;
    rcVcopy(createParams.bmin, pmesh->bmin);
    rcVcopy(createParams.bmax, pmesh->bmax);
    createParams.cs = config.cs;
    createParams.ch = config.ch;
    createParams.buildBvTree = true;

    unsigned char* navData = 0;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&createParams, &navData, &navDataSize))
        return false;
    if (!navmesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0))
        return false;


    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);

    return true;
}

DllExport [[maybe_unused]] bool AddPrebuiltTile(float* verts, unsigned int vcount, unsigned int* indices, unsigned int* neighbors, unsigned int icount, unsigned int nvp, int tx, int ty, float orig[3]) {
    unsigned short* sVertices = new unsigned short[vcount * 3];
    for (int i = 0; i < vcount; i++) {
        sVertices[i * 3 + 0] = verts[i * 3 + 0] / config.cs;
        sVertices[i * 3 + 1] = verts[i * 3 + 1] / config.ch;
        sVertices[i * 3 + 2] = verts[i * 3 + 2] / config.cs;
    }

    unsigned short* polys = new unsigned short[icount * 2];
    for (int i = 0; i < icount / nvp; i++) { // index per poly
        for (int j = 0; j < nvp; j++) { // index per vertex in that poly
            polys[(i * nvp * 2) + j] = indices[(i * nvp) + j];
        }
        for (int j = 0; j < nvp; j++) {
            polys[(i * nvp * 2) + nvp + j] = neighbors[(i * nvp) + j];
        }
    }


    unsigned char* polyAreas = new unsigned char[icount / nvp];
    for (int i = 0; i < icount / nvp; i++)
        polyAreas[i] = 0;

    unsigned short* polyFlags = new unsigned short[icount / nvp];
    for (int i = 0; i < icount / nvp; i++)
        polyFlags[i] = 0;

    float bmin[3];
    bmin[0] = orig[0] - (config.borderSize * config.cs);
    bmin[1] = orig[1] - (config.borderSize * config.cs);
    bmin[2] = orig[2] - (config.borderSize * config.cs);

    float bmax[3];
    bmax[0] = orig[0] + ((TILE_WIDTH)*config.cs) + (config.borderSize * config.cs);
    bmax[1] = orig[1] + ((TILE_WIDTH)*config.cs) + (config.borderSize * config.cs);
    bmax[2] = orig[2] + ((TILE_WIDTH)*config.cs) + (config.borderSize * config.cs);


    dtNavMeshCreateParams createParams;
    memset(&createParams, 0, sizeof(createParams));
    createParams.verts = sVertices;
    createParams.vertCount = vcount;
    createParams.polys = polys;
    createParams.polyAreas = polyAreas;
    createParams.polyFlags = polyFlags;
    createParams.polyCount = icount / nvp;
    createParams.nvp = nvp;
    //createParams.detailMeshes = nullptr;
    //createParams.detailVerts = nullptr;
    //createParams.detailVertsCount = 0;
    //createParams.detailTris = nullptr;
    //createParams.detailTriCount = 0;
    //createParams.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
    //createParams.offMeshConRad = m_geom->getOffMeshConnectionRads();
    //createParams.offMeshConDir = m_geom->getOffMeshConnectionDirs();
    //createParams.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
    //createParams.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
    //createParams.offMeshConUserID = m_geom->getOffMeshConnectionId();
    //createParams.offMeshConCount = m_geom->getOffMeshConnectionCount();
    createParams.walkableHeight = config.walkableHeight;
    createParams.walkableRadius = config.walkableRadius;
    createParams.walkableClimb = config.walkableClimb;
    createParams.tileX = tx;
    createParams.tileY = ty;
    createParams.tileLayer = 0;
    rcVcopy(createParams.bmin, bmin);
    rcVcopy(createParams.bmax, bmax);
    createParams.cs = config.cs;
    createParams.ch = config.ch;
    createParams.buildBvTree = true;

    unsigned char* navData = 0;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&createParams, &navData, &navDataSize))
        return false;
    if (!navmesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0))
        return false;

    return true;
}

DllExport [[maybe_unused]] void ClearTiles() {
    dtFreeNavMesh(navmesh);
    navmesh = nullptr;
}

DllExport [[maybe_unused]] int GetMaxVertsPerPoly(int tx, int ty) {
    if (navmesh != nullptr) {
        return DT_VERTS_PER_POLYGON;
    }
    return -1;
}
DllExport [[maybe_unused]] int GetVertCount(int tx, int ty) {
    if (navmesh != nullptr) {
        //return navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0))->header->vertCount;

        //TEST DETAIL STUFF
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        return tile->header->vertCount + tile->header->detailVertCount;
        
    }
    return -1;
}
DllExport [[maybe_unused]] int GetPolyCount(int tx, int ty) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        //return tile->header->polyCount;

        // TEST DETAIL STUFF
        int detailMeshCount = 0;
        for (int i = 0; i < tile->header->detailMeshCount; i++) { // Figure how many meshes actually have data
            dtPolyDetail* meshDef = &tile->detailMeshes[i];

            if (meshDef->triCount > 0)
                detailMeshCount++;
        }

        return tile->header->polyCount - detailMeshCount + tile->header->detailTriCount;
        
    }
    return -1;
}
DllExport [[maybe_unused]] void GetVerts(int tx, int ty, void* buffer) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        //memcpy(buffer, tile->verts, tile->header->vertCount * sizeof(float) * 3);

        // TEST DETAIL STUFF
        memcpy(buffer, tile->verts, tile->header->vertCount * sizeof(float) * 3);
        memcpy(&((float*)buffer)[tile->header->vertCount * 3], tile->detailVerts, tile->header->detailVertCount * sizeof(float) * 3);
        
    }
}
DllExport [[maybe_unused]] void GetBoundingBox(int tx, int ty, float* buffer) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        buffer[0] = tile->header->bmin[0];
        buffer[1] = tile->header->bmin[1];
        buffer[2] = tile->header->bmin[2];
        buffer[3] = tile->header->bmax[0];
        buffer[4] = tile->header->bmax[1];
        buffer[5] = tile->header->bmax[2];
    }
}

DllExport [[maybe_unused]] void GetPolyVerts(int tx, int ty, void* buffer, unsigned int polyIndex) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        //memcpy(buffer, tile->polys[polyIndex].verts, tile->polys[polyIndex].vertCount * sizeof(unsigned short));
        
        //

        /*
        dtPolyDetail* meshDef;
        for (unsigned int d = 0, p = 0; d < tile->header->detailMeshCount; d++) {
            meshDef = &tile->detailMeshes[polyIndex];

            p += meshDef->triCount;

            if (p >= polyIndex)
                break;
        }

        unsigned char* tri = &tile->detailTris[polyIndex * 4];
        ((unsigned short*)buffer)[0] = tri[0] + meshDef->vertBase;
        ((unsigned short*)buffer)[1] = tri[1] + meshDef->vertBase;
        ((unsigned short*)buffer)[2] = tri[2] + meshDef->vertBase;
        */

        //

        // TEST DETAIL STUFF
        int i = 0;
        for (int p = 0; p < tile->header->polyCount; p++) {
            dtPolyDetail* mesh = &tile->detailMeshes[p];
            if (mesh->vertCount > 0) {

                for (int d = 0; d < mesh->triCount; d++) {
                    if (i + d == polyIndex) {
                        unsigned char* tri = &tile->detailTris[(mesh->triBase + d) * 4];
                        for (int k = 0; k < 3; k++) {
                            if (tri[k] < tile->polys[p].vertCount)
                                ((unsigned short*)buffer)[k] = tile->polys[p].verts[tri[k]];
                            else
                                ((unsigned short*)buffer)[k] = tile->header->vertCount + (mesh->vertBase + tri[k] - tile->polys[p].vertCount);
                        }
                        //((unsigned short*)buffer)[0] = tri[0] + mesh->vertBase;
                        //((unsigned short*)buffer)[1] = tri[1] + mesh->vertBase;
                        //((unsigned short*)buffer)[2] = tri[2] + mesh->vertBase;
                        return;
                    }
                }

                i += tile->detailMeshes[p].triCount;
            }
            else {
                if (i == polyIndex) {
                    memcpy(buffer, tile->polys[p].verts, tile->polys[p].vertCount * sizeof(unsigned short));
                    return;
                }

                i++;
            }
        }

        
    }
}

DllExport [[maybe_unused]] unsigned char GetPolyVertCount(int tx, int ty, unsigned int polyIndex) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        //return tile->polys[polyIndex].vertCount;

        // TEST DETAIL STUFF
        int i = 0;
        for (int p = 0; p < tile->header->polyCount; p++) {
            dtPolyDetail* mesh = &tile->detailMeshes[p];
            if (mesh->vertCount > 0) {

                for (int d = 0; d < mesh->triCount; d++) {
                    if (i + d == polyIndex) {
                        return 3;
                    }
                }

                i += tile->detailMeshes[p].triCount;
            }
            else {
                if (i == polyIndex) {
                    return tile->polys[p].vertCount;
                }

                i++;
            }
        }
        
    }

    return 0;
}

unsigned int GetRemappedPolyIndex(int tx, int ty, int polyIndex) {
    const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

    int i = 0;
    for (int p = 0; p < tile->header->polyCount; p++) {
        if (p == polyIndex)
            return i;

        dtPolyDetail* mesh = &tile->detailMeshes[p];
        if (mesh->vertCount > 0) {
            i += tile->detailMeshes[p].triCount;
        }
        else {
            i++;
        }
    }

    return -1;
}

unsigned int GetRemappedDetailTriIndex(int tx, int ty, int polyIndex, int detailTriIndex) {
    const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

    int i = 0;
    for (int p = 0; p < tile->header->polyCount; p++) {
        dtPolyDetail* mesh = &tile->detailMeshes[p];
        if (mesh->vertCount > 0) {
            if (p == polyIndex) {
                for (int d = 0; d < mesh->triCount; d++) {
                    if (d == detailTriIndex) {
                        return i + d;
                    }
                }
            }

            i += tile->detailMeshes[p].triCount;
        }
        else {
            i++;
        }
    }

    return -1;
}

DllExport [[maybe_unused]] void GetPolyNeighborInfo(int tx, int ty, void* buffer, unsigned int polyIndex) {
    if (navmesh != nullptr) {
        const dtMeshTile* tile = navmesh->getTileByRef(navmesh->getTileRefAt(tx, ty, 0));

        //memcpy(buffer, tile->polys[polyIndex].neis, tile->polys[polyIndex].vertCount * sizeof(unsigned short));

        //

        /*
        dtPolyDetail* meshDef;
        for (unsigned int d = 0, p = 0; d < tile->header->detailMeshCount; d++) {
            meshDef = &tile->detailMeshes[polyIndex];

            p += meshDef->triCount;

            if (p >= polyIndex)
                break;
        }

        unsigned char* tri = &tile->detailTris[polyIndex * 4];
        ((unsigned short*)buffer)[0] = tri[3];
        */


        // TEST NONWORKING DETAIL STUFF
        unsigned int polyIndex = 0xFFFFFFFF;
        unsigned int detailTriIndex = 0xFFFFFFFF;


        std::vector<unsigned short> polyNeis;

        int i = 0;
        bool foundPoly = false;
        for (int p = 0; p < tile->header->polyCount && !foundPoly; p++) {
            dtPolyDetail* mesh = &tile->detailMeshes[p];
            if (mesh->vertCount > 0) {
                for (int d = 0; d < mesh->triCount; d++) {
                    if (i + d == polyIndex) {
                        for (unsigned short nei : tile->polys[p].neis)
                            polyNeis.push_back(nei - 1); // -1 is decoding thing

                        polyNeis.push_back(p); // Check this poly for detail neighbors.

                        polyIndex = p;
                        detailTriIndex = d;

                        foundPoly = true;
                        break;
                    }
                }

                i += tile->detailMeshes[p].triCount;
            }
            else {
                if (i == polyIndex) {
                    for (unsigned short nei : tile->polys[p].neis)
                        polyNeis.push_back(nei - 1); // -1 is decoding thing
                    
                    polyIndex = p;

                    foundPoly = true;

                    //memcpy(polyNeis, tile->polys[p].neis, tile->polys[p].vertCount * sizeof(unsigned short));
                }

                i++;
            }
        }

        unsigned int remappedPolyIndex;
        if (detailTriIndex == 0xFFFFFFFF)
            remappedPolyIndex = GetRemappedPolyIndex(tx, ty, polyIndex);
        else
            remappedPolyIndex = GetRemappedDetailTriIndex(tx, ty, polyIndex, detailTriIndex);

        unsigned char polyVertCount;
        polyVertCount = GetPolyVertCount(tx, ty, remappedPolyIndex);
        unsigned short* polyVerts = new unsigned short[polyVertCount];
        GetPolyVerts(tx, ty, polyVerts, remappedPolyIndex);

        std::vector<unsigned short> finalNeis;

        for (unsigned short pNei : polyNeis) {
            dtPolyDetail* mesh = &tile->detailMeshes[pNei];

            if (mesh->vertCount > 0) {
                for (int dNei = 0; dNei < mesh->vertCount; dNei++) {
                    if (pNei == polyIndex && dNei == detailTriIndex)
                        continue; // Don't return itself as a neighbor.

                    unsigned short neighborDetailTriVerts[3];
                    GetPolyVerts(tx, ty, &neighborDetailTriVerts, GetRemappedDetailTriIndex(tx, ty, pNei, dNei));

                    for (int v = 0; v < polyVertCount; v++) {
                        for (int nV = 0; nV < 3; nV++) {
                            if (neighborDetailTriVerts[nV] == polyVerts[v] && neighborDetailTriVerts[(nV + 1) % 3] == polyVerts[(v + 1) % polyVertCount])
                                finalNeis.push_back(GetRemappedDetailTriIndex(tx, ty, pNei, dNei));
                        }
                    }
                }
            }
            else {
                finalNeis.push_back(GetRemappedPolyIndex(tx, ty, pNei));
            }
        }
        
        delete[] polyVerts;
    }
}

struct PolyTileLink {
    unsigned int srcPolyIndex;
    unsigned int srcEdgeIndex;
    unsigned int destPolyIndex;
    unsigned int destEdgeIndex;
    int srcTileX;
    int srcTileY;
    int destTileX;
    int destTileY;
    unsigned char bmin;
    unsigned char bmax;
};

// Finds links for polygon 
DllExport [[maybe_unused]] void GetPolyLinks(int stx, int sty, PolyTileLink buffer[], unsigned int polyIndex) {
    if (navmesh != nullptr) {
        const dtMeshTile* srcTile = navmesh->getTileByRef(navmesh->getTileRefAt(stx, sty, 0));
        const dtPoly* srcPoly = &srcTile->polys[polyIndex];


        unsigned int resLinkIndex = 0; // Not the index in the tile, the index in what we return 
        for (unsigned int i = srcPoly->firstLink; i != DT_NULL_LINK; i = srcTile->links[i].next) {
            const dtLink link = srcTile->links[i];

            PolyTileLink res = buffer[resLinkIndex];

            res.srcPolyIndex = polyIndex;
            res.srcEdgeIndex = link.edge;
            res.srcTileX = srcTile->header->x;
            res.srcTileY = srcTile->header->y;

            unsigned int destPolySalt, destTileIndex, destPolyIndex;
            navmesh->decodePolyId(link.ref, destPolySalt, destTileIndex, destPolyIndex);
            res.destPolyIndex = destPolyIndex;

            const dtMeshTile* destTile;
            const dtPoly* destPoly;
            navmesh->getTileAndPolyByRef(link.ref, &destTile, &destPoly);

            if (srcTile == destTile)
                continue;

            unsigned int destEdgeIndex = 0xFFFFFFFF;
            for (unsigned int j = destPoly->firstLink; j != DT_NULL_LINK; j = destTile->links[j].next) {

                const dtLink candidateLink = destTile->links[j];

                const dtMeshTile* candidateTile;
                const dtPoly* candidatePoly;
                navmesh->getTileAndPolyByRef(candidateLink.ref, &candidateTile, &candidatePoly);

                if (candidatePoly == srcPoly) {
                    destEdgeIndex = candidateLink.edge;
                    continue;
                }
            }

            res.destEdgeIndex = destEdgeIndex;

            res.destTileX = destTile->header->x;
            res.destTileY = destTile->header->y;

            res.bmin = link.bmin;
            res.bmax = link.bmax;

            buffer[resLinkIndex] = res;

            resLinkIndex++;
        }

        // To get dest edge:
        // * Perhaps we could look at the linked poly's links and find what links back to this poly?
        //   A poly can only be linked to any one poly once, right?
    }
}


DllExport [[maybe_unused]] unsigned int GetPolyLinkCount(int stx, int sty, unsigned int polyIndex) {
    if (navmesh != nullptr) {
        const dtMeshTile* srcTile = navmesh->getTileByRef(navmesh->getTileRefAt(stx, sty, 0));
        const dtPoly* srcPoly = &srcTile->polys[polyIndex];

        unsigned int count = 0;
        for (unsigned int i = srcPoly->firstLink; i != DT_NULL_LINK; i = srcTile->links[i].next) {
            const dtLink link = srcTile->links[i];

            const dtMeshTile* destTile;
            const dtPoly* destPoly;
            navmesh->getTileAndPolyByRef(link.ref, &destTile, &destPoly);

            if (srcTile != destTile)
                count++;
        }

        return count;
    }
}

















/*
DllExport [[maybe_unused]] void GetMergedMeshVerts(void* buffer) {
    if (mergedTiles != nullptr) {
        memcpy(buffer, mergedTiles->verts, mergedTiles->nverts * sizeof(short) * 3);
    }
}
DllExport [[maybe_unused]] void GetMergedMeshPolys(void* buffer) {
    if (mergedTiles != nullptr) {
        memcpy(buffer, mergedTiles->polys, mergedTiles->npolys * sizeof(short) * 2 * mergedTiles->nvp);
    }
}
DllExport [[maybe_unused]] void GetMergedMeshAreas(void* buffer) {
    if (mergedTiles != nullptr) {
        memcpy(buffer, mergedTiles->areas, mergedTiles->npolys * sizeof(char));
    }
}
DllExport [[maybe_unused]] void GetMergedMeshRegions(void* buffer) {
    if (mergedTiles != nullptr) {
        memcpy(buffer, mergedTiles->regs, mergedTiles->npolys * sizeof(short));
    }
}
*/








/*
rcPolyMesh *currentMesh = nullptr;

float bmin[3];
float bmax[3];



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

DllExport [[maybe_unused]] void GetMeshAreas(void* buffer) {
    if (currentMesh != nullptr) {
        memcpy(buffer, currentMesh->areas, currentMesh->npolys * sizeof(char));
    }
}

DllExport [[maybe_unused]] void GetMeshRegions(void* buffer) {
    if (currentMesh != nullptr) {
        memcpy(buffer, currentMesh->regs, currentMesh->npolys * sizeof(short));
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

*/