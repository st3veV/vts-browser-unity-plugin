/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

using System.Collections.Generic;
using TMPro;
using UnityEngine;
using vts;

namespace vts
{
    public static partial class Extensions
    {
        internal static TValue GetOrCreate<TKey, TValue>(this IDictionary<TKey, TValue> dict, TKey key) where TValue : new()
        {
            TValue val;
            if (!dict.TryGetValue(key, out val))
            {
                val = new TValue();
                dict.Add(key, val);
            }
            return val;
        }
    }
}

public class VtsCameraObjects : VtsCameraBase
{
    protected override void Start()
    {
        base.Start();
        partsGroup = new GameObject(name + " - parts").transform;
    }

    protected void OnDestroy()
    {
        Destroy(partsGroup);
    }

    protected override void CameraDraw()
    {
        shiftingOriginMap = mapObject.GetComponent<VtsMapShiftingOrigin>();
        conv = Math.Mul44x44(Math.Mul44x44(VtsUtil.U2V44(mapTrans.localToWorldMatrix), VtsUtil.U2V44(VtsUtil.SwapYZ)), Math.Inverse44(draws.camera.view));
        UpdateSurfacesCache(opaquePrefab, draws.opaque, opaquePartsCache);
        UpdateSurfacesCache(transparentPrefab, draws.transparent, transparentPartsCache);
        UpdateGeodataCache(draws.geodata, geodataPartsCache);
        originHasShifted = false;
    }

    public override void OriginShifted()
    {
        originHasShifted = true;
    }

    private void UpdateSurfacesCache(GameObject prefab, List<DrawSurfaceTask> tasks, Dictionary<VtsMesh, List<Part>> partsCache)
    {
        // organize tasks by meshes
        Dictionary<VtsMesh, List<DrawSurfaceTask>> tasksByMesh = new Dictionary<VtsMesh, List<DrawSurfaceTask>>();
        foreach (var t in tasks)
        {
            List<DrawSurfaceTask> ts = tasksByMesh.GetOrCreate(t.mesh as VtsMesh);
            ts.Add(t);
        }

        // remove obsolete cache entries
        {
            tmpKeys.UnionWith(partsCache.Keys);
            tmpKeys.ExceptWith(tasksByMesh.Keys);
            foreach (var k in tmpKeys)
            {
                foreach (var j in partsCache[k])
                    Destroy(j.go);
                partsCache.Remove(k);
            }
            tmpKeys.Clear();
        }

        // update remaining cache entries
        foreach (var k in tasksByMesh)
        {
            List<DrawSurfaceTask> ts = k.Value;
            List<Part> ps = partsCache.GetOrCreate(k.Key);

            // update current objects
            int updatable = ts.Count < ps.Count ? ts.Count : ps.Count;
            for (int i = 0; i < updatable; i++)
                UpdatePart(ps[i], ts[i]);

            // update the rest of the cache
            int changeCount = ts.Count - ps.Count;
            while (changeCount > 0)
            {
                // inflate
                ps.Add(InitPart(prefab, ts[updatable++]));
                changeCount--;
            }
            if (changeCount < 0)
            {
                // deflate
                foreach (var p in ps.GetRange(updatable, -changeCount))
                    Destroy(p.go);
                ps.RemoveRange(updatable, -changeCount);
            }
            Debug.Assert(ts.Count == ps.Count);
        }
    }

    private void UpdateGeodataCache(List<DrawGeodataTask> geodata, Dictionary<VtsGeodataObject, List<GeodataPart>> geodataPartsCache)
    {
        // organize tasks by geodata
        Dictionary<VtsGeodataObject, List<DrawGeodataObjectTask>> tasksByGeodata = new();

        var colors = new List<Color>{Color.cyan, Color.red, Color.green, Color.blue, Color.yellow, Color.magenta};
        var currentColor = 0;

        foreach (var t in geodata)
        {
            if (t.geodata is not VtsGeodataTile gd)
                continue;
            VtsGeodataObject[] gdos = gd.GetObjects();
            if(gdos.Length == 0)
                continue;
            foreach (var gdo in gdos)
            {
                DrawGeodataObjectTask ot = new()
                {
                    geodata = gdo,
                    color = colors[currentColor++%colors.Count],
                };
                List<DrawGeodataObjectTask> ts = tasksByGeodata.GetOrCreate(gdo);
                ts.Add(ot);
            }
        }

        // remove obsolete cache entries
        {
            tmpGeoKeys.UnionWith(geodataPartsCache.Keys);
            tmpGeoKeys.ExceptWith(tasksByGeodata.Keys);
            foreach (var k in tmpGeoKeys)
            {
                foreach (var j in geodataPartsCache[k])
                {
                    if (j.go != null)
                        Destroy(j.go);
                }
                geodataPartsCache.Remove(k);
            }
            tmpGeoKeys.Clear();
        }


        // update remaining cache entries
        foreach (var k in tasksByGeodata)
        {
            List<DrawGeodataObjectTask> ts = k.Value;
            List<GeodataPart> ps = geodataPartsCache.GetOrCreate(k.Key);

            // update current objects
            int updatable = ts.Count < ps.Count ? ts.Count : ps.Count;
            for (int i = 0; i < updatable; i++)
                UpdateGeodataPart(ps[i], ts[i]);

            // update the rest of the cache
            int changeCount = ts.Count - ps.Count;

            while (changeCount > 0)
            {
                // inflate
                ps.Add(InitGeodataPart(ts[updatable++]));
                changeCount--;
            }

        }
    }

    private GeodataPart InitGeodataPart(DrawGeodataObjectTask task)
    {
        VtsGeodataObject gd = task.geodata;
        GeodataPart part = new();
        GameObject pb = GetGeoPrefab(gd.tile.type);
        if (pb == null)
        {
            return part;
        }
        bool isValid = false;

        GameObject go = Instantiate(pb, partsGroup);
        Renderer[] renderers = go.GetComponentsInChildren<Renderer>();
        foreach (var r in renderers)
        {
            r.material.color = task.color;
        }
        var tmp = go.GetComponentInChildren<TextMeshPro>();
        if (tmp != null)
        {
            tmp.text = gd.id;
        }
        if (gd.positions.Length == 1)
        {
            // rendering a point
            isValid = UpdateGeodataTransformPoint(go, gd.positions[0], gd);
        }

        if (!isValid)
        {
            Destroy(go);
            return part;
        }

        if (shiftingOriginMap)
            go.GetOrAddComponent<VtsObjectShiftingOrigin>().map = shiftingOriginMap;
        part.go = go;

        return part;
    }

    private GameObject GetGeoPrefab(GeodataType type)
    {
        switch (type)
        {
            case GeodataType.IconScreen:
                return pointPrefab;
            case GeodataType.Invalid:
                break;
            case GeodataType.Triangles:
                break;
            case GeodataType.LineFlat:
                break;
            case GeodataType.PointFlat:
                return pointPrefab;
            case GeodataType.IconFlat:
                break;
            case GeodataType.LabelFlat:
                break;
            case GeodataType.LineScreen:
                break;
            case GeodataType.PointScreen:
                break;
            case GeodataType.LabelScreen:
                break;
            default:
                return null;
        }
        return null;
    }

    private void UpdateGeodataPart(GeodataPart geodataPart, DrawGeodataObjectTask task)
    {
        if (!originHasShifted)
            return;

        VtsGeodataObject gd = task.geodata;
        if(!geodataPart.go)
        {
            return;
        }

        if (gd.positions.Length == 1)
        {
            // rendering a point
            UpdateGeodataTransformPoint(geodataPart.go, gd.positions[0], gd);
        }
    }

    private void UpdateTransform(Part part, DrawSurfaceTask task)
    {
        VtsUtil.Matrix2Transform(part.go.transform, VtsUtil.V2U44(Math.Mul44x44(conv, System.Array.ConvertAll(task.data.mv, System.Convert.ToDouble))));
    }

    private bool UpdateGeodataTransformPoint(GameObject go, Point3D position, VtsGeodataObject gd)
    {
        if (position.ToVector3().magnitude > 1_000_000_000)
        {
            return false;
        }

        var model = System.Array.ConvertAll(gd.tile.model, System.Convert.ToDouble);

        var worldPosition = Math.Mul44x4(model, position.ToArray4());
        worldPosition[3] = 1;
        var worldUp = Math.Normalize4(worldPosition);


        var projPos = Math.Mul44x4(draws.camera.view, worldPosition);
        projPos[3] = 1;

        // use model up
        var mp4 = position.ToArray4();
        mp4[3] = 1;
        var wp4 = Math.Mul44x4(model, mp4);
        var wp3 = new double[3]{wp4[0], wp4[1], wp4[2]};
        var wn3 = Math.Normalize3(wp3);
        var wn4 = new double[4]{wn3[0], wn3[1], wn3[2], 0};
        var invModel = Math.Inverse44(model);
        var mn4 = Math.Mul44x4(invModel, wn4);
        var mn3 = new double[3]{mn4[0], mn4[1], mn4[2]};
        //var mn = Math.Normalize3(mn3);
        
        var mn4mul = Math.Mul44x4(draws.camera.view, mn4);
        var mn3mul = new double[3]{mn4mul[0], mn4mul[1], mn4mul[2]};
        var mn = Math.Normalize3(mn3mul);


        
        //var rot = Matrix4x4.Rotate(Quaternion.LookRotation(-VtsUtil.V2U3(draws.camera.eye), VtsUtil.V2U3(mn))); //
        //var rot = Matrix4x4.Rotate(Quaternion.LookRotation(ucam.transform.forward, VtsUtil.V2U3(mn))); //
        var rot = Matrix4x4.Rotate(Quaternion.LookRotation(VtsUtil.V2U3(draws.camera.eye), VtsUtil.V2U3(mn))); //
        

        var m =  Matrix4x4.Translate(VtsUtil.V2U4(projPos)) * rot * Matrix4x4.Scale(Vector3.one * 100000);
        var t = VtsUtil.V2U44(Math.Mul44x44(conv, VtsUtil.U2V44(m)));
        var pp = t.GetPosition();
        if (float.IsNaN(pp[0]) || float.IsNaN(pp[1]) || float.IsNaN(pp[2]))
        {
            return false;
        }
        VtsUtil.Matrix2Transform(go.transform, t);

        return true;
    }

    private Part InitPart(GameObject prefab, DrawSurfaceTask task)
    {
        Part part = new Part();
        part.go = Instantiate(prefab, partsGroup);
        UpdateTransform(part, task);
        part.mf = part.go.GetOrAddComponent<MeshFilter>();
        part.mf.mesh = (task.mesh as VtsMesh).Get();
        InitMaterial(propertyBlock, task);
        part.mr = part.go.GetOrAddComponent<MeshRenderer>();
        part.mr.SetPropertyBlock(propertyBlock);
        if (shiftingOriginMap)
            part.go.GetOrAddComponent<VtsObjectShiftingOrigin>().map = shiftingOriginMap;
        return part;
    }

    private void UpdatePart(Part part, DrawSurfaceTask task)
    {
        if (originHasShifted)
            UpdateTransform(part, task);
        part.mr.GetPropertyBlock(propertyBlock);
        UpdateMaterial(propertyBlock, task);
        part.mr.SetPropertyBlock(propertyBlock);
    }

    public GameObject opaquePrefab;
    public GameObject transparentPrefab;

    public GameObject pointPrefab;

    private readonly HashSet<VtsMesh> tmpKeys = new HashSet<VtsMesh>();
    private readonly HashSet<VtsGeodataObject> tmpGeoKeys = new HashSet<VtsGeodataObject>();
    private readonly Dictionary<VtsMesh, List<Part>> opaquePartsCache = new Dictionary<VtsMesh, List<Part>>();
    private readonly Dictionary<VtsMesh, List<Part>> transparentPartsCache = new Dictionary<VtsMesh, List<Part>>();
    private readonly Dictionary<VtsGeodataObject, List<GeodataPart>> geodataPartsCache = new Dictionary<VtsGeodataObject, List<GeodataPart>>();
    private Transform partsGroup;

    private double[] conv;
    private VtsMapShiftingOrigin shiftingOriginMap;
    private bool originHasShifted = false;
}

internal class Part
{
    public GameObject go;
    public MeshFilter mf;
    public MeshRenderer mr;
}

internal class GeodataPart
{
    public GameObject go;
}

internal struct DrawGeodataObjectTask
{
    public VtsGeodataObject geodata;
    public Color color;
}